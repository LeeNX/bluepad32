// SPDX-License-Identifier: Apache-2.0
// Nordic UART Service (NuS) on Bluepad32's BLE service. See uni_bt_nus.h.

#include "bt/uni_bt_nus.h"

#include <stdio.h>
#include <string.h>

#include <driver/gpio.h>
#include <nvs.h>

#include "bt/uni_bt_service.gatt.h"
#include "bt/uni_bt_service.h"
#ifdef CONFIG_BLUEPAD32_OTA
#include "uni_ota.h"
#endif
#include "sdkconfig.h"
#include "uni_common.h"
#include "uni_log.h"

#define NUS_RX_HANDLE ATT_CHARACTERISTIC_6E400002_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE
#define NUS_TX_HANDLE ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE
#define NUS_TX_CCCD_HANDLE ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_CLIENT_CONFIGURATION_HANDLE

#ifndef CONFIG_BLUEPAD32_BLE_NUS_MAX_CLIENTS
#define CONFIG_BLUEPAD32_BLE_NUS_MAX_CLIENTS 2
#endif
#ifndef CONFIG_BLUEPAD32_BLE_NUS_TX_BUFFER_SIZE
#define CONFIG_BLUEPAD32_BLE_NUS_TX_BUFFER_SIZE 1024
#endif

#define MAX_CLIENTS CONFIG_BLUEPAD32_BLE_NUS_MAX_CLIENTS
#define TX_BUFFER_SIZE CONFIG_BLUEPAD32_BLE_NUS_TX_BUFFER_SIZE
// Minimum ATT MTU (23) minus the notification header (3).
#define DEFAULT_PAYLOAD 20

typedef struct {
    hci_con_handle_t handle;  // HCI_CON_HANDLE_INVALID when the slot is free
    bool subscribed;
    bool notify_pending;  // a "can send" request is outstanding
    uint16_t payload;
    // Ring buffer of bytes waiting to be notified.
    uint8_t tx[TX_BUFFER_SIZE];
    uint16_t tx_head;
    uint16_t tx_len;
    btstack_context_callback_registration_t can_send;
} nus_client_t;

static nus_client_t clients[MAX_CLIENTS];
static uni_bt_nus_rx_callback_t rx_callback;
static uni_bt_nus_subscribe_callback_t subscribe_callback;
static uint32_t dropped_bytes;

static void request_send(nus_client_t* c);

static nus_client_t* client_for_handle(hci_con_handle_t handle) {
    if (handle == HCI_CON_HANDLE_INVALID)
        return NULL;
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (clients[i].handle == handle)
            return &clients[i];
    return NULL;
}

static nus_client_t* free_slot(void) {
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (clients[i].handle == HCI_CON_HANDLE_INVALID)
            return &clients[i];
    return NULL;
}

static void on_can_send(void* context) {
    nus_client_t* c = (nus_client_t*)context;
    c->notify_pending = false;
    if (c->handle == HCI_CON_HANDLE_INVALID || !c->subscribed || c->tx_len == 0)
        return;

    uint16_t n = c->tx_len < c->payload ? c->tx_len : c->payload;
    // Copy out of the ring buffer; a chunk can wrap around. Static: this runs on the BTstack thread only, and the
    // BTstack thread's stack (the "main" task) is small.
    static uint8_t chunk[256];
    if (n > sizeof(chunk))
        n = sizeof(chunk);
    for (uint16_t i = 0; i < n; i++)
        chunk[i] = c->tx[(c->tx_head + i) % TX_BUFFER_SIZE];

    uint8_t status = att_server_notify(c->handle, NUS_TX_HANDLE, chunk, n);
    if (status == ERROR_CODE_SUCCESS) {
        c->tx_head = (c->tx_head + n) % TX_BUFFER_SIZE;
        c->tx_len -= n;
    } else if (status != BTSTACK_ACL_BUFFERS_FULL && status != ERROR_CODE_COMMAND_DISALLOWED) {
        // Not a transient condition: drop what we tried to send so we don't spin on it.
        logi("NuS: notify failed, status=%#x, dropping %u bytes\n", status, n);
        c->tx_head = (c->tx_head + n) % TX_BUFFER_SIZE;
        c->tx_len -= n;
    }
    if (c->tx_len > 0)
        request_send(c);
}

static void request_send(nus_client_t* c) {
    if (c->notify_pending || c->handle == HCI_CON_HANDLE_INVALID || !c->subscribed)
        return;
    c->can_send.callback = &on_can_send;
    c->can_send.context = c;
    // BTstack can invoke the callback from inside this call when it can send right away, and the callback clears
    // notify_pending. So set it first, and undo it only if the request was refused.
    c->notify_pending = true;
    if (att_server_request_to_send_notification(&c->can_send, c->handle) != ERROR_CODE_SUCCESS)
        c->notify_pending = false;
}

static void enqueue(nus_client_t* c, const uint8_t* data, uint16_t len) {
    uint16_t room = TX_BUFFER_SIZE - c->tx_len;
    uint16_t n = len < room ? len : room;
    for (uint16_t i = 0; i < n; i++)
        c->tx[(c->tx_head + c->tx_len + i) % TX_BUFFER_SIZE] = data[i];
    c->tx_len += n;
    dropped_bytes += len - n;
    request_send(c);
}

//
// Access gate
//
#ifdef CONFIG_BLUEPAD32_BLE_NUS_GATE

#ifndef CONFIG_BLUEPAD32_BLE_NUS_SWITCH_GPIO
#define CONFIG_BLUEPAD32_BLE_NUS_SWITCH_GPIO -1
#endif
#ifndef CONFIG_BLUEPAD32_BLE_NUS_BUTTON_GPIO
#define CONFIG_BLUEPAD32_BLE_NUS_BUTTON_GPIO -1
#endif
#ifndef CONFIG_BLUEPAD32_BLE_NUS_BUTTON_HOLD_MS
#define CONFIG_BLUEPAD32_BLE_NUS_BUTTON_HOLD_MS 1000
#endif
#ifndef CONFIG_BLUEPAD32_BLE_NUS_OPEN_SECONDS
#define CONFIG_BLUEPAD32_BLE_NUS_OPEN_SECONDS 300
#endif
#define GATE_POLL_MS 50
#define GATE_NVS_NAMESPACE "bp32_nus"
#define GATE_NVS_KEY_LATCH "latch"

static struct {
    bool open;
    bool latched;
    bool switch_on;
    bool hold;
    bool window_forever;
    uint32_t window_left_s;
    uint32_t button_held_ms;
    bool button_fired;
} gate;
static btstack_timer_source_t gate_tick_timer;
static btstack_timer_source_t gate_poll_timer;

static bool gate_wanted(void) {
    return gate.latched || gate.switch_on || gate.hold || gate.window_forever || gate.window_left_s > 0;
}

static void gate_apply(void) {
    bool want = gate_wanted();
    if (want != gate.open) {
        gate.open = want;
        logi("NuS gate %s\n", want ? "open" : "closed");
        if (!want) {
            // Drop every central that connected to the BLE service. Gamepad links are not in this table.
            for (int i = 0; i < MAX_CLIENTS; i++)
                if (clients[i].handle != HCI_CON_HANDLE_INVALID)
                    gap_disconnect(clients[i].handle);
        }
    }
    uni_bt_service_update_advertising();
}

static void gate_latch_store(bool latched) {
    nvs_handle_t h;
    if (nvs_open(GATE_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
        return;
    nvs_set_u8(h, GATE_NVS_KEY_LATCH, latched);
    nvs_commit(h);
    nvs_close(h);
}

static bool gate_latch_load(void) {
    nvs_handle_t h;
    uint8_t v = 0;
    if (nvs_open(GATE_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return false;
    nvs_get_u8(h, GATE_NVS_KEY_LATCH, &v);
    nvs_close(h);
    return v != 0;
}

static void on_gate_tick(btstack_timer_source_t* ts) {
    if (gate.window_left_s > 0) {
        gate.window_left_s--;
        if (gate.window_left_s == 0)
            gate_apply();
    }
    btstack_run_loop_set_timer(ts, 1000);
    btstack_run_loop_add_timer(ts);
}

static void on_gate_poll(btstack_timer_source_t* ts) {
#if CONFIG_BLUEPAD32_BLE_NUS_SWITCH_GPIO >= 0
#ifdef CONFIG_BLUEPAD32_BLE_NUS_SWITCH_ACTIVE_LOW
    bool on = gpio_get_level(CONFIG_BLUEPAD32_BLE_NUS_SWITCH_GPIO) == 0;
#else
    bool on = gpio_get_level(CONFIG_BLUEPAD32_BLE_NUS_SWITCH_GPIO) == 1;
#endif
    if (on != gate.switch_on) {
        gate.switch_on = on;
        gate_apply();
    }
#endif
#if CONFIG_BLUEPAD32_BLE_NUS_BUTTON_GPIO >= 0
#ifdef CONFIG_BLUEPAD32_BLE_NUS_BUTTON_ACTIVE_LOW
    bool pressed = gpio_get_level(CONFIG_BLUEPAD32_BLE_NUS_BUTTON_GPIO) == 0;
#else
    bool pressed = gpio_get_level(CONFIG_BLUEPAD32_BLE_NUS_BUTTON_GPIO) == 1;
#endif
    if (pressed) {
        gate.button_held_ms += GATE_POLL_MS;
        if (gate.button_held_ms >= CONFIG_BLUEPAD32_BLE_NUS_BUTTON_HOLD_MS && !gate.button_fired) {
            gate.button_fired = true;
            uni_bt_nus_gate_open(CONFIG_BLUEPAD32_BLE_NUS_OPEN_SECONDS);
        }
    } else {
        gate.button_held_ms = 0;
        gate.button_fired = false;
    }
#endif
    btstack_run_loop_set_timer(ts, GATE_POLL_MS);
    btstack_run_loop_add_timer(ts);
}

static void gate_init(void) {
    memset(&gate, 0, sizeof(gate));
    gate.latched = gate_latch_load();

#if CONFIG_BLUEPAD32_BLE_NUS_SWITCH_GPIO >= 0 || CONFIG_BLUEPAD32_BLE_NUS_BUTTON_GPIO >= 0
    gpio_config_t io = {.mode = GPIO_MODE_INPUT};
    uint64_t mask = 0;
#if CONFIG_BLUEPAD32_BLE_NUS_SWITCH_GPIO >= 0
    mask |= 1ULL << CONFIG_BLUEPAD32_BLE_NUS_SWITCH_GPIO;
#endif
#if CONFIG_BLUEPAD32_BLE_NUS_BUTTON_GPIO >= 0
    mask |= 1ULL << CONFIG_BLUEPAD32_BLE_NUS_BUTTON_GPIO;
#endif
    io.pin_bit_mask = mask;
#if defined(CONFIG_BLUEPAD32_BLE_NUS_SWITCH_ACTIVE_LOW) || defined(CONFIG_BLUEPAD32_BLE_NUS_BUTTON_ACTIVE_LOW)
    io.pull_up_en = GPIO_PULLUP_ENABLE;
#endif
    gpio_config(&io);
#endif

    btstack_run_loop_remove_timer(&gate_tick_timer);
    gate_tick_timer.process = &on_gate_tick;
    btstack_run_loop_set_timer(&gate_tick_timer, 1000);
    btstack_run_loop_add_timer(&gate_tick_timer);
    btstack_run_loop_remove_timer(&gate_poll_timer);
    gate_poll_timer.process = &on_gate_poll;
    btstack_run_loop_set_timer(&gate_poll_timer, GATE_POLL_MS);
    btstack_run_loop_add_timer(&gate_poll_timer);

    gate.open = gate_wanted();
    logi("NuS gate: %s (latched=%d)\n", gate.open ? "open" : "closed", gate.latched);
}

bool uni_bt_nus_gate_is_open(void) {
    return gate.open;
}

void uni_bt_nus_gate_open(uint32_t seconds) {
    if (seconds == 0) {
        gate.window_forever = true;
    } else if (!gate.window_forever && seconds > gate.window_left_s) {
        gate.window_left_s = seconds;
    }
    logi("NuS gate: open request (%us)\n", (unsigned)seconds);
    gate_apply();
}

void uni_bt_nus_gate_close(void) {
    gate.window_forever = false;
    gate.window_left_s = 0;
    if (gate.latched) {
        gate.latched = false;
        gate_latch_store(false);
    }
    gate_apply();
}

void uni_bt_nus_gate_set_latch(bool latched) {
    if (gate.latched == latched)
        return;
    gate.latched = latched;
    gate_latch_store(latched);
    gate_apply();
}

bool uni_bt_nus_gate_is_latched(void) {
    return gate.latched;
}

void uni_bt_nus_gate_hold(bool hold) {
    if (gate.hold == hold)
        return;
    gate.hold = hold;
    gate_apply();
}

uint32_t uni_bt_nus_gate_seconds_left(void) {
    return gate.window_left_s;
}

#else  // !CONFIG_BLUEPAD32_BLE_NUS_GATE: always open

static void gate_init(void) {}
bool uni_bt_nus_gate_is_open(void) {
    return true;
}
void uni_bt_nus_gate_open(uint32_t seconds) {
    ARG_UNUSED(seconds);
}
void uni_bt_nus_gate_close(void) {}
void uni_bt_nus_gate_set_latch(bool latched) {
    ARG_UNUSED(latched);
}
bool uni_bt_nus_gate_is_latched(void) {
    return false;
}
void uni_bt_nus_gate_hold(bool hold) {
    ARG_UNUSED(hold);
}
uint32_t uni_bt_nus_gate_seconds_left(void) {
    return 0;
}

#endif  // CONFIG_BLUEPAD32_BLE_NUS_GATE

void uni_bt_nus_init(void) {
    memset(clients, 0, sizeof(clients));
    for (int i = 0; i < MAX_CLIENTS; i++)
        clients[i].handle = HCI_CON_HANDLE_INVALID;
    gate_init();
}

void uni_bt_nus_set_rx_callback(uni_bt_nus_rx_callback_t cb) {
    rx_callback = cb;
}

void uni_bt_nus_set_subscribe_callback(uni_bt_nus_subscribe_callback_t cb) {
    subscribe_callback = cb;
}

int uni_bt_nus_send(hci_con_handle_t client, const uint8_t* data, uint16_t len) {
    int queued = 0;
    if (client != HCI_CON_HANDLE_INVALID) {
        nus_client_t* c = client_for_handle(client);
        if (c && c->subscribed) {
            enqueue(c, data, len);
            queued++;
        }
        return queued;
    }
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].handle != HCI_CON_HANDLE_INVALID && clients[i].subscribed) {
            enqueue(&clients[i], data, len);
            queued++;
        }
    }
    return queued;
}

int uni_bt_nus_vprintf(hci_con_handle_t client, const char* fmt, va_list ap) {
    char buf[192];
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n < 0)
        return n;
    if (n >= (int)sizeof(buf))
        n = sizeof(buf) - 1;
    return uni_bt_nus_send(client, (const uint8_t*)buf, (uint16_t)n);
}

int uni_bt_nus_printf(hci_con_handle_t client, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int r = uni_bt_nus_vprintf(client, fmt, ap);
    va_end(ap);
    return r;
}

int uni_bt_nus_client_count(void) {
    int n = 0;
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (clients[i].handle != HCI_CON_HANDLE_INVALID)
            n++;
    return n;
}

int uni_bt_nus_subscriber_count(void) {
    int n = 0;
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (clients[i].handle != HCI_CON_HANDLE_INVALID && clients[i].subscribed)
            n++;
    return n;
}

uint32_t uni_bt_nus_dropped_bytes(void) {
    return dropped_bytes;
}

uint16_t uni_bt_nus_payload_size(hci_con_handle_t client) {
    nus_client_t* c = client_for_handle(client);
    return c ? c->payload : 0;
}

bool uni_bt_nus_has_free_slot(void) {
    return uni_bt_nus_client_count() < MAX_CLIENTS;
}

void uni_bt_nus_on_connected(hci_con_handle_t handle) {
    // The ATT server also reports links where Bluepad32 is the central (its gamepads). Only peripheral-role links
    // are NuS clients.
    if (gap_get_role(handle) != HCI_ROLE_SLAVE)
        return;
    nus_client_t* c = free_slot();
    if (!c) {
        logi("NuS: no free slot for connection %#x\n", handle);
        return;
    }
    memset(c, 0, sizeof(*c));
    c->handle = handle;
    c->payload = DEFAULT_PAYLOAD;
    logi("NuS: client connected, handle=%#x (%d/%d)\n", handle, uni_bt_nus_client_count(), MAX_CLIENTS);
}

void uni_bt_nus_on_disconnected(hci_con_handle_t handle) {
    nus_client_t* c = client_for_handle(handle);
    if (!c)
        return;
    bool was_subscribed = c->subscribed;
#ifdef CONFIG_BLUEPAD32_OTA
    uni_ota_on_client_disconnected(handle);
#endif
    memset(c, 0, sizeof(*c));
    c->handle = HCI_CON_HANDLE_INVALID;
    logi("NuS: client disconnected, handle=%#x\n", handle);
    if (was_subscribed && subscribe_callback)
        subscribe_callback(handle, false);
}

void uni_bt_nus_on_mtu(hci_con_handle_t handle) {
    nus_client_t* c = client_for_handle(handle);
    if (!c)
        return;
    uint16_t mtu = att_server_get_mtu(handle);
    c->payload = mtu > 3 ? mtu - 3 : DEFAULT_PAYLOAD;
    if (c->payload > 244)
        c->payload = 244;
    logi("NuS: handle=%#x payload=%u\n", handle, c->payload);
}

bool uni_bt_nus_on_write(hci_con_handle_t handle, uint16_t att_handle, const uint8_t* data, uint16_t len, int* result) {
    if (att_handle == NUS_RX_HANDLE) {
        *result = ATT_ERROR_SUCCESS;
        if (rx_callback)
            rx_callback(handle, data, len);
        return true;
    }
    if (att_handle == NUS_TX_CCCD_HANDLE) {
        nus_client_t* c = client_for_handle(handle);
        if (!c || len < 2) {
            *result = ATT_ERROR_REQUEST_NOT_SUPPORTED;
            return true;
        }
        bool sub = little_endian_read_16(data, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        if (sub != c->subscribed) {
            c->subscribed = sub;
            if (!sub) {
                c->tx_head = c->tx_len = 0;
            }
            logi("NuS: handle=%#x subscribed=%d\n", handle, sub);
            if (subscribe_callback)
                subscribe_callback(handle, sub);
        }
        *result = ATT_ERROR_SUCCESS;
        return true;
    }
    return false;
}
