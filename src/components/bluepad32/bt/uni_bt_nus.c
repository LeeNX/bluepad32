// SPDX-License-Identifier: Apache-2.0
// Nordic UART Service (NuS) on Bluepad32's BLE service. See uni_bt_nus.h.

#include "bt/uni_bt_nus.h"

#include <stdio.h>
#include <string.h>

#include "bt/uni_bt_service.gatt.h"
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
    if (att_server_request_to_send_notification(&c->can_send, c->handle) == ERROR_CODE_SUCCESS)
        c->notify_pending = true;
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

void uni_bt_nus_init(void) {
    memset(clients, 0, sizeof(clients));
    for (int i = 0; i < MAX_CLIENTS; i++)
        clients[i].handle = HCI_CON_HANDLE_INVALID;
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
