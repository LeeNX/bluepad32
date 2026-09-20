// SPDX-License-Identifier: Apache-2.0
// Over-the-air firmware update, over the Nordic UART Service (NuS). See uni_ota.h for the protocol.

#include "uni_ota.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <driver/gpio.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <nvs_flash.h>
#include <esp_random.h>
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>

#include "bt/uni_bt.h"
#include "bt/uni_bt_nus.h"
#include "sdkconfig.h"
#include "uni_common.h"
#include "uni_log.h"
#include "uni_system.h"

#ifndef CONFIG_BLUEPAD32_OTA_ARM_SECONDS
#define CONFIG_BLUEPAD32_OTA_ARM_SECONDS 120
#endif
#ifndef CONFIG_BLUEPAD32_OTA_BUTTON_GPIO
#define CONFIG_BLUEPAD32_OTA_BUTTON_GPIO -1
#endif
#ifndef CONFIG_BLUEPAD32_OTA_BUTTON_HOLD_MS
#define CONFIG_BLUEPAD32_OTA_BUTTON_HOLD_MS 1500
#endif
#ifndef CONFIG_BLUEPAD32_OTA_FACTORY_RESET_HOLD_MS
#define CONFIG_BLUEPAD32_OTA_FACTORY_RESET_HOLD_MS 10000
#endif
#ifndef CONFIG_BLUEPAD32_OTA_AUTO_CONFIRM_SEC
#define CONFIG_BLUEPAD32_OTA_AUTO_CONFIRM_SEC 15
#endif
#ifndef CONFIG_BLUEPAD32_OTA_PSK
#define CONFIG_BLUEPAD32_OTA_PSK ""
#endif

#ifdef CONFIG_BLUEPAD32_OTA_ALLOW_AUTH
#define ALLOW_AUTH (sizeof(CONFIG_BLUEPAD32_OTA_PSK) > 1)
#else
#define ALLOW_AUTH 0
#endif
#ifdef CONFIG_BLUEPAD32_OTA_ALLOW_UNAUTH
#define ALLOW_UNAUTH 1
#else
#define ALLOW_UNAUTH 0
#endif
#ifdef CONFIG_BLUEPAD32_OTA_REQUIRE_ARM_FOR_AUTH
#define REQUIRE_ARM_FOR_AUTH 1
#else
#define REQUIRE_ARM_FOR_AUTH 0
#endif

#define FRAME_HEADER_LEN 5  // magic + u32 offset
// Largest payload per frame: a 244-byte notification payload (ATT MTU 247) minus the 5-byte frame header.
#define MAX_CHUNK 239
// Bytes the client may send before it has to wait for an ack. Bigger is faster; the client's write-without-response
// queue and our flash writes are the limit.
#define ACK_WINDOW 8192
#define NONCE_LEN 32
#define NONCE_LIFETIME_MS 30000
#define REBOOT_DELAY_MS 1500
#define BUTTON_POLL_MS 50

typedef enum {
    STATE_IDLE,
    STATE_RECEIVING,
} ota_state_t;

static struct {
    ota_state_t state;
    hci_con_handle_t client;

    bool armed;
    uint32_t arm_left_s;

    uint8_t nonce[NONCE_LEN];
    bool nonce_valid;

    const esp_partition_t* partition;
    esp_ota_handle_t handle;
    uint32_t size;
    uint32_t offset;
    uint32_t acked;
    uint8_t expected_sha[32];
    mbedtls_sha256_context sha;
} g;

static btstack_timer_source_t arm_timer;
static btstack_timer_source_t nonce_timer;
static btstack_timer_source_t reboot_timer;
static btstack_timer_source_t confirm_timer;
static btstack_timer_source_t button_timer;
static uint32_t button_held_ms;
static bool button_fired;
static bool button_factory_fired;

static void reply(hci_con_handle_t client, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
static void reply(hci_con_handle_t client, const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    if (n >= (int)sizeof(buf))
        n = sizeof(buf) - 1;
    logi("OTA: %s", buf);
    uni_bt_nus_send(client, (const uint8_t*)buf, (uint16_t)n);
}

//
// Hex helpers
//
static bool hex_decode(const char* hex, uint8_t* out, size_t out_len) {
    if (strlen(hex) != out_len * 2)
        return false;
    for (size_t i = 0; i < out_len; i++) {
        if (!isxdigit((unsigned char)hex[2 * i]) || !isxdigit((unsigned char)hex[2 * i + 1]))
            return false;
        char b[3] = {hex[2 * i], hex[2 * i + 1], 0};
        out[i] = (uint8_t)strtoul(b, NULL, 16);
    }
    return true;
}

static void hex_encode(const uint8_t* in, size_t len, char* out) {
    for (size_t i = 0; i < len; i++)
        sprintf(out + 2 * i, "%02x", in[i]);
}

static bool ct_equal(const uint8_t* a, const uint8_t* b, size_t len) {
    uint8_t diff = 0;
    for (size_t i = 0; i < len; i++)
        diff |= a[i] ^ b[i];
    return diff == 0;
}

//
// Arming
//
static void on_arm_tick(btstack_timer_source_t* ts) {
    if (!g.armed)
        return;
    // Arming only has to last until a transfer starts. Never pull it out from under one.
    if (g.arm_left_s > 0 && g.state != STATE_RECEIVING)
        g.arm_left_s--;
    if (g.arm_left_s == 0) {
        g.armed = false;
        logi("OTA: arm timed out\n");
        uni_bt_nus_printf(HCI_CON_HANDLE_INVALID, "OTA disarmed (timeout)\n");
        return;
    }
    btstack_run_loop_set_timer(ts, 1000);
    btstack_run_loop_add_timer(ts);
}

void uni_ota_arm(uint32_t seconds) {
    if (seconds == 0)
        seconds = CONFIG_BLUEPAD32_OTA_ARM_SECONDS;
    btstack_run_loop_remove_timer(&arm_timer);
    g.armed = true;
    g.arm_left_s = seconds;
    arm_timer.process = &on_arm_tick;
    btstack_run_loop_set_timer(&arm_timer, 1000);
    btstack_run_loop_add_timer(&arm_timer);
    logi("OTA: armed for %us\n", (unsigned)seconds);
    // OTA needs a way in. Arming is a physical action, so it also opens the BLE access gate for as long.
    uni_bt_nus_gate_open(seconds);
    uni_bt_nus_printf(HCI_CON_HANDLE_INVALID, "OTA armed %us\n", (unsigned)seconds);
}

void uni_ota_disarm(void) {
    if (!g.armed)
        return;
    btstack_run_loop_remove_timer(&arm_timer);
    g.armed = false;
    g.arm_left_s = 0;
    logi("OTA: disarmed\n");
    uni_bt_nus_printf(HCI_CON_HANDLE_INVALID, "OTA disarmed\n");
}

bool uni_ota_is_armed(void) {
    return g.armed;
}

static void on_button_poll(btstack_timer_source_t* ts) {
#if CONFIG_BLUEPAD32_OTA_BUTTON_GPIO >= 0
#ifdef CONFIG_BLUEPAD32_OTA_BUTTON_ACTIVE_LOW
    const int active = 0;
#else
    const int active = 1;
#endif
    if (gpio_get_level(CONFIG_BLUEPAD32_OTA_BUTTON_GPIO) == active) {
        button_held_ms += BUTTON_POLL_MS;
        if (button_held_ms >= CONFIG_BLUEPAD32_OTA_BUTTON_HOLD_MS && !button_fired) {
            button_fired = true;
            uni_ota_arm(0);
        }
        // A much longer hold is a factory reset. Being at the device is the authentication.
        if (CONFIG_BLUEPAD32_OTA_FACTORY_RESET_HOLD_MS > 0 &&
            button_held_ms >= CONFIG_BLUEPAD32_OTA_FACTORY_RESET_HOLD_MS && !button_factory_fired) {
            button_factory_fired = true;
            logi("OTA: button held %u ms, factory reset\n", (unsigned)button_held_ms);
            uni_ota_factory_reset();
        }
    } else {
        button_held_ms = 0;
        button_fired = false;
        button_factory_fired = false;
    }
#endif
    btstack_run_loop_set_timer(ts, BUTTON_POLL_MS);
    btstack_run_loop_add_timer(ts);
}

//
// Session
//
// A transfer wants the radio to itself. The ESP32 shares it between the gamepad links, scanning for new gamepads,
// advertising for new centrals and the NuS link doing the transfer; scanning in particular eats connection events.
static bool quiet_was_scanning;

static void set_quiet(bool quiet) {
    if (quiet) {
        quiet_was_scanning = uni_bt_is_scanning();
        if (quiet_was_scanning)
            uni_bt_stop_scanning_unsafe();
        uni_bt_nus_pause_advertising(true);
    } else {
        uni_bt_nus_pause_advertising(false);
        if (quiet_was_scanning)
            uni_bt_start_scanning_and_autoconnect_unsafe();
        quiet_was_scanning = false;
    }
}

static void abort_session(void) {
    if (g.state == STATE_RECEIVING) {
        esp_ota_abort(g.handle);
        mbedtls_sha256_free(&g.sha);
    }
    g.state = STATE_IDLE;
    g.client = HCI_CON_HANDLE_INVALID;
    uni_bt_nus_gate_hold(false);
    set_quiet(false);
}

static void on_reboot(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);
    uni_system_reboot();
}

static void on_nonce_expired(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);
    g.nonce_valid = false;
}

static const esp_partition_t* factory_partition(void) {
    return esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
}

static const char* auth_modes(void) {
    if (ALLOW_AUTH && ALLOW_UNAUTH)
        return "psk+none";
    if (ALLOW_AUTH)
        return "psk";
    if (ALLOW_UNAUTH)
        return "none";
    return "off";
}

static void cmd_status(hci_con_handle_t client) {
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_app_desc_t desc = {0};
    if (running)
        esp_ota_get_partition_description(running, &desc);
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    if (running)
        esp_ota_get_state_partition(running, &st);
    reply(client, "OTA status armed=%d arm_left=%u state=%s auth=%s running=%s ver=%s pending_verify=%d factory=%d\n",
          g.armed, (unsigned)g.arm_left_s, g.state == STATE_IDLE ? "idle" : "receiving", auth_modes(),
          running ? running->label : "?", desc.version, st == ESP_OTA_IMG_PENDING_VERIFY, factory_partition() != NULL);
}

static void cmd_challenge(hci_con_handle_t client) {
    if (!ALLOW_AUTH) {
        reply(client, "OTA err auth disabled\n");
        return;
    }
    esp_fill_random(g.nonce, sizeof(g.nonce));
    g.nonce_valid = true;
    btstack_run_loop_remove_timer(&nonce_timer);
    nonce_timer.process = &on_nonce_expired;
    btstack_run_loop_set_timer(&nonce_timer, NONCE_LIFETIME_MS);
    btstack_run_loop_add_timer(&nonce_timer);

    char hex[NONCE_LEN * 2 + 1];
    hex_encode(g.nonce, sizeof(g.nonce), hex);
    reply(client, "OTA nonce %s\n", hex);
}

// HMAC-SHA256(psk, nonce || "<size>" || "<sha256 hex>")
static bool verify_hmac(uint32_t size, const char* sha_hex, const uint8_t* given) {
    char msg[NONCE_LEN + 16 + 64 + 1];
    size_t n = 0;
    memcpy(msg, g.nonce, NONCE_LEN);
    n += NONCE_LEN;
    n += (size_t)snprintf(msg + n, sizeof(msg) - n, "%u", (unsigned)size);
    memcpy(msg + n, sha_hex, 64);
    n += 64;

    uint8_t mac[32];
    const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (mbedtls_md_hmac(info, (const uint8_t*)CONFIG_BLUEPAD32_OTA_PSK, strlen(CONFIG_BLUEPAD32_OTA_PSK),
                        (const uint8_t*)msg, n, mac) != 0)
        return false;
    return ct_equal(mac, given, sizeof(mac));
}

static void cmd_begin(hci_con_handle_t client, char* size_s, char* sha_s, char* hmac_s) {
    if (g.state != STATE_IDLE) {
        reply(client, "OTA err busy\n");
        return;
    }
    if (!size_s || !sha_s) {
        reply(client, "OTA err usage: ota begin <size> <sha256> [<hmac>]\n");
        return;
    }
    uint32_t size = (uint32_t)strtoul(size_s, NULL, 10);
    uint8_t sha[32];
    if (size == 0 || !hex_decode(sha_s, sha, sizeof(sha))) {
        reply(client, "OTA err bad size or sha256\n");
        return;
    }

    bool authed = hmac_s != NULL;
    if (authed) {
        uint8_t mac[32];
        bool ok = ALLOW_AUTH && g.nonce_valid && hex_decode(hmac_s, mac, sizeof(mac));
        bool nonce_was_valid = g.nonce_valid;
        g.nonce_valid = false;  // single use, whatever the outcome
        if (!ALLOW_AUTH) {
            reply(client, "OTA err auth disabled\n");
            return;
        }
        if (!nonce_was_valid) {
            reply(client, "OTA err no challenge\n");
            return;
        }
        if (!ok || !verify_hmac(size, sha_s, mac)) {
            reply(client, "OTA err auth failed\n");
            return;
        }
    } else if (!ALLOW_UNAUTH) {
        reply(client, "OTA err auth required\n");
        return;
    }

    // Arming is a physical-presence check. Unauthenticated always needs it; authenticated does by default.
    if ((!authed || REQUIRE_ARM_FOR_AUTH) && !g.armed) {
        reply(client, "OTA err not armed\n");
        return;
    }

    uint16_t payload = uni_bt_nus_payload_size(client);
    if (payload < FRAME_HEADER_LEN + 16) {
        reply(client, "OTA err mtu too small (payload %u)\n", payload);
        return;
    }
    uint16_t chunk = payload - FRAME_HEADER_LEN;
    if (chunk > MAX_CHUNK)
        chunk = MAX_CHUNK;

    const esp_partition_t* part = esp_ota_get_next_update_partition(NULL);
    if (!part) {
        reply(client, "OTA err no update partition\n");
        return;
    }
    if (size > part->size) {
        reply(client, "OTA err image too big (%u > %u)\n", (unsigned)size, (unsigned)part->size);
        return;
    }
    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        reply(client, "OTA err flash begin %s\n", esp_err_to_name(err));
        return;
    }

    g.state = STATE_RECEIVING;
    uni_bt_nus_gate_hold(true);  // don't close the BLE access gate on the client mid-transfer
    set_quiet(true);
    g.client = client;
    g.partition = part;
    g.handle = handle;
    g.size = size;
    g.offset = 0;
    g.acked = 0;
    memcpy(g.expected_sha, sha, sizeof(sha));
    mbedtls_sha256_init(&g.sha);
    mbedtls_sha256_starts(&g.sha, 0);
    reply(client, "OTA ready chunk=%u window=%u\n", chunk, ACK_WINDOW);
}

static void cmd_end(hci_con_handle_t client) {
    if (g.state != STATE_RECEIVING || g.client != client) {
        reply(client, "OTA err no transfer\n");
        return;
    }
    if (g.offset != g.size) {
        reply(client, "OTA err incomplete %u/%u\n", (unsigned)g.offset, (unsigned)g.size);
        return;
    }
    uint8_t digest[32];
    mbedtls_sha256_finish(&g.sha, digest);
    mbedtls_sha256_free(&g.sha);
    bool sha_ok = ct_equal(digest, g.expected_sha, sizeof(digest));

    esp_err_t err = esp_ota_end(g.handle);  // also validates the image
    g.state = STATE_IDLE;
    g.client = HCI_CON_HANDLE_INVALID;
    uni_bt_nus_gate_hold(false);
    set_quiet(false);
    if (!sha_ok) {
        reply(client, "OTA err sha256 mismatch\n");
        return;
    }
    if (err != ESP_OK) {
        reply(client, "OTA err image invalid %s\n", esp_err_to_name(err));
        return;
    }
    err = esp_ota_set_boot_partition(g.partition);
    if (err != ESP_OK) {
        reply(client, "OTA err set boot %s\n", esp_err_to_name(err));
        return;
    }
    uni_ota_disarm();
    reply(client, "OTA done\n");
    reboot_timer.process = &on_reboot;
    btstack_run_loop_set_timer(&reboot_timer, REBOOT_DELAY_MS);
    btstack_run_loop_add_timer(&reboot_timer);
}

//
// Factory reset
//
static void on_factory_reboot(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);
    uni_system_reboot();
}

bool uni_ota_factory_reset(void) {
    if (g.state != STATE_IDLE)
        abort_session();

    const esp_partition_t* otadata = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
    bool to_factory = factory_partition() != NULL;
    // With otadata erased the bootloader runs the factory app, or the first OTA slot if there is no factory app.
    if (otadata && esp_partition_erase_range(otadata, 0, otadata->size) != ESP_OK)
        return false;
    // Stored settings: Bluetooth bonds, the NuS access latch, everything else in NVS.
    nvs_flash_deinit();
    if (nvs_flash_erase() != ESP_OK)
        return false;

    logi("OTA: factory reset done (%s), rebooting\n", to_factory ? "factory image" : "settings only");
    uni_ota_disarm();
    reboot_timer.process = &on_factory_reboot;
    btstack_run_loop_set_timer(&reboot_timer, REBOOT_DELAY_MS);
    btstack_run_loop_add_timer(&reboot_timer);
    return true;
}

// "ota factory-reset [<hmac>]": same policy as "ota begin". hmac = HMAC-SHA256(psk, nonce || "factory-reset").
static void cmd_factory_reset(hci_con_handle_t client, char* hmac_s) {
    if (g.state != STATE_IDLE) {
        reply(client, "OTA err busy\n");
        return;
    }
    bool authed = hmac_s != NULL;
    if (authed) {
        bool nonce_was_valid = g.nonce_valid;
        g.nonce_valid = false;  // single use
        if (!ALLOW_AUTH) {
            reply(client, "OTA err auth disabled\n");
            return;
        }
        if (!nonce_was_valid) {
            reply(client, "OTA err no challenge\n");
            return;
        }
        uint8_t given[32], mac[32];
        static const char kMsg[] = "factory-reset";
        uint8_t msg[NONCE_LEN + sizeof(kMsg) - 1];
        memcpy(msg, g.nonce, NONCE_LEN);
        memcpy(msg + NONCE_LEN, kMsg, sizeof(kMsg) - 1);
        const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        if (!hex_decode(hmac_s, given, sizeof(given)) ||
            mbedtls_md_hmac(info, (const uint8_t*)CONFIG_BLUEPAD32_OTA_PSK, strlen(CONFIG_BLUEPAD32_OTA_PSK), msg,
                            sizeof(msg), mac) != 0 ||
            !ct_equal(mac, given, sizeof(mac))) {
            reply(client, "OTA err auth failed\n");
            return;
        }
    } else if (!ALLOW_UNAUTH) {
        reply(client, "OTA err auth required\n");
        return;
    }
    if ((!authed || REQUIRE_ARM_FOR_AUTH) && !g.armed) {
        reply(client, "OTA err not armed\n");
        return;
    }
    if (!uni_ota_factory_reset()) {
        reply(client, "OTA err factory reset failed\n");
        return;
    }
    reply(client, "OTA done factory-reset mode=%s\n", factory_partition() ? "factory" : "settings");
}

bool uni_ota_handle_line(hci_con_handle_t client, const char* line) {
    if (strncasecmp(line, "ota", 3) != 0 || (line[3] != '\0' && !isspace((unsigned char)line[3])))
        return false;

    char buf[224];
    strlcpy(buf, line, sizeof(buf));
    char* save = NULL;
    strtok_r(buf, " \t", &save);  // "ota"
    char* cmd = strtok_r(NULL, " \t", &save);
    if (!cmd) {
        reply(client, "OTA err usage: ota status|challenge|begin|end|abort\n");
        return true;
    }
    if (strcasecmp(cmd, "status") == 0) {
        cmd_status(client);
    } else if (strcasecmp(cmd, "challenge") == 0) {
        cmd_challenge(client);
    } else if (strcasecmp(cmd, "begin") == 0) {
        char* a = strtok_r(NULL, " \t", &save);
        char* b = strtok_r(NULL, " \t", &save);
        char* c = strtok_r(NULL, " \t", &save);
        cmd_begin(client, a, b, c);
    } else if (strcasecmp(cmd, "end") == 0) {
        cmd_end(client);
    } else if (strcasecmp(cmd, "factory-reset") == 0) {
        cmd_factory_reset(client, strtok_r(NULL, " \t", &save));
    } else if (strcasecmp(cmd, "abort") == 0) {
        abort_session();
        reply(client, "OTA ok aborted\n");
    } else {
        reply(client, "OTA err unknown command\n");
    }
    return true;
}

bool uni_ota_handle_frame(hci_con_handle_t client, const uint8_t* data, uint16_t len) {
    if (len < 1 || data[0] != UNI_OTA_FRAME_MAGIC)
        return false;
    if (g.state != STATE_RECEIVING || g.client != client) {
        reply(client, "OTA err no transfer\n");
        return true;
    }
    if (len <= FRAME_HEADER_LEN) {
        reply(client, "OTA err short frame\n");
        return true;
    }
    uint32_t offset = little_endian_read_32(data, 1);
    const uint8_t* payload = data + FRAME_HEADER_LEN;
    uint16_t n = len - FRAME_HEADER_LEN;

    if (offset != g.offset) {
        // Let the client resynchronise.
        reply(client, "OTA err offset expected=%u\n", (unsigned)g.offset);
        return true;
    }
    if (g.offset + n > g.size) {
        abort_session();
        reply(client, "OTA err too much data\n");
        return true;
    }
    esp_err_t err = esp_ota_write(g.handle, payload, n);
    if (err != ESP_OK) {
        abort_session();
        reply(client, "OTA err flash write %s\n", esp_err_to_name(err));
        return true;
    }
    mbedtls_sha256_update(&g.sha, payload, n);
    g.offset += n;

    if (g.offset - g.acked >= ACK_WINDOW || g.offset == g.size) {
        g.acked = g.offset;
        reply(client, "OTA ack %u\n", (unsigned)g.offset);
    }
    return true;
}

void uni_ota_on_client_disconnected(hci_con_handle_t client) {
    if (g.state == STATE_RECEIVING && g.client == client) {
        logi("OTA: client %#x went away, aborting transfer\n", client);
        abort_session();
    }
}

//
// Rollback
//
void uni_ota_mark_valid(void) {
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK)
        logi("OTA: running image confirmed\n");
    else if (err != ESP_ERR_NOT_SUPPORTED)
        logi("OTA: confirm: %s\n", esp_err_to_name(err));
}

static void on_confirm(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);
    uni_ota_mark_valid();
}

void uni_ota_init(void) {
    memset(&g, 0, sizeof(g));
    g.client = HCI_CON_HANDLE_INVALID;

    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (running && esp_ota_get_state_partition(running, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        logi("OTA: new image is pending verification\n");
        if (CONFIG_BLUEPAD32_OTA_AUTO_CONFIRM_SEC > 0) {
            confirm_timer.process = &on_confirm;
            btstack_run_loop_set_timer(&confirm_timer, CONFIG_BLUEPAD32_OTA_AUTO_CONFIRM_SEC * 1000);
            btstack_run_loop_add_timer(&confirm_timer);
        }
    }

#if CONFIG_BLUEPAD32_OTA_BUTTON_GPIO >= 0
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_BLUEPAD32_OTA_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
#ifdef CONFIG_BLUEPAD32_OTA_BUTTON_ACTIVE_LOW
        .pull_up_en = GPIO_PULLUP_ENABLE,
#else
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
#endif
    };
    gpio_config(&io);
    button_timer.process = &on_button_poll;
    btstack_run_loop_set_timer(&button_timer, BUTTON_POLL_MS);
    btstack_run_loop_add_timer(&button_timer);
    logi("OTA: arm button on GPIO %d (hold %d ms)\n", CONFIG_BLUEPAD32_OTA_BUTTON_GPIO,
         CONFIG_BLUEPAD32_OTA_BUTTON_HOLD_MS);
#endif
    logi("OTA: ready, auth=%s\n", auth_modes());
}
