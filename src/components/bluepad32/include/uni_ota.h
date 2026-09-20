// SPDX-License-Identifier: Apache-2.0
// Over-the-air firmware update, over the Nordic UART Service (NuS).
//
// Lets a BLE central update the firmware without a wired connection (a robot may be hard to open, or have no USB
// port). Everything must be called from the BTstack thread.
//
// Safety:
//  - OTA must be *armed* first, by a physical action: a button (CONFIG_BLUEPAD32_OTA_BUTTON_GPIO), or a platform
//    calling uni_ota_arm(), e.g. from a gamepad button chord. Arming times out.
//  - Authenticated mode: the client proves knowledge of a pre-shared key (HMAC-SHA256 over a device nonce, the image
//    size and its SHA-256).
//  - Unauthenticated mode is only available if CONFIG_BLUEPAD32_OTA_ALLOW_UNAUTH is set, and still needs arming.
//  - The SHA-256 of the whole image is checked before the boot partition is switched.
//  - With the bootloader's app rollback enabled, a new image must call uni_ota_mark_valid() (done automatically after
//    CONFIG_BLUEPAD32_OTA_AUTO_CONFIRM_SEC seconds) or the previous one is restored on the next reset.
//
// Wire protocol, all over NuS:
//   client -> device, text lines (one NuS write each, or split across writes; the platform assembles lines and passes
//   them to uni_ota_handle_line()):
//     ota status
//     ota challenge
//     ota begin <size> <sha256 hex> [<hmac hex>]     hmac = HMAC-SHA256(psk, nonce || "<size>" || "<sha256 hex>")
//     ota end
//     ota abort
//   client -> device, binary (one NuS write each), passed to uni_ota_handle_frame():
//     0xB3, offset (u32 LE), payload...
//   device -> requesting client, text lines:
//     OTA status armed=<0|1> arm_left=<s> state=<idle|receiving> auth=<none|psk|psk+none> running=<label> ver=<ver>
//     OTA nonce <64 hex>
//     OTA ready chunk=<max payload> window=<bytes>
//     OTA ack <next offset>
//     OTA done                                          (the device reboots shortly after)
//     OTA err <reason>

#ifndef UNI_OTA_H
#define UNI_OTA_H

#include <stdbool.h>
#include <stdint.h>

#include <btstack.h>

#define UNI_OTA_FRAME_MAGIC 0xB3

void uni_ota_init(void);

// Arm OTA for `seconds` (0 = CONFIG_BLUEPAD32_OTA_ARM_SECONDS).
void uni_ota_arm(uint32_t seconds);
void uni_ota_disarm(void);
bool uni_ota_is_armed(void);

// Returns true if `line` (no terminator) was an "ota ..." command and has been handled.
bool uni_ota_handle_line(hci_con_handle_t client, const char* line);

// Returns true if `data` is an OTA data frame (starts with UNI_OTA_FRAME_MAGIC) and has been handled.
bool uni_ota_handle_frame(hci_con_handle_t client, const uint8_t* data, uint16_t len);

// A NuS client went away: abort its transfer, if any.
void uni_ota_on_client_disconnected(hci_con_handle_t client);

// Confirm the running image (cancel rollback). Safe to call when rollback isn't pending.
void uni_ota_mark_valid(void);

#endif  // UNI_OTA_H
