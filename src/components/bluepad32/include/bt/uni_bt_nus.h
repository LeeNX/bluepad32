// SPDX-License-Identifier: Apache-2.0
// Nordic UART Service (NuS) on Bluepad32's BLE service.
//
// A byte pipe that lets one or more BLE centrals (a phone, a PC, a test rig) monitor and control the device
// while gamepads are connected. Several centrals can be connected at the same time
// (CONFIG_BLUEPAD32_BLE_NUS_MAX_CLIENTS); output is broadcast to every subscribed one.
//
// Everything here must be called from the BTstack thread, e.g. from a platform callback.
// The BLE service must be enabled first: uni_bt_enable_service_safe(true).

#ifndef UNI_BT_NUS_H
#define UNI_BT_NUS_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include <btstack.h>

// Called for every write from a central. Each ATT write is one call, so message boundaries are kept.
// `client` identifies the connection; pass it to uni_bt_nus_send() to reply to that central only.
typedef void (*uni_bt_nus_rx_callback_t)(hci_con_handle_t client, const uint8_t* data, uint16_t len);

// Called when a central subscribes to (or unsubscribes from) TX notifications.
typedef void (*uni_bt_nus_subscribe_callback_t)(hci_con_handle_t client, bool subscribed);

void uni_bt_nus_set_rx_callback(uni_bt_nus_rx_callback_t cb);
void uni_bt_nus_set_subscribe_callback(uni_bt_nus_subscribe_callback_t cb);

// Queue `len` bytes for `client`, or for every subscribed client when `client` is HCI_CON_HANDLE_INVALID.
// Bytes that don't fit in a client's TX buffer are dropped (counted, see uni_bt_nus_dropped_bytes()).
// Returns the number of clients the data was queued for.
int uni_bt_nus_send(hci_con_handle_t client, const uint8_t* data, uint16_t len);

// printf() to `client`, or to every subscribed client when `client` is HCI_CON_HANDLE_INVALID.
int uni_bt_nus_vprintf(hci_con_handle_t client, const char* fmt, va_list ap);
int uni_bt_nus_printf(hci_con_handle_t client, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

// Number of connected centrals, and how many of those subscribed to TX.
int uni_bt_nus_client_count(void);
int uni_bt_nus_subscriber_count(void);

// Total bytes dropped because a client's TX buffer was full.
uint32_t uni_bt_nus_dropped_bytes(void);

// Usable payload per notification for `client` (ATT MTU - 3), 0 if unknown.
uint16_t uni_bt_nus_payload_size(hci_con_handle_t client);

// --- Used by uni_bt_service.c ---
void uni_bt_nus_init(void);
void uni_bt_nus_on_connected(hci_con_handle_t handle);
void uni_bt_nus_on_disconnected(hci_con_handle_t handle);
void uni_bt_nus_on_mtu(hci_con_handle_t handle);
// Returns true if the write was for NuS (and sets *result to the ATT error code).
bool uni_bt_nus_on_write(hci_con_handle_t handle, uint16_t att_handle, const uint8_t* data, uint16_t len, int* result);
// True if another central could still connect.
bool uni_bt_nus_has_free_slot(void);

#endif  // UNI_BT_NUS_H
