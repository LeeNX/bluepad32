// SPDX-License-Identifier: Apache-2.0
// SInput HID gamepads (Handheld Legend). Spec: https://docs.handheldlegend.com/s/sinput

#ifndef UNI_HID_PARSER_SINPUT_H
#define UNI_HID_PARSER_SINPUT_H

#include <stdint.h>

#include "parser/uni_hid_parser.h"

// Fallback VID/PID from the SInput spec. Intended for testing; real products should register their own PID.
#define UNI_HID_PARSER_SINPUT_VID 0x2e8a
#define UNI_HID_PARSER_SINPUT_PID 0x10c6

void uni_hid_parser_sinput_setup(struct uni_hid_device_s* d);
void uni_hid_parser_sinput_init_report(struct uni_hid_device_s* d);
void uni_hid_parser_sinput_parse_input_report(struct uni_hid_device_s* d, const uint8_t* report, uint16_t len);

#endif  // UNI_HID_PARSER_SINPUT_H
