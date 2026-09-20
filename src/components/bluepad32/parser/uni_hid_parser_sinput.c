// SPDX-License-Identifier: Apache-2.0
// SInput HID gamepads (Handheld Legend). Spec: https://docs.handheldlegend.com/s/sinput
//
// Phase 1: input only. Parses the fixed-layout 64-byte input report (ID 0x01): buttons, sticks, triggers, battery.
// Not yet implemented: feature discovery (command 0x02), IMU, touchpads, haptics, player LEDs, RGB.
//
// Byte offsets follow SDL's SInput HIDAPI driver (SDL_hidapi_sinput.c). Bluepad32's BLE path delivers the report
// with the Report ID as report[0], so the offsets below include it.

#include "parser/uni_hid_parser_sinput.h"

#include "controller/uni_controller.h"
#include "uni_common.h"
#include "uni_hid_device.h"
#include "uni_log.h"

#define SINPUT_REPORT_ID_INPUT 0x01
#define SINPUT_REPORT_MIN_LEN 19  // Up to and including right trigger.

#define IDX_PLUG_STATUS 1
#define IDX_CHARGE_LEVEL 2
#define IDX_BUTTONS_0 3
#define IDX_BUTTONS_1 4
#define IDX_BUTTONS_2 5
#define IDX_BUTTONS_3 6
#define IDX_LEFT_X 7
#define IDX_LEFT_Y 9
#define IDX_RIGHT_X 11
#define IDX_RIGHT_Y 13
#define IDX_LEFT_TRIGGER 15
#define IDX_RIGHT_TRIGGER 17

// buttons[0]
// Face buttons in A, B, X, Y order (bit 0 = south/A). SDL's driver names bit 0 "east" and bit 1 "south", but its
// mapping string is a:b0,b:b1,x:b2,y:b3, i.e. bit 0 is A. Confirmed on hardware against ESP32-BLE-Gamepad's SInput mode.
#define B0_SOUTH 0x01
#define B0_EAST 0x02
#define B0_WEST 0x04
#define B0_NORTH 0x08
#define B0_DPAD_UP 0x10
#define B0_DPAD_DOWN 0x20
#define B0_DPAD_LEFT 0x40
#define B0_DPAD_RIGHT 0x80
// buttons[1]
#define B1_LEFT_STICK 0x01
#define B1_RIGHT_STICK 0x02
#define B1_LEFT_BUMPER 0x04
#define B1_RIGHT_BUMPER 0x08
#define B1_LEFT_TRIGGER 0x10
#define B1_RIGHT_TRIGGER 0x20
// buttons[2]
#define B2_START 0x01
#define B2_BACK 0x02
#define B2_GUIDE 0x04
#define B2_CAPTURE 0x08

// Plug status values.
#define PLUG_NO_BATTERY 1
#define PLUG_CHARGING 2
#define PLUG_CHARGED 3
#define PLUG_ON_BATTERY 4

static inline int16_t read_s16(const uint8_t* data, int idx) {
    return (int16_t)(data[idx] | (data[idx + 1] << 8));
}

// int16 stick -> Bluepad32 -512..511.
static inline int32_t stick_to_axis(int16_t v) {
    return v >> 6;
}

// int16 trigger -> Bluepad32 0..1023.
// ESP32-BLE-Gamepad's SInput mode sends 0 (released) .. 32767 (fully pressed), and this is what we assume. Negative
// values are clamped to released, so a device that idles at INT16_MIN (SDL's default for absent triggers) also reads
// as released. TODO: confirm the range against the spec / a real SInput device.
static inline int32_t trigger_to_pedal(int16_t v) {
    if (v < 0)
        return 0;
    return v >> 5;
}

void uni_hid_parser_sinput_setup(struct uni_hid_device_s* d) {
    uni_hid_device_set_ready_complete(d);
}

void uni_hid_parser_sinput_init_report(struct uni_hid_device_s* d) {
    // Each report contains the full state.
    uni_controller_t* ctl = &d->controller;
    memset(ctl, 0, sizeof(*ctl));
    ctl->klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    ctl->battery = 255;  // Not available, until a report says otherwise.
}

void uni_hid_parser_sinput_parse_input_report(struct uni_hid_device_s* d, const uint8_t* report, uint16_t len) {
    if (len < 1 || report[0] != SINPUT_REPORT_ID_INPUT) {
        logd("SInput: ignoring report id=0x%02x len=%d\n", len ? report[0] : 0, len);
        return;
    }
    if (len < SINPUT_REPORT_MIN_LEN) {
        loge("SInput: report too short; got %d, want >= %d\n", len, SINPUT_REPORT_MIN_LEN);
        return;
    }

    uni_gamepad_t* gp = &d->controller.gamepad;

    const uint8_t b0 = report[IDX_BUTTONS_0];
    const uint8_t b1 = report[IDX_BUTTONS_1];
    const uint8_t b2 = report[IDX_BUTTONS_2];

    // SInput names buttons by position (south/east/west/north). Bluepad32 uses the Xbox layout:
    // A=south, B=east, X=west, Y=north.
    gp->buttons = 0;
    gp->buttons |= (b0 & B0_SOUTH) ? BUTTON_A : 0;
    gp->buttons |= (b0 & B0_EAST) ? BUTTON_B : 0;
    gp->buttons |= (b0 & B0_WEST) ? BUTTON_X : 0;
    gp->buttons |= (b0 & B0_NORTH) ? BUTTON_Y : 0;
    gp->buttons |= (b1 & B1_LEFT_BUMPER) ? BUTTON_SHOULDER_L : 0;
    gp->buttons |= (b1 & B1_RIGHT_BUMPER) ? BUTTON_SHOULDER_R : 0;
    gp->buttons |= (b1 & B1_LEFT_TRIGGER) ? BUTTON_TRIGGER_L : 0;
    gp->buttons |= (b1 & B1_RIGHT_TRIGGER) ? BUTTON_TRIGGER_R : 0;
    gp->buttons |= (b1 & B1_LEFT_STICK) ? BUTTON_THUMB_L : 0;
    gp->buttons |= (b1 & B1_RIGHT_STICK) ? BUTTON_THUMB_R : 0;

    gp->misc_buttons = 0;
    gp->misc_buttons |= (b2 & B2_START) ? MISC_BUTTON_START : 0;
    gp->misc_buttons |= (b2 & B2_BACK) ? MISC_BUTTON_SELECT : 0;
    gp->misc_buttons |= (b2 & B2_GUIDE) ? MISC_BUTTON_SYSTEM : 0;
    gp->misc_buttons |= (b2 & B2_CAPTURE) ? MISC_BUTTON_CAPTURE : 0;

    gp->dpad = 0;
    gp->dpad |= (b0 & B0_DPAD_UP) ? DPAD_UP : 0;
    gp->dpad |= (b0 & B0_DPAD_DOWN) ? DPAD_DOWN : 0;
    gp->dpad |= (b0 & B0_DPAD_LEFT) ? DPAD_LEFT : 0;
    gp->dpad |= (b0 & B0_DPAD_RIGHT) ? DPAD_RIGHT : 0;

    gp->axis_x = stick_to_axis(read_s16(report, IDX_LEFT_X));
    gp->axis_y = stick_to_axis(read_s16(report, IDX_LEFT_Y));
    gp->axis_rx = stick_to_axis(read_s16(report, IDX_RIGHT_X));
    gp->axis_ry = stick_to_axis(read_s16(report, IDX_RIGHT_Y));
    gp->brake = trigger_to_pedal(read_s16(report, IDX_LEFT_TRIGGER));
    gp->throttle = trigger_to_pedal(read_s16(report, IDX_RIGHT_TRIGGER));

    // Battery: 0..100 % -> 0..254. "No battery" (wired) leaves it as "not available".
    const uint8_t plug = report[IDX_PLUG_STATUS];
    if (plug == PLUG_NO_BATTERY) {
        d->controller.battery = 255;
    } else if (plug == PLUG_CHARGED) {
        d->controller.battery = 254;
    } else if (plug == PLUG_CHARGING || plug == PLUG_ON_BATTERY) {
        uint8_t pct = report[IDX_CHARGE_LEVEL];
        if (pct > 100)
            pct = 100;
        d->controller.battery = (uint8_t)(pct * 254 / 100);
    }
}
