/* controller_xbox360.c — Xbox 360 Wired Controller for Ghost-Control
 * XInput protocol, no handshake needed — controller streams immediately.
 * Button layout verified from joypad-os-ds4-main xinput_descriptors.h
 */

#include "controller_xbox360.h"
#include <string.h>

#ifdef __PROSPERO__
#include <ps5/klog.h>
#define LOG(...) klog_printf("[GC] " __VA_ARGS__)
#else
#include <stdio.h>
#define LOG(...) fprintf(stderr, __VA_ARGS__)
#endif

/* ── helpers ──────────────────────────────────────────────────────────── */

#define DEADZONE 7849

static uint8_t stick_x(int16_t v) {
    return (v > DEADZONE || v < -DEADZONE) ? (uint8_t)((v + 32768) >> 8) : 128u;
}

static uint8_t stick_y(int16_t v) {
    return (v > DEADZONE || v < -DEADZONE) ? (uint8_t)(255 - ((v + 32768) >> 8)) : 128u;
}

/* ── input parsing ────────────────────────────────────────────────────── */

void xbox360_parse_input(const uint8_t *b, ScePadData *o) {
    uint8_t b0 = b[2];
    uint8_t b1 = b[3];
    uint8_t lt = b[4];
    uint8_t rt = b[5];
    int16_t lx = (int16_t)((uint16_t)b[6]  | ((uint16_t)b[7]  << 8));
    int16_t ly = (int16_t)((uint16_t)b[8]  | ((uint16_t)b[9]  << 8));
    int16_t rx = (int16_t)((uint16_t)b[10] | ((uint16_t)b[11] << 8));
    int16_t ry = (int16_t)((uint16_t)b[12] | ((uint16_t)b[13] << 8));

    o->leftStick.x      = stick_x(lx);
    o->leftStick.y      = stick_y(ly);
    o->rightStick.x     = stick_x(rx);
    o->rightStick.y     = stick_y(ry);
    o->analogButtons.l2 = lt;
    o->analogButtons.r2 = rt;

    uint32_t btn = 0;

    /* b[2]: dpad + start/back + stick clicks */
    if (b0 & 0x01u) btn |= SCE_PAD_BUTTON_UP;
    if (b0 & 0x02u) btn |= SCE_PAD_BUTTON_DOWN;
    if (b0 & 0x04u) btn |= SCE_PAD_BUTTON_LEFT;
    if (b0 & 0x08u) btn |= SCE_PAD_BUTTON_RIGHT;
    if (b0 & 0x10u) btn |= SCE_PAD_BUTTON_OPTIONS;  /* Start → Options */
    if (b0 & 0x20u) btn |= SCE_PAD_BUTTON_SHARE;    /* Back  → Share   */
    if (b0 & 0x40u) btn |= SCE_PAD_BUTTON_L3;       /* LS    → L3      */
    if (b0 & 0x80u) btn |= SCE_PAD_BUTTON_R3;       /* RS    → R3      */

    /* b[3]: bumpers + guide + face buttons */
    if (b1 & 0x01u) btn |= SCE_PAD_BUTTON_L1;       /* LB    → L1      */
    if (b1 & 0x02u) btn |= SCE_PAD_BUTTON_R1;       /* RB    → R1      */
    if (b1 & 0x04u) btn |= SCE_PAD_BUTTON_PS;       /* Guide → PS      */
    if (b1 & 0x10u) btn |= SCE_PAD_BUTTON_CROSS;    /* A     → Cross   */
    if (b1 & 0x20u) btn |= SCE_PAD_BUTTON_CIRCLE;   /* B     → Circle  */
    if (b1 & 0x40u) btn |= SCE_PAD_BUTTON_SQUARE;   /* X     → Square  */
    if (b1 & 0x80u) btn |= SCE_PAD_BUTTON_TRIANGLE; /* Y     → Triangle*/

    /* Triggers: analog + digital threshold */
    if (lt > 16u) btn |= SCE_PAD_BUTTON_L2;
    if (rt > 16u) btn |= SCE_PAD_BUTTON_R2;

    /* Combo: LB+Start OR RB+Back → Touchpad */
    if (((b1 & 0x01u) && (b0 & 0x10u)) || ((b1 & 0x02u) && (b0 & 0x20u))) {
        if (b1 & 0x01u) btn &= ~(SCE_PAD_BUTTON_L1 | SCE_PAD_BUTTON_OPTIONS);
        if (b1 & 0x02u) btn &= ~(SCE_PAD_BUTTON_R1 | SCE_PAD_BUTTON_SHARE);
        btn |= SCE_PAD_BUTTON_TOUCH_PAD;
    }

    o->buttons   = btn;
    o->connected = 1;
    o->quat.w    = 1.0f;
}

/* ── packet handler ───────────────────────────────────────────────────── */

int xbox360_handle_packet(const uint8_t *buf, uint32_t len, ScePadData *out_pad) {
    /* XInput report: rid=0x00, buf[1]=0x14 (20 bytes), need at least 14 */
    if (buf[0] == 0x00 && len >= 14 && buf[1] == 0x14) {
        xbox360_parse_input(buf, out_pad);
        return 1;
    }
    return 0;
}
