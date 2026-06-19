/* controller_ds4.c — DualShock 4 / DS4-compatible (XIM4 etc.) for Ghost-Control
 *
 * DS4 USB wire format is the public Sony report (ID 0x01, 64 bytes).
 * No init or handshake required: open the IN endpoint, reports stream at ~250 Hz.
 *
 * Touchpad layout (bytes 29-40):
 *   29:  finger1 contact (bit7=NOT touching, bits6:0=id)
 *   30:  finger1 X low 8
 *   31:  finger1 X high 4 (low nibble) | Y high 4 (high nibble)
 *   32:  finger1 Y low 8
 *   33-36: finger1 second sample (history, ignored)
 *   37:  finger2 contact
 *   38:  finger2 X low 8
 *   39:  finger2 X high 4 (low nibble) | Y high 4 (high nibble)
 *   40:  finger2 Y low 8
 *   Resolution: 1920 x 942 (fits uint16_t directly, no scaling).
 */

#include "controller_ds4.h"
#include <string.h>

/* Hat lookup: index 0..8 → (up, right, down, left) bits */
static const uint8_t HAT_DPAD[9] = {
    /* 0 N  */ SCE_PAD_BUTTON_UP,
    /* 1 NE */ SCE_PAD_BUTTON_UP   | SCE_PAD_BUTTON_RIGHT,
    /* 2 E  */ SCE_PAD_BUTTON_RIGHT,
    /* 3 SE */ SCE_PAD_BUTTON_DOWN | SCE_PAD_BUTTON_RIGHT,
    /* 4 S  */ SCE_PAD_BUTTON_DOWN,
    /* 5 SW */ SCE_PAD_BUTTON_DOWN | SCE_PAD_BUTTON_LEFT,
    /* 6 W  */ SCE_PAD_BUTTON_LEFT,
    /* 7 NW */ SCE_PAD_BUTTON_UP   | SCE_PAD_BUTTON_LEFT,
    /* 8 -- */ 0u,
};

void ds4_parse_input(const uint8_t *b, ScePadData *o) {
    o->leftStick.x      = b[1];
    o->leftStick.y      = b[2];
    o->rightStick.x     = b[3];
    o->rightStick.y     = b[4];
    o->analogButtons.l2 = b[8];
    o->analogButtons.r2 = b[9];

    uint32_t btn = 0;

    /* b[5]: dpad (low nibble, hat 0..8) + face buttons (high nibble) */
    uint8_t hat = b[5] & 0x0Fu;
    if (hat <= 8) btn |= HAT_DPAD[hat];
    if (b[5] & 0x10u) btn |= SCE_PAD_BUTTON_SQUARE;
    if (b[5] & 0x20u) btn |= SCE_PAD_BUTTON_CROSS;
    if (b[5] & 0x40u) btn |= SCE_PAD_BUTTON_CIRCLE;
    if (b[5] & 0x80u) btn |= SCE_PAD_BUTTON_TRIANGLE;

    /* b[6]: shoulders + select/start + stick clicks */
    if (b[6] & 0x01u) btn |= SCE_PAD_BUTTON_L1;
    if (b[6] & 0x02u) btn |= SCE_PAD_BUTTON_R1;
    if (b[6] & 0x04u) btn |= SCE_PAD_BUTTON_L2;
    if (b[6] & 0x08u) btn |= SCE_PAD_BUTTON_R2;
    if (b[6] & 0x10u) btn |= SCE_PAD_BUTTON_SHARE;    /* Share → Create */
    if (b[6] & 0x20u) btn |= SCE_PAD_BUTTON_OPTIONS;
    if (b[6] & 0x40u) btn |= SCE_PAD_BUTTON_L3;
    if (b[6] & 0x80u) btn |= SCE_PAD_BUTTON_R3;

    /* b[7]: PS + touchpad-click (low 2 bits) */
    if (b[7] & 0x01u) btn |= SCE_PAD_BUTTON_PS;
    if (b[7] & 0x02u) btn |= SCE_PAD_BUTTON_TOUCH_PAD;

    o->buttons   = btn;
    o->connected = 1;
    o->quat.w    = 1.0f;
}

/* Parse one 4-byte touch sample into ScePadTouch.
 * DS4 contact byte: bit 7 = NOT touching (active low); bits 6:0 = contact ID.
 * ScePadTouch.finger: 0 = not touching, non-zero = active contact ID. */
static void parse_touch_sample(const uint8_t *s, ScePadTouch *t) {
    uint8_t contact = s[0];
    if (contact & 0x80u) {
        /* Not touching */
        t->finger = 0;
        t->x = 0;
        t->y = 0;
    } else {
        t->finger = contact & 0x7Fu;
        t->x = (uint16_t)s[1] | ((uint16_t)(s[2] & 0x0Fu) << 8);
        t->y = ((uint16_t)(s[2] & 0xF0u) << 4) | (uint16_t)s[3];
    }
    t->pad[0] = t->pad[1] = t->pad[2] = 0;
}

int ds4_handle_packet(int fd, struct usb_fs_endpoint *eps,
                      const uint8_t *buf, uint32_t len,
                      ScePadData *out_pad) {
    (void)fd; (void)eps;

    /* DS4 USB uses report ID 0x01. Real DS4 sends 64 bytes; HORI third-party
     * pads (and XIM4) often truncate to ~27 bytes — bytes [1..9] are identical
     * so anything ≥ 10 bytes with [0]==0x01 is parseable. */
    if (len >= 10 && buf[0] == 0x01) {
        ds4_parse_input(buf, out_pad);

        /* Touchpad data: finger 1 at offset 29 (4 bytes), finger 2 at offset 37 (4 bytes).
         * Second-sample history (offsets 33-36, 41-44) is ignored. */
        if (len >= 33) {
            parse_touch_sample(&buf[29], &out_pad->touchData.touch[0]);
            uint8_t fingers = (out_pad->touchData.touch[0].finger != 0) ? 1 : 0;

            if (len >= 41) {
                parse_touch_sample(&buf[37], &out_pad->touchData.touch[1]);
                if (out_pad->touchData.touch[1].finger != 0) fingers++;
            }
            out_pad->touchData.fingers = fingers;
        }

        return 1;
    }
    return 0;
}
