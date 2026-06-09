#pragma once
#include <stdint.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>
#include "gc_types.h"

/*
 * Xbox 360 Wired Controller (VID=0x045E PID=0x028E) — XInput protocol
 *
 * Endpoints: IN=0x81 OUT=0x02 (interface 0, maxpkt=32)
 *
 * XInput report format (20 bytes, report_id=0x00):
 *   [0]=0x00 rid  [1]=0x14 (length=20)
 *   [2]=buttons0: bit0=DUp bit1=DDn bit2=DLt bit3=DRt
 *                 bit4=Start bit5=Back bit6=LS bit7=RS
 *   [3]=buttons1: bit0=LB bit1=RB bit2=Guide
 *                 bit4=A bit5=B bit6=X bit7=Y
 *   [4]=LT (0-255)  [5]=RT (0-255)
 *   [6..7] = LX int16 LE (center=0)
 *   [8..9] = LY int16 LE (center=0, positive=up)
 *   [10..11]= RX int16 LE
 *   [12..13]= RY int16 LE
 *   [14..19]= padding
 *
 * Verified from joypad-os-ds4-main/src/usb/usbd/descriptors/xinput_descriptors.h
 */

#define XBOX360_EP_IN   0x81
#define XBOX360_EP_OUT  0x02

#define VID_XBOX360  0x045eu
#define PID_XBOX360  0x028eu

/* Parse XInput report into ScePadData */
void xbox360_parse_input(const uint8_t *buf, ScePadData *o);

/* Handle one IN packet. Returns 1 if pad updated, 0 to skip. */
int xbox360_handle_packet(const uint8_t *buf, uint32_t len, ScePadData *out_pad);
