/* -----------------------------------------------------------------------------
 * File: vitalink_proto.h
 * Purpose: Defines the 16-byte telemetry packet we notify over BLE once/second.
 * Notes:
 * - Little-endian layout; fixed-size for easy parsing on the Pi.
 * - "flags" bit meanings are defined below.
 * --------------------------------------------------------------------------- */

#pragma once
#include <stdint.h>

/* 16-byte, little-endian packet */
struct __packed vitals_pkt {
  uint8_t  ver;            /* Protocol version; bump if you change layout */
  uint16_t seq;            /* Sequence number (detect drops/out-of-order) */
  uint32_t ts_ms;          /* Milliseconds since boot (k_uptime_get_32)   */
  uint8_t  flags;          /* See bit meanings below                      */
  uint8_t  hr_bpm;         /* Heart rate (BPM)                            */
  uint8_t  spo2;           /* SpO2 (%)                                    */
  int16_t  skin_c_x100;    /* Skin temp x100C (36.50C -> 3650)            */
  uint16_t act_rms_x100;   /* Activity metric x100                        */
  uint8_t  crc8;           /* CRC-8 over bytes [0..13]                    */
  uint8_t  rfu;            /* Reserved/pad                                */
};

/* flags bit map (adjust as needed) */
#define VFLAG_MOTION_ARTIFACT  (1u << 0)
#define VFLAG_LOW_BATT         (1u << 1)
#define VFLAG_SENSOR_FAULT     (1u << 2)
#define VFLAG_ALERT            (1u << 3)
