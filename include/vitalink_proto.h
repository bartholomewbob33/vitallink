/* -----------------------------------------------------------------------------
 * File: vitalink_proto.h
 * Purpose: Defines the 16-byte telemetry packet we notify over BLE and
 *          small helpers to compute/verify the checksum and manipulate flags.
 *
 * Packet layout (little-endian):
 *   Byte  [0]      : uint8_t  ver          (protocol version)
 *   Bytes [1..2]   : uint16_t seq          (sequence id, LE)
 *   Bytes [3..6]   : uint32_t ts_ms        (ms since boot, LE)
 *   Byte  [7]      : uint8_t  flags        (bit flags below)
 *   Byte  [8]      : uint8_t  hr_bpm       (beats per minute)
 *   Byte  [9]      : uint8_t  spo2         (%)
 *   Bytes [10..11] : int16_t  skin_c_x100  (centi-deg C, LE; 36.50C => 3650)
 *   Bytes [12..13] : uint16_t act_rms_x100 (activity metric x100, LE)
 *   Byte  [14]     : uint8_t  checksum     (sum of bytes [0..13] mod 256)
 *   Byte  [15]     : uint8_t  rfu          (reserved/pad, transmit as 0)
 *
 * Integrity (checksum):
 *   checksum = (sum of pkt bytes 0..13) & 0xFF
 *
 * Flags (bit meanings):
 *   bit0 : Motion artifact
 *   bit1 : Low battery
 *   bit2 : Sensor fault
 *   bit3 : Alert tier (moderate concern)
 *   bit4 : Emergency tier (critical concern)
 *   bit5..7 : reserved
 *
 * Notes:
 *   - NORMAL = both ALERT and EMERGENCY bits clear (0).
 *   - Bump VITALINK_PROTO_VER if you change this layout.
 *   - Includes vitals_finalize() to set ver, zero RFU, and fill checksum.
 * --------------------------------------------------------------------------- */

#pragma once
#include <stdint.h>

#ifndef __packed
#define __packed __attribute__((__packed__))
#endif

#define VITALINK_PROTO_VER 1

struct __packed vitals_pkt {
  uint8_t  ver;            /* [0]  protocol version (VITALINK_PROTO_VER)         */
  uint16_t seq;            /* [1..2] LE sequence number                           */
  uint32_t ts_ms;          /* [3..6] LE ms since boot                             */
  uint8_t  flags;          /* [7]  bit flags (see below)                          */
  uint8_t  hr_bpm;         /* [8]  heart rate (BPM)                               */
  uint8_t  spo2;           /* [9]  SpO2 (%)                                       */
  int16_t  skin_c_x100;    /* [10..11] LE 36.50C => 3650                          */
  uint16_t act_rms_x100;   /* [12..13] LE activity metric x100                    */
  uint8_t  checksum;       /* [14] sum of bytes [0..13] & 0xFF                    */
  uint8_t  rfu;            /* [15] reserved (0)                                   */
};

/* Compile-time guard (Zephyr-style assert if available) */
#if defined(CONFIG_ZEPHYR)
  #include <zephyr/sys/util_macro.h>
  BUILD_ASSERT(sizeof(struct vitals_pkt) == 16, "vitals_pkt must be 16 bytes");
#else
  typedef char _vitals_pkt_must_be_16_bytes[ (sizeof(struct vitals_pkt) == 16) ? 1 : -1 ];
#endif

/* Flag bit definitions */
#define VFLAG_MOTION_ARTIFACT  (1u << 0)
#define VFLAG_LOW_BATT         (1u << 1)
#define VFLAG_SENSOR_FAULT     (1u << 2)
#define VFLAG_ALERT            (1u << 3)
#define VFLAG_EMERGENCY        (1u << 4)

/* Checksum helpers */
static inline uint8_t vitals_checksum_compute(const struct vitals_pkt *p)
{
  const uint8_t *b = (const uint8_t *)p;
  uint16_t sum = 0;
  for (int i = 0; i < 14; i++) sum += b[i];
  return (uint8_t)(sum & 0xFF);
}
static inline void vitals_checksum_fill(struct vitals_pkt *p)
{
  p->checksum = vitals_checksum_compute(p);
}
static inline int vitals_checksum_ok(const struct vitals_pkt *p)
{
  return p->checksum == vitals_checksum_compute(p);
}

/* Tier convenience */
static inline void vitals_flags_set_alert(struct vitals_pkt *p)
{
  p->flags |= VFLAG_ALERT;
  p->flags &= (uint8_t)~VFLAG_EMERGENCY;
}
static inline void vitals_flags_set_emergency(struct vitals_pkt *p)
{
  p->flags |= VFLAG_EMERGENCY;
  p->flags &= (uint8_t)~VFLAG_ALERT;
}
static inline void vitals_flags_clear_tiers(struct vitals_pkt *p)
{
  p->flags &= (uint8_t)~(VFLAG_ALERT | VFLAG_EMERGENCY);
}

/* Finalize before TX: set version, zero RFU, and compute checksum */
static inline void vitals_finalize(struct vitals_pkt *p)
{
  p->ver = VITALINK_PROTO_VER;
  p->rfu = 0u;
  vitals_checksum_fill(p);
}
