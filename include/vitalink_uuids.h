/* -----------------------------------------------------------------------------
 * File: vitalink_uuids.h
 * Purpose: Holds the custom 128-bit BLE UUIDs used by the project.
 *
 * Notes:
 * - Keep these UUIDs in sync with your Raspberry Pi subscriber.
 * - Service UUID identifies the "container"; Characteristic UUID is the data pipe.
 * - This version exports ready-to-use const objects, simple accessors, and a
 *   convenience macro to include the Service UUID in advertising data.
 * --------------------------------------------------------------------------- */

#pragma once
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/* ---- VitalLink Service UUID----------------------------- */
#define VITALINK_SERVICE_UUID_INIT \
  BT_UUID_INIT_128(0xa1,0xa3,0xb2,0xd8,0xe1,0x3f,0x5f,0x9b,0x4b,0x4a,0xa8,0x22,0xf1,0x84,0x5a,0x8f)

/* ---- Vitals Notify Characteristic UUID  ------------------ */
#define VITALS_CHAR_UUID_INIT \
  BT_UUID_INIT_128(0x99,0x11,0x3f,0x5e,0x2a,0x7d,0x5a,0x8d,0x2a,0x4b,0xb2,0x39,0xb7,0xc4,0xe2,0xd3)

/* ---- Exported const objects--------------------- */
static const struct bt_uuid_128 VITALINK_SERVICE_UUID = VITALINK_SERVICE_UUID_INIT;
static const struct bt_uuid_128 VITALS_CHAR_UUID      = VITALS_CHAR_UUID_INIT;

/* ---- Typed accessors for APIs that want struct bt_uuid* ------------------ */
static inline const struct bt_uuid *vitalink_service_uuid(void)
{
  return &VITALINK_SERVICE_UUID.uuid;
}
static inline const struct bt_uuid *vitals_char_uuid(void)
{
  return &VITALS_CHAR_UUID.uuid;
}

/* ---- Advertising helper: 128-bit Service UUID in AD (LSB-first) ---------- */
#define VITALINK_AD_SERVICE_UUID \
  BT_DATA_BYTES(BT_DATA_UUID128_ALL, \
    0xa1,0xa3,0xb2,0xd8,0xe1,0x3f,0x5f,0x9b,0x4b,0x4a,0xa8,0x22,0xf1,0x84,0x5a,0x8f)
