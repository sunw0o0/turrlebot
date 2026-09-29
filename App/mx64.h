/*
 * mx64.h - MX-64(2.0) (Protocol 2.0) control table and wheel helpers
 */
#ifndef MX64_H
#define MX64_H

#include <stdint.h>

/* EEPROM (writable only while torque is off) */
#define MX64_ADDR_ID             7
#define MX64_ADDR_BAUD_RATE      8
#define MX64_ADDR_OPERATING_MODE 11
#define MX64_ADDR_VELOCITY_LIMIT 44   /* 4 bytes */

/* RAM */
#define MX64_ADDR_TORQUE_ENABLE    64
#define MX64_ADDR_LED              65
#define MX64_ADDR_HARDWARE_ERROR   70
#define MX64_ADDR_GOAL_VELOCITY    104  /* 4 bytes, signed */
#define MX64_ADDR_PRESENT_VELOCITY 128  /* 4 bytes, signed */

#define MX64_MODE_VELOCITY 1
#define MX64_MODE_POSITION 3

/* Goal velocity unit: 0.229 rpm */

int mx64_torque(uint8_t id, uint8_t on);
/* Turns torque off, switches to velocity mode (EEPROM is written only if
 * the mode differs) and turns torque back on. */
int mx64_set_wheel_mode(uint8_t id);
int mx64_set_speed(uint8_t id, int32_t speed);
/* Sends the speeds of `count` motors in one sync-write packet. */
int mx64_set_speeds(const uint8_t *ids, const int32_t *speeds, uint8_t count);

#endif /* MX64_H */
