/*
 * mx64.h - MX-64 (Protocol 1.0) control table and wheel-mode helpers
 */
#ifndef MX64_H
#define MX64_H

#include <stdint.h>

/* EEPROM */
#define MX64_ADDR_ID              3
#define MX64_ADDR_BAUD_RATE       4
#define MX64_ADDR_CW_ANGLE_LIMIT  6   /* 2 bytes */
#define MX64_ADDR_CCW_ANGLE_LIMIT 8   /* 2 bytes */

/* RAM */
#define MX64_ADDR_TORQUE_ENABLE   24
#define MX64_ADDR_LED             25
#define MX64_ADDR_MOVING_SPEED    32  /* 2 bytes */
#define MX64_ADDR_PRESENT_SPEED   38  /* 2 bytes */

/* Wheel-mode speed: 0..1023, about 0.114 rpm per unit */
#define MX64_SPEED_MAX 1023

int mx64_torque(uint8_t id, uint8_t on);
/* Sets both angle limits to 0 (wheel mode). EEPROM is written only if
 * the limits differ. */
int mx64_set_wheel_mode(uint8_t id);
/* speed: -1023..1023, positive = CCW */
int mx64_set_speed(uint8_t id, int16_t speed);
/* Sends the speeds of `count` motors in one sync-write packet. */
int mx64_set_speeds(const uint8_t *ids, const int16_t *speeds, uint8_t count);

#endif /* MX64_H */
