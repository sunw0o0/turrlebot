/*
 * dxl.h - Dynamixel Protocol 2.0 driver (USART3 + MAX13488E RS-485)
 *
 * MAX13488E switches TX/RX direction automatically, so no DE/RE GPIO is
 * needed. Because RE is tied to GND, our own TX bytes echo back on RX;
 * the driver skips that echo.
 *
 * USART3 is generated with the LL driver (CubeMX). RX is polled while a
 * transaction is in progress, so no USART interrupt is needed.
 */
#ifndef DXL_H
#define DXL_H

#include <stdint.h>
#include "main.h"
#include "stm32f4xx_hal.h"

#define DXL_BROADCAST_ID 0xFE

/* Return codes (>= 0 is the status packet's error byte) */
#define DXL_ALERT        0x80  /* error byte bit: hardware error latched */
#define DXL_OK           0
#define DXL_ERR_TX      -1
#define DXL_ERR_TIMEOUT -2
#define DXL_ERR_ARG     -3

void dxl_init(USART_TypeDef *usart);
void dxl_set_baud(uint32_t baud);

/* Total bytes received since dxl_init (includes the TX echo) */
extern volatile uint32_t dxl_rx_count;

uint16_t dxl_crc(const uint8_t *data, uint16_t len);

int dxl_ping(uint8_t id);
/* Broadcast ping: every motor on the bus answers. Stores up to `max`
 * IDs and returns how many answered. */
int dxl_scan(uint8_t *ids, int max);

/* Reboot clears a latched hardware error (torque stays off until then). */
int dxl_reboot(uint8_t id);

int dxl_read(uint8_t id, uint16_t addr, uint16_t len, uint8_t *out);
int dxl_write(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len);
int dxl_write_u8(uint8_t id, uint16_t addr, uint8_t value);
int dxl_write_u32(uint8_t id, uint16_t addr, uint32_t value);

/* Writes `len` bytes at `addr` to each id; data holds len bytes per id.
 * No status packet is returned. */
int dxl_sync_write(uint16_t addr, uint16_t len, const uint8_t *ids,
                   const uint8_t *data, uint8_t count);

#endif /* DXL_H */
