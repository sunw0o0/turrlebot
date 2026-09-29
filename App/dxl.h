/*
 * dxl.h - Dynamixel Protocol 1.0 driver (USART3 + MAX13488E RS-485)
 *
 * MAX13488E switches TX/RX direction automatically, so no DE/RE GPIO is
 * needed. Because RE is tied to GND, our own TX bytes may echo back on RX;
 * the driver discards that echo.
 */
#ifndef DXL_H
#define DXL_H

#include <stdint.h>
#include "main.h"

#define DXL_BROADCAST_ID 0xFE

/* Return codes (>= 0 is the status packet's error byte) */
#define DXL_OK           0
#define DXL_ERR_TX      -1
#define DXL_ERR_TIMEOUT -2
#define DXL_ERR_ARG     -3

void dxl_init(UART_HandleTypeDef *huart);

int dxl_ping(uint8_t id);
int dxl_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out);
int dxl_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len);
int dxl_write_u8(uint8_t id, uint8_t addr, uint8_t value);
int dxl_write_u16(uint8_t id, uint8_t addr, uint16_t value);

/* Writes `len` bytes at `addr` to each id; data holds len bytes per id.
 * No status packet is returned. */
int dxl_sync_write(uint8_t addr, uint8_t len, const uint8_t *ids,
                   const uint8_t *data, uint8_t count);

#endif /* DXL_H */
