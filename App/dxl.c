/*
 * dxl.c - Dynamixel Protocol 2.0 driver
 *
 * Packet: FF FF FD 00 ID LEN_L LEN_H INST PARAM... CRC_L CRC_H
 *   LEN = number of (stuffed) params + 3
 *   CRC = CRC-16 (poly 0x8005) over everything before the CRC
 *   Byte stuffing: FF FF FD inside INST/PARAM is sent as FF FF FD FD
 * Status packets use INST = 0x55 followed by an error byte.
 *
 * RX is polled (LL driver): bytes arriving while we transmit (the echo)
 * are drained into rx_buf, and recv_status() polls the USART directly.
 */
#include "dxl.h"
#include "stm32f4xx_ll_usart.h"
#include <string.h>

#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_REBOOT     0x08
#define INST_SYNC_WRITE 0x83
#define INST_STATUS     0x55

#define TX_TIMEOUT_MS 20

#define RX_BUF_SIZE 256        /* power of two */
#define PKT_MAX     96

static USART_TypeDef *dxl_usart;
static uint32_t status_timeout_ms = 10;

static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_head;
static uint16_t rx_tail;

volatile uint32_t dxl_rx_count;

static uint8_t tx_pkt[PKT_MAX];

uint16_t dxl_crc(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
  }
  return crc;
}

/* Moves a received byte (if any) into rx_buf and clears overrun. */
static void rx_poll(void)
{
  if (LL_USART_IsActiveFlag_ORE(dxl_usart))
    LL_USART_ClearFlag_ORE(dxl_usart);
  if (LL_USART_IsActiveFlag_RXNE(dxl_usart)) {
    rx_buf[rx_head & (RX_BUF_SIZE - 1)] = LL_USART_ReceiveData8(dxl_usart);
    rx_head++;
    dxl_rx_count++;
  }
}

static void rx_flush(void)
{
  while (LL_USART_IsActiveFlag_RXNE(dxl_usart))
    (void)LL_USART_ReceiveData8(dxl_usart);
  if (LL_USART_IsActiveFlag_ORE(dxl_usart))
    LL_USART_ClearFlag_ORE(dxl_usart);
  rx_tail = rx_head;
}

static int rx_get(uint8_t *b, uint32_t deadline)
{
  while (rx_tail == rx_head) {
    rx_poll();
    if (rx_tail == rx_head && (int32_t)(HAL_GetTick() - deadline) >= 0)
      return 0;
  }
  *b = rx_buf[rx_tail & (RX_BUF_SIZE - 1)];
  rx_tail++;
  return 1;
}

static int tx_bytes(const uint8_t *data, uint16_t len)
{
  uint32_t deadline = HAL_GetTick() + TX_TIMEOUT_MS + 1;

  for (uint16_t i = 0; i < len; i++) {
    while (!LL_USART_IsActiveFlag_TXE(dxl_usart)) {
      rx_poll();
      if ((int32_t)(HAL_GetTick() - deadline) >= 0)
        return DXL_ERR_TX;
    }
    LL_USART_TransmitData8(dxl_usart, data[i]);
    rx_poll();
  }
  while (!LL_USART_IsActiveFlag_TC(dxl_usart)) {
    rx_poll();
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
      return DXL_ERR_TX;
  }
  return DXL_OK;
}

static int send_packet(uint8_t id, uint8_t inst, const uint8_t *params,
                       uint16_t nparams)
{
  uint16_t n = 8;

  tx_pkt[0] = 0xFF;
  tx_pkt[1] = 0xFF;
  tx_pkt[2] = 0xFD;
  tx_pkt[3] = 0x00;
  tx_pkt[4] = id;
  tx_pkt[7] = inst;
  for (uint16_t i = 0; i < nparams; i++) {
    if (n + 3 > PKT_MAX)
      return DXL_ERR_ARG;
    tx_pkt[n++] = params[i];
    if (tx_pkt[n - 1] == 0xFD && tx_pkt[n - 2] == 0xFF && tx_pkt[n - 3] == 0xFF)
      tx_pkt[n++] = 0xFD;       /* byte stuffing */
  }
  uint16_t len = n - 8 + 3;     /* inst + params + crc */
  tx_pkt[5] = len & 0xFF;
  tx_pkt[6] = len >> 8;
  uint16_t crc = dxl_crc(tx_pkt, n);
  tx_pkt[n++] = crc & 0xFF;
  tx_pkt[n++] = crc >> 8;

  rx_flush();
  return tx_bytes(tx_pkt, n);
}

/* Receives the next status packet (the echo of our instruction is
 * skipped because its INST is not 0x55). Returns the error byte and
 * fills *id and params (destuffed), or DXL_ERR_TIMEOUT. */
static int recv_any(uint32_t deadline, uint8_t *id, uint8_t *params,
                    uint16_t nparams)
{
  uint8_t pkt[PKT_MAX];
  uint8_t b;

  for (;;) {
    /* Header FF FF FD 00 */
    if (!rx_get(&b, deadline)) return DXL_ERR_TIMEOUT;
    if (b != 0xFF) continue;
    if (!rx_get(&b, deadline)) return DXL_ERR_TIMEOUT;
    if (b != 0xFF) continue;
    if (!rx_get(&b, deadline)) return DXL_ERR_TIMEOUT;
    if (b != 0xFD) continue;
    if (!rx_get(&b, deadline)) return DXL_ERR_TIMEOUT;
    if (b != 0x00) continue;

    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = 0xFD; pkt[3] = 0x00;
    for (int i = 4; i < 7; i++)
      if (!rx_get(&pkt[i], deadline)) return DXL_ERR_TIMEOUT;
    uint16_t len = pkt[5] | (pkt[6] << 8);
    if (len < 4 || len + 7 > PKT_MAX) continue;
    for (uint16_t i = 0; i < len; i++)
      if (!rx_get(&pkt[7 + i], deadline)) return DXL_ERR_TIMEOUT;

    uint16_t total = len + 7;
    uint16_t crc = pkt[total - 2] | (pkt[total - 1] << 8);
    if (crc != dxl_crc(pkt, total - 2)) continue;
    if (pkt[7] != INST_STATUS) continue;   /* our own echo */

    /* Destuff params (between error byte and CRC) */
    uint16_t out = 0;
    for (uint16_t i = 9; i < total - 2; i++) {
      if (pkt[i] == 0xFD && i >= 3 && pkt[i - 1] == 0xFD &&
          pkt[i - 2] == 0xFF && pkt[i - 3] == 0xFF)
        continue;
      if (out < nparams)
        params[out] = pkt[i];
      out++;
    }
    if (out < nparams) continue;

    *id = pkt[4];
    return pkt[8];
  }
}

static int recv_status(uint8_t id, uint8_t *params, uint16_t nparams)
{
  uint32_t deadline = HAL_GetTick() + status_timeout_ms + 1;
  uint8_t got;
  int ret;

  do {
    ret = recv_any(deadline, &got, params, nparams);
  } while (ret >= 0 && got != id);
  return ret;
}

void dxl_init(USART_TypeDef *usart)
{
  dxl_usart = usart;
  rx_head = 0;
  rx_tail = 0;
  if (!LL_USART_IsEnabled(usart))
    LL_USART_Enable(usart);
  rx_flush();
}

void dxl_set_baud(uint32_t baud)
{
  while (!LL_USART_IsActiveFlag_TC(dxl_usart))
    ;
  LL_USART_Disable(dxl_usart);
  LL_USART_SetBaudRate(dxl_usart, HAL_RCC_GetPCLK1Freq(),
                       LL_USART_OVERSAMPLING_16, baud);
  LL_USART_Enable(dxl_usart);
  /* ~30-byte packet air time + return delay margin */
  status_timeout_ms = 10 + 300000 / baud;
  rx_flush();
}

int dxl_ping(uint8_t id)
{
  uint8_t info[3];
  int ret = send_packet(id, INST_PING, NULL, 0);
  if (ret != DXL_OK)
    return ret;
  return recv_status(id, info, 3);
}

int dxl_scan(uint8_t *ids, int max)
{
  uint8_t info[3];
  uint8_t id;
  int count = 0;

  if (send_packet(DXL_BROADCAST_ID, INST_PING, NULL, 0) != DXL_OK)
    return 0;
  /* Each motor answers after its own ID-based delay */
  uint32_t deadline = HAL_GetTick() + 3 * status_timeout_ms + 10;
  while (recv_any(deadline, &id, info, 3) >= 0) {
    if (count < max)
      ids[count] = id;
    count++;
  }
  return count < max ? count : max;
}

int dxl_reboot(uint8_t id)
{
  int ret = send_packet(id, INST_REBOOT, NULL, 0);
  if (ret != DXL_OK)
    return ret;
  return recv_status(id, NULL, 0);
}

int dxl_read(uint8_t id, uint16_t addr, uint16_t len, uint8_t *out)
{
  uint8_t params[4] = { addr & 0xFF, addr >> 8, len & 0xFF, len >> 8 };
  if (len + 11 > PKT_MAX)
    return DXL_ERR_ARG;
  int ret = send_packet(id, INST_READ, params, 4);
  if (ret != DXL_OK)
    return ret;
  return recv_status(id, out, len);
}

int dxl_write(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len)
{
  uint8_t params[PKT_MAX];
  if (len + 2 > PKT_MAX - 12)
    return DXL_ERR_ARG;
  params[0] = addr & 0xFF;
  params[1] = addr >> 8;
  memcpy(&params[2], data, len);
  int ret = send_packet(id, INST_WRITE, params, len + 2);
  if (ret != DXL_OK || id == DXL_BROADCAST_ID)
    return ret;
  return recv_status(id, NULL, 0);
}

int dxl_write_u8(uint8_t id, uint16_t addr, uint8_t value)
{
  return dxl_write(id, addr, &value, 1);
}

int dxl_write_u32(uint8_t id, uint16_t addr, uint32_t value)
{
  uint8_t data[4] = { value & 0xFF, (value >> 8) & 0xFF,
                      (value >> 16) & 0xFF, value >> 24 };
  return dxl_write(id, addr, data, 4);
}

int dxl_sync_write(uint16_t addr, uint16_t len, const uint8_t *ids,
                   const uint8_t *data, uint8_t count)
{
  uint8_t params[PKT_MAX];
  uint16_t n = 0;

  if (4 + count * (len + 1) > PKT_MAX - 12)
    return DXL_ERR_ARG;
  params[n++] = addr & 0xFF;
  params[n++] = addr >> 8;
  params[n++] = len & 0xFF;
  params[n++] = len >> 8;
  for (uint8_t i = 0; i < count; i++) {
    params[n++] = ids[i];
    memcpy(&params[n], &data[i * len], len);
    n += len;
  }
  return send_packet(DXL_BROADCAST_ID, INST_SYNC_WRITE, params, n);
}
