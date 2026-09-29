/*
 * dxl.c - Dynamixel Protocol 1.0 driver
 *
 * Packet: FF FF ID LEN INST/ERR PARAM... CHK
 *   LEN = number of params + 2
 *   CHK = ~(ID + LEN + INST + PARAM...)
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
#define INST_SYNC_WRITE 0x83

#define TX_TIMEOUT_MS     10
#define STATUS_TIMEOUT_MS 10   /* MX-64 default return delay is 500us */

#define RX_BUF_SIZE 256        /* power of two */
#define PKT_MAX     64

static USART_TypeDef *dxl_usart;

static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_head;
static uint16_t rx_tail;

static uint8_t tx_pkt[PKT_MAX];
static uint8_t tx_len;

/* Moves a received byte (if any) into rx_buf and clears overrun. */
static void rx_poll(void)
{
  if (LL_USART_IsActiveFlag_ORE(dxl_usart))
    LL_USART_ClearFlag_ORE(dxl_usart);
  if (LL_USART_IsActiveFlag_RXNE(dxl_usart)) {
    rx_buf[rx_head & (RX_BUF_SIZE - 1)] = LL_USART_ReceiveData8(dxl_usart);
    rx_head++;
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

static int tx_bytes(const uint8_t *data, uint8_t len)
{
  uint32_t deadline = HAL_GetTick() + TX_TIMEOUT_MS + 1;

  for (uint8_t i = 0; i < len; i++) {
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

static uint8_t checksum(const uint8_t *pkt)
{
  uint8_t sum = 0;
  for (uint8_t i = 2; i < pkt[3] + 3; i++)
    sum += pkt[i];
  return (uint8_t)~sum;
}

static int send_packet(uint8_t id, uint8_t inst, const uint8_t *params,
                       uint8_t nparams)
{
  if (nparams + 6 > PKT_MAX)
    return DXL_ERR_ARG;

  tx_pkt[0] = 0xFF;
  tx_pkt[1] = 0xFF;
  tx_pkt[2] = id;
  tx_pkt[3] = nparams + 2;
  tx_pkt[4] = inst;
  if (nparams)
    memcpy(&tx_pkt[5], params, nparams);
  tx_pkt[5 + nparams] = checksum(tx_pkt);
  tx_len = nparams + 6;

  rx_flush();
  return tx_bytes(tx_pkt, tx_len);
}

/* Waits for a status packet from `id`, skipping the echo of our own
 * instruction packet. Returns the error byte, or DXL_ERR_TIMEOUT. */
static int recv_status(uint8_t id, uint8_t *params, uint8_t nparams)
{
  uint32_t deadline = HAL_GetTick() + STATUS_TIMEOUT_MS + 1;
  uint8_t pkt[PKT_MAX];
  uint8_t b;

  for (;;) {
    /* Header: FF FF, then ID must not be FF */
    if (!rx_get(&b, deadline)) return DXL_ERR_TIMEOUT;
    if (b != 0xFF) continue;
    if (!rx_get(&b, deadline)) return DXL_ERR_TIMEOUT;
    if (b != 0xFF) continue;
    do {
      if (!rx_get(&b, deadline)) return DXL_ERR_TIMEOUT;
    } while (b == 0xFF);

    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = b;
    if (!rx_get(&pkt[3], deadline)) return DXL_ERR_TIMEOUT;
    if (pkt[3] < 2 || pkt[3] + 4 > PKT_MAX) continue;

    for (uint8_t i = 0; i < pkt[3]; i++)
      if (!rx_get(&pkt[4 + i], deadline)) return DXL_ERR_TIMEOUT;

    uint8_t len = pkt[3] + 4;
    if (pkt[len - 1] != checksum(pkt)) continue;
    if (len == tx_len && memcmp(pkt, tx_pkt, len) == 0) continue; /* echo */
    if (pkt[2] != id) continue;
    if (pkt[3] != nparams + 2) continue;

    if (nparams)
      memcpy(params, &pkt[5], nparams);
    return pkt[4];
  }
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

int dxl_ping(uint8_t id)
{
  int ret = send_packet(id, INST_PING, NULL, 0);
  if (ret != DXL_OK)
    return ret;
  return recv_status(id, NULL, 0);
}

int dxl_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out)
{
  uint8_t params[2] = { addr, len };
  if (len + 6 > PKT_MAX)
    return DXL_ERR_ARG;
  int ret = send_packet(id, INST_READ, params, 2);
  if (ret != DXL_OK)
    return ret;
  return recv_status(id, out, len);
}

int dxl_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len)
{
  uint8_t params[PKT_MAX];
  if (len + 7 > PKT_MAX)
    return DXL_ERR_ARG;
  params[0] = addr;
  memcpy(&params[1], data, len);
  int ret = send_packet(id, INST_WRITE, params, len + 1);
  if (ret != DXL_OK || id == DXL_BROADCAST_ID)
    return ret;
  return recv_status(id, NULL, 0);
}

int dxl_write_u8(uint8_t id, uint8_t addr, uint8_t value)
{
  return dxl_write(id, addr, &value, 1);
}

int dxl_write_u16(uint8_t id, uint8_t addr, uint16_t value)
{
  uint8_t data[2] = { value & 0xFF, value >> 8 };
  return dxl_write(id, addr, data, 2);
}

int dxl_sync_write(uint8_t addr, uint8_t len, const uint8_t *ids,
                   const uint8_t *data, uint8_t count)
{
  uint8_t params[PKT_MAX];
  uint8_t n = 0;

  if (2 + count * (len + 1) + 6 > PKT_MAX)
    return DXL_ERR_ARG;
  params[n++] = addr;
  params[n++] = len;
  for (uint8_t i = 0; i < count; i++) {
    params[n++] = ids[i];
    memcpy(&params[n], &data[i * len], len);
    n += len;
  }
  return send_packet(DXL_BROADCAST_ID, INST_SYNC_WRITE, params, n);
}
