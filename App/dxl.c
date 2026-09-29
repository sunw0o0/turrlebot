/*
 * dxl.c - Dynamixel Protocol 1.0 driver
 *
 * Packet: FF FF ID LEN INST/ERR PARAM... CHK
 *   LEN = number of params + 2
 *   CHK = ~(ID + LEN + INST + PARAM...)
 *
 * RX runs on a 1-byte interrupt reception feeding a ring buffer, so the
 * USART3 global interrupt must be enabled in CubeMX (NVIC Settings).
 */
#include "dxl.h"
#include <string.h>

#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_SYNC_WRITE 0x83

#define TX_TIMEOUT_MS     10
#define STATUS_TIMEOUT_MS 10   /* MX-64 default return delay is 500us */

#define RX_BUF_SIZE 256        /* power of two */
#define PKT_MAX     64

static UART_HandleTypeDef *dxl_huart;

static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head;   /* written by ISR */
static uint16_t rx_tail;            /* read by main loop */
static uint8_t rx_byte;

static uint8_t tx_pkt[PKT_MAX];
static uint8_t tx_len;

static void rx_start(void)
{
  HAL_UART_Receive_IT(dxl_huart, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != dxl_huart)
    return;
  rx_buf[rx_head & (RX_BUF_SIZE - 1)] = rx_byte;
  rx_head++;
  rx_start();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != dxl_huart)
    return;
  /* ORE aborts the reception; PE/FE/NE keep it running */
  if (huart->RxState == HAL_UART_STATE_READY)
    rx_start();
}

static void rx_flush(void)
{
  rx_tail = rx_head;
}

static int rx_get(uint8_t *b, uint32_t deadline)
{
  while (rx_tail == rx_head) {
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
      return 0;
  }
  *b = rx_buf[rx_tail & (RX_BUF_SIZE - 1)];
  rx_tail++;
  return 1;
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
  if (HAL_UART_Transmit(dxl_huart, tx_pkt, tx_len, TX_TIMEOUT_MS) != HAL_OK)
    return DXL_ERR_TX;
  return DXL_OK;
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

void dxl_init(UART_HandleTypeDef *huart)
{
  dxl_huart = huart;
  rx_head = 0;
  rx_tail = 0;
  rx_start();
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
