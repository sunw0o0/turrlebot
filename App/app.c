/*
 * app.c - drive both wheels at a fixed speed
 *
 * Motors: MX-64, Protocol 1.0, 1 Mbps, left = ID 1, right = ID 2.
 * The right wheel is mounted mirrored, so its sign is flipped to make
 * both wheels roll forward.
 */
#include "app.h"
#include "main.h"
#include "dxl.h"
#include "mx64.h"

#define LEFT_ID   1
#define RIGHT_ID  2
#define LEFT_DIR  (+1)
#define RIGHT_DIR (-1)

#define FIXED_SPEED 100          /* ~11.4 rpm; max 1023 */

#define DXL_BAUD 1000000

#define RETRY_PERIOD_MS  500
#define UPDATE_PERIOD_MS 50

static const uint8_t ids[2] = { LEFT_ID, RIGHT_ID };

/* Watch these in the debugger (Live Expressions) */
volatile int g_dxl_ping[2] = { DXL_ERR_TIMEOUT, DXL_ERR_TIMEOUT };
volatile int g_dxl_ready;

/* Filled by scan_bus() when setup fails: motors found at any baud rate.
 * g_scan_rx_bytes > 0 means at least our own TX echo came back on RX. */
#define SCAN_MAX 4
volatile int g_scan_count;
volatile uint32_t g_scan_baud[SCAN_MAX];
volatile uint8_t g_scan_id[SCAN_MAX];
volatile uint32_t g_scan_rx_bytes;

static uint32_t last_tick;

static int motors_setup(void)
{
  int ok = 1;

  for (int i = 0; i < 2; i++) {
    g_dxl_ping[i] = dxl_ping(ids[i]);
    if (g_dxl_ping[i] < 0) {
      ok = 0;
      continue;
    }
    if (mx64_torque(ids[i], 0) < 0 ||
        mx64_set_wheel_mode(ids[i]) < 0 ||
        mx64_set_speed(ids[i], 0) < 0 ||
        mx64_torque(ids[i], 1) < 0)
      ok = 0;
  }
  return ok;
}

/* Pings IDs 0..20 at common MX baud rates to find where the motors are. */
static void scan_bus(void)
{
  static const uint32_t bauds[] = { 1000000, 57600, 115200 };
  uint32_t rx_before = dxl_rx_count;

  g_scan_count = 0;
  for (unsigned b = 0; b < sizeof(bauds) / sizeof(bauds[0]); b++) {
    dxl_set_baud(bauds[b]);
    for (uint8_t id = 0; id <= 20; id++) {
      if (dxl_ping(id) >= 0 && g_scan_count < SCAN_MAX) {
        g_scan_baud[g_scan_count] = bauds[b];
        g_scan_id[g_scan_count] = id;
        g_scan_count++;
      }
    }
  }
  g_scan_rx_bytes = dxl_rx_count - rx_before;
  dxl_set_baud(DXL_BAUD);
}

void app_init(void)
{
  dxl_init(USART3);
  HAL_Delay(100);                /* let the motors boot */
  g_dxl_ready = motors_setup();
  if (!g_dxl_ready)
    scan_bus();
  last_tick = HAL_GetTick();
}

void app_loop(void)
{
  uint32_t now = HAL_GetTick();

  if (!g_dxl_ready) {
    /* 12V may come up after the MCU; keep retrying */
    if (now - last_tick >= RETRY_PERIOD_MS) {
      last_tick = now;
      g_dxl_ready = motors_setup();
    }
    return;
  }

  if (now - last_tick >= UPDATE_PERIOD_MS) {
    last_tick = now;
    const int16_t speeds[2] = { LEFT_DIR * FIXED_SPEED,
                                RIGHT_DIR * FIXED_SPEED };
    mx64_set_speeds(ids, speeds, 2);
  }
}
