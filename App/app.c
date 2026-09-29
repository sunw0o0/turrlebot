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

#define RETRY_PERIOD_MS  500
#define UPDATE_PERIOD_MS 50

static const uint8_t ids[2] = { LEFT_ID, RIGHT_ID };

/* Watch these in the debugger (Live Expressions) */
volatile int g_dxl_ping[2] = { DXL_ERR_TIMEOUT, DXL_ERR_TIMEOUT };
volatile int g_dxl_ready;

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

void app_init(void)
{
  dxl_init(USART3);
  HAL_Delay(100);                /* let the motors boot */
  g_dxl_ready = motors_setup();
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
