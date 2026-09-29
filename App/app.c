/*
 * app.c - drive both wheels at a fixed speed
 *
 * Motors: MX-64(2.0), Protocol 2.0. At startup the bus is scanned with a
 * broadcast ping at several baud rates, so the motors' baud rate and IDs
 * do not have to be known in advance. The lower ID is the left wheel.
 * The right wheel is mounted mirrored, so its sign is flipped to make
 * both wheels roll forward.
 */
#include "app.h"
#include "main.h"
#include "dxl.h"
#include "mx64.h"

#define LEFT_DIR  (+1)
#define RIGHT_DIR (-1)

#define FIXED_SPEED 50           /* x 0.229 rpm = ~11.5 rpm */

#define RETRY_PERIOD_MS  500
#define UPDATE_PERIOD_MS 50

static const uint32_t bauds[] = { 1000000, 57600, 115200, 2000000, 9600 };

/* Watch these in the debugger (Expressions) */
volatile uint32_t g_dxl_baud;     /* baud rate the motors answered at */
volatile int g_dxl_count;         /* motors found (0..2) */
volatile uint8_t g_dxl_ids[2];    /* [0] = left, [1] = right */
volatile int g_dxl_err;           /* last setup error (<0 comm, >0 motor) */
volatile uint8_t g_dxl_hw_err[2]; /* Hardware Error Status seen (addr 70) */
volatile int g_dxl_reboots;       /* reboots done to clear hardware errors */
volatile int g_dxl_ready;
volatile uint32_t g_dxl_rx_bytes; /* > 0: at least our echo comes back */

static uint8_t ids[2];
static int count;
static uint32_t last_tick;

static int find_motors(void)
{
  for (unsigned b = 0; b < sizeof(bauds) / sizeof(bauds[0]); b++) {
    dxl_set_baud(bauds[b]);
    count = dxl_scan(ids, 2);
    if (count > 0) {
      if (count == 2 && ids[0] > ids[1]) {
        uint8_t t = ids[0]; ids[0] = ids[1]; ids[1] = t;
      }
      g_dxl_baud = bauds[b];
      return 1;
    }
  }
  return 0;
}

static int motors_setup(void)
{
  g_dxl_count = 0;
  g_dxl_baud = 0;
  if (!find_motors()) {
    g_dxl_rx_bytes = dxl_rx_count;
    return 0;
  }
  g_dxl_count = count;
  for (int i = 0; i < count; i++) {
    uint8_t hw = 0;
    int ret;

    g_dxl_ids[i] = ids[i];
    /* A latched hardware error keeps torque off until the motor reboots */
    ret = dxl_read(ids[i], MX64_ADDR_HARDWARE_ERROR, 1, &hw);
    if (ret >= 0 && hw) {
      g_dxl_hw_err[i] = hw;
      dxl_reboot(ids[i]);
      g_dxl_reboots++;
      HAL_Delay(1000);
    }
    ret = mx64_set_wheel_mode(ids[i]);
    if (MX64_FAILED(ret) || (ret & DXL_ALERT)) {
      g_dxl_err = ret;
      return 0;
    }
  }
  g_dxl_err = 0;
  return 1;
}

void app_init(void)
{
  dxl_init(USART3);
  HAL_Delay(300);                /* let the motors boot */
  g_dxl_ready = motors_setup();
  last_tick = HAL_GetTick();
}

void app_loop(void)
{
  uint32_t now = HAL_GetTick();

  if (!g_dxl_ready) {
    /* 12V may come up after the MCU; keep retrying */
    if (now - last_tick >= RETRY_PERIOD_MS) {
      g_dxl_ready = motors_setup();
      last_tick = HAL_GetTick();
    }
    return;
  }

  if (now - last_tick >= UPDATE_PERIOD_MS) {
    last_tick = now;
    const int32_t speeds[2] = { LEFT_DIR * FIXED_SPEED,
                                RIGHT_DIR * FIXED_SPEED };
    mx64_set_speeds(ids, speeds, (uint8_t)count);
  }
}
