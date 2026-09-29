#include "mx64.h"
#include "dxl.h"

#define MAX_MOTORS 8

static uint16_t speed_to_raw(int16_t speed)
{
  if (speed > MX64_SPEED_MAX)  speed = MX64_SPEED_MAX;
  if (speed < -MX64_SPEED_MAX) speed = -MX64_SPEED_MAX;
  /* 0..1023 = CCW, 1024..2047 = CW */
  return speed >= 0 ? (uint16_t)speed : (uint16_t)(1024 - speed);
}

int mx64_torque(uint8_t id, uint8_t on)
{
  return dxl_write_u8(id, MX64_ADDR_TORQUE_ENABLE, on ? 1 : 0);
}

int mx64_set_wheel_mode(uint8_t id)
{
  uint8_t limits[4];
  int ret = dxl_read(id, MX64_ADDR_CW_ANGLE_LIMIT, 4, limits);
  if (ret != DXL_OK)
    return ret;
  if (limits[0] == 0 && limits[1] == 0 && limits[2] == 0 && limits[3] == 0)
    return DXL_OK;

  const uint8_t zero[4] = { 0, 0, 0, 0 };
  return dxl_write(id, MX64_ADDR_CW_ANGLE_LIMIT, zero, 4);
}

int mx64_set_speed(uint8_t id, int16_t speed)
{
  return dxl_write_u16(id, MX64_ADDR_MOVING_SPEED, speed_to_raw(speed));
}

int mx64_set_speeds(const uint8_t *ids, const int16_t *speeds, uint8_t count)
{
  uint8_t data[MAX_MOTORS * 2];
  if (count > MAX_MOTORS)
    return DXL_ERR_ARG;
  for (uint8_t i = 0; i < count; i++) {
    uint16_t raw = speed_to_raw(speeds[i]);
    data[i * 2]     = raw & 0xFF;
    data[i * 2 + 1] = raw >> 8;
  }
  return dxl_sync_write(MX64_ADDR_MOVING_SPEED, 2, ids, data, count);
}
