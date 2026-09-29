#include "mx64.h"
#include "dxl.h"

#define MAX_MOTORS 8

int mx64_torque(uint8_t id, uint8_t on)
{
  return dxl_write_u8(id, MX64_ADDR_TORQUE_ENABLE, on ? 1 : 0);
}

int mx64_set_wheel_mode(uint8_t id)
{
  uint8_t mode;
  int ret;

  if ((ret = mx64_torque(id, 0)) < 0)
    return ret;
  if ((ret = dxl_read(id, MX64_ADDR_OPERATING_MODE, 1, &mode)) < 0)
    return ret;
  if (mode != MX64_MODE_VELOCITY &&
      (ret = dxl_write_u8(id, MX64_ADDR_OPERATING_MODE, MX64_MODE_VELOCITY)) < 0)
    return ret;
  if ((ret = mx64_set_speed(id, 0)) < 0)
    return ret;
  return mx64_torque(id, 1);
}

int mx64_set_speed(uint8_t id, int32_t speed)
{
  return dxl_write_u32(id, MX64_ADDR_GOAL_VELOCITY, (uint32_t)speed);
}

int mx64_set_speeds(const uint8_t *ids, const int32_t *speeds, uint8_t count)
{
  uint8_t data[MAX_MOTORS * 4];
  if (count > MAX_MOTORS)
    return DXL_ERR_ARG;
  for (uint8_t i = 0; i < count; i++) {
    uint32_t v = (uint32_t)speeds[i];
    data[i * 4]     = v & 0xFF;
    data[i * 4 + 1] = (v >> 8) & 0xFF;
    data[i * 4 + 2] = (v >> 16) & 0xFF;
    data[i * 4 + 3] = v >> 24;
  }
  return dxl_sync_write(MX64_ADDR_GOAL_VELOCITY, 4, ids, data, count);
}
