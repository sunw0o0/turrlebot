#include "sw.h"
#include "main.h"

#define SW_PINS (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

/* Configured here so it works even if CubeMX leaves PB12..15 unused */
void sw_init(void)
{
  GPIO_InitTypeDef init = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  init.Pin = SW_PINS;
  init.Mode = GPIO_MODE_INPUT;
  init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &init);
}

uint8_t sw_read(void)
{
  return (uint8_t)((GPIOB->IDR >> 12) & 0x0F);
}

uint8_t sw_on(void)
{
  return (uint8_t)(~sw_read() & 0x0F);
}
