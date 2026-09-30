/*
 * sw.c - 보드 슬라이드 스위치 S1~S4 (PB12~PB15)
 */
#include "sw.h"
#include "main.h"

/*
 * PB12~PB15 를 입력으로 설정한다.
 * CubeMX 에서 이 핀들을 설정하지 않아도 동작하도록 여기서 직접 한다.
 */
void sw_init(void)
{
  GPIO_InitTypeDef init = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();         /* GPIOB 에 클럭 공급 */

  init.Pin  = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  init.Mode = GPIO_MODE_INPUT;
  init.Pull = GPIO_PULLUP;              /* 아무것도 연결 안 되면 HIGH(꺼짐) */
  HAL_GPIO_Init(GPIOB, &init);
}

/* 켜진(LOW) 스위치를 비트로 모아서 돌려준다 */
uint8_t sw_on(void)
{
  uint8_t on = 0;

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
    on |= SW1;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) {
    on |= SW2;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) {
    on |= SW3;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) {
    on |= SW4;
  }
  return on;
}
