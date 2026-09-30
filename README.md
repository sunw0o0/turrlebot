# turrlebot

STM32F446RE + MX-64(2.0) x2 (RS-485, MAX13488E, Protocol 2.0) 바퀴 구동.

## 구조

CubeMX 재생성 시 `Core/`의 USER CODE 밖은 덮어써지므로 직접 작성한 코드는 모두 `App/`에 둔다.

| 파일 | 내용 |
|---|---|
| `App/dxl.c/h` | Dynamixel Protocol 2.0 (ping / broadcast ping / read / write / sync write, CRC, byte stuffing, TX echo 무시) |
| `App/mx64.c/h` | MX-64(2.0) Control Table, 속도 모드(Operating Mode 1), Goal Velocity |
| `App/app.c/h` | 스위치에 따라 두 바퀴 구동 |
| `App/sw.c/h` | 보드 슬라이드 스위치 S1~S4 (PB12~PB15) 읽기 |
| `App/host.c/h` | 젯슨과 시리얼 통신 (USART6, 인터럽트 수신). 지금은 받은 바이트를 그대로 돌려보내는 에코 테스트 |
| `jetson/echo_test.py` | 젯슨에서 돌리는 에코 테스트 스크립트 |

`main.c`:

```c
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
app_init();
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
app_loop();
/* USER CODE END 3 */
```

STM32CubeIDE: `App` 폴더를 Source Location에 추가하고 (Project Properties > C/C++ General > Paths and Symbols > Source Location), Include paths에도 `App`을 추가한다.

## CubeMX 설정

- USART3: Asynchronous, 1 Mbps, 8N1, **TX = PC10**, RX = PC11 (PB10은 LED4)
- USART3 드라이버는 LL (Project Manager > Advanced Settings). 송수신은 폴링이라 USART3 인터럽트는 필요 없음

## 모터

- 펌웨어 MX-64(2.0), Protocol 2.0. 통신 속도와 ID는 코드가 자동으로 찾는다.
  - 1M, 57600, 115200, 2M, 9600bps 순서로 broadcast ping을 보내 처음 응답한 속도를 쓴다.
  - 찾은 ID 중 작은 쪽이 왼쪽 바퀴다. 두 모터의 ID는 서로 달라야 한다.
- 속도 모드(Operating Mode = 1)는 `app_init()`에서 자동으로 설정한다 (EEPROM은 값이 다를 때만 쓴다).

## 동작 확인 (디버거 Expressions)

| 변수 | 의미 |
|---|---|
| `g_dxl_ready` | 1이면 설정 완료, 바퀴 구동 중 |
| `g_dxl_baud` | 모터가 응답한 통신 속도 (0이면 못 찾음) |
| `g_dxl_count`, `g_dxl_ids` | 찾은 모터 수와 ID ([0] 왼쪽, [1] 오른쪽) |
| `g_dxl_err` | 설정 중 마지막 에러 (-2 응답 없음, 양수는 모터 에러 코드, 128(0x80)은 하드웨어 에러 걸림) |
| `g_dxl_hw_err` | 모터에 걸려 있던 Hardware Error Status (0x01 전압, 0x04 과열, 0x08 엔코더, 0x10 전기 충격, 0x20 과부하). 있으면 Reboot로 지우고 다시 설정한다 |
| `g_dxl_reboots` | 하드웨어 에러를 지우려고 재부팅한 횟수 (계속 늘어나면 원인이 남아 있음) |
| `g_dxl_rx_bytes` | 받은 바이트 수. **모터를 못 찾았을 때만** 기록되고, 찾으면 0으로 남는다. 못 찾았는데 0이면 에코도 안 돌아옴 = MCU~RS-485 칩 배선 문제, 0보다 크면 모터 쪽(A/B 배선, 12V, 통신 속도) 문제 |
| `g_dxl_volt`, `g_dxl_volt_min`, `g_dxl_volt_max` | 모터가 측정한 전압과 허용 범위 (0.1V 단위, 120 = 12.0V). 전압 에러(0x01) 확인용 |

- 12V가 늦게 들어와도 500ms마다 재시도한다
- 속도는 `app.c`의 `SPEED_1` / `SPEED_2` (1 = 0.229 rpm), 방향은 `LEFT_DIR` / `RIGHT_DIR`

## 스위치 (켜짐 = 핀 LOW)

| 스위치 | 동작 |
|---|---|
| S1 (PB12) | 켜져 있는 동안 구동, 꺼지면 정지 |
| S2 (PB13) | 켜져 있으면 정지 (다른 스위치보다 우선) |
| S3 (PB14) | 켜져 있으면 2단 속도 (`SPEED_2`) |
| S4 (PB15) | 켜져 있으면 1단 속도 유지 (`SPEED_1`, S3보다 우선) |

S3/S4 둘 다 꺼져 있으면 1단 속도. 디버거에서 `g_sw_on`(bit0=S1 ... bit3=S4), `g_speed`로 확인.

## 젯슨 통신 (USART6)

| STM32 (J10) | 젯슨 Orin Nano 40핀 |
|---|---|
| 1 GND | 6 GND |
| 3 PC6 USART6_TX | 10 RXD |
| 4 PC7 USART6_RX | 8 TXD |
| 2 +5V | 연결하지 않음 |

- CubeMX: USART6 Asynchronous 115200 8N1, 드라이버 LL, NVIC USART6 global interrupt 켜기
- 코드 생성 후 `Core/Src/stm32f4xx_it.c` 에 두 줄 추가 (USER CODE 구역이라 재생성해도 유지됨)
  - `/* USER CODE BEGIN Includes */` 아래: `#include "host.h"`
  - `/* USER CODE BEGIN USART6_IRQn 0 */` 아래: `host_irq();`
- 젯슨: `python3 jetson/echo_test.py /dev/ttyTHS2` 로 보낸 문자열이 그대로 돌아오면 성공
- 젯슨 Orin Nano Super 에서 40핀 8/10번은 `/dev/ttyTHS2` (ttyTHS1 아님)
- STM32 가 켜질 때 `STM32 ready` 를 한 번 보낸다. J10 3번-4번을 점퍼로 이으면 에코가 계속 돌아 rx/tx 카운터가 늘어난다 (STM32 쪽 자체 점검)
- 디버거: `g_host_rx_bytes` (받은 바이트 수), `g_host_tx_bytes` (보낸 바이트 수), `g_host_overflow` (버퍼 넘침)
