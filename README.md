# turrlebot

STM32F446RE + MX-64(2.0) x2 (RS-485, MAX13488E, Protocol 2.0) 바퀴 구동.

## 구조

CubeMX 재생성 시 `Core/`의 USER CODE 밖은 덮어써지므로 직접 작성한 코드는 모두 `App/`에 둔다.

| 파일 | 내용 |
|---|---|
| `App/dxl.c/h` | Dynamixel Protocol 2.0 (ping / broadcast ping / read / write / sync write, CRC, byte stuffing, TX echo 무시) |
| `App/mx64.c/h` | MX-64(2.0) Control Table, 속도 모드(Operating Mode 1), Goal Velocity |
| `App/app.c/h` | 두 바퀴를 고정 속도로 구동 |

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
| `g_dxl_rx_bytes` | 받은 바이트 수 (0이면 에코도 안 돌아옴 = MCU~RS-485 칩 배선 문제) |

- 12V가 늦게 들어와도 500ms마다 재시도한다
- 속도는 `app.c`의 `FIXED_SPEED` (1 = 0.229 rpm), 방향은 `LEFT_DIR` / `RIGHT_DIR`
