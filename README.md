# turrlebot

STM32F446RE + MX-64 x2 (RS-485, MAX13488E) 바퀴 구동.

## 구조

CubeMX 재생성 시 `Core/`의 USER CODE 밖은 덮어써지므로 직접 작성한 코드는 모두 `App/`에 둔다.

| 파일 | 내용 |
|---|---|
| `App/dxl.c/h` | Dynamixel Protocol 1.0 (ping / read / write / sync write, TX echo 무시) |
| `App/mx64.c/h` | MX-64 Control Table, 휠 모드, 속도 명령 |
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
- NVIC: **USART3 global interrupt 켜기** (수신이 인터럽트 기반)

## 모터 설정 (Dynamixel Wizard, 한 번만)

두 모터 모두 공장 출하값이 ID 1 / 57600bps라서 **한 개씩만 연결해서** 바꾼다.

| | ID | Baud Rate (값) | Protocol |
|---|---|---|---|
| 왼쪽 | 1 | 1 Mbps (1) | 1.0 |
| 오른쪽 | 2 | 1 Mbps (1) | 1.0 |

휠 모드(CW/CCW Angle Limit = 0)는 `app_init()`에서 자동으로 설정한다.

## 동작 확인

- 디버거 Live Expressions에서 `g_dxl_ping` (0이면 응답 OK, -2면 타임아웃), `g_dxl_ready` 확인
- 12V가 늦게 들어와도 500ms마다 재시도한다
- 속도는 `app.c`의 `FIXED_SPEED` (1 = 약 0.114 rpm), 방향은 `LEFT_DIR` / `RIGHT_DIR`
