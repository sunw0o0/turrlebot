/*
 * app.c - 스위치로 두 바퀴(MX-64) 를 굴린다
 *
 * ============ 전체 흐름 ============
 *
 *  app_init()  (전원 켜고 한 번)
 *    1. 스위치 핀 준비
 *    2. 모터 찾기   : 여러 통신 속도로 "누구 있어?" (broadcast ping)
 *    3. 모터 설정   : 에러 있으면 재부팅 -> 속도 모드 -> 토크 ON
 *
 *  app_loop()  (while(1) 안에서 계속)
 *    - 준비 안 됐으면 : 0.5초마다 2~3 다시 시도 (12V 를 늦게 켜도 됨)
 *    - 준비 됐으면     : 0.05초마다 스위치 읽기 -> 속도 정하기 -> 모터에 전송
 *
 * ============ 스위치 (켜짐 = 핀 LOW) ============
 *
 *   S1 켜짐 -> 달린다 (꺼지면 정지)
 *   S2 켜짐 -> 정지   (다른 스위치보다 우선)
 *   S3 켜짐 -> 2단 속도
 *   S4 켜짐 -> 1단 속도 유지 (S3 보다 우선)
 *   S3, S4 둘 다 꺼짐 -> 1단 속도
 */
#include "app.h"
#include "main.h"
#include "dxl.h"
#include "mx64.h"
#include "sw.h"

/* ==================== 바꿔 쓰는 설정값 ==================== */

/* 속도 (1 = 0.229 rpm) */
#define SPEED_1 50               /* 약 11.5 rpm */
#define SPEED_2 100              /* 약 22.9 rpm */

/* 바퀴 방향. 오른쪽 모터는 반대로 달려 있어서 -1 을 곱해야 같이 앞으로 간다.
   바퀴가 반대로 돌면 부호를 바꾸면 된다. */
#define LEFT_DIR  (+1)
#define RIGHT_DIR (-1)

#define UPDATE_PERIOD_MS 50      /* 속도 명령 보내는 주기 */
#define RETRY_PERIOD_MS  500     /* 모터를 못 찾았을 때 다시 찾는 주기 */

/* 모터를 찾을 때 시도할 통신 속도 (앞에서부터 차례로) */
static const uint32_t bauds[] = { 1000000, 57600, 115200, 2000000, 9600 };
#define NUM_BAUDS (sizeof(bauds) / sizeof(bauds[0]))


/* ==================== 디버거로 보는 변수 ==================== */
/* volatile: 디버거(Live Expressions)에서 항상 최신 값이 보이게 한다 */

volatile int      g_dxl_ready;      /* 1 = 설정 끝, 달리는 중 */
volatile uint32_t g_dxl_baud;       /* 모터를 찾은 통신 속도 (0 = 못 찾음) */
volatile int      g_dxl_count;      /* 찾은 모터 수 */
volatile uint8_t  g_dxl_ids[2];     /* [0] 왼쪽, [1] 오른쪽 모터 ID */
volatile int      g_dxl_err;        /* 설정 실패 이유 (-2 = 응답 없음) */
volatile uint8_t  g_dxl_hw_err[2];  /* 모터에 걸려 있던 하드웨어 에러 */
volatile int      g_dxl_reboots;    /* 에러 때문에 재부팅한 횟수 */
volatile uint32_t g_dxl_rx_bytes;   /* 받은 바이트 수 (0 = 배선 문제) */
volatile uint8_t  g_sw_on;          /* 켜진 스위치 (bit0 = S1 ... bit3 = S4) */
volatile int32_t  g_speed;          /* 지금 보내는 속도 */


/* ==================== 이 파일 안에서만 쓰는 변수 ==================== */

static uint8_t  ids[2];             /* 찾은 모터 ID ([0] 왼쪽, [1] 오른쪽) */
static int      count;              /* 찾은 모터 수 */
static uint32_t last_tick;          /* 마지막으로 일한 시각 (ms) */


/*
 * 모터 찾기
 * bauds[] 의 속도를 하나씩 바꿔가며 broadcast ping 을 보낸다.
 * 한 대라도 응답하면 그 속도로 정하고 1 을 돌려준다.
 */
static int find_motors(void)
{
  for (unsigned i = 0; i < NUM_BAUDS; i++) {
    dxl_set_baud(bauds[i]);
    count = dxl_scan(ids, 2);

    if (count > 0) {
      /* ID 가 작은 모터를 왼쪽([0]) 으로 */
      if (count == 2 && ids[0] > ids[1]) {
        uint8_t temp = ids[0];
        ids[0] = ids[1];
        ids[1] = temp;
      }
      g_dxl_baud = bauds[i];
      return 1;
    }
  }
  return 0;
}

/*
 * 모터 하나 설정
 * 성공하면 0, 실패하면 에러 값을 돌려준다.
 */
static int setup_one_motor(int index)
{
  uint8_t id = ids[index];
  uint8_t hw_error = 0;
  int ret;

  /* 1) 하드웨어 에러 확인.
        과부하 등으로 에러가 걸린 모터는 재부팅하기 전까지 토크가 안 켜진다. */
  ret = dxl_read(id, MX64_ADDR_HARDWARE_ERROR, 1, &hw_error);
  if (ret >= 0 && hw_error != 0) {
    g_dxl_hw_err[index] = hw_error;
    dxl_reboot(id);
    g_dxl_reboots++;
    HAL_Delay(1000);             /* 재부팅 끝날 때까지 기다림 */
  }

  /* 2) 바퀴용 설정 (속도 모드 + 토크 ON) */
  ret = mx64_set_wheel_mode(id);
  if (mx64_failed(ret)) {
    return ret;
  }
  if (ret & DXL_ALERT) {
    return ret;                  /* 재부팅 후에도 하드웨어 에러가 남아 있음 */
  }
  return 0;
}

/*
 * 모터 찾기 + 모두 설정
 * 다 되면 1, 하나라도 실패하면 0.
 */
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
    g_dxl_ids[i] = ids[i];

    int ret = setup_one_motor(i);
    if (ret != 0) {
      g_dxl_err = ret;
      return 0;
    }
  }

  g_dxl_err = 0;
  return 1;
}

/*
 * 스위치 상태로 속도를 정한다. 0 이면 정지.
 *   on & SW1  : S1 이 켜져 있으면 참
 *   !(...)    : 반대 (꺼져 있으면 참)
 */
static int32_t speed_from_switches(uint8_t on)
{
  /* S1 이 꺼져 있으면 정지 */
  if (!(on & SW1)) {
    return 0;
  }
  /* S2 가 켜져 있으면 정지 */
  if (on & SW2) {
    return 0;
  }
  /* S4 가 켜져 있으면 1단 (S3 보다 우선) */
  if (on & SW4) {
    return SPEED_1;
  }
  /* S3 가 켜져 있으면 2단 */
  if (on & SW3) {
    return SPEED_2;
  }
  /* 아무것도 없으면 1단 */
  return SPEED_1;
}


/* ==================== main.c 에서 부르는 함수 ==================== */

void app_init(void)
{
  sw_init();
  dxl_init(USART3);
  HAL_Delay(300);                /* 모터가 전원 켜고 부팅할 시간 */

  g_dxl_ready = motors_setup();
  last_tick = HAL_GetTick();
}

void app_loop(void)
{
  uint32_t now = HAL_GetTick();

  /* ---- 아직 준비 안 됨: 0.5초마다 다시 시도 ---- */
  if (!g_dxl_ready) {
    if (now - last_tick >= RETRY_PERIOD_MS) {
      g_dxl_ready = motors_setup();
      last_tick = HAL_GetTick();
    }
    return;
  }

  /* ---- 준비 됨: 0.05초마다 스위치 읽고 속도 보내기 ---- */
  if (now - last_tick >= UPDATE_PERIOD_MS) {
    last_tick = now;

    g_sw_on = sw_on();
    g_speed = speed_from_switches(g_sw_on);

    int32_t speeds[2];
    speeds[0] = LEFT_DIR * g_speed;    /* 왼쪽 */
    speeds[1] = RIGHT_DIR * g_speed;   /* 오른쪽 */
    mx64_set_speeds(ids, speeds, (uint8_t)count);
  }
}
