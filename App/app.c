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
 *  app_loop()  (while(1) 안에서 계속, 맨 아래에 있음)
 *    handle_host()   : 젯슨에서 온 패킷 처리 (왼쪽/오른쪽 바퀴 속도 명령 저장)
 *    task_psd()      : 0.01초마다 PSD 측정, 0.05초마다 젯슨에 전송
 *    task_status()   : 0.1초마다 젯슨에 상태 보고
 *    준비 안 됐으면 :
 *      task_retry()  : 0.5초마다 2~3 다시 시도 (12V 를 늦게 켜도 됨)
 *    준비 됐으면 :
 *      task_check()  : 1초마다 모터 전압/에러/토크 확인
 *      task_encoder(): 0.05초마다 엔코더(위치/속도) 읽고 젯슨에 전송
 *      task_drive()  : 0.05초마다 속도 정하기 -> 모터에 전송
 *
 * 처음 읽을 때는 맨 아래 app_loop() 부터 보고, 궁금한 함수를 위로 찾아 올라가면 된다.
 *
 * ============ 어떻게 움직일지 (위에서부터 우선) ============
 *
 *   S2 켜짐                          -> 비상 정지
 *   S1 꺼짐                          -> 정지
 *   젯슨 바퀴 명령이 0.3초 안에 옴   -> 그 명령대로 주행
 *   S3 또는 S4 켜짐                  -> 고정 속도 테스트 (S4 = 1단, S3 = 2단)
 *   그 외 (ROS 명령 끊김)            -> 정지
 *
 * ============ 바퀴 속도 -> MX-64 값 ============
 *
 *   젯슨(팀 stm 패키지)이 cmd_vel 을 왼쪽/오른쪽 바퀴 선속도(mm/s)로 바꿔서 보낸다.
 *   바퀴 각속도 (rad/s) = 바퀴 선속도 (m/s) / r      (r = 바퀴 반지름)
 *   MX-64 값 = rpm / 0.229,   rpm = rad/s * 60 / (2 * pi)
 */
#include "app.h"
#include "main.h"
#include "dxl.h"
#include "mx64.h"
#include "sw.h"
#include "host.h"
#include "proto.h"
#include "psd.h"

/* ==================== 바꿔 쓰는 설정값 ==================== */

/* 속도 (1 = 0.229 rpm) */
#define SPEED_1 50               /* 약 11.5 rpm */
#define SPEED_2 100              /* 약 22.9 rpm */

/* 바퀴 방향. 오른쪽 모터는 반대로 달려 있어서 -1 을 곱해야 같이 앞으로 간다.
   바퀴가 반대로 돌면 부호를 바꾸면 된다. */
#define LEFT_DIR  (+1)
#define RIGHT_DIR (-1)

/* 어느 모터가 왼쪽인지. 1 = ID 가 작은 모터가 왼쪽, 0 = ID 가 큰 모터가 왼쪽.
   teleop 에서 j(왼쪽 회전)를 눌렀는데 오른쪽으로 돌면서 i 는 앞으로 가면
   이 값을 바꾸고 LEFT_DIR / RIGHT_DIR 부호도 서로 바꾼다. */
#define LEFT_IS_SMALLER_ID 0   /* 로봇 앞쪽 기준: 왼쪽 바퀴 = ID 2, 오른쪽 바퀴 = ID 1 */

/* ---- 로봇 치수 (임시값! 실제 로봇에 맞게 바꾸기) ---- */
#define WHEEL_RADIUS_M     0.05f   /* 바퀴 반지름 r (m) */
/* 좌우 바퀴 사이 거리는 젯슨 쪽 (stm_bridge.yaml 의 wheel_separation) 에서 쓴다 */

/* ---- 안전 제한 ---- */
#define MAX_WHEEL_MPS   0.30f      /* 바퀴 선속도 최대 (m/s) */
#define MAX_GOAL        250        /* MX-64 값 최대 (약 57 rpm) */
#define TEST_GOAL_MAX   285        /* 디버거 시험(g_test_goal) 최대 = 모터 속도 제한 (약 65 rpm) */
#define CMD_TIMEOUT_MS  300        /* 이 시간 동안 ROS 명령이 없으면 정지 */

#define STATUS_PERIOD_MS 100       /* 젯슨에 상태 보고하는 주기 */
#define UPDATE_PERIOD_MS 50        /* 속도 명령 보내는 주기 */
#define RETRY_PERIOD_MS  500     /* 모터를 못 찾았을 때 다시 찾는 주기 */
#define CHECK_PERIOD_MS  1000      /* 모터 전압 / 에러 / 토크를 다시 읽는 주기 */
#define VEL_PERIOD_MS    50        /* 모터 실제 속도 / 위치(엔코더)를 읽고 젯슨에 보내는 주기 */
#define PSD_PERIOD_MS    10        /* PSD 측정 주기 */
#define PSD_SEND_MS      50        /* 젯슨에 PSD 보내는 주기 */

/* 바퀴 모터 수. 이만큼 다 찾아야 "준비 완료" (덜 찾으면 0.5초마다 다시 찾는다) */
#define NUM_MOTORS 2
/* 한 통신 속도에서 broadcast ping 을 몇 번 보낼지 (응답을 놓쳐도 다시 잡도록) */
#define SCAN_TRIES 3

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
volatile uint8_t  g_dxl_last_hw_err[2]; /* 0 이 아니었던 마지막 하드웨어 에러 (재부팅으로 지워져도 남음) */
volatile int      g_dxl_reboots;    /* 에러 때문에 재부팅한 횟수 */
/* 전압 (0.1V 단위, 예: 120 = 12.0V). 에러 1(입력 전압 이상) 확인용 */
volatile uint16_t g_dxl_volt[2];    /* 모터가 측정한 지금 전압 */
volatile uint16_t g_dxl_volt_min[2];/* 모터에 설정된 전압 하한 */
volatile uint16_t g_dxl_volt_max[2];/* 모터에 설정된 전압 상한 */
volatile uint32_t g_dxl_vel_limit[2]; /* 켜질 때 모터에 설정돼 있던 속도 제한 (285 보다 작으면 285 로 올림) */
volatile uint8_t  g_dxl_torque[2];  /* 모터 토크 (1 = 켜짐, 0 = 모터가 스스로 끔) */
volatile uint32_t g_dxl_rx_bytes;   /* 받은 바이트 수 (0 = 배선 문제) */
volatile uint8_t  g_sw_on;          /* 켜진 스위치 (bit0 = S1 ... bit3 = S4) */
volatile int32_t  g_speed;          /* 스위치 테스트 모드 속도 */
volatile int16_t  g_cmd_left_mm;    /* 받은 왼쪽 바퀴 속도 (mm/s) */
volatile int16_t  g_cmd_right_mm;   /* 받은 오른쪽 바퀴 속도 (mm/s) */
volatile int32_t  g_goal[2];        /* 모터에 보내는 값 [0] 왼쪽, [1] 오른쪽 */
volatile int32_t  g_present_vel[2]; /* 모터가 엔코더로 잰 실제 속도 (g_goal 과 같은 단위/방향, 1 = 0.229 rpm) */
volatile int32_t  g_present_pos[2]; /* 모터 엔코더 위치 (4096 = 바퀴 1바퀴, 앞으로 가면 커짐, 여러 바퀴 누적) */
volatile float    g_goal_rpm[2];    /* g_goal 을 rpm 으로 */
volatile float    g_present_rpm[2]; /* g_present_vel 을 rpm 으로 */
/* 디버거 속도 시험: Live Expressions 에서 이 값을 바꾸면 (0 이 아니면)
   젯슨 명령 대신 두 바퀴를 이 값으로 돌린다. 1 = 0.229 rpm, 최대 TEST_GOAL_MAX.
   S1 켜짐 + S2 꺼짐 일 때만 동작. 끝나면 0 으로 되돌린다. */
volatile int32_t  g_test_goal;
volatile uint8_t  g_state;          /* PROTO_STATE_* (proto.h) */


/* ==================== 이 파일 안에서만 쓰는 변수 ==================== */

static uint8_t  ids[2];             /* 찾은 모터 ID ([0] 왼쪽, [1] 오른쪽) */
static int      count;              /* 찾은 모터 수 */
static uint32_t last_tick;          /* 마지막으로 일한 시각 (ms) */
static uint32_t last_status_tick;   /* 마지막으로 상태를 보고한 시각 */
static uint32_t last_check_tick;    /* 마지막으로 모터 상태를 읽은 시각 */
static uint32_t last_vel_tick;      /* 마지막으로 실제 속도 / 위치를 읽은 시각 */
static uint32_t last_psd_tick;      /* 마지막으로 PSD 를 잰 시각 */
static uint32_t last_psd_send_tick; /* 마지막으로 PSD 를 보낸 시각 */
static uint32_t last_cmd_tick;      /* 마지막으로 ROS 명령을 받은 시각 */
static int      has_cmd;            /* ROS 명령을 한 번이라도 받았는지 */


/* ==================== 바이트 <-> 숫자 바꾸기 도우미 ====================
 *
 * 통신은 1바이트(0~255)씩 오가므로, 큰 숫자는 여러 바이트로 쪼개서 보낸다.
 * 다이나믹셀과 젯슨 둘 다 "작은 자리 먼저" (little-endian) 순서를 쓴다.
 *   예) 300 = 0x012C  ->  [0x2C] [0x01]
 */

/* 2바이트 -> 16비트 숫자 (부호 없음) */
static uint16_t to_uint16(const uint8_t *b)
{
  return (uint16_t)(b[0] | (b[1] << 8));
}

/* 4바이트 -> 32비트 숫자 (부호 있음, 음수도 됨) */
static int32_t to_int32(const uint8_t *b)
{
  uint32_t value = b[0];
  value = value | ((uint32_t)b[1] << 8);
  value = value | ((uint32_t)b[2] << 16);
  value = value | ((uint32_t)b[3] << 24);
  return (int32_t)value;
}

/* 16비트 숫자 -> 2바이트 (out[0], out[1] 에 넣는다) */
static void put_16(uint8_t *out, uint16_t value)
{
  out[0] = (uint8_t)(value & 0xFF);
  out[1] = (uint8_t)(value >> 8);
}

/* 32비트 숫자 -> 4바이트 (out[0] ~ out[3] 에 넣는다) */
static void put_32(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)(value & 0xFF);
  out[1] = (uint8_t)((value >> 8) & 0xFF);
  out[2] = (uint8_t)((value >> 16) & 0xFF);
  out[3] = (uint8_t)((value >> 24) & 0xFF);
}


/* ==================== 모터 찾기 ==================== */

/* ids[] 에 이 ID 가 이미 들어 있으면 1, 없으면 0 */
static int already_have(uint8_t id)
{
  for (int m = 0; m < count; m++)
  {
    if (ids[m] == id)
    {
      return 1;
    }
  }
  return 0;
}

/* 이번에 찾은 ID 들 중 처음 보는 것만 ids[] 에 추가한다 */
static void add_new_ids(const uint8_t *found, int n)
{
  for (int k = 0; k < n; k++)
  {
    if (already_have(found[k]))
    {
      continue;                  /* 이미 있음 -> 건너뜀 */
    }
    if (count < NUM_MOTORS)
    {
      ids[count] = found[k];
      count++;
    }
  }
}

/* 왼쪽 모터가 ids[0] 에 오도록 순서를 맞춘다 (LEFT_IS_SMALLER_ID 참고) */
static void put_left_first(void)
{
  if (count != 2)
  {
    return;
  }
  int first_is_bigger = (ids[0] > ids[1]);
  if (first_is_bigger == LEFT_IS_SMALLER_ID)
  {
    /* 두 값 바꾸기 */
    uint8_t temp = ids[0];
    ids[0] = ids[1];
    ids[1] = temp;
  }
}

/*
 * 모터 찾기
 * bauds[] 의 속도를 하나씩 바꿔가며 broadcast ping 을 보낸다.
 * 한 속도에서 SCAN_TRIES 번 보내서 찾은 ID 를 모두 모은다.
 * (두 모터가 연달아 대답할 때 하나를 놓쳐도 다음 번에 잡을 수 있게)
 * 한 대라도 응답하면 그 속도로 정하고 1 을 돌려준다.
 */
static int find_motors(void)
{
  for (unsigned i = 0; i < NUM_BAUDS; i++)
  {
    dxl_set_baud(bauds[i]);
    count = 0;

    for (int t = 0; t < SCAN_TRIES && count < NUM_MOTORS; t++)
    {
      uint8_t found[NUM_MOTORS];
      int n = dxl_scan(found, NUM_MOTORS);
      if (t == 0 && n == 0)
      {
        break;                   /* 이 속도엔 아무도 없음 -> 바로 다음 속도로 */
      }

      add_new_ids(found, n);
    }

    if (count > 0)
    {
      put_left_first();
      g_dxl_baud = bauds[i];
      return 1;
    }
  }
  return 0;
}

/*
 * 모터 토크가 실제로 켜져 있으면 1.
 * (과부하/과열 등으로 모터가 스스로 토크를 끈 상태인지 확인)
 */
static int torque_is_on(uint8_t id)
{
  uint8_t on = 0;
  if (dxl_read(id, MX64_ADDR_TORQUE_ENABLE, 1, &on) < 0)
  {
    return 0;
  }
  return on == 1;
}

/* ==================== 모터 설정 ==================== */

/* 하드웨어 에러 값을 기록한다. 0 이 아니면 "마지막 에러" 에도 남긴다 */
static void record_hw_error(int index, uint8_t value)
{
  g_dxl_hw_err[index] = value;
  if (value != 0)
  {
    g_dxl_last_hw_err[index] = value;
  }
}

/* 진단용: 모터의 전압, 전압 허용 범위, 속도 제한을 읽어서 디버거 변수에 넣는다 */
static void read_diagnostics(int index)
{
  uint8_t id = ids[index];
  uint8_t buf[4];

  if (dxl_read(id, MX64_ADDR_PRESENT_VOLTAGE, 2, buf) >= 0)
  {
    g_dxl_volt[index] = to_uint16(buf);
  }
  if (dxl_read(id, MX64_ADDR_MIN_VOLTAGE, 2, buf) >= 0)
  {
    g_dxl_volt_min[index] = to_uint16(buf);
  }
  if (dxl_read(id, MX64_ADDR_MAX_VOLTAGE, 2, buf) >= 0)
  {
    g_dxl_volt_max[index] = to_uint16(buf);
  }
  if (dxl_read(id, MX64_ADDR_VELOCITY_LIMIT, 4, buf) >= 0)
  {
    g_dxl_vel_limit[index] = (uint32_t)to_int32(buf);
  }
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

  /* 0) 진단용 값 읽기 */
  read_diagnostics(index);

  /* 1) 바퀴용 설정 (속도 모드 + 토크 ON) */
  ret = mx64_set_wheel_mode(id);
  if (mx64_failed(ret))
  {
    return ret;
  }

  /* 2) 에러 표시(Alert)가 있으면 어떤 에러인지 기록한다.
        전압 에러(0x01)처럼 토크를 끄지 않는 에러도 있으므로,
        실패 여부는 "토크가 실제로 켜졌는지"로 판단한다. */
  if (ret & DXL_ALERT)
  {
    if (dxl_read(id, MX64_ADDR_HARDWARE_ERROR, 1, &hw_error) >= 0)
    {
      record_hw_error(index, hw_error);
    }
    if (torque_is_on(id))
    {
      return 0;                  /* 경고는 있지만 움직일 수 있음 */
    }

    /* 3) 토크가 안 켜졌을 때만 재부팅해서 에러를 지우고 한 번 더 시도 */
    dxl_reboot(id);
    g_dxl_reboots++;
    HAL_Delay(1000);             /* 재부팅 끝날 때까지 기다림 */

    ret = mx64_set_wheel_mode(id);
    if (mx64_failed(ret))
    {
      return ret;
    }
    if (!torque_is_on(id))
    {
      return DXL_ALERT;          /* 재부팅 후에도 토크가 안 켜짐 */
    }
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

  if (!find_motors())
  {
    g_dxl_rx_bytes = dxl_rx_count;
    return 0;
  }
  g_dxl_count = count;
  g_dxl_ids[0] = ids[0];
  g_dxl_ids[1] = (count > 1) ? ids[1] : 0;

  /* 바퀴 모터를 다 못 찾았으면 준비 안 됨 -> 0.5초 뒤 다시 찾는다.
     (한 대만 찾은 채로 달리면 한쪽 바퀴만 돈다) */
  if (count < NUM_MOTORS)
  {
    g_dxl_err = DXL_ERR_TIMEOUT;
    return 0;
  }

  for (int i = 0; i < count; i++)
  {
    g_dxl_ids[i] = ids[i];

    int ret = setup_one_motor(i);
    if (ret != 0)
    {
      g_dxl_err = ret;
      return 0;
    }
    g_dxl_torque[i] = (uint8_t)torque_is_on(ids[i]);
  }

  g_dxl_err = 0;
  return 1;
}

/*
 * 달리는 중에 모터 상태 다시 읽기 (1초마다)
 * 전압, 하드웨어 에러, 토크를 읽어 디버거 변수와 젯슨 상태 보고에 반영한다.
 * 모터가 과부하 등으로 토크를 스스로 껐으면 0 을 돌려준다 -> 다시 설정(재부팅)한다.
 */
static int check_motors(void)
{
  int ok = 1;

  for (int i = 0; i < count; i++)
  {
    uint8_t buf[2];
    uint8_t value;

    if (dxl_read(ids[i], MX64_ADDR_PRESENT_VOLTAGE, 2, buf) >= 0)
    {
      g_dxl_volt[i] = to_uint16(buf);
    }
    if (dxl_read(ids[i], MX64_ADDR_HARDWARE_ERROR, 1, &value) >= 0)
    {
      record_hw_error(i, value);
    }
    if (dxl_read(ids[i], MX64_ADDR_TORQUE_ENABLE, 1, &value) >= 0)
    {
      g_dxl_torque[i] = value;
      if (value == 0)
      {
        ok = 0;                  /* 토크가 꺼짐 -> 명령을 보내도 안 돈다 */
      }
    }
  }
  return ok;
}

/*
 * 모터 실제 속도와 위치(엔코더) 읽기
 *   128번 Present Velocity (4바이트) 와 132번 Present Position (4바이트) 는
 *   붙어 있어서 128번부터 8바이트를 한 번에 읽는다.
 * 앞으로 가는 방향이 + 가 되도록 LEFT_DIR / RIGHT_DIR 을 곱한다.
 * 읽기에 실패하면 이전 값을 그대로 둔다.
 */
static void read_present_velocity(void)
{
  const int dir[2] = { LEFT_DIR, RIGHT_DIR };

  for (int i = 0; i < count; i++)
  {
    uint8_t buf[8];
    if (dxl_read(ids[i], MX64_ADDR_PRESENT_VELOCITY, 8, buf) >= 0)
    {
      g_present_vel[i] = dir[i] * to_int32(&buf[0]);   /* 128~131: 속도 */
      g_present_pos[i] = dir[i] * to_int32(&buf[4]);   /* 132~135: 위치 */
      g_present_rpm[i] = g_present_vel[i] * 0.229f;
    }
  }
}

/*
 * 스위치 상태로 속도를 정한다. 0 이면 정지.
 *   on & SW1  : S1 이 켜져 있으면 참
 *   !(...)    : 반대 (꺼져 있으면 참)
 */
static int32_t speed_from_switches(uint8_t on)
{
  /* S1 이 꺼져 있으면 정지 */
  if (!(on & SW1))
  {
    return 0;
  }
  /* S2 가 켜져 있으면 정지 */
  if (on & SW2)
  {
    return 0;
  }
  /* S4 가 켜져 있으면 1단 (S3 보다 우선) */
  if (on & SW4)
  {
    return SPEED_1;
  }
  /* S3 가 켜져 있으면 2단 */
  if (on & SW3)
  {
    return SPEED_2;
  }
  /* 아무것도 없으면 1단 */
  return SPEED_1;
}


/* ==================== 젯슨 통신 ==================== */

/* 받은 바이트를 패킷으로 조립하고, 속도 명령이면 저장한다 */
static void handle_host(void)
{
  uint8_t b;
  proto_packet_t pkt;

  while (host_read(&b))
  {
    if (!proto_feed(b, &pkt))
    {
      continue;                  /* 아직 패킷이 다 안 모임 */
    }
    if (pkt.cmd == PROTO_CMD_WHEEL && pkt.len == 4)
    {
      /* 2바이트씩 작은 자리 먼저 -> 부호 있는 16비트 정수 */
      g_cmd_left_mm  = (int16_t)to_uint16(&pkt.data[0]);
      g_cmd_right_mm = (int16_t)to_uint16(&pkt.data[2]);
      last_cmd_tick = HAL_GetTick();
      has_cmd = 1;
    }
  }
}

/* 젯슨에 상태 패킷을 보낸다 */
static void send_status(void)
{
  uint8_t data[PROTO_STATUS_LEN];
  uint8_t buf[PROTO_STATUS_LEN + 5];

  data[0] = (uint8_t)g_dxl_ready;
  data[1] = g_state;
  data[2] = g_sw_on;
  data[3] = g_dxl_hw_err[0];
  data[4] = g_dxl_hw_err[1];
  put_16(&data[5], g_dxl_volt[0]);           /* 5, 6번: 전압 */
  data[7] = (uint8_t)g_dxl_count;            /* 찾은 모터 수 */
  data[8] = g_dxl_ids[0];                    /* 왼쪽 모터 ID */
  data[9] = g_dxl_ids[1];                    /* 오른쪽 모터 ID */
  data[10] = (uint8_t)(g_dxl_torque[0] | (g_dxl_torque[1] << 1));  /* bit0 왼쪽, bit1 오른쪽 */
  /* 목표 속도와 실제 속도 (int16, 1 = 0.229 rpm, 앞으로 = +) */
  put_16(&data[11], (uint16_t)(int16_t)g_goal[0]);          /* 11, 12번: 왼쪽 목표 */
  put_16(&data[13], (uint16_t)(int16_t)g_goal[1]);          /* 13, 14번: 오른쪽 목표 */
  put_16(&data[15], (uint16_t)(int16_t)g_present_vel[0]);   /* 15, 16번: 왼쪽 실제 */
  put_16(&data[17], (uint16_t)(int16_t)g_present_vel[1]);   /* 17, 18번: 오른쪽 실제 */

  uint16_t n = proto_build(PROTO_CMD_STATUS, data, PROTO_STATUS_LEN, buf);
  host_write(buf, n);
}


/* 젯슨에 PSD 거리 패킷을 보낸다 (왼쪽, 앞, 오른쪽 mm) */
static void send_psd(void)
{
  uint8_t data[6];
  uint8_t buf[6 + 5];

  for (int i = 0; i < PSD_COUNT; i++)
  {
    put_16(&data[i * 2], g_psd_mm[i]);       /* 0,1 왼쪽 / 2,3 앞 / 4,5 오른쪽 */
  }
  uint16_t n = proto_build(PROTO_CMD_PSD, data, 6, buf);
  host_write(buf, n);
}


/* 젯슨에 엔코더 패킷을 보낸다 (위치 int32 왼쪽/오른쪽, 속도 int16 왼쪽/오른쪽) */
static void send_encoder(void)
{
  uint8_t data[PROTO_ENCODER_LEN];
  uint8_t buf[PROTO_ENCODER_LEN + 5];

  put_32(&data[0], (uint32_t)g_present_pos[0]);            /* 0~3번: 왼쪽 위치 */
  put_32(&data[4], (uint32_t)g_present_pos[1]);            /* 4~7번: 오른쪽 위치 */
  put_16(&data[8], (uint16_t)(int16_t)g_present_vel[0]);   /* 8, 9번: 왼쪽 속도 */
  put_16(&data[10], (uint16_t)(int16_t)g_present_vel[1]);  /* 10, 11번: 오른쪽 속도 */
  uint16_t n = proto_build(PROTO_CMD_ENCODER, data, PROTO_ENCODER_LEN, buf);
  host_write(buf, n);
}


/* ==================== 바퀴 속도 -> MX-64 값 ==================== */

/* 바퀴 각속도 (rad/s) -> MX-64 Goal Velocity 값 (소수) */
static float wheel_to_goal(float wheel_rad_s)
{
  const float TWO_PI = 6.2831853f;
  float rpm = wheel_rad_s * 60.0f / TWO_PI;
  return rpm / 0.229f;
}

/* 소수 -> 정수 (반올림) */
static int32_t round_to_int(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value + 0.5f);
  }
  return (int32_t)(value - 0.5f);
}

/* 절댓값 */
static float absf(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }
  return value;
}

/* 왼쪽/오른쪽 바퀴 선속도(mm/s) -> 왼쪽/오른쪽 MX-64 값 */
static void wheels_to_goals(int16_t left_mm, int16_t right_mm,
                            int32_t *left, int32_t *right)
{
  float left_mps  = left_mm / 1000.0f;
  float right_mps = right_mm / 1000.0f;

  float left_rad_s  = left_mps / WHEEL_RADIUS_M;
  float right_rad_s = right_mps / WHEEL_RADIUS_M;

  float l = wheel_to_goal(left_rad_s);
  float r = wheel_to_goal(right_rad_s);

  /* 한쪽이라도 최대를 넘으면 두 바퀴를 같은 비율로 줄인다.
     (한쪽만 자르면 도는 모양이 바뀌기 때문) */
  float biggest = absf(l);
  if (absf(r) > biggest)
  {
    biggest = absf(r);
  }
  float limit = (float)MAX_GOAL;
  float limit_by_speed = wheel_to_goal(MAX_WHEEL_MPS / WHEEL_RADIUS_M);
  if (limit_by_speed < limit)
  {
    limit = limit_by_speed;
  }
  if (biggest > limit)
  {
    float scale = limit / biggest;
    l = l * scale;
    r = r * scale;
  }

  *left  = round_to_int(l);
  *right = round_to_int(r);
}

/* 값을 -max ~ +max 사이로 자른다 */
static int32_t clamp(int32_t value, int32_t max)
{
  if (value > max)
  {
    return max;
  }
  if (value < -max)
  {
    return -max;
  }
  return value;
}

/* ROS 명령을 받은 지 0.3초가 안 지났으면 1 */
static int ros_cmd_is_fresh(uint32_t now)
{
  if (!has_cmd)
  {
    return 0;                    /* 한 번도 못 받음 */
  }
  return (now - last_cmd_tick) < CMD_TIMEOUT_MS;
}

/*
 * 지금 어떻게 움직일지 정한다.
 * left, right 에 "앞으로 가는 방향 기준" 바퀴 값을 넣고 상태를 돌려준다.
 * (모터 방향 LEFT_DIR / RIGHT_DIR 은 보내기 직전에 곱한다)
 */
static uint8_t decide(uint8_t on, uint32_t now, int32_t *left, int32_t *right)
{
  *left = 0;
  *right = 0;

  if (on & SW2)
  {
    return PROTO_STATE_ESTOP;
  }
  if (!(on & SW1))
  {
    return PROTO_STATE_STOP_SW;
  }
  if (g_test_goal != 0)
  {
    int32_t goal = clamp(g_test_goal, TEST_GOAL_MAX);
    *left = goal;
    *right = goal;
    return PROTO_STATE_MANUAL;   /* 디버거 시험 중 */
  }
  if (ros_cmd_is_fresh(now))
  {
    wheels_to_goals(g_cmd_left_mm, g_cmd_right_mm, left, right);
    return PROTO_STATE_ROS;
  }
  if (on & (SW3 | SW4))
  {
    g_speed = speed_from_switches(on);
    *left = g_speed;
    *right = g_speed;
    return PROTO_STATE_MANUAL;
  }
  return PROTO_STATE_NO_CMD;
}


/* ==================== main.c 에서 처음 한 번 부르는 함수 ==================== */

void app_init(void)
{
  sw_init();
  host_init();                   /* 젯슨 통신 (USART6) */
  psd_init();                    /* PSD 거리 센서 (ADC1 + DMA) */
  dxl_init(USART3);
  HAL_Delay(300);                /* 모터가 전원 켜고 부팅할 시간 */

  g_dxl_ready = motors_setup();
  last_tick = HAL_GetTick();
}


/* ==================== app_loop 가 하는 일들 ====================
 *
 * 각 함수는 "시간이 됐으면 한 번 일하고, 아니면 그냥 돌아가는" 모양이다.
 *   now - last_xxx_tick >= 주기   : 지난번 일한 뒤로 주기만큼 시간이 지났나?
 *   last_xxx_tick = now           : 지금 일했다고 시각을 적어 둔다
 */

/* PSD: 0.01초마다 재고, 0.05초마다 젯슨에 보낸다 */
static void task_psd(uint32_t now)
{
  if (now - last_psd_tick >= PSD_PERIOD_MS)
  {
    last_psd_tick = now;
    psd_update();
  }
  if (now - last_psd_send_tick >= PSD_SEND_MS)
  {
    last_psd_send_tick = now;
    send_psd();
  }
}

/* 0.1초마다 젯슨에 상태 보고 */
static void task_status(uint32_t now)
{
  if (now - last_status_tick < STATUS_PERIOD_MS)
  {
    return;                      /* 아직 시간 안 됨 */
  }
  last_status_tick = now;

  g_sw_on = sw_on();
  if (!g_dxl_ready)
  {
    g_state = PROTO_STATE_NOT_READY;
  }
  send_status();
}

/* 모터 준비가 안 됐을 때: 0.5초마다 다시 찾고 설정한다 */
static void task_retry(uint32_t now)
{
  if (now - last_tick < RETRY_PERIOD_MS)
  {
    return;
  }
  g_dxl_ready = motors_setup();
  last_tick = HAL_GetTick();     /* 설정에 시간이 걸리므로 끝난 시각으로 */
}

/* 1초마다 모터 상태 확인. 모터가 멀쩡하면 1, 토크가 꺼졌으면 0 */
static int task_check(uint32_t now)
{
  if (now - last_check_tick < CHECK_PERIOD_MS)
  {
    return 1;                    /* 아직 확인할 때 아님 -> 멀쩡하다고 봄 */
  }
  last_check_tick = now;
  return check_motors();
}

/* 0.05초마다 실제 속도 / 위치 읽고 젯슨에 엔코더 패킷 보내기 */
static void task_encoder(uint32_t now)
{
  if (now - last_vel_tick < VEL_PERIOD_MS)
  {
    return;
  }
  last_vel_tick = now;
  read_present_velocity();
  send_encoder();
}

/* 정한 속도를 디버거 변수에 기록한다 (앞으로 = + 기준) */
static void save_goals(int32_t left, int32_t right)
{
  g_goal[0] = left;
  g_goal[1] = right;
  g_goal_rpm[0] = left * 0.229f;
  g_goal_rpm[1] = right * 0.229f;
}

/* 모터에 속도를 보낸다. 모터 방향(LEFT_DIR / RIGHT_DIR) 을 여기서 곱한다 */
static void send_goals(int32_t left, int32_t right)
{
  int32_t speeds[2];
  speeds[0] = LEFT_DIR * left;   /* 왼쪽 */
  speeds[1] = RIGHT_DIR * right; /* 오른쪽 */
  mx64_set_speeds(ids, speeds, (uint8_t)count);
}

/* 0.05초마다 속도 정해서 모터에 보내기 */
static void task_drive(uint32_t now)
{
  if (now - last_tick < UPDATE_PERIOD_MS)
  {
    return;
  }
  last_tick = now;

  int32_t left;
  int32_t right;
  g_sw_on = sw_on();
  g_state = decide(g_sw_on, now, &left, &right);
  save_goals(left, right);
  send_goals(left, right);
}


/* ==================== main.c 에서 계속 부르는 함수 ==================== */

void app_loop(void)
{
  uint32_t now = HAL_GetTick();  /* 전원 켠 뒤 지난 시간 (ms) */

  /* 1) 모터 상태와 상관없이 항상 하는 일 */
  handle_host();                 /* 젯슨에서 온 명령 받기 */
  task_psd(now);                 /* PSD 재고 보내기 */
  task_status(now);              /* 상태 보고 */

  /* 2) 모터가 아직 준비 안 됨 -> 다시 시도만 하고 끝 */
  if (!g_dxl_ready)
  {
    task_retry(now);
    return;
  }

  /* 3) 모터 준비 됨 */
  if (!task_check(now))
  {
    g_dxl_ready = 0;             /* 토크 꺼짐 -> 다음 번에 task_retry 가 재부팅까지 해 준다 */
    return;
  }
  task_encoder(now);             /* 엔코더 읽고 보내기 */
  task_drive(now);               /* 속도 정해서 모터에 보내기 */
}
