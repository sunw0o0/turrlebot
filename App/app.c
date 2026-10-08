/*
 * ============================================================================
 *  app.c - 로봇의 "두뇌". 우리가 직접 쓴 코드의 중심 파일
 * ============================================================================
 *
 * [이 파일이 하는 일]
 *   1. 젯슨(Jetson)이 USB 선으로 "왼쪽 바퀴 몇, 오른쪽 바퀴 몇" 하고 보내면 받는다.
 *   2. 스위치(S1~S4) 상태를 보고 "지금 움직여도 되나?" 를 정한다.
 *   3. 다이나믹셀 모터 2개(MX-64)에 속도를 보낸다.
 *   4. 모터 상태, PSD 거리, 엔코더 값을 젯슨에 다시 알려준다.
 *
 * [주석 읽는 법]
 *   "슬래시 별" 로 시작해서 "별 슬래시" 로 끝나는 글자는 "주석" 이다.
 *   컴퓨터는 무시하고, 사람만 읽는다. 이 설명 글도 전부 주석이다.
 *
 * [STM32 프로그램이 도는 방식]
 *   main.c (CubeMX 가 만든 파일) 안에 이런 모양이 있다.
 *
 *       app_init();        <- 전원 켜면 딱 한 번
 *       while (1)          <- 1 은 "참" 이라서 영원히 반복
 *       {
 *         app_loop();      <- 이게 1초에 수만 번 불린다
 *       }
 *
 *   그래서 app_loop() 는 "한 번 훑어보고 할 일 있으면 하고 바로 끝나는" 함수여야 한다.
 *   오래 기다리는 코드를 넣으면 다른 일(젯슨 통신, PSD 등)이 다 멈춘다.
 *
 * [처음 읽는 순서 추천]
 *   1) 맨 아래 app_loop()       : 무슨 일을 반복하는지 목록
 *   2) 그 위 task_...() 함수들   : 각 일을 "언제" 하는지
 *   3) decide()                 : 스위치 / 젯슨 명령으로 속도 정하기
 *   4) 맨 위 설정값과 변수들     : 이름이 무슨 뜻인지 다시 확인
 *
 * [어떻게 움직일지 - 위에 있을수록 우선]
 *   S2 켜짐                          -> 비상 정지
 *   S1 꺼짐                          -> 정지
 *   디버거에서 g_test_goal 을 넣음   -> 그 속도로 시험 주행
 *   젯슨 바퀴 명령이 0.3초 안에 옴   -> 그 명령대로 주행
 *   S3 또는 S4 켜짐                  -> 고정 속도 테스트 (S4 = 1단, S3 = 2단)
 *   그 외 (ROS 명령 끊김)            -> 정지
 */


/* ============================================================================
 *  #include : 다른 파일에 적힌 "이름표" 를 가져오기
 * ============================================================================
 *
 * C 에서는 함수나 변수를 쓰기 전에 "이런 게 있다" 고 먼저 알려줘야 한다.
 * .h 파일(헤더 파일)에 그 목록이 적혀 있고, #include 로 그 목록을 붙여 넣는다.
 * "책 맨 앞 목차를 복사해 오는 것" 이라고 생각하면 된다.
 */
#include "app.h"     /* app_init(), app_loop() 목록 (main.c 가 부를 함수) */
#include "main.h"    /* CubeMX 가 만든 것: HAL_GetTick(), HAL_Delay(), USART3 등 */
#include "dxl.h"     /* 다이나믹셀 통신: dxl_read(), dxl_scan() 등 */
#include "mx64.h"    /* MX-64 모터 전용: 속도 모드 설정, 속도 보내기, 주소 번호들 */
#include "sw.h"      /* 스위치: sw_on(), SW1~SW4 */
#include "host.h"    /* 젯슨과 USB 통신: host_read(), host_write() */
#include "proto.h"   /* 젯슨과 주고받는 패킷 모양: PROTO_CMD_..., PROTO_STATE_... */
#include "psd.h"     /* PSD 거리 센서: psd_update(), g_psd_mm[] */


/* ============================================================================
 *  #define : 숫자에 이름 붙이기 (바꿔 쓰는 설정값)
 * ============================================================================
 *
 * #define 이름 값   이라고 쓰면, 컴파일할 때 "이름" 이 전부 "값" 으로 바뀐다.
 *   예) #define SPEED_1 50   이면 코드의 SPEED_1 이 전부 50 이 된다.
 * 숫자를 그냥 50 이라고 쓰면 무슨 뜻인지 모르니까 이름을 붙이는 것이다.
 * 값을 바꾸고 싶으면 여기 한 곳만 고치면 된다.
 *
 * 이름이 전부 대문자인 것은 "#define 으로 만든 고정값" 이라는 약속이다.
 */

/* ---- 스위치 테스트 속도 ----
 * MX-64 모터의 속도 단위: 1 = 0.229 rpm (rpm = 1분에 몇 바퀴)
 *   50  x 0.229 = 약 11.5 rpm
 *   100 x 0.229 = 약 22.9 rpm */
#define SPEED_1 50               /* 1단 (S4) */
#define SPEED_2 100              /* 2단 (S3) */

/* ---- 바퀴 방향 ----
 * 두 모터는 로봇 양쪽에 서로 마주 보게 달려 있다.
 * 그래서 둘 다 "+ 방향" 으로 돌리면 한쪽은 앞으로, 한쪽은 뒤로 간다.
 * 오른쪽에 -1 을 곱해서 둘 다 앞으로 가게 맞춘다.
 * 바퀴가 반대로 돌면 여기 부호(+1, -1)를 바꾸면 된다. */
#define LEFT_DIR  (+1)
#define RIGHT_DIR (-1)

/* ---- 어느 모터가 왼쪽인가 ----
 * 모터마다 번호(ID)가 있다. 우리 로봇은 ID 1 과 ID 2.
 *   1 = ID 가 작은 모터(1번)가 왼쪽
 *   0 = ID 가 큰 모터(2번)가 왼쪽  <- 지금 설정
 * 로봇을 다시 조립해서 좌우가 바뀌면 이 값을 바꾼다. */
#define LEFT_IS_SMALLER_ID 0

/* ---- 로봇 치수 (임시값! 자로 재서 바꾸기) ----
 * 숫자 뒤의 f 는 "float(소수) 숫자" 라는 표시. 0.05f = 0.05 */
#define WHEEL_RADIUS_M     0.05f   /* 바퀴 반지름 (m). _M = 미터 단위라는 뜻 */

/* ---- 안전 제한 ---- */
#define MAX_WHEEL_MPS   0.30f      /* 바퀴 최고 속도 0.3 m/s. MPS = Meter Per Second */
#define MAX_GOAL        250        /* 모터에 보낼 수 있는 최대값 (250 x 0.229 = 약 57 rpm) */
#define TEST_GOAL_MAX   285        /* 디버거 시험(g_test_goal) 최대값 (약 65 rpm) */
#define CMD_TIMEOUT_MS  300        /* 젯슨 명령이 0.3초(300ms) 안 오면 멈춘다. MS = 밀리초 */

/* ---- 일하는 주기 (단위: ms, 1000ms = 1초) ----
 * PERIOD = 주기 = "몇 ms 마다 한 번" */
#define STATUS_PERIOD_MS 100       /* 0.1초마다 젯슨에 상태 보고 */
#define UPDATE_PERIOD_MS 50        /* 0.05초마다 모터에 속도 보내기 */
#define RETRY_PERIOD_MS  500       /* 모터를 못 찾았으면 0.5초마다 다시 찾기 */
#define CHECK_PERIOD_MS  1000      /* 1초마다 모터 전압/에러/토크 확인 */
#define VEL_PERIOD_MS    50        /* 0.05초마다 엔코더(속도/위치) 읽고 젯슨에 보내기. VEL = velocity(속도) */
#define PSD_PERIOD_MS    10        /* 0.01초마다 PSD 거리 측정 */
#define PSD_SEND_MS      50        /* 0.05초마다 PSD 거리를 젯슨에 보내기 */

/* 바퀴 모터 개수. 2개를 다 찾아야 "준비 완료" 로 본다 */
#define NUM_MOTORS 2
/* 모터 찾을 때 "누구 있어?" 를 몇 번 물어볼지 (대답을 놓칠 수도 있어서 3번) */
#define SCAN_TRIES 3

/* ---- 모터를 찾을 때 시도할 통신 속도 목록 ----
 * 통신 속도(baud) = 1초에 몇 비트를 보내는지. 양쪽 속도가 같아야 대화가 된다.
 * 모터가 어떤 속도로 설정돼 있는지 모르니까 하나씩 다 시도한다.
 *
 * 문법 설명:
 *   static        : 이 파일(app.c) 안에서만 쓰는 것
 *   const         : 바꿀 수 없는 값 (읽기 전용)
 *   uint32_t      : 0 ~ 약 42억 까지 담는 숫자 칸 (아래 "자료형" 설명 참고)
 *   bauds[]       : [] 는 "배열" = 같은 종류 칸 여러 개를 한 줄로 붙인 것
 *   { a, b, c }   : 배열에 처음 넣을 값들
 *   bauds[0] = 1000000, bauds[1] = 57600 ... (번호는 0 부터 센다!) */
static const uint32_t bauds[] = { 1000000, 57600, 115200, 2000000, 9600 };

/* 배열 칸 개수 = 전체 크기 / 한 칸 크기.  sizeof = "몇 바이트인지" 재는 것.
 * 위 배열은 5칸이니까 NUM_BAUDS = 5 */
#define NUM_BAUDS (sizeof(bauds) / sizeof(bauds[0]))


/* ============================================================================
 *  [자료형] 숫자를 담는 칸의 종류
 * ============================================================================
 *
 *   이름        크기     담을 수 있는 값            u 가 붙으면 unsigned(음수 없음)
 *   uint8_t     1바이트  0 ~ 255
 *   int16_t     2바이트  -32768 ~ 32767
 *   uint16_t    2바이트  0 ~ 65535
 *   int32_t     4바이트  약 -21억 ~ 21억
 *   uint32_t    4바이트  0 ~ 약 42억
 *   int         4바이트  int32_t 와 같다고 보면 됨
 *   float       4바이트  소수 (예: 0.229)
 *
 *   _t 는 "type(자료형)" 이라는 뜻의 꼬리표일 뿐이다.
 *   숫자(8, 16, 32)는 비트 수다. 8비트 = 1바이트.
 */


/* ============================================================================
 *  디버거로 보는 변수 (전역 변수)
 * ============================================================================
 *
 * 변수 = 값을 담아두는 이름 붙은 칸.
 *   uint8_t  g_sw_on;   ->  "g_sw_on 이라는 1바이트 칸을 하나 만든다"
 *
 * 이름 규칙 (우리끼리 정한 약속):
 *   g_     : global(전역). 모든 함수에서 쓸 수 있고, 디버거에서도 볼 수 있다
 *   dxl    : 다이나믹셀(DYNAMIXEL) 관련
 *   [2]    : 칸 2개짜리 배열. [0] = 왼쪽, [1] = 오른쪽
 *
 * volatile : "이 값은 언제든 바뀔 수 있으니 매번 진짜 메모리에서 읽어라" 라는 표시.
 *            이게 없으면 컴파일러가 최적화하면서 디버거(Live Expressions)에
 *            옛날 값이 보일 수 있다. 그래서 디버거로 볼 변수에 붙인다.
 *
 * 전역 변수는 처음에 자동으로 0 이 들어 있다.
 */

/* ---- 모터 준비 상태 ---- */
volatile int      g_dxl_ready;      /* 1 = 모터 2개 다 찾고 설정 끝 (달릴 수 있음), 0 = 아직 */
volatile uint32_t g_dxl_baud;       /* 모터를 찾은 통신 속도 (예: 1000000). 0 = 못 찾음 */
volatile int      g_dxl_count;      /* 찾은 모터 개수 (정상이면 2) */
volatile uint8_t  g_dxl_ids[2];     /* 모터 번호. [0] = 왼쪽 모터 ID, [1] = 오른쪽 모터 ID */
volatile int      g_dxl_err;        /* 설정이 실패한 이유. 0 = 성공, -2 = 모터가 대답 안 함 */

/* ---- 모터 에러 ----
 * 하드웨어 에러 값 (여러 개가 겹치면 더해진다. 예: 33 = 32 + 1)
 *   1 = 전압 이상, 4 = 과열, 8 = 엔코더 이상, 16 = 전기 충격, 32 = 과부하 */
volatile uint8_t  g_dxl_hw_err[2];      /* 지금 모터에 걸려 있는 에러 */
volatile uint8_t  g_dxl_last_hw_err[2]; /* 0 이 아니었던 마지막 에러 (재부팅으로 지워져도 남겨 둠) */
volatile int      g_dxl_reboots;        /* 에러 때문에 모터를 재부팅한 횟수 */

/* ---- 모터 전압 (단위 0.1V. 예: 120 = 12.0V) ---- */
volatile uint16_t g_dxl_volt[2];    /* 모터가 잰 지금 전압 */
volatile uint16_t g_dxl_volt_min[2];/* 모터에 설정된 "이보다 낮으면 에러" 전압 */
volatile uint16_t g_dxl_volt_max[2];/* 모터에 설정된 "이보다 높으면 에러" 전압 */

volatile uint32_t g_dxl_vel_limit[2]; /* 모터에 설정돼 있던 최고 속도 (285 보다 작으면 285 로 올림) */
volatile uint8_t  g_dxl_torque[2];  /* 토크(힘) 켜짐 여부. 1 = 켜짐, 0 = 꺼짐(모터가 과부하로 스스로 끔) */
volatile uint32_t g_dxl_rx_bytes;   /* 모터한테서 받은 바이트 수. 0 이면 배선 문제 (rx = receive = 받기) */

/* ---- 스위치 / 젯슨 명령 ---- */
volatile uint8_t  g_sw_on;          /* 켜진 스위치. 비트로 표시 (아래 "비트" 설명 참고) */
volatile int32_t  g_speed;          /* 스위치 테스트 모드에서 쓰는 속도 */
volatile int16_t  g_cmd_left_mm;    /* 젯슨이 보낸 왼쪽 바퀴 속도 (mm/s). cmd = command(명령) */
volatile int16_t  g_cmd_right_mm;   /* 젯슨이 보낸 오른쪽 바퀴 속도 (mm/s) */

/* ---- 속도 (단위: 1 = 0.229 rpm, 앞으로 가면 +) ----
 * goal = 목표 (우리가 "이 속도로 돌아" 라고 보낸 값)
 * present = 현재 (모터가 엔코더로 직접 잰 진짜 값) */
volatile int32_t  g_goal[2];        /* 목표 속도. [0] 왼쪽, [1] 오른쪽 */
volatile int32_t  g_present_vel[2]; /* 실제 속도 (vel = velocity) */
volatile int32_t  g_present_pos[2]; /* 바퀴 위치 (pos = position). 4096 = 1바퀴. 앞으로 갈수록 커짐 */
volatile float    g_goal_rpm[2];    /* g_goal 을 rpm 으로 바꾼 것 (보기 편하라고) */
volatile float    g_present_rpm[2]; /* g_present_vel 을 rpm 으로 바꾼 것 */

/* ---- 디버거 속도 시험 ----
 * 디버거 Live Expressions 에서 이 값을 0 이 아닌 수로 바꾸면,
 * 젯슨 명령 대신 두 바퀴를 이 속도로 돌린다. (S1 켜짐 + S2 꺼짐일 때만)
 * 다 보면 0 으로 되돌린다. */
volatile int32_t  g_test_goal;

/* ---- 지금 로봇 상태 ----
 * 0 = S1 꺼져서 정지, 1 = 젯슨 명령대로 주행, 2 = 비상 정지,
 * 3 = 젯슨 명령 끊김, 4 = 모터 준비 안 됨, 5 = 스위치/디버거 테스트 (proto.h 참고) */
volatile uint8_t  g_state;


/* ============================================================================
 *  이 파일 안에서만 쓰는 변수
 * ============================================================================
 *
 * static : 이 파일(app.c) 안에서만 보이는 변수. 다른 파일이 실수로 못 건드린다.
 * tick   : "시계 눈금" = 시각. HAL_GetTick() 은 전원 켠 뒤 지난 ms 를 알려준다.
 *          예) 전원 켜고 3초 지났으면 3000
 * last_..._tick : "마지막으로 ...한 시각". 주기 계산에 쓴다 (task_ 함수들 참고)
 */
static uint8_t  ids[2];             /* 찾은 모터 ID. [0] 왼쪽, [1] 오른쪽 (실제로 쓰는 값) */
static int      count;              /* 찾은 모터 수 */
static uint32_t last_tick;          /* 마지막으로 모터에 속도를 보낸 시각 (또는 다시 찾은 시각) */
static uint32_t last_status_tick;   /* 마지막으로 젯슨에 상태를 보고한 시각 */
static uint32_t last_check_tick;    /* 마지막으로 모터 상태를 확인한 시각 */
static uint32_t last_vel_tick;      /* 마지막으로 엔코더를 읽은 시각 */
static uint32_t last_psd_tick;      /* 마지막으로 PSD 를 잰 시각 */
static uint32_t last_psd_send_tick; /* 마지막으로 PSD 를 젯슨에 보낸 시각 */
static uint32_t last_cmd_tick;      /* 마지막으로 젯슨 명령을 받은 시각 */
static int      has_cmd;            /* 젯슨 명령을 한 번이라도 받았으면 1 */


/* ============================================================================
 *  [함수] 읽는 법
 * ============================================================================
 *
 *   static  int32_t   clamp   (int32_t value, int32_t max)
 *   ──────  ───────   ─────   ────────────────────────────
 *   이 파일   돌려주는   함수     받는 값들 (매개변수)
 *   전용      값 종류    이름
 *
 *   {
 *     ... 할 일 ...
 *     return 결과;     <- 결과를 돌려주고 함수 끝
 *   }
 *
 *   void : "돌려주는 값 없음" 또는 "받는 값 없음"
 *   부를 때:  int32_t x = clamp(300, 285);   -> x 에 285 가 들어간다
 */


/* ============================================================================
 *  바이트 <-> 숫자 바꾸기 도우미
 * ============================================================================
 *
 * 통신선으로는 한 번에 1바이트(0~255)씩만 보낼 수 있다.
 * 그래서 300 같은 큰 숫자는 여러 바이트로 쪼개서 보낸다.
 *
 *   300 을 16진수로 쓰면 0x012C  (0x 는 "16진수" 표시)
 *     큰 자리 바이트 = 0x01, 작은 자리 바이트 = 0x2C
 *   다이나믹셀과 젯슨은 "작은 자리 먼저" 보낸다 (little-endian 이라고 부름)
 *     -> [0x2C] [0x01]
 *
 * [비트 연산 기호]
 *   <<  : 왼쪽으로 밀기.  0x01 << 8  = 0x0100  (8비트 = 1바이트 = 16진수 두 자리)
 *   >>  : 오른쪽으로 밀기. 0x012C >> 8 = 0x01
 *   |   : OR (합치기).    0x0100 | 0x2C = 0x012C
 *   &   : AND (골라내기). 0x012C & 0xFF = 0x2C  (아래 1바이트만 남김)
 *
 * [포인터 * 와 주소 &]
 *   const uint8_t *b : "바이트들이 있는 곳의 위치(주소)" 를 받는다.
 *                      b[0], b[1] 로 그 위치부터 차례로 읽을 수 있다.
 *   &data[5]         : "data 배열 5번 칸의 위치". 함수에 "여기부터 써" 하고 알려줄 때 쓴다.
 *   const            : 받은 곳을 읽기만 하고 바꾸지 않겠다는 약속
 *
 * [(자료형) 값] : 형 변환(cast). 값을 다른 종류의 칸에 맞게 바꾼다.
 *   (uint8_t)300 -> 44  (1바이트에 안 들어가는 윗부분은 잘림)
 */

/* 2바이트 -> 16비트 숫자.  예) b = [0x2C, 0x01]  ->  0x012C = 300 */
static uint16_t to_uint16(const uint8_t *b)
{
  /* b[1] 을 8비트 왼쪽으로 밀어서(큰 자리로 보내서) b[0] 과 합친다 */
  return (uint16_t)(b[0] | (b[1] << 8));
}

/* 4바이트 -> 32비트 숫자 (음수도 됨) */
static int32_t to_int32(const uint8_t *b)
{
  uint32_t value = b[0];                    /* 가장 작은 자리 */
  value = value | ((uint32_t)b[1] << 8);    /* 두 번째 자리 붙이기 */
  value = value | ((uint32_t)b[2] << 16);   /* 세 번째 자리 붙이기 */
  value = value | ((uint32_t)b[3] << 24);   /* 가장 큰 자리 붙이기 */
  return (int32_t)value;                    /* 부호 있는 숫자로 (맨 위 비트가 1 이면 음수) */
}

/* 16비트 숫자 -> 2바이트.  out[0] 에 작은 자리, out[1] 에 큰 자리 */
static void put_16(uint8_t *out, uint16_t value)
{
  out[0] = (uint8_t)(value & 0xFF);         /* 아래 8비트만 */
  out[1] = (uint8_t)(value >> 8);           /* 위 8비트를 아래로 내려서 */
}

/* 32비트 숫자 -> 4바이트.  out[0] 이 가장 작은 자리 */
static void put_32(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)(value & 0xFF);
  out[1] = (uint8_t)((value >> 8) & 0xFF);
  out[2] = (uint8_t)((value >> 16) & 0xFF);
  out[3] = (uint8_t)((value >> 24) & 0xFF);
}


/* ============================================================================
 *  모터 찾기
 * ============================================================================
 *
 * [for 반복문 읽는 법]
 *   for (int m = 0; m < count; m++)
 *        ─────────  ─────────  ───
 *        시작:       조건:       한 번 끝날 때마다:
 *        m 을 0 으로  m 이 count  m 을 1 늘림 (m++ 는 m = m + 1)
 *                    보다 작은 동안
 *   -> m = 0, 1, 2, ... count-1 까지 { } 안을 반복한다.
 *
 * [비교 기호]
 *   ==  같다  (= 하나는 "넣기", == 두 개는 "같은지 비교". 헷갈리기 쉬움!)
 *   !=  다르다
 *   <   작다,  <=  작거나 같다,  >  크다,  >=  크거나 같다
 *
 * [참 / 거짓]
 *   C 에서는 0 = 거짓, 0 이 아닌 수 = 참.
 *   !  는 "반대".  !1 = 0,  !0 = 1
 *   && 는 "그리고",  || 는 "또는"
 */

/* ids[] 에 이 ID 가 이미 있으면 1(참), 없으면 0(거짓) */
static int already_have(uint8_t id)
{
  for (int m = 0; m < count; m++)          /* 지금까지 찾은 모터를 하나씩 보면서 */
  {
    if (ids[m] == id)                      /* 같은 번호가 있으면 */
    {
      return 1;                            /* "이미 있음" 하고 바로 끝 */
    }
  }
  return 0;                                /* 끝까지 봤는데 없음 */
}

/* 이번에 찾은 ID 들(found 배열, n 개) 중 처음 보는 것만 ids[] 에 추가한다 */
static void add_new_ids(const uint8_t *found, int n)
{
  for (int k = 0; k < n; k++)              /* 찾은 것들을 하나씩 */
  {
    if (already_have(found[k]))
    {
      continue;                            /* continue = 이번 차례는 건너뛰고 다음 k 로 */
    }
    if (count < NUM_MOTORS)                /* 아직 2개가 안 찼으면 */
    {
      ids[count] = found[k];               /* 빈 칸에 넣고 */
      count++;                             /* 개수 1 늘리기 */
    }
  }
}

/* 왼쪽 모터가 ids[0] 에 오도록 순서를 맞춘다 */
static void put_left_first(void)
{
  if (count != 2)
  {
    return;                                /* 2개가 아니면 바꿀 게 없음 */
  }
  /* (ids[0] > ids[1]) 은 참이면 1, 거짓이면 0 이 된다 */
  int first_is_bigger = (ids[0] > ids[1]);
  if (first_is_bigger == LEFT_IS_SMALLER_ID)
  {
    /* 두 칸의 값 맞바꾸기: 임시 칸(temp)에 하나를 잠깐 맡겨 둔다
       (컵 두 개의 물을 바꾸려면 빈 컵이 하나 더 필요한 것과 같다) */
    uint8_t temp = ids[0];
    ids[0] = ids[1];
    ids[1] = temp;
  }
}

/*
 * 모터 찾기
 *   통신 속도를 bauds[] 순서대로 바꿔가며 "거기 누구 있어?" (broadcast ping) 를 보낸다.
 *   broadcast = 모든 모터에게 한 번에 묻는 것. 있는 모터는 자기 ID 로 대답한다.
 *   한 속도에서 3번 물어서 대답한 ID 를 다 모은다.
 *   한 대라도 대답하면 그 속도로 정하고 1 을 돌려준다. 아무도 없으면 0.
 */
static int find_motors(void)
{
  /* i = 0, 1, 2, 3, 4 : 통신 속도 5개를 차례로 */
  for (unsigned i = 0; i < NUM_BAUDS; i++)
  {
    dxl_set_baud(bauds[i]);                /* STM 의 통신 속도를 바꾼다 */
    count = 0;                             /* 이 속도에서 새로 센다 */

    /* 최대 3번 물어보기. 단, 2개를 다 찾으면 (count < NUM_MOTORS 가 거짓) 일찍 끝 */
    for (int t = 0; t < SCAN_TRIES && count < NUM_MOTORS; t++)
    {
      uint8_t found[NUM_MOTORS];           /* 이번에 대답한 ID 를 받을 칸 2개 */
      int n = dxl_scan(found, NUM_MOTORS); /* 물어보기. n = 대답한 모터 수 */
      if (t == 0 && n == 0)
      {
        break;                             /* break = 반복문 탈출. 첫 번째에 아무도 없으면 이 속도는 포기 */
      }

      add_new_ids(found, n);
    }

    if (count > 0)                         /* 이 속도에서 한 대라도 찾았으면 */
    {
      put_left_first();
      g_dxl_baud = bauds[i];               /* 디버거에서 보려고 기록 */
      return 1;
    }
  }
  return 0;                                /* 5개 속도 다 해봤는데 아무도 없음 */
}

/*
 * 모터 토크(힘)가 실제로 켜져 있으면 1.
 * 모터는 과부하/과열이 되면 스스로 토크를 끈다. 그걸 확인하는 함수.
 */
static int torque_is_on(uint8_t id)
{
  uint8_t on = 0;
  /* dxl_read(모터ID, 주소, 몇 바이트, 받을 곳)
     모터 안에는 "주소" 가 붙은 칸들이 있다 (Control Table). 64번 칸 = 토크 켜짐 여부.
     &on : on 변수의 위치를 알려줘서 dxl_read 가 거기에 값을 써 넣게 한다.
     실패하면 음수를 돌려준다. */
  if (dxl_read(id, MX64_ADDR_TORQUE_ENABLE, 1, &on) < 0)
  {
    return 0;                              /* 읽기 실패 -> 꺼진 걸로 본다 */
  }
  return on == 1;                          /* 1 이면 참(1), 아니면 거짓(0) 을 돌려준다 */
}


/* ============================================================================
 *  모터 설정
 * ============================================================================ */

/* 하드웨어 에러 값 기록. index = 0(왼쪽) 또는 1(오른쪽) */
static void record_hw_error(int index, uint8_t value)
{
  g_dxl_hw_err[index] = value;             /* 지금 에러 (0 일 수도 있음) */
  if (value != 0)
  {
    g_dxl_last_hw_err[index] = value;      /* 에러가 있었으면 따로 남겨 둔다 */
  }
}

/* 진단용: 모터의 전압, 전압 허용 범위, 속도 제한을 읽어서 디버거 변수에 넣는다 */
static void read_diagnostics(int index)
{
  uint8_t id = ids[index];                 /* 이 모터의 번호 */
  uint8_t buf[4];                          /* 읽은 바이트를 잠깐 담을 칸 (buf = buffer = 임시 그릇) */

  /* ">= 0" = 읽기 성공했으면. 실패하면 값을 안 바꾸고 넘어간다 */
  if (dxl_read(id, MX64_ADDR_PRESENT_VOLTAGE, 2, buf) >= 0)
  {
    g_dxl_volt[index] = to_uint16(buf);    /* 2바이트를 숫자로 */
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
 * 모터 하나 설정. index = 0(왼쪽) 또는 1(오른쪽)
 * 성공하면 0, 실패하면 0 이 아닌 에러 값을 돌려준다.
 */
static int setup_one_motor(int index)
{
  uint8_t id = ids[index];
  uint8_t hw_error = 0;
  int ret;                                 /* ret = return value = 함수가 돌려준 결과 */

  /* 0) 진단용 값 읽기 */
  read_diagnostics(index);

  /* 1) 바퀴용으로 설정: 토크 끄기 -> 속도 모드 -> 속도 0 -> 토크 켜기 (mx64.c) */
  ret = mx64_set_wheel_mode(id);
  if (mx64_failed(ret))
  {
    return ret;                            /* 통신 자체가 실패 -> 에러 값 그대로 돌려주기 */
  }

  /* 2) 모터가 "나 에러 있어" 표시(Alert)를 했으면 무슨 에러인지 읽어서 기록한다.
        ret & DXL_ALERT : ret 안에 Alert 비트(0x80)가 켜져 있으면 참.
        전압 에러처럼 토크는 켜 두는 에러도 있어서, 진짜 문제인지는
        "토크가 실제로 켜졌는지" 로 판단한다. */
  if (ret & DXL_ALERT)
  {
    if (dxl_read(id, MX64_ADDR_HARDWARE_ERROR, 1, &hw_error) >= 0)
    {
      record_hw_error(index, hw_error);
    }
    if (torque_is_on(id))
    {
      return 0;                            /* 경고는 있지만 움직일 수 있음 -> 성공 */
    }

    /* 3) 토크가 안 켜졌을 때만: 모터를 재부팅해서 에러를 지우고 한 번 더 시도 */
    dxl_reboot(id);
    g_dxl_reboots++;                       /* 재부팅 횟수 1 증가 */
    HAL_Delay(1000);                       /* 1초(1000ms) 기다림. 모터가 다시 켜지는 시간 */

    ret = mx64_set_wheel_mode(id);
    if (mx64_failed(ret))
    {
      return ret;
    }
    if (!torque_is_on(id))                 /* ! = 반대. "토크가 안 켜졌으면" */
    {
      return DXL_ALERT;                    /* 재부팅해도 안 됨 -> 실패 */
    }
  }
  return 0;                                /* 성공 */
}

/*
 * 모터 찾기 + 2개 다 설정
 * 다 되면 1, 하나라도 실패하면 0.
 */
static int motors_setup(void)
{
  g_dxl_count = 0;
  g_dxl_baud = 0;

  if (!find_motors())                      /* 한 대도 못 찾았으면 */
  {
    g_dxl_rx_bytes = dxl_rx_count;         /* 받은 바이트 수 기록 (0 이면 배선 확인) */
    return 0;
  }
  g_dxl_count = count;
  g_dxl_ids[0] = ids[0];
  /* 조건 ? A : B  =  조건이 참이면 A, 거짓이면 B.
     모터가 2개 이상이면 ids[1], 아니면 0 */
  g_dxl_ids[1] = (count > 1) ? ids[1] : 0;

  /* 2개를 다 못 찾았으면 준비 안 됨 -> 0.5초 뒤 다시 찾는다.
     (한 대만 찾은 채로 달리면 한쪽 바퀴만 돌아서 위험) */
  if (count < NUM_MOTORS)
  {
    g_dxl_err = DXL_ERR_TIMEOUT;           /* -2: 대답 없음 */
    return 0;
  }

  for (int i = 0; i < count; i++)          /* i = 0(왼쪽), 1(오른쪽) */
  {
    g_dxl_ids[i] = ids[i];

    int ret = setup_one_motor(i);
    if (ret != 0)                          /* 실패했으면 */
    {
      g_dxl_err = ret;                     /* 이유를 기록하고 */
      return 0;                            /* 포기 (0.5초 뒤 처음부터 다시) */
    }
    g_dxl_torque[i] = (uint8_t)torque_is_on(ids[i]);
  }

  g_dxl_err = 0;
  return 1;                                /* 모두 성공 */
}

/*
 * 달리는 중에 모터 상태 다시 읽기 (1초마다 불림)
 * 전압, 에러, 토크를 읽어서 디버거 변수와 젯슨 상태 보고에 반영한다.
 * 모터가 스스로 토크를 껐으면 0 을 돌려준다 -> app_loop 가 다시 설정(재부팅)한다.
 */
static int check_motors(void)
{
  int ok = 1;                              /* 일단 괜찮다고 보고 시작 */

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
        ok = 0;                            /* 토크 꺼짐 -> 속도를 보내도 안 돈다 */
      }
    }
  }
  return ok;
}

/*
 * 모터의 실제 속도와 위치(엔코더) 읽기
 *   엔코더 = 모터 축이 얼마나 돌았는지 재는 센서. 모터 안에 들어 있다.
 *   모터 칸 128번(속도 4바이트) 과 132번(위치 4바이트) 은 붙어 있어서
 *   128번부터 8바이트를 한 번에 읽는다.
 *   LEFT_DIR / RIGHT_DIR 을 곱해서 "앞으로 가면 +" 가 되게 맞춘다.
 *   읽기에 실패하면 이전 값을 그대로 둔다.
 */
static void read_present_velocity(void)
{
  const int dir[2] = { LEFT_DIR, RIGHT_DIR };   /* dir[0] = +1(왼쪽), dir[1] = -1(오른쪽) */

  for (int i = 0; i < count; i++)
  {
    uint8_t buf[8];                        /* 8바이트 받을 그릇 */
    if (dxl_read(ids[i], MX64_ADDR_PRESENT_VELOCITY, 8, buf) >= 0)
    {
      /* &buf[0] = buf 의 0번 칸부터, &buf[4] = 4번 칸부터 4바이트를 숫자로 */
      g_present_vel[i] = dir[i] * to_int32(&buf[0]);   /* buf[0~3] = 128~131번 칸: 속도 */
      g_present_pos[i] = dir[i] * to_int32(&buf[4]);   /* buf[4~7] = 132~135번 칸: 위치 */
      g_present_rpm[i] = g_present_vel[i] * 0.229f;    /* 보기 편하게 rpm 으로 */
    }
  }
}

/*
 * 스위치 상태로 테스트 속도를 정한다. 0 이면 정지.
 *
 * [비트로 스위치 4개를 한 숫자에 담기]
 *   on 은 1바이트(8비트) 숫자인데, 아래 4비트를 스위치 4개로 쓴다.
 *     비트 번호:   3    2    1    0
 *     스위치:     S4   S3   S2   S1
 *   예) S1 과 S3 만 켜짐 -> 0101 (2진수) = 5
 *
 *   SW1 = 0001, SW2 = 0010, SW3 = 0100, SW4 = 1000 (sw.h)
 *   on & SW3  -> S3 자리만 남긴다. S3 가 켜져 있으면 0 이 아님(참), 꺼져 있으면 0(거짓)
 */
static int32_t speed_from_switches(uint8_t on)
{
  if (!(on & SW1))                         /* S1 이 꺼져 있으면 */
  {
    return 0;                              /* 정지 */
  }
  if (on & SW2)                            /* S2(비상정지)가 켜져 있으면 */
  {
    return 0;                              /* 정지 */
  }
  if (on & SW4)                            /* S4 가 켜져 있으면 (S3 보다 먼저 봄) */
  {
    return SPEED_1;                        /* 1단 */
  }
  if (on & SW3)                            /* S3 가 켜져 있으면 */
  {
    return SPEED_2;                        /* 2단 */
  }
  return SPEED_1;                          /* 아무것도 없으면 1단 */
}


/* ============================================================================
 *  젯슨 통신
 * ============================================================================
 *
 * [패킷] 정해진 모양으로 묶은 데이터 한 덩어리. 우리 패킷 모양:
 *
 *   AA  55  | CMD | LEN | DATA ... | CHK
 *   ──────    ───   ───   ───────    ───
 *   시작 표시  종류  DATA   내용      검사값 (CMD+LEN+DATA 를 다 더한 값의 아래 1바이트)
 *              번호  길이
 *
 *   CHK 로 "중간에 잡음 때문에 바이트가 깨졌는지" 확인한다.
 */

/*
 * 젯슨에서 온 바이트들을 패킷으로 조립하고, 바퀴 속도 명령이면 저장한다.
 *
 * [while 반복문]  while (조건) { ... }  = 조건이 참인 동안 계속 반복
 * [구조체] proto_packet_t 는 여러 값을 한 묶음으로 만든 것 (proto.h).
 *          pkt.cmd, pkt.len, pkt.data[] 처럼 점(.)으로 안의 값을 꺼낸다.
 */
static void handle_host(void)
{
  uint8_t b;                               /* 받은 바이트 1개 */
  proto_packet_t pkt;                      /* 조립이 끝난 패킷 */

  /* host_read 가 받은 바이트를 b 에 넣어 주고 1 을 돌려준다. 더 없으면 0 -> 반복 끝 */
  while (host_read(&b))
  {
    /* proto_feed : 바이트를 하나씩 넣다가 패킷이 완성되면 1 */
    if (!proto_feed(b, &pkt))
    {
      continue;                            /* 아직 다 안 모임 -> 다음 바이트 */
    }
    /* 바퀴 명령(CMD 0x01)이고 길이가 4바이트면 */
    if (pkt.cmd == PROTO_CMD_WHEEL && pkt.len == 4)
    {
      /* data[0~1] = 왼쪽 mm/s, data[2~3] = 오른쪽 mm/s.
         (int16_t) 로 바꿔야 뒤로 가는 음수 속도가 제대로 나온다 */
      g_cmd_left_mm  = (int16_t)to_uint16(&pkt.data[0]);
      g_cmd_right_mm = (int16_t)to_uint16(&pkt.data[2]);
      last_cmd_tick = HAL_GetTick();       /* 명령 받은 시각 기록 (0.3초 시간 초과 검사용) */
      has_cmd = 1;
    }
  }
}

/* 젯슨에 상태 패킷(CMD 0x81, 19바이트)을 보낸다 */
static void send_status(void)
{
  uint8_t data[PROTO_STATUS_LEN];          /* 내용 19바이트 */
  uint8_t buf[PROTO_STATUS_LEN + 5];       /* 완성된 패킷: 내용 + 앞 4바이트(AA 55 CMD LEN) + 뒤 1바이트(CHK) */

  data[0] = (uint8_t)g_dxl_ready;          /* 0번: 모터 준비 */
  data[1] = g_state;                       /* 1번: 상태 */
  data[2] = g_sw_on;                       /* 2번: 스위치 */
  data[3] = g_dxl_hw_err[0];               /* 3번: 왼쪽 에러 */
  data[4] = g_dxl_hw_err[1];               /* 4번: 오른쪽 에러 */
  put_16(&data[5], g_dxl_volt[0]);         /* 5, 6번: 전압 */
  data[7] = (uint8_t)g_dxl_count;          /* 7번: 찾은 모터 수 */
  data[8] = g_dxl_ids[0];                  /* 8번: 왼쪽 모터 ID */
  data[9] = g_dxl_ids[1];                  /* 9번: 오른쪽 모터 ID */
  /* 10번: 토크 두 개를 한 바이트에. 왼쪽은 bit0, 오른쪽은 1칸 밀어서 bit1 */
  data[10] = (uint8_t)(g_dxl_torque[0] | (g_dxl_torque[1] << 1));
  /* 11~18번: 목표 속도와 실제 속도 (2바이트씩) */
  put_16(&data[11], (uint16_t)(int16_t)g_goal[0]);          /* 11, 12번: 왼쪽 목표 */
  put_16(&data[13], (uint16_t)(int16_t)g_goal[1]);          /* 13, 14번: 오른쪽 목표 */
  put_16(&data[15], (uint16_t)(int16_t)g_present_vel[0]);   /* 15, 16번: 왼쪽 실제 */
  put_16(&data[17], (uint16_t)(int16_t)g_present_vel[1]);   /* 17, 18번: 오른쪽 실제 */

  /* proto_build : AA 55 CMD LEN 을 앞에, CHK 를 뒤에 붙여서 buf 에 완성. n = 전체 길이 */
  uint16_t n = proto_build(PROTO_CMD_STATUS, data, PROTO_STATUS_LEN, buf);
  host_write(buf, n);                      /* USB 로 젯슨에 보내기 */
}

/* 젯슨에 PSD 거리 패킷(CMD 0x10, 6바이트)을 보낸다. 단위 mm */
static void send_psd(void)
{
  uint8_t data[6];
  uint8_t buf[6 + 5];

  /* i = 0(왼쪽), 1(앞), 2(오른쪽). 각각 2바이트씩 data[0~1], data[2~3], data[4~5] 에 */
  for (int i = 0; i < PSD_COUNT; i++)
  {
    put_16(&data[i * 2], g_psd_mm[i]);
  }
  uint16_t n = proto_build(PROTO_CMD_PSD, data, 6, buf);
  host_write(buf, n);
}

/* 젯슨에 엔코더 패킷(CMD 0x11, 12바이트)을 보낸다 */
static void send_encoder(void)
{
  uint8_t data[PROTO_ENCODER_LEN];
  uint8_t buf[PROTO_ENCODER_LEN + 5];

  put_32(&data[0], (uint32_t)g_present_pos[0]);            /* 0~3번: 왼쪽 위치 (4바이트) */
  put_32(&data[4], (uint32_t)g_present_pos[1]);            /* 4~7번: 오른쪽 위치 */
  put_16(&data[8], (uint16_t)(int16_t)g_present_vel[0]);   /* 8, 9번: 왼쪽 속도 (2바이트) */
  put_16(&data[10], (uint16_t)(int16_t)g_present_vel[1]);  /* 10, 11번: 오른쪽 속도 */
  uint16_t n = proto_build(PROTO_CMD_ENCODER, data, PROTO_ENCODER_LEN, buf);
  host_write(buf, n);
}


/* ============================================================================
 *  바퀴 속도(m/s) -> 모터 값 계산
 * ============================================================================
 *
 *   젯슨은 "바퀴가 땅 위를 몇 mm/s 로 굴러가라" 고 보낸다 (선속도).
 *   모터는 "1분에 몇 바퀴 돌아라" (rpm) 를 0.229 단위로 받는다.
 *
 *   1) mm/s -> m/s           : 1000 으로 나누기
 *   2) m/s  -> rad/s (각속도) : 반지름으로 나누기   (바퀴 둘레 = 2 x pi x 반지름)
 *   3) rad/s -> rpm           : x 60 / (2 x pi)     (1바퀴 = 2 x pi rad, 1분 = 60초)
 *   4) rpm  -> 모터 값        : / 0.229
 */

/* 바퀴 각속도 (rad/s) -> 모터 값 (소수) */
static float wheel_to_goal(float wheel_rad_s)
{
  const float TWO_PI = 6.2831853f;         /* 2 x 3.14159... */
  float rpm = wheel_rad_s * 60.0f / TWO_PI;
  return rpm / 0.229f;
}

/* 소수 -> 정수로 반올림.  2.6 -> 3,  -2.6 -> -3
   (int32_t) 로 바꾸면 소수점 아래가 그냥 버려지므로 0.5 를 더하거나 빼고 바꾼다 */
static int32_t round_to_int(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value + 0.5f);
  }
  return (int32_t)(value - 0.5f);
}

/* 절댓값 (음수면 + 로).  -3.5 -> 3.5 */
static float absf(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }
  return value;
}

/*
 * 왼쪽/오른쪽 바퀴 선속도(mm/s) -> 왼쪽/오른쪽 모터 값
 *
 * 함수는 return 으로 값을 하나만 돌려줄 수 있다. 그런데 여기선 2개(왼쪽, 오른쪽)가 필요하다.
 * 그래서 "결과를 써 넣을 곳의 위치" 를 포인터(int32_t *left)로 받는다.
 *   *left = 3;   ->  "left 가 가리키는 곳에 3 을 넣어라"
 * 부르는 쪽은 wheels_to_goals(..., &l, &r) 처럼 자기 변수 위치를 넘겨준다.
 */
static void wheels_to_goals(int16_t left_mm, int16_t right_mm,
                            int32_t *left, int32_t *right)
{
  float left_mps  = left_mm / 1000.0f;     /* mm/s -> m/s */
  float right_mps = right_mm / 1000.0f;

  float left_rad_s  = left_mps / WHEEL_RADIUS_M;   /* m/s -> rad/s */
  float right_rad_s = right_mps / WHEEL_RADIUS_M;

  float l = wheel_to_goal(left_rad_s);     /* rad/s -> 모터 값 */
  float r = wheel_to_goal(right_rad_s);

  /* 한쪽이라도 최대를 넘으면 두 바퀴를 같은 비율로 줄인다.
     예) 왼쪽 300, 오른쪽 150, 최대 250 이면 -> 둘 다 x (250/300) -> 250, 125
     (한쪽만 자르면 왼쪽:오른쪽 비율이 바뀌어서 도는 모양이 달라진다) */
  float biggest = absf(l);                 /* 둘 중 큰 것 찾기 */
  if (absf(r) > biggest)
  {
    biggest = absf(r);
  }
  float limit = (float)MAX_GOAL;           /* 최대값 = 250 과 */
  float limit_by_speed = wheel_to_goal(MAX_WHEEL_MPS / WHEEL_RADIUS_M);   /* 0.3 m/s 에 해당하는 값 중 */
  if (limit_by_speed < limit)
  {
    limit = limit_by_speed;                /* 더 작은 쪽을 최대로 */
  }
  if (biggest > limit)
  {
    float scale = limit / biggest;         /* 줄일 비율 (1 보다 작음) */
    l = l * scale;
    r = r * scale;
  }

  *left  = round_to_int(l);                /* 결과를 부른 쪽 변수에 써 넣기 */
  *right = round_to_int(r);
}

/* 값을 -max ~ +max 사이로 자른다.  clamp(300, 285) -> 285,  clamp(-300, 285) -> -285 */
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

/* 젯슨 명령을 받은 지 0.3초가 안 지났으면 1 */
static int ros_cmd_is_fresh(uint32_t now)
{
  if (!has_cmd)
  {
    return 0;                              /* 한 번도 못 받음 */
  }
  /* now - last_cmd_tick = 마지막 명령 뒤로 지난 시간(ms) */
  return (now - last_cmd_tick) < CMD_TIMEOUT_MS;
}

/*
 * 지금 어떻게 움직일지 정한다. (이 파일에서 제일 중요한 판단)
 *   on    : 켜진 스위치
 *   now   : 지금 시각
 *   left, right : 결과(바퀴 속도)를 써 넣을 곳. "앞으로 = +" 기준
 *   돌려주는 값 : 상태 번호 (PROTO_STATE_...)
 *
 * 위에서부터 차례로 검사해서 처음 맞는 것에서 return 으로 끝난다.
 * 그래서 위에 있는 것이 우선순위가 높다.
 */
static uint8_t decide(uint8_t on, uint32_t now, int32_t *left, int32_t *right)
{
  *left = 0;                               /* 기본은 정지 */
  *right = 0;

  if (on & SW2)                            /* 1순위: S2 켜짐 -> 비상 정지 */
  {
    return PROTO_STATE_ESTOP;
  }
  if (!(on & SW1))                         /* 2순위: S1 꺼짐 -> 정지 */
  {
    return PROTO_STATE_STOP_SW;
  }
  if (g_test_goal != 0)                    /* 3순위: 디버거 시험 값이 있으면 */
  {
    int32_t goal = clamp(g_test_goal, TEST_GOAL_MAX);   /* 너무 크면 자르고 */
    *left = goal;
    *right = goal;
    return PROTO_STATE_MANUAL;
  }
  if (ros_cmd_is_fresh(now))               /* 4순위: 젯슨 명령이 살아 있으면 */
  {
    wheels_to_goals(g_cmd_left_mm, g_cmd_right_mm, left, right);
    return PROTO_STATE_ROS;
  }
  if (on & (SW3 | SW4))                    /* 5순위: S3 또는 S4 가 켜져 있으면 (SW3 | SW4 = 둘 중 하나라도) */
  {
    g_speed = speed_from_switches(on);
    *left = g_speed;
    *right = g_speed;
    return PROTO_STATE_MANUAL;
  }
  return PROTO_STATE_NO_CMD;               /* 그 외: 명령 없음 -> 정지 (속도는 위에서 0 으로 해 둠) */
}


/* ============================================================================
 *  main.c 가 전원 켤 때 딱 한 번 부르는 함수
 * ============================================================================ */

void app_init(void)
{
  sw_init();                               /* 스위치 핀 준비 (sw.c) */
  host_init();                             /* 젯슨 통신 준비 (USART6, host.c) */
  psd_init();                              /* PSD 거리 센서 준비 (ADC + DMA, psd.c) */
  dxl_init(USART3);                        /* 다이나믹셀 통신 준비 (USART3, dxl.c) */
  HAL_Delay(300);                          /* 0.3초 기다림: 모터가 전원 켜고 깨어나는 시간 */

  g_dxl_ready = motors_setup();            /* 모터 찾고 설정. 성공 1, 실패 0 */
  last_tick = HAL_GetTick();               /* 지금 시각 기록 */
}


/* ============================================================================
 *  app_loop 가 하는 일들 (task = 할 일)
 * ============================================================================
 *
 * 모든 task_ 함수는 같은 모양이다:
 *
 *   if (now - last_xxx_tick < 주기)   <- 지난번 일한 뒤로 아직 주기만큼 안 지났으면
 *   {
 *     return;                         <- 아무것도 안 하고 바로 끝
 *   }
 *   last_xxx_tick = now;              <- "지금 일했음" 하고 시각을 적어 두고
 *   ... 일하기 ...
 *
 *   예) 주기 100ms, 지난번 1000ms 에 일함
 *       now = 1050 -> 1050 - 1000 = 50  < 100 -> 그냥 끝
 *       now = 1100 -> 1100 - 1000 = 100 -> 일함, last = 1100
 *
 * 이렇게 하면 HAL_Delay() 로 멈춰 기다리지 않고도 여러 일을 각자 다른 주기로 할 수 있다.
 * (알람시계 여러 개를 맞춰 두고, 울린 것만 처리하는 것과 같다)
 */

/* PSD: 0.01초마다 재고, 0.05초마다 젯슨에 보낸다 */
static void task_psd(uint32_t now)
{
  if (now - last_psd_tick >= PSD_PERIOD_MS)        /* 0.01초 지났으면 */
  {
    last_psd_tick = now;
    psd_update();                                  /* 거리 계산 (psd.c) */
  }
  if (now - last_psd_send_tick >= PSD_SEND_MS)     /* 0.05초 지났으면 */
  {
    last_psd_send_tick = now;
    send_psd();                                    /* 젯슨에 보내기 */
  }
}

/* 0.1초마다 젯슨에 상태 보고 */
static void task_status(uint32_t now)
{
  if (now - last_status_tick < STATUS_PERIOD_MS)
  {
    return;                                        /* 아직 시간 안 됨 */
  }
  last_status_tick = now;

  g_sw_on = sw_on();                               /* 스위치 읽기 */
  if (!g_dxl_ready)
  {
    g_state = PROTO_STATE_NOT_READY;               /* 모터 준비 안 됨 */
  }
  send_status();
}

/* 모터 준비가 안 됐을 때: 0.5초마다 다시 찾고 설정한다 (12V 를 늦게 켜도 알아서 잡힘) */
static void task_retry(uint32_t now)
{
  if (now - last_tick < RETRY_PERIOD_MS)
  {
    return;
  }
  g_dxl_ready = motors_setup();
  last_tick = HAL_GetTick();     /* 설정하는 데 시간이 걸리므로 now 가 아니라 끝난 시각을 다시 읽어 기록 */
}

/* 1초마다 모터 상태 확인. 괜찮으면 1, 토크가 꺼졌으면 0 */
static int task_check(uint32_t now)
{
  if (now - last_check_tick < CHECK_PERIOD_MS)
  {
    return 1;                                      /* 아직 확인할 때 아님 -> 괜찮다고 봄 */
  }
  last_check_tick = now;
  return check_motors();
}

/* 0.05초마다 엔코더(실제 속도 / 위치) 읽고 젯슨에 보내기 */
static void task_encoder(uint32_t now)
{
  if (now - last_vel_tick < VEL_PERIOD_MS)
  {
    return;
  }
  last_vel_tick = now;
  read_present_velocity();                         /* 모터에서 읽기 */
  send_encoder();                                  /* 젯슨에 보내기 */
}

/* 정한 속도를 디버거 변수에 적어 둔다 (앞으로 = + 기준) */
static void save_goals(int32_t left, int32_t right)
{
  g_goal[0] = left;
  g_goal[1] = right;
  g_goal_rpm[0] = left * 0.229f;                   /* 모터 값 x 0.229 = rpm */
  g_goal_rpm[1] = right * 0.229f;
}

/* 모터에 속도 보내기. 모터 방향(LEFT_DIR / RIGHT_DIR)은 보내기 직전에 여기서 곱한다 */
static void send_goals(int32_t left, int32_t right)
{
  int32_t speeds[2];
  speeds[0] = LEFT_DIR * left;                     /* 왼쪽:   +1 x left  */
  speeds[1] = RIGHT_DIR * right;                   /* 오른쪽: -1 x right (반대로 달려 있어서) */
  /* 두 모터에 한 번에 보내기 (Sync Write, mx64.c) */
  mx64_set_speeds(ids, speeds, (uint8_t)count);
}

/* 0.05초마다: 스위치 읽기 -> 속도 정하기 -> 기록 -> 모터에 보내기 */
static void task_drive(uint32_t now)
{
  if (now - last_tick < UPDATE_PERIOD_MS)
  {
    return;
  }
  last_tick = now;

  int32_t left;                                    /* decide 가 결과를 써 넣을 칸 */
  int32_t right;
  g_sw_on = sw_on();
  /* &left, &right : "여기에 결과 써 줘" 하고 위치를 넘긴다 */
  g_state = decide(g_sw_on, now, &left, &right);
  save_goals(left, right);
  send_goals(left, right);
}


/* ============================================================================
 *  main.c 가 while(1) 안에서 계속 부르는 함수  <- 여기부터 읽기 시작!
 * ============================================================================ */

void app_loop(void)
{
  uint32_t now = HAL_GetTick();  /* 전원 켠 뒤 지난 시간 (ms). 아래 task 들이 다 이 값을 쓴다 */

  /* 1) 모터 상태와 상관없이 항상 하는 일 */
  handle_host();                 /* 젯슨에서 온 명령 받기 */
  task_psd(now);                 /* PSD 재고 보내기 */
  task_status(now);              /* 상태 보고 */

  /* 2) 모터가 아직 준비 안 됨 -> 다시 시도만 하고 이번 차례는 끝 */
  if (!g_dxl_ready)
  {
    task_retry(now);
    return;                      /* return = 여기서 함수 끝. 아래는 실행 안 함 */
  }

  /* 3) 모터 준비 됨 */
  if (!task_check(now))          /* 모터 토크가 꺼졌으면 */
  {
    g_dxl_ready = 0;             /* "준비 안 됨" 으로 바꾸면 다음 차례에 task_retry 가 재부팅까지 해 준다 */
    return;
  }
  task_encoder(now);             /* 엔코더 읽고 보내기 */
  task_drive(now);               /* 속도 정해서 모터에 보내기 */
}
