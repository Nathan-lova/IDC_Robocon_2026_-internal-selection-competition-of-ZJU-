/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : M2006/C610 + PS2 + stepper + dual servo control
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "ps2.h"
#include "pid.h"
/* CIR button: auto-forward */
#define FORWARD_DURATION_MS  4000
#define FORWARD_TARGET_RPM   2500.0f
/* aggressive PID for slope climb: more integral to fight gravity */
#define CLIMB_KP      3.0f
#define CLIMB_KI      1.2f
#define CLIMB_KD      0.50f
#define CLIMB_ILIMIT  6500
/* wheel sync: cross-couple correction to prevent yaw drift on slope */
#define SYNC_GAIN     0.35f    /* 0=off, higher=stricter sync */
#define SYNC_MAX      400.0f   /* max RPM correction per side (P term clamp) */
#define SYNC_KI       0.08f    /* cross-couple integral gain (0=off) */
#define SYNC_ILIMIT   250.0f   /* max RPM correction from integral alone */
/* normal driving PID (restored after climb) */
#define NORM_KP       1.6f
#define NORM_KI       0.25f
#define NORM_KD       0.70f
#define NORM_ILIMIT   4000
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

volatile uint16_t dbg_servo180_target = 0;
volatile uint16_t dbg_servo1_pulse = 0;
/* USER CODE BEGIN PV */
#define JOY_DEAD      20
#define JOY_MAX       127.0f
#define WHEEL_MAX_RPM 4000.0f
#define TURN_MAX_RPM  3000.0f
#define SERVO360_STOP_US  1500u
#define SERVO360_SPEED_SLOW 150u  /* initial speed */
#define SERVO360_SPEED_FAST 350u  /* speed after holding */
#define SERVO360_RAMP_CNT    50    /* frames (150ms) before speed-up */
#define SLEW_RATE         200.0f  /* RPM per 10ms tick, limit target ramp */

PID_TypeDef motor_pid[2];

/* ================= Real PS2 key mapping ================= */

typedef struct
{
    uint8_t bank;   // 1 = btn1, 2 = btn2
    uint8_t mask;
} ps2_key_t;

#define PS2_BANK_BTN1  1
#define PS2_BANK_BTN2  2

#define K_BTN1(mask)   {PS2_BANK_BTN1, (mask)}
#define K_BTN2(mask)   {PS2_BANK_BTN2, (mask)}


/* 你们实测出来的物理按键映射 */
static ps2_key_t KEY_PHY_UP       = K_BTN1(0x08);
static ps2_key_t KEY_PHY_RIGHT    = K_BTN1(0x10);
static ps2_key_t KEY_PHY_DOWN     = K_BTN1(0x20);
static ps2_key_t KEY_PHY_LEFT     = K_BTN1(0x40);

static ps2_key_t KEY_PHY_L2       = K_BTN1(0x80);  /* confirmed: btn1 255→127 */
static ps2_key_t KEY_PHY_R2       = K_BTN2(0x01);  /* confirmed: btn2 127→126 */
static ps2_key_t KEY_PHY_L1       = K_BTN2(0x02);
static ps2_key_t KEY_PHY_R1       = K_BTN2(0x04);


uint16_t TIM_COUNT[2];
static uint32_t led1_tick = 0;
static uint32_t motor_tick = 0;
static uint32_t step_cnt = 0;
static uint8_t  pul_state = 0;
static int8_t   stepper_dir = 0;  /* -1=DOWN, 0=STOP, 1=UP */
static ps2_state_t ps2;
static float    speed_scale = 1.0f;
static float    actual_target_l = 0.0f;
static float    actual_target_r = 0.0f;
static uint8_t  prev_btn2 = 0xFF;       /* edge detection for L1/R1 */
static uint32_t ps2_ok_cnt = 0;
static uint32_t ps2_fail_cnt = 0;
static uint32_t startup_guard_cnt = 0;       /* skip motor output first ~500ms */

static uint8_t  cal_ok = 0;
static uint8_t  cly, crx;
static uint8_t  relay_on;
static uint8_t  prev_circle_bit = 1;   /* btn2 bit4 idle=1 (not pressed) */
static uint8_t  debounce_left;
static uint8_t  debounce_right;
static uint8_t  servo360_hold_cnt = 0;  /* hold counter for servo speed ramp */
static uint8_t  debounce_l2;
static uint8_t  debounce_r2;
static uint8_t  s1_locked = 0;       /* servo1 lock flag */
static uint16_t s1_start = 0;        /* servo1 pulse at press start */
#define S1_MAX_TRAVEL  150           /* max pulse change per continuous press */
static uint8_t  debounce_up;
static uint8_t  debounce_down;

/* Nathan-style PS2 reconnect */
static uint8_t  ps2_connected = 0;
static uint32_t ps2_reinit_tick = 0;
static uint8_t  ps2_reconnect_cnt = 0;

/* CIR button: auto-forward 7s @ 1500 RPM for 30° slope climb */
static uint8_t  forward_active = 0;
static uint32_t forward_start_tick = 0;
static uint8_t  debounce_cir = 0;
static float    sync_i = 0.0f;        /* cross-couple integral accumulator */

/* stepper acceleration */
static uint16_t stepper_period = 4000;   /* ~10500 steps/s start */
/* USER CODE END PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
static uint8_t ps2_key_pressed(ps2_key_t key)
{
    uint8_t pressed;

    if (key.bank == PS2_BANK_BTN1)
    {
        pressed = (uint8_t)(~ps2.btn1);
    }
    else
    {

        pressed = (uint8_t)(~ps2.btn2) & 0x7F;
    }

    return (pressed & key.mask) != 0;
}
/* USER CODE END 0 */

static float ramp_target(float target, float current)
{
    float diff = target - current;
    if (diff >  SLEW_RATE) return current + SLEW_RATE;
    if (diff < -SLEW_RATE) return current - SLEW_RATE;
    return target;
}

int main(void)
{
  /* ====================== boot debug LED sequence ======================
     PF14 (LED1) 状态指示启动进度:
       灭      → 还未上电或硬件故障
       亮(常亮) → 卡在 HAL_Init()
       亮→灭→亮 → 卡在 SystemClock_Config() / HSE 起振
       闪烁     → 已进入主循环, 正常运行
     =================================================================== */

  /* ---- 第0步: 绕过HAL, 直接亮LED ---- */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIOF->MODER = (GPIOF->MODER & ~(3u << 28)) | (1u << 28);  /* PF14 output */

  /* 阶段1: HAL_Init 前 → LED ON */
  GPIOF->BSRR = GPIO_PIN_14;

  HAL_Init();

  /* 阶段1完成: HAL_Init 通过 → LED OFF */
  GPIOF->BSRR = (uint32_t)GPIO_PIN_14 << 16;

  /* 阶段2: SystemClock_Config 前 → LED ON */
  GPIOF->BSRR = GPIO_PIN_14;

  SystemClock_Config();

  /* 阶段2完成: SystemClock_Config 通过 → LED OFF */
  GPIOF->BSRR = (uint32_t)GPIO_PIN_14 << 16;

  /* ====================== boot debug end ====================== */

  MX_GPIO_Init();  /* re-inits PF14, LED1 goes OFF here */
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  my_can_filter_init_recv_all(&hcan1);
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  hcan1.Instance->MCR |= CAN_MCR_ABOM;  /* auto bus-off recovery */

  MX_MOTO_GPIO_Init();
  MX_EMAG_GPIO_Init();
  MX_TIM3_Init();
  MX_SERVO_GPIO_Init();
  MX_TIM4_Init();
  servo_drv_init();
  ps2_init();
  /* PID speed control init */
  pid_init(&motor_pid[0]);
  pid_init(&motor_pid[1]);
  motor_pid[0].f_param_init(&motor_pid[0], PID_Speed, 8000, 4000, 20, 10, 500, 0, 1.6f, 0.25f, 0.70f);
  motor_pid[1].f_param_init(&motor_pid[1], PID_Speed, 8000, 4000, 20, 10, 500, 0, 1.6f, 0.25f, 0.70f);


  /* USER CODE END 2 */

  static uint32_t emag_test_tick = 0;

  while (1)
  {
    if(HAL_GetTick() - led1_tick > 1000)
    {
      led1_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }

    /* 继电器测试代码已注释，用ST-Link看relay_on变量确认按键检测 */

    if(HAL_GetTick() - motor_tick >= 10)
    {
      motor_tick = HAL_GetTick();

      u8 ps2_ok = ps2_read(&ps2);

      if (ps2_ok) {
				
        ps2_ok_cnt++;

        /* reconnection debounce: 30 consecutive OK frames (300ms) */
        if (!ps2_connected) {
          if (++ps2_reconnect_cnt >= 30) {
            ps2_reconnect_cnt = 0;
            ps2_connected = 1;
          }
        } else {
          ps2_fail_cnt = 0;

        if (!cal_ok) {
          cal_ok = 1;
          cly = ps2.joy_ly;
          crx = ps2.joy_rx;
        }

        /* ---- CIR button: trigger auto-forward 7s @ 1500 RPM (3-frame debounce) ---- */
        {
          uint8_t cir_pressed = ((uint8_t)(~ps2.btn2) & PS2_CIR) ? 1 : 0;

          if (cir_pressed && debounce_cir < 3) debounce_cir++;
          else if (!cir_pressed)              debounce_cir = 0;

          if (debounce_cir == 3) {          /* just confirmed */
            debounce_cir = 4;               /* lock until release */
            forward_active = 1;
            forward_start_tick = HAL_GetTick();
            sync_i = 0.0f;
            motor_pid[0].f_pid_reset(&motor_pid[0], CLIMB_KP, CLIMB_KI, CLIMB_KD);
            motor_pid[1].f_pid_reset(&motor_pid[1], CLIMB_KP, CLIMB_KI, CLIMB_KD);
            motor_pid[0].IntegralLimit = CLIMB_ILIMIT;
            motor_pid[1].IntegralLimit = CLIMB_ILIMIT;
            actual_target_l = 0.0f;
            actual_target_r = 0.0f;
            motor_pid[0].target = 0.0f;
            motor_pid[1].target = 0.0f;
            /* stop stepper during forward auto-run */
            HAL_TIM_Base_Stop_IT(&htim3);
            htim3.Instance->CNT = 0;
            stepper_period = 4000;
            pul_state = 0;
            GPIOB->BSRR = (uint32_t)MOTO_PUL_Pin << 16;
            stepper_dir = 0;
          }
          prev_circle_bit = (cir_pressed) ? 0 : 1;  /* sync relay edge detect */
        }

        if (!forward_active) {

        /* ---- speed: L1=×0.75, R1=×1.33 (edge-triggered, 0=pressed) ---- */
				{
						uint8_t btn2_effective;
						uint8_t edge2;

						btn2_effective = ((uint8_t)(~ps2.btn2)) & 0x7F;
						edge2 = btn2_effective & (uint8_t)(~prev_btn2);

						if (edge2 & KEY_PHY_R1.mask)
						{
								speed_scale *= 1.25f;  /* R1: speed up */
								if (speed_scale > 2.5f)
										speed_scale = 2.5f;
						}

						if (edge2 & KEY_PHY_L1.mask)
						{
								speed_scale *= 0.8f;   /* L1: slow down */
								if (speed_scale < 0.2f)
										speed_scale = 0.2f;
						}

						prev_btn2 = btn2_effective;
				}

        /* ---- joystick normalize ---- */
        int16_t ly = (int16_t)ps2.joy_ly - cly;
        int16_t rx = (int16_t)ps2.joy_rx - crx;

        if (ly > -JOY_DEAD && ly < JOY_DEAD) ly = 0;
        if (rx > -JOY_DEAD && rx < JOY_DEAD) rx = 0;

        float fwd = 0.0f, trn = 0.0f;
        if (ly != 0) { fwd = -(float)ly / (JOY_MAX - JOY_DEAD); if (fwd >  1.0f) fwd =  1.0f; if (fwd < -1.0f) fwd = -1.0f; }
        if (rx != 0) { trn =  (float)rx / (JOY_MAX - JOY_DEAD); if (trn >  1.0f) trn =  1.0f; if (trn < -1.0f) trn = -1.0f; }

        float fwd_rpm = fwd * WHEEL_MAX_RPM * speed_scale;
        float trn_rpm = trn * TURN_MAX_RPM  * speed_scale;
        float t_l = fwd_rpm + trn_rpm, t_r = fwd_rpm - trn_rpm;
        if (t_l >  WHEEL_MAX_RPM) t_l =  WHEEL_MAX_RPM; if (t_l < -WHEEL_MAX_RPM) t_l = -WHEEL_MAX_RPM;
        if (t_r >  WHEEL_MAX_RPM) t_r =  WHEEL_MAX_RPM; if (t_r < -WHEEL_MAX_RPM) t_r = -WHEEL_MAX_RPM;
        actual_target_l = ramp_target(t_l, actual_target_l);
        actual_target_r = ramp_target(t_r, actual_target_r);
        motor_pid[0].target = actual_target_l;
        motor_pid[1].target = actual_target_r;

        /* ---- stepper: D-pad UP/DOWN (3-frame debounce) ---- */
        {
          uint8_t up_now   = ps2_key_pressed(KEY_PHY_UP);
          uint8_t down_now = ps2_key_pressed(KEY_PHY_DOWN);

          if (up_now   && debounce_up   < 5) debounce_up++;
          else if (!up_now)                  debounce_up = 0;
          if (down_now && debounce_down < 5) debounce_down++;
          else if (!down_now)                debounce_down = 0;

          int8_t desired = 0;
          if (debounce_down > 2)
            desired = 1;
          else if (debounce_up > 2)
            desired = -1;

          if (desired != stepper_dir) {
            /* direction change or start/stop — full reset before switch */
            HAL_TIM_Base_Stop_IT(&htim3);
            htim3.Instance->CNT = 0;
            stepper_period = 4000;
            htim3.Instance->ARR = stepper_period;
            pul_state = 0;
            GPIOB->BSRR = (uint32_t)MOTO_PUL_Pin << 16;  /* PUL LOW */

            if (desired == 1) {
              GPIOB->BSRR = MOTO_DIR_Pin;              /* DIR HIGH */
              HAL_TIM_Base_Start_IT(&htim3);
            } else if (desired == -1) {
              GPIOB->BSRR = (uint32_t)MOTO_DIR_Pin << 16; /* DIR LOW */
              HAL_TIM_Base_Start_IT(&htim3);
            }
            stepper_dir = desired;
          }

          /* accel ramp */
          if (stepper_dir != 0 && stepper_period > 2000) {
            stepper_period -= 280;
            if (stepper_period < 2000) stepper_period = 2000;
            htim3.Instance->ARR = stepper_period;
          }
        }

        /*
         * Servo0 (PC0, D-pad LEFT/RIGHT): 360° continuous rotation
         *   1500us=STOP, 500us=max CW, 2500us=max CCW
         * Servo1 (PC1, L2/R2): 180° angle servo — holds position on release
         *   500us=0°, 1500us=90°, 2500us=180°
         */
        #define SERVO360_DB  5u    /* debounce frames */
        #define S_180_STEP  7

        /* ---- servo0: 360° two-speed ramp, stop on release ---- */
        {
          uint8_t left_now  = ps2_key_pressed(KEY_PHY_LEFT);
          uint8_t right_now = ps2_key_pressed(KEY_PHY_RIGHT);

          if (left_now && !right_now) {
            if (debounce_left  < 10) debounce_left++;
            debounce_right = 0;
          } else if (right_now && !left_now) {
            if (debounce_right < 10) debounce_right++;
            debounce_left = 0;
          } else {
            debounce_left = 0;
            debounce_right = 0;
          }

          /* ramp-up counter: count hold time, reset on release */
          if (debounce_left >= SERVO360_DB || debounce_right >= SERVO360_DB) {
            if (servo360_hold_cnt < 255) servo360_hold_cnt++;
          } else {
            servo360_hold_cnt = 0;
          }

          /* pick speed: fast after RAMP_CNT frames, slow otherwise */
          uint16_t speed = (servo360_hold_cnt >= SERVO360_RAMP_CNT)
                           ? SERVO360_SPEED_FAST : SERVO360_SPEED_SLOW;

          uint16_t s0;
          if (debounce_left >= SERVO360_DB)
            s0 = SERVO360_STOP_US - speed;
          else if (debounce_right >= SERVO360_DB)
            s0 = SERVO360_STOP_US + speed;
          else
            s0 = SERVO360_STOP_US;

          servo_set_pulse(SERVO_CH0, s0);
        }

        /* ---- servo1: 180°, L2=收 R2=放 (3-frame debounce + stall lock) ---- */
        {
          uint8_t l2_now = ps2_key_pressed(KEY_PHY_L2);
          uint8_t r2_now = ps2_key_pressed(KEY_PHY_R2);

          if (l2_now && debounce_l2 < 5) debounce_l2++;
          else if (!l2_now)             debounce_l2 = 0;
          if (r2_now && debounce_r2 < 5) debounce_r2++;
          else if (!r2_now)             debounce_r2 = 0;

          /* detect fresh press → record start + unlock */
          if (debounce_l2 == 3 || debounce_r2 == 3) {
            s1_locked = 0;
            s1_start = servo_get_pulse(SERVO_CH1);
          }

          if (!s1_locked) {
            uint16_t s1 = servo_get_pulse(SERVO_CH1);
            uint16_t orig = s1;

            if (debounce_l2 > 2) {
              if (s1 > 1360) s1 -= S_180_STEP;
            } else if (debounce_r2 > 2) {
              if (s1 < 1735) s1 += S_180_STEP;
            }

            /* stall lock: if travel exceeds limit, clamp and lock */
            uint16_t travel = (s1 > s1_start) ? (s1 - s1_start) : (s1_start - s1);
            if (travel > S1_MAX_TRAVEL) {
              s1_locked = 1;
              s1 = (s1 > s1_start) ? (s1_start + S1_MAX_TRAVEL)
                                   : (s1_start - S1_MAX_TRAVEL);
              /* re-clamp to hardware limits */
              if (s1 < 1360) s1 = 1360;
              if (s1 > 1735) s1 = 1735;
            }
            servo_set_pulse(SERVO_CH1, s1);
          }
          /* when locked: hold position, do nothing until button released+repressed */
        }
	

        } /* !forward_active */

        }
      } else {
        ps2_reconnect_cnt = 0;
        ps2_fail_cnt++;
        if (!forward_active) {
          servo_set_pulse(SERVO_CH0, SERVO360_STOP_US);
          /* 连续10次失败(100ms)才判定真断连, 防止偶尔噪声误触发 */
          if (ps2_connected && ps2_fail_cnt > 10) {
            ps2_connected = 0;
            motor_pid[0].f_pid_reset(&motor_pid[0], 1.6f, 0.25f, 0.70f);
            motor_pid[1].f_pid_reset(&motor_pid[1], 1.6f, 0.25f, 0.70f);
            actual_target_l = 0.0f;
            actual_target_r = 0.0f;
            motor_pid[0].target = 0.0f;
            motor_pid[1].target = 0.0f;
            set_moto_current(&hcan1, 0, 0);
            HAL_TIM_Base_Stop_IT(&htim3);
            stepper_period = 4000;
            stepper_dir = 0;
            pul_state = 0;
            GPIOB->BSRR = (uint32_t)MOTO_PUL_Pin << 16;
            relay_on = 0;
            relay_set_pulse(RELAY_PULSE_OFF);  /* 2000us = 断开 */
          }
          if (!ps2_connected && HAL_GetTick() - ps2_reinit_tick > 200) {
            ps2_reinit_tick = HAL_GetTick();
            ps2_init();
          }
        }
      }

      /* ---- forward motion: CIR auto-run 7s @ 1500 RPM (30° slope climb) ---- */
      if (forward_active) {
        if (HAL_GetTick() - forward_start_tick >= FORWARD_DURATION_MS) {
          /* time elapsed → stop, ramp down, restore normal PID */
          forward_active = 0;
          sync_i = 0.0f;
          motor_pid[0].f_pid_reset(&motor_pid[0], NORM_KP, NORM_KI, NORM_KD);
          motor_pid[1].f_pid_reset(&motor_pid[1], NORM_KP, NORM_KI, NORM_KD);
          motor_pid[0].IntegralLimit = NORM_ILIMIT;
          motor_pid[1].IntegralLimit = NORM_ILIMIT;
          actual_target_l = ramp_target(0.0f, actual_target_l);
          actual_target_r = ramp_target(0.0f, actual_target_r);
          motor_pid[0].target = 0.0f;
          motor_pid[1].target = 0.0f;
        } else {
          /* ramp to target, then cross-couple wheel speeds for straight climb */
          float base = ramp_target(FORWARD_TARGET_RPM,
                       (actual_target_l + actual_target_r) * 0.5f);
          /* speed diff: positive = left faster */
          float spd_l =  moto_chassis[0].speed_rpm;
          float spd_r = -moto_chassis[1].speed_rpm;  /* motor1 is negated */
          float diff  = spd_l - spd_r;
          /* PI cross-couple: P term + I term to eliminate steady-state yaw */
          float p_term = diff * SYNC_GAIN;
          sync_i += diff * SYNC_KI;
          /* clamp integral (anti-windup) */
          if (sync_i >  SYNC_ILIMIT) sync_i =  SYNC_ILIMIT;
          if (sync_i < -SYNC_ILIMIT) sync_i = -SYNC_ILIMIT;
          float sync_corr = p_term + sync_i;
          if (sync_corr >  SYNC_MAX) sync_corr =  SYNC_MAX;
          if (sync_corr < -SYNC_MAX) sync_corr = -SYNC_MAX;
          actual_target_l = base - sync_corr;
          actual_target_r = base + sync_corr;
          motor_pid[0].target = actual_target_l;
          motor_pid[1].target = actual_target_r;
        }
      }

      /* ---- common motor PID + CAN output ---- */
      {
        int16_t cur_l = 0, cur_r = 0;
        if (startup_guard_cnt > 200) {
          cur_l = (int16_t)motor_pid[0].f_cal_pid(&motor_pid[0],
                                           moto_chassis[0].speed_rpm);
          cur_r = (int16_t)motor_pid[1].f_cal_pid(&motor_pid[1],
                                           -moto_chassis[1].speed_rpm);
        } else {
          startup_guard_cnt++;
        }

        if (cur_l >  8000) cur_l =  8000;
        if (cur_l < -8000) cur_l = -8000;
        if (cur_r >  8000) cur_r =  8000;
        if (cur_r < -8000) cur_r = -8000;

        set_moto_current(&hcan1, cur_l, -cur_r);
      }
    }
  }
}

/** System Clock Configuration */
void SystemClock_Config(void)
{
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

/* ---- timer callbacks ---- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  if (htim->Instance == TIM3) {
    pul_state = !pul_state;
    if (pul_state)
      GPIOB->BSRR = GPIO_PIN_0;
    else
      GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;
    if (++step_cnt >= 24000) {
      step_cnt = 0;
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
  }
  if (htim->Instance == TIM4) {
    servo_tim4_period_elapsed();
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  servo_tim4_oc_match(htim->Channel);
}

void _Error_Handler(char * file, int line)
{
  while(1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
