/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_WHO_AM_I     0x75 // to verify the bus
#define MPU_ADDR         (0x68 << 1) // mpu6050 i2c address
#define MPU_PWR_MGMT_1   0x6B // wake, PLL with X gyro
#define MPU_CONFIG       0x1A
#define MPU_SMPLRT_DIV   0x19
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_INT          0x38
#define MPU_USER_CTRL    0x6A
#define MPU_ACCEL_XOUT_H 0x3B
#define ACCEL_LSB_PER_G  16384.0f
#define GYRO_LSB_PER_DPS 131.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Minimal runtime state (diagnostic counters removed for simplicity) */
static volatile uint8_t i2c_busy = 0;      /* 1 while an I2C DMA read is in progress */
static volatile uint8_t data_ready = 0;    /* Set when a new sample has been received */
static volatile uint8_t uart_tx_ready = 1; /* 1 when UART DMA is idle */
static char uart_tx_buf[256];              /* persistent UART TX buffer */
/* 1-second pacing (TIM6 at 200 Hz -> 200 ticks = 1 second) */
static volatile uint16_t tick_count = 0;
static volatile uint8_t second_flag = 0;   /* Set each second to trigger UART output */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* MPU6050 sample buffer (ACCEL[6] + TEMP[2] + GYRO[6] = 14 bytes) */
static uint8_t mpu_buf[14];

/* Raw sensor readings */
static int16_t ax_raw, ay_raw, az_raw;
static int16_t gx_raw, gy_raw, gz_raw;
static int16_t temp_raw;

/* Converted physical units */
static float ax_g, az_g;
// static float ay_g;
static float  gy_dps;
// static float gx_dps, gz_dps
// static float temp_c;


/* -------------------------------------------------------------------------- */
/* USER CONFIGURABLE PARAMETERS (EDIT THESE FOR TUNING)                       */
/* -------------------------------------------------------------------------- */

/* PID gains (output units: PWM counts if u_max = ARR). */
static float Kp = 1.0f;  /* Start modest; adjust later */
static float Ki = 0.0f;   /* Begin at 0 to tune P & D first */
static float Kd = 0.0f;   /* Derivative on gyro (see formula) */

/* Safety & scaling. */
static float max_power_scale = 0.8f;   /* scale to 80% of 4199 (3359) */
static float min_power_scale = 0.6f;   /* scale to 60% of 4199 (2519) */

static float max_deadband = 1.0f;   /* Angle above which output is allowed */
static float min_deadband = 1.0f;   /* Angle below which output/integral are reset */

static float safety_angle_deg = 35.0f; /* Disable motors beyond this tilt degree */

static float offset_angle = -2.0f; /* Manual angle offsets. */


/* Integral windup guard (absolute value clamp). */
static float integral_limit = 5000.0f;

/* Complementary filter coefficient (alpha close to 1 trusts gyro more). */
static float cf_alpha = 0.95f;

/* Timing constants. */
static const float dt = 0.005f;  /* 1/200 Hz (TIM6) */

/* -------------------------------------------------------------------------- */
/* USER CODE CONFIGURATION END                                                */
/* -------------------------------------------------------------------------- */


/* Degree-based attitude & PID state.*/
static float pitch_acc_deg = 0.0f;     /* Instant pitch from accelerometer */
static float pitch_deg = 0.0f;         /* Fused angle after complementary filter */
static float error;
static float prev_pitch_deg = 0.0f;    /* Previous fused angle for integration base */
static float pid_P = 0.0f, pid_I = 0.0f, pid_D = 0.0f; /* PID term contributions */
static float pid_u = 0.0f;             /* Total control output (signed PWM counts) */
static uint8_t attitude_initialized = 0; /* Set after first accel-based seed */

/* Loop timing diagnostics (microseconds) */
static uint32_t loop_time_us = 0;        /* Duration of last control loop (us) */

/* Cycle counter for timing */
uint32_t arr;
float duty;
float max_duty;
float min_duty;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef mpu_read_u8(uint8_t reg, uint8_t *val)
{
  return HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

static HAL_StatusTypeDef mpu_write_u8(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static HAL_StatusTypeDef mpu_read_burst_dma(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
  /* HAL will perform: [S | Addr(W) | start_reg] then repeated start [Addr(R) | read len] using DMA */
  return HAL_I2C_Mem_Read_DMA(&hi2c2, MPU_ADDR, start_reg, I2C_MEMADD_SIZE_8BIT, buf, len);
}

static bool mpu_init_200hz(void)
{
  /* Read WHO_AM_I to confirm presence and set address */
  uint8_t who = 0;
  if (mpu_read_u8(MPU_WHO_AM_I, &who) != HAL_OK) {
    return false;
  }

  /* Accept common IDs: MPU6050=0x68/0x69, MPU6500/9250=0x70 */
  if (!(who == 0x68 || who == 0x69 || who == 0x70)) {
    return false;
  }

  /* Wake up, select PLL clock (recommended) */
  if (mpu_write_u8(MPU_PWR_MGMT_1, 0x01) != HAL_OK) return false; /* CLKSEL=PLL X gyro */
  HAL_Delay(10);

  /* DLPF = 3 → internal sample rate = 1 kHz; gyro BW ≈ 42 Hz, accel BW ≈ 44 Hz */
  if (mpu_write_u8(MPU_CONFIG, 0x03) != HAL_OK) return false;

  /* SMPLRT_DIV = 4 → 1000 / (1+4) = 200 Hz output data rate */
  if (mpu_write_u8(MPU_SMPLRT_DIV, 0x04) != HAL_OK) return false;

  /* Gyro full-scale ±250 dps (FS_SEL=0), Accel ±2 g (AFS_SEL=0) */
  if (mpu_write_u8(MPU_GYRO_CONFIG,  0x00) != HAL_OK) return false;
  if (mpu_write_u8(MPU_ACCEL_CONFIG, 0x00) != HAL_OK) return false;

  /* Ensure interrupts/FIFO/master are off (we poll via I2C) */
  (void)mpu_write_u8(MPU_INT, 0x00);
  (void)mpu_write_u8(MPU_USER_CTRL,  0x00);

  return true;
}


static void transmit_once_per_second(void)
{
  if (!second_flag || !uart_tx_ready) return;
  second_flag = 0;
  /* Single concise snapshot (last computed values). */
  int n = snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                   "P= %.1f I= %.1f D= %.1f | Angle= %.2f error= %.2f PWM= %.0f | loop_us= %lu\r\n",
                   pid_P, pid_I, pid_D, pitch_deg, error, duty, (unsigned long)loop_time_us);
  if (n > 0) {
    uart_tx_ready = 0;
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart_tx_buf, (uint16_t)n) != HAL_OK) {
      uart_tx_ready = 1;
    }
  }
}


/* ===================== Control & PID =====================*/

/* Simple cycle counter timing (idempotent call OK) */
/* cycle_counter_init()
   Enables the DWT cycle counter so we can measure how long each control loop
   takes. This helps verify we stay under the 5 ms (200 Hz) budget. */
static void cycle_counter_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* Enable trace */
#ifdef DWT_LAR
  DWT->LAR = 0xC5ACCE55; /* Unlock on some cores */
#endif
  DWT->CYCCNT = 0;                        /* Reset counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;     /* Start counting CPU cycles */
}


/* cycles_to_us(cycles)
   Converts raw CPU cycle counts to microseconds using SystemCoreClock. */
static uint32_t cycles_to_us(uint32_t cycles)
{
  return (uint32_t)((uint64_t)cycles * 1000000ULL / SystemCoreClock);
}


/* motor_set_output(u_counts, pitch)
   Takes signed control output (pid_u) in PWM counts.
   - Sign decides direction pin.
   - Magnitude (after limits & deadband) becomes PWM duty.
   - Enforces safety tilt cutoff & caps maximum power for testing. */
static void motor_set_output(float u_counts, float current_pitch_deg)
{
  /* Single PWM (TIM1 CH1) drives both ENA & ENB (wired together). Direction via two pins:
     DIR_FWD high / DIR_REV low  -> forward both motors
     DIR_FWD low  / DIR_REV high -> reverse both motors
  */

  /* Safety angle cutoff */
  if (fabsf(current_pitch_deg) > safety_angle_deg) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    pid_u = 0.0f;
    pid_I = 0.0f;
    pid_D = 0.0f;

    HAL_GPIO_WritePin(DIR_FWD_GPIO_Port, DIR_FWD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_REV_GPIO_Port, DIR_REV_Pin, GPIO_PIN_RESET);
    return;
  }


  /* Double deadband logic: angle-based */
  float abs_angle = fabsf(current_pitch_deg);

  if (abs_angle < min_deadband) {
    // Within tight upright zone: reset output and integral
	duty = 0.0f;
    pid_I = 0.0f;
  }
  else if (abs_angle > max_deadband) {
    // Outside deadband: allow output
    duty = fabsf(u_counts);
    duty = min_duty + ((duty / arr) * (max_duty - min_duty));
    if (duty > max_duty) duty = max_duty;
  }
  else {
    // In between: keep previous duty, but do not reset integral
  }


  /* Set PWM duty */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty);

  /* Direction logic */
  if (duty == 0.0f) {
    HAL_GPIO_WritePin(DIR_FWD_GPIO_Port, DIR_FWD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_REV_GPIO_Port, DIR_REV_Pin, GPIO_PIN_RESET);
  } else if (u_counts >= 0.0f) {
    HAL_GPIO_WritePin(DIR_FWD_GPIO_Port, DIR_FWD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR_REV_GPIO_Port, DIR_REV_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(DIR_FWD_GPIO_Port, DIR_FWD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_REV_GPIO_Port, DIR_REV_Pin, GPIO_PIN_SET);
  }
}


/* control_update()
   Runs every sensor sample (200 Hz):
     1. Compute pitch from accel & integrate gyro.
     2. Fuse with complementary filter.
     3. Compute PID.
     4. Drive motor (with safety + power limits).
     5. Measure loop time & log.
*/
static void control_update(void)
{
  uint32_t start_cycles = DWT->CYCCNT;

  /* Angle calc in degrees */
  /* Simple 2-axis pitch using only X (forward) and Z (up) acceleration.
    pitch ≈ atan2(-Ax, Az). This ignores Ay so it's a bit less noise sensitive for pure forward/back motion. */

  pitch_acc_deg = atan2f(-ax_g, -az_g) * 57.2957795131f; /* rad->deg */

  if (!attitude_initialized) {
    prev_pitch_deg = pitch_acc_deg;
    attitude_initialized = 1;
  }

  /* Complementary filter */
  pitch_deg = cf_alpha * (prev_pitch_deg + gy_dps * dt) + (1.0f - cf_alpha) * pitch_acc_deg;
  prev_pitch_deg = pitch_deg;


  error = offset_angle - pitch_deg; // Target is offset_angle deg (upright)
  pid_P = Kp * error;
  pid_D = Kd * (-gy_dps); // D term fights fast tilting
  pid_I += Ki * error * dt;
  if (pid_I > integral_limit) pid_I = integral_limit; else if (pid_I < -integral_limit) pid_I = -integral_limit;
  pid_u = pid_P + pid_I + pid_D;

  /* Motor output */
  motor_set_output(pid_u, error);

  loop_time_us = cycles_to_us(DWT->CYCCNT - start_cycles); /* Last loop duration */
}

static void process_sample_and_stream(void)
{
  /* Parse big-endian 16-bit values */
  ax_raw   = (int16_t)((mpu_buf[0]  << 8) | mpu_buf[1]);
  ay_raw   = (int16_t)((mpu_buf[2]  << 8) | mpu_buf[3]);
  az_raw   = (int16_t)((mpu_buf[4]  << 8) | mpu_buf[5]);
  temp_raw = (int16_t)((mpu_buf[6]  << 8) | mpu_buf[7]);
  gx_raw   = (int16_t)((mpu_buf[8]  << 8) | mpu_buf[9]);
  gy_raw   = (int16_t)((mpu_buf[10] << 8) | mpu_buf[11]);
  gz_raw   = (int16_t)((mpu_buf[12] << 8) | mpu_buf[13]);

  /* Convert to physical units */
  ax_g = (float)ax_raw / ACCEL_LSB_PER_G;
  // ay_g = (float)ay_raw / ACCEL_LSB_PER_G; // Remove for performance
  az_g = (float)az_raw / ACCEL_LSB_PER_G;
  // gx_dps = (float)gx_raw / GYRO_LSB_PER_DPS; // Remove for performance
  gy_dps = (float)gy_raw / GYRO_LSB_PER_DPS;
  // gz_dps = (float)gz_raw / GYRO_LSB_PER_DPS; // Remove for performance
  // temp_c = (float)temp_raw / 340.0f + 36.53f; // Remove for performance

  control_update();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  cycle_counter_init();
  HAL_Delay(50);
  if (!mpu_init_200hz()) {
      /* If this fails, blink or stay here for debugging */
      char err[] = "MPU init failed\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)err, sizeof(err)-1, HAL_MAX_DELAY);
      while (1) {
        HAL_Delay(1000);
      }
    } else {
      char ok[] = "MPU init OK\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)ok, sizeof(ok)-1, HAL_MAX_DELAY);
    }

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); /* PWM output ready (duty stays 0) */
  arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  max_duty = max_power_scale * arr;
  min_duty = min_power_scale * arr; /* Minimum PWM to overcome friction */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (data_ready) {
	  data_ready = 0;
	  process_sample_and_stream();
	}
  transmit_once_per_second();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  /* TIM6 NVIC configured once in HAL_TIM_Base_MspInit; duplicate removed */
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DIR_FWD_Pin|DIR_REV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR_FWD_Pin DIR_REV_Pin */
  GPIO_InitStruct.Pin = DIR_FWD_Pin|DIR_REV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    if (!i2c_busy) {
      if (mpu_read_burst_dma(MPU_ACCEL_XOUT_H, mpu_buf, sizeof(mpu_buf)) == HAL_OK) {
        i2c_busy = 1;
      }
    }
    /* 200 Hz -> generate 1 Hz flag */
    if (++tick_count >= 100) {
      tick_count = 0;
      second_flag = 1;
    }
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c2) {
  i2c_busy = 0;
  data_ready = 1; /* signal main loop to process */
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c2) {
  i2c_busy = 0;  /* allow retry on next tick */
    /* Optionally, you could call HAL_I2C_DeInit/Init for severe errors */
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    uart_tx_ready = 1;
  }
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
