/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "balance_control.h"
#include "imu.h"
#include "line_follower.h"
#include "motor_control.h"
#include "oled_status.h"
#include "usart.h"
#include "ultrasonic.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t now_ms;
  uint32_t data_ready_missed;
  uint16_t loop_dt_ms;
  int16_t bt_target_speed;
  int16_t bt_target_turn;
  uint32_t bt_rx_count;
  uint8_t bt_last_byte;
  uint8_t oled_status;
  BalanceSample_t sample;
  MotorTelemetry_t motor;
  BalanceOutput_t output;
  LineFollowerOutput_t line;
} AppTelemetryFrame_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define BALANCE_TELEMETRY_PERIOD_MS    50U         /* UART debug print period. */
#define OLED_STATUS_PERIOD_MS          250U        /* OLED status refresh period. */
#define IMU_DATA_READY_THREAD_FLAG     0x00000001U /* FreeRTOS flag set by MPU6050 PB9 interrupt. */
#define CONTROL_IMU_SAMPLES_PER_STEP   2U          /* 5 ms IMU sample * 2 = 10 ms control period. */

/* 避障参数集中在此处，控制循环每 10 ms 使用一次。                    */
/* 这些数值对应本车的 target_speed/PWM 标定；名称中的 mm/s、dps 同时    */
/* 保留实验报告中的物理含义，实际输出仍由原平衡环完成单位换算。        */
#define OBSTACLE_AVOIDANCE_TRIGGER_DISTANCE_MM       400U
#define OBSTACLE_AVOIDANCE_CONFIRM_SAMPLES             2U
#define OBSTACLE_AVOIDANCE_CONFIRM_TIMEOUT_MS        200U
#define OBSTACLE_AVOIDANCE_HARD_BRAKE_CYCLES           0U
#define OBSTACLE_AVOIDANCE_BRAKE_SETTLE_MS            10U

#define OBSTACLE_AVOIDANCE_TURN_RATE_DPS              45U
#define OBSTACLE_AVOIDANCE_SEARCH_TURN_RATE_DPS       50U
#define OBSTACLE_AVOIDANCE_RIGHT_OUT_TURN_DEG       95.0f
#define OBSTACLE_AVOIDANCE_TURN_TARGET_DEG           80.0f
#define OBSTACLE_AVOIDANCE_RETURN_TURN_DEG           60.0f
#define OBSTACLE_AVOIDANCE_TURN_TIMEOUT_MS          2500U

#define OBSTACLE_AVOIDANCE_BYPASS_SPEED_MM_S         300
#define OBSTACLE_AVOIDANCE_TURN_FORWARD_SPEED_MM_S   100
#define OBSTACLE_AVOIDANCE_CONFIRM_CREEP_SPEED_MM_S   40
#define OBSTACLE_AVOIDANCE_SLOWDOWN_DISTANCE_MM      450U
#define OBSTACLE_AVOIDANCE_SLOWDOWN_MIN_SPEED_MM_S   200
#define OBSTACLE_AVOIDANCE_RETURN_SPEED_MM_S         300
#define OBSTACLE_AVOIDANCE_RETURN_SEEK_TURN_PWM       90
#define OBSTACLE_AVOIDANCE_SEARCH_SPEED_MM_S          60
#define OBSTACLE_AVOIDANCE_REACQUIRE_SPEED_MM_S       60
#define OBSTACLE_AVOIDANCE_REACQUIRE_TURN_DPS         27U
#define OBSTACLE_AVOIDANCE_REACQUIRE_RIGHT_MS        650U
#define OBSTACLE_AVOIDANCE_LINE_CONFIRM_CYCLES         5U

#define OBSTACLE_AVOIDANCE_SHIFT_OUT_MS              850U
#define OBSTACLE_AVOIDANCE_PASS_FORWARD_MS          1500U
#define OBSTACLE_AVOIDANCE_MIN_SHIFT_OUT_MS          500U
#define OBSTACLE_AVOIDANCE_MIN_PASS_FORWARD_MS       700U
#define OBSTACLE_AVOIDANCE_RETURN_TIMEOUT_MS        4000U
#define OBSTACLE_AVOIDANCE_SEARCH_SWEEP_MS           700U

/* 当前平衡环的 target_turn 单位是 PWM，不是 dps；45 dps 对应已验证的
 * 约 240 PWM，搜索/重捕线按比例降到约 145 PWM。方向符号遵循本车实测：
 * target_turn<0 为右转，target_turn>0 为左转。 */
#define OBSTACLE_AVOIDANCE_TURN_PWM_MAG             240
#define OBSTACLE_AVOIDANCE_SEARCH_TURN_PWM_MAG      145
#define OBSTACLE_AVOIDANCE_RIGHT_TURN_PWM   (-OBSTACLE_AVOIDANCE_TURN_PWM_MAG)
#define OBSTACLE_AVOIDANCE_LEFT_TURN_PWM    (OBSTACLE_AVOIDANCE_TURN_PWM_MAG)
#define OBSTACLE_AVOIDANCE_SEARCH_RIGHT_PWM (-OBSTACLE_AVOIDANCE_SEARCH_TURN_PWM_MAG)
#define OBSTACLE_AVOIDANCE_REARM_DELAY_MS          2500U
#define OBSTACLE_AVOIDANCE_STRAIGHT_MAX_ERROR       120
#define OBSTACLE_AVOIDANCE_CLEAR_DISTANCE_MM       800U
#define OBSTACLE_AVOIDANCE_CLEAR_CONFIRM_COUNT       3U

#define BT_CMD_FORWARD                 0x41U       /* APP arrow up: move forward. */
#define BT_CMD_RIGHT_FRONT             0x42U       /* APP arrow upper-right: forward + right turn. */
#define BT_CMD_RIGHT                   0x43U       /* APP arrow right: turn right in place. */
#define BT_CMD_RIGHT_BACK              0x44U       /* APP arrow lower-right: backward + right turn. */
#define BT_CMD_BACK                    0x45U       /* APP arrow down: move backward. */
#define BT_CMD_LEFT_BACK               0x46U       /* APP arrow lower-left: backward + left turn. */
#define BT_CMD_LEFT                    0x47U       /* APP arrow left: turn left in place. */
#define BT_CMD_LEFT_FRONT              0x48U       /* APP arrow upper-left: forward + left turn. */
#define BT_CMD_GRAVITY_MODE            0x49U       /* APP gravity-control mode key. */
#define BT_CMD_JOYSTICK_MODE           0x4AU       /* APP joystick-control mode key. */
#define BT_CMD_BUTTON_MODE             0x4BU       /* APP button-control mode key. */
#define BT_CMD_SPEED_UP                0x58U       /* APP speed-up key. */
#define BT_CMD_SPEED_DOWN              0x59U       /* APP speed-down key. */
#define BT_CMD_STOP                    0x5AU       /* APP stop/brake command used by WHEELTEC firmware. */

#define BT_CMD_TIMEOUT_MS              500U        /* Clear target if no BT command is received for this time. */
#define BT_SPEED_STEP_COUNTS           25          /* Speed target increment for each speed-up/down command. */
#define BT_SPEED_MIN_COUNTS            80          /* Minimum forward/back target, in encoder-count-sum units. */
#define BT_SPEED_MAX_COUNTS            500         /* Maximum forward/back target; larger means faster remote drive. */
#define BT_SPEED_DEFAULT_COUNTS        300         /* Default forward/back target after power-on. */
#define BT_TURN_STEP_PWM               70          /* Turn target increment for each speed-up/down command. */
#define BT_TURN_MIN_PWM                180         /* Minimum turn differential PWM target. */
#define BT_TURN_MAX_PWM                800         /* Maximum turn differential PWM target; larger means faster turning. */
#define BT_TURN_DEFAULT_PWM            380         /* Default turn differential PWM target after power-on. */

#define BT_TARGET_SPEED_SIGN           1           /* Change to -1 if forward/backward direction is reversed. */
#define BT_TARGET_TURN_SIGN            -1           /* Change to -1 if left/right direction is reversed. */
/* USER CODE END PD */

/* USER CODE BEGIN Variables */
static volatile uint32_t app_imu_data_ready_sequence;
static volatile int16_t app_bt_target_speed;
static volatile int16_t app_bt_target_turn;
static volatile int16_t app_bt_speed_level = BT_SPEED_DEFAULT_COUNTS;
static volatile int16_t app_bt_turn_level = BT_TURN_DEFAULT_PWM;
static volatile uint8_t app_bt_mode = BT_CMD_JOYSTICK_MODE;
static volatile uint32_t app_bt_last_cmd_ms;
static volatile uint32_t app_bt_rx_count;
static volatile uint8_t app_bt_last_byte;
typedef enum
{
  OBSTACLE_CLEAR = 0,
  OBSTACLE_CONFIRM,
  OBSTACLE_BRAKE,
  OBSTACLE_TURN_RIGHT_OUT,
  OBSTACLE_SHIFT_RIGHT,
  OBSTACLE_TURN_LEFT_FORWARD,
  OBSTACLE_PASS_FORWARD,
  OBSTACLE_TURN_LEFT_RETURN,
  OBSTACLE_RETURN_TO_LINE,
  OBSTACLE_SEARCH_LINE,
  OBSTACLE_REACQUIRE_RIGHT
} ObstacleState_t;
static volatile ObstacleState_t app_obstacle_state;
static volatile uint8_t app_obstacle_hits;
static volatile uint32_t app_obstacle_since_ms;
static volatile uint32_t app_obstacle_rearm_ms;
static volatile uint8_t app_obstacle_reacquire_hits;
static volatile float app_obstacle_turn_progress_deg;
static volatile int8_t app_obstacle_search_direction;
static volatile uint8_t app_obstacle_approach_slowdown;
static volatile uint8_t app_obstacle_ready;
static volatile uint8_t app_obstacle_clear_hits;
static volatile int16_t app_speed_command;
/* USER CODE END Variables */

/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask", .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for TelemetryQueue */
osMessageQueueId_t TelemetryQueueHandle;
const osMessageQueueAttr_t TelemetryQueue_attributes = {
  .name = "TelemetryQueue"
};

/* USER CODE BEGIN FunctionPrototypes */
static void App_SendText(const char *text);
static void App_SendRtosStats(void);
static void App_SendBanner(void);
static void App_SendControlConfig(void);
static void App_SetStoppedOutput(BalanceOutput_t *output,
                                 BalanceState_t state);
static void App_SetLineStoppedOutput(LineFollowerOutput_t *output);
static void App_QueueTelemetry(uint32_t now_ms,
                               uint16_t loop_dt_ms,
                               uint32_t data_ready_missed,
                               const BalanceSample_t *sample,
                               const MotorTelemetry_t *telemetry,
                               const BalanceOutput_t *output,
                               const LineFollowerOutput_t *line);
static void App_SendTelemetry(const AppTelemetryFrame_t *frame);
static void App_FormatFloat(char *out, size_t out_size, float value,
                            uint8_t decimals);
static int32_t App_RoundFloat(float value);
static const char *App_StateText(BalanceState_t state);
static void App_BluetoothPoll(uint32_t now_ms);
static void App_BluetoothParseByte(uint8_t byte, uint32_t now_ms);
static void App_BluetoothClearTarget(void);
static int16_t App_BluetoothClampInt16(int16_t value, int16_t min_value,
                                       int16_t max_value);
static int16_t App_BluetoothGetTargetSpeed(void);
static int16_t App_BluetoothGetTargetTurn(void);
static uint8_t App_ReadLineSensorLevels(void);
static int16_t App_SlewSpeed(int16_t current, int16_t target,
                             int16_t accel_step, int16_t decel_step);
static void App_ResetObstacleState(void);
static void App_ObstacleStartState(ObstacleState_t state, uint32_t now_ms);
static uint32_t App_ObstacleElapsed(uint32_t now_ms);
static void App_ObstacleAccumulateTurn(const BalanceSample_t *sample,
                                       uint16_t dt_ms);
static uint8_t App_ObstacleTurnComplete(uint32_t now_ms, float target_deg);
static uint8_t App_ObstacleLineSeen(uint8_t filtered_bitmap);
static int16_t App_ObstacleApproachSpeed(int16_t current_speed,
                                          uint16_t distance_mm);
/* USER CODE END FunctionPrototypes */

static void App_ResetObstacleState(void)
{
  app_obstacle_state = OBSTACLE_CLEAR;
  app_obstacle_hits = 0U;
  app_obstacle_since_ms = 0U;
  app_obstacle_rearm_ms = 0U;
  app_obstacle_reacquire_hits = 0U;
  app_obstacle_turn_progress_deg = 0.0f;
  app_obstacle_search_direction = 1;
  app_obstacle_approach_slowdown = 0U;
  app_obstacle_ready = 0U;
  app_obstacle_clear_hits = 0U;
}

static void App_ObstacleStartState(ObstacleState_t state, uint32_t now_ms)
{
  app_obstacle_state = state;
  app_obstacle_since_ms = now_ms;
  app_obstacle_turn_progress_deg = 0.0f;
}

static uint32_t App_ObstacleElapsed(uint32_t now_ms)
{
  return now_ms - app_obstacle_since_ms;
}

static void App_ObstacleAccumulateTurn(const BalanceSample_t *sample,
                                       uint16_t dt_ms)
{
  float yaw_rate;

  if (sample == NULL)
  {
    return;
  }
  yaw_rate = sample->gyro_yaw_dps;
  if (yaw_rate < 0.0f)
  {
    yaw_rate = -yaw_rate;
  }
  app_obstacle_turn_progress_deg += yaw_rate * ((float)dt_ms / 1000.0f);
}

static uint8_t App_ObstacleTurnComplete(uint32_t now_ms, float target_deg)
{
  return (uint8_t)((app_obstacle_turn_progress_deg >= target_deg) ||
                   (App_ObstacleElapsed(now_ms) >=
                    OBSTACLE_AVOIDANCE_TURN_TIMEOUT_MS));
}

static uint8_t App_ObstacleLineSeen(uint8_t filtered_bitmap)
{
  return (uint8_t)((filtered_bitmap != 0U) &&
                   (filtered_bitmap != LINE_SENSOR_MASK));
}

static int16_t App_ObstacleApproachSpeed(int16_t current_speed,
                                          uint16_t distance_mm)
{
  int32_t speed = current_speed;
  int32_t span = (int32_t)OBSTACLE_AVOIDANCE_SLOWDOWN_DISTANCE_MM -
                 (int32_t)OBSTACLE_AVOIDANCE_TRIGGER_DISTANCE_MM;
  int32_t from_trigger;

  if (speed < OBSTACLE_AVOIDANCE_SLOWDOWN_MIN_SPEED_MM_S)
  {
    return (int16_t)speed;
  }
  if ((distance_mm >= OBSTACLE_AVOIDANCE_SLOWDOWN_DISTANCE_MM) ||
      (span <= 0))
  {
    return (int16_t)speed;
  }
  from_trigger = (int32_t)distance_mm -
                 (int32_t)OBSTACLE_AVOIDANCE_TRIGGER_DISTANCE_MM;
  if (from_trigger < 0)
  {
    from_trigger = 0;
  }
  speed = OBSTACLE_AVOIDANCE_SLOWDOWN_MIN_SPEED_MM_S +
          ((speed - OBSTACLE_AVOIDANCE_SLOWDOWN_MIN_SPEED_MM_S) *
           from_trigger) / span;
  return (int16_t)speed;
}

void StartControlTask(void *argument);
void StartTelemetryTask(void *argument);

void MX_FREERTOS_Init(void);

/**
 * @brief FreeRTOS 初始化入口。
 *
 * 创建遥测消息队列、控制任务和串口遥测任务，
 * 同时初始化蓝牙遥控的默认速度/转向等级。
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  app_imu_data_ready_sequence = 0U;
  App_BluetoothClearTarget();
  app_bt_speed_level = BT_SPEED_DEFAULT_COUNTS;
  app_bt_turn_level = BT_TURN_DEFAULT_PWM;
  app_bt_mode = BT_CMD_JOYSTICK_MODE;
  app_bt_last_cmd_ms = 0U;
  app_bt_rx_count = 0U;
  app_bt_last_byte = 0U;
  app_obstacle_state = OBSTACLE_CLEAR;
  app_obstacle_hits = 0U;
  app_obstacle_since_ms = 0U;
  app_obstacle_rearm_ms = 0U;
  app_obstacle_reacquire_hits = 0U;
  app_obstacle_turn_progress_deg = 0.0f;
  app_obstacle_search_direction = 1;
  app_obstacle_approach_slowdown = 0U;
  app_obstacle_ready = 0U;
  app_obstacle_clear_hits = 0U;
  app_speed_command = 0;
  Ultrasonic_Init();
  /* USER CODE END Init */

  /* creation of TelemetryQueue */
  TelemetryQueueHandle = osMessageQueueNew(4, sizeof(AppTelemetryFrame_t),
                                            &TelemetryQueue_attributes);
  if (TelemetryQueueHandle == NULL)
  {
    App_SendText("#balance,error,telemetry_queue_create_failed\r\n");
  }

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL,
                                  &ControlTask_attributes);
  if (ControlTaskHandle == NULL)
  {
    App_SendText("#balance,error,control_task_create_failed\r\n");
  }

  /* creation of TelemetryTask */
  TelemetryTaskHandle = NULL;
  if (TelemetryQueueHandle != NULL)
  {
    TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL,
                                      &TelemetryTask_attributes);
  }
  if (TelemetryTaskHandle == NULL)
  {
    App_SendText("#balance,error,telemetry_task_create_failed\r\n");
  }

  UltrasonicTaskHandle = osThreadNew(Ultrasonic_Task, NULL,
                                     &UltrasonicTask_attributes);
  if (UltrasonicTaskHandle == NULL)
  {
    App_SendText("#ultrasonic,error,task_create_failed\r\n");
  }

  /* USER CODE BEGIN RTOS_THREADS */
  App_SendRtosStats();
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief PB9 MPU6050 data-ready synchronized balance control task.
  * @param argument Not used.
  */
/* USER CODE END Header_StartControlTask */
/**
 * @brief 平衡车主控制任务。
 *
 * 该任务负责整车闭环控制：初始化电机和 MPU6050，
 * 等待 PB9 数据就绪中断，同步读取 IMU 和编码器，
 * 调用 BalanceControl_Update() 计算左右轮 PWM，
 * 并处理蓝牙遥控目标与安全保护。
 */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  BalanceController_t controller;
  LineFollower_t line_follower;
  LineFollowerOutput_t line_output;
  BalanceSample_t sample;
  BalanceOutput_t output;
  const MotorTelemetry_t *telemetry;
  uint32_t current_sequence;
  uint32_t last_sequence;
  uint32_t last_sample_ms;
  uint32_t last_control_ms = 0U;
  uint32_t last_telemetry_ms = 0U;
  uint32_t data_ready_missed = 0U;
  uint32_t obstacle_last_ultrasonic_sequence = 0U;
  uint16_t loop_dt_ms = 0U;
  uint8_t control_divider = 0U;

  (void)argument;
  (void)memset(&sample, 0, sizeof(sample));
  (void)memset(&line_output, 0, sizeof(line_output));
  App_SetStoppedOutput(&output, BALANCE_STATE_TIMEOUT);
  App_SetLineStoppedOutput(&line_output);
  Motor_Init();
  BalanceControl_Init(&controller);
  LineFollower_Init(&line_follower);
  App_SendBanner();
  App_SendControlConfig();
  OledStatus_Init();

  if (Imu_Init() == 0U)
  {
    App_SetStoppedOutput(&output, BALANCE_STATE_IMU_ERROR);
    Motor_Brake();
    App_SendText("#balance,error,imu_init_failed\r\n");
    for (;;)
    {
      telemetry = Motor_GetTelemetry();
      sample.timestamp_ms = HAL_GetTick();
      sample.valid = 0U;
      App_QueueTelemetry(sample.timestamp_ms, 0U, data_ready_missed,
                         &sample, telemetry, &output, &line_output);
      osDelay(BALANCE_TELEMETRY_PERIOD_MS);
    }
  }

  App_SendText("#balance,imu_init_ok\r\n");
  last_sequence = app_imu_data_ready_sequence;
  (void)osThreadFlagsClear(IMU_DATA_READY_THREAD_FLAG);
  last_sample_ms = HAL_GetTick();

  for (;;)
  {
    uint32_t flags;
    uint32_t now_ms;
    uint32_t sequence_delta;
    uint32_t sample_dt_ms;
    uint8_t timing_fault = 0U;

    flags = osThreadFlagsWait(IMU_DATA_READY_THREAD_FLAG, osFlagsWaitAny,
                              BALANCE_SAMPLE_TIMEOUT_MS);
    now_ms = HAL_GetTick();
    App_BluetoothPoll(now_ms);

    if ((flags & osFlagsError) != 0U)
    {
      app_obstacle_state = OBSTACLE_CLEAR;
      app_obstacle_hits = 0U;
      app_obstacle_rearm_ms = 0U;
      app_obstacle_reacquire_hits = 0U;
      app_obstacle_approach_slowdown = 0U;
      app_obstacle_turn_progress_deg = 0.0f;
      app_obstacle_search_direction = 1;
      app_speed_command = 0;
      BalanceControl_Reset(&controller);
      LineFollower_Init(&line_follower);
      App_SetLineStoppedOutput(&line_output);
      Motor_Brake();
      App_SetStoppedOutput(&output, BALANCE_STATE_TIMEOUT);
      control_divider = 0U;
      last_control_ms = 0U;
      loop_dt_ms = 0U;
      telemetry = Motor_GetTelemetry();
      if ((now_ms - last_telemetry_ms) >= BALANCE_TELEMETRY_PERIOD_MS)
      {
        App_QueueTelemetry(now_ms, loop_dt_ms, data_ready_missed,
                           &sample, telemetry, &output, &line_output);
        last_telemetry_ms = now_ms;
      }
      continue;
    }

    current_sequence = app_imu_data_ready_sequence;
    sequence_delta = current_sequence - last_sequence;
    last_sequence = current_sequence;
    if (sequence_delta != 1U)
    {
      data_ready_missed += (sequence_delta > 1U) ?
                           (sequence_delta - 1U) : 1U;
      timing_fault = 1U;
    }

    sample_dt_ms = now_ms - last_sample_ms;
    last_sample_ms = now_ms;
    (void)Imu_UpdateKalman5ms(&sample);
    sample.data_ready_sequence = current_sequence;
    sample.sample_dt_ms = (sample_dt_ms > 65535U) ? 65535U :
                          (uint16_t)sample_dt_ms;
    Encoder_Update5ms();
    telemetry = Motor_GetTelemetry();

    if (sample.valid == 0U)
    {
      app_obstacle_state = OBSTACLE_CLEAR;
      app_obstacle_hits = 0U;
      app_obstacle_rearm_ms = 0U;
      app_obstacle_reacquire_hits = 0U;
      app_obstacle_approach_slowdown = 0U;
      app_obstacle_turn_progress_deg = 0.0f;
      app_obstacle_search_direction = 1;
      app_speed_command = 0;
      BalanceControl_Reset(&controller);
      LineFollower_Init(&line_follower);
      App_SetLineStoppedOutput(&line_output);
      Motor_Brake();
      App_SetStoppedOutput(&output, BALANCE_STATE_IMU_ERROR);
      control_divider = 0U;
      last_control_ms = 0U;
      loop_dt_ms = 0U;
    }
    else if (timing_fault != 0U)
    {
      app_obstacle_state = OBSTACLE_CLEAR;
      app_obstacle_hits = 0U;
      app_obstacle_rearm_ms = 0U;
      app_obstacle_reacquire_hits = 0U;
      app_obstacle_approach_slowdown = 0U;
      app_obstacle_turn_progress_deg = 0.0f;
      app_obstacle_search_direction = 1;
      app_speed_command = 0;
      BalanceControl_Reset(&controller);
      LineFollower_Init(&line_follower);
      App_SetLineStoppedOutput(&line_output);
      Motor_Brake();
      App_SetStoppedOutput(&output, BALANCE_STATE_OVERRUN);
      control_divider = 0U;
      last_control_ms = 0U;
      loop_dt_ms = 0U;
    }
    else
    {
      control_divider++;
      if (control_divider >= CONTROL_IMU_SAMPLES_PER_STEP)
      {
        uint32_t control_dt_ms;

        control_divider = 0U;
        if (last_control_ms != 0U)
        {
          control_dt_ms = now_ms - last_control_ms;
          loop_dt_ms = (control_dt_ms > 65535U) ? 65535U :
                       (uint16_t)control_dt_ms;
        }
        last_control_ms = now_ms;

        /* 避障期间仍更新传感器滤波，但冻结八字路线内部状态：
         * 绕瓶过程中压到黑线不能消耗右、左、左、右路口指令，
         * 也不能提前触发终点。 */
        {
          LineFollower_t follower_before = line_follower;
          line_output = LineFollower_Update(&line_follower,
                                             App_ReadLineSensorLevels(),
                                             now_ms);
          if (app_obstacle_state != OBSTACLE_CLEAR)
          {
            LineFollower_t follower_after = line_follower;
            line_follower = follower_before;
            (void)memcpy(line_follower.sample_history,
                         follower_after.sample_history,
                         sizeof(line_follower.sample_history));
            line_follower.history_index = follower_after.history_index;
            line_output.route_index = follower_before.route_index;
            line_output.route_total = 4U;
          }
        }
        {
          uint16_t distance = Ultrasonic_GetDistanceMm();
          uint8_t distance_valid = Ultrasonic_IsValid();
          uint32_t ultrasonic_sequence = Ultrasonic_GetSequence();
          uint8_t new_ultrasonic_sample =
            (uint8_t)(ultrasonic_sequence != obstacle_last_ultrasonic_sequence);
          obstacle_last_ultrasonic_sequence = ultrasonic_sequence;
          /* 上电/重新回线后先确认前方连续空旷，滤掉 HC-SR04 的启动残留
           * 和一次性跳变；只有 ready=1 才允许第一圈触发避障。 */
          if ((app_obstacle_state == OBSTACLE_CLEAR) &&
              ((int32_t)(now_ms - app_obstacle_rearm_ms) >= 0) &&
              (app_obstacle_ready == 0U))
          {
            if ((distance_valid != 0U) &&
                (distance >= OBSTACLE_AVOIDANCE_CLEAR_DISTANCE_MM))
            {
              if (app_obstacle_clear_hits < OBSTACLE_AVOIDANCE_CLEAR_CONFIRM_COUNT)
              {
                app_obstacle_clear_hits++;
              }
              if (app_obstacle_clear_hits >= OBSTACLE_AVOIDANCE_CLEAR_CONFIRM_COUNT)
              {
                app_obstacle_ready = 1U;
              }
            }
            else if ((distance_valid != 0U) &&
                     (distance < OBSTACLE_AVOIDANCE_TRIGGER_DISTANCE_MM))
            {
              app_obstacle_clear_hits = 0U;
            }
          }
          /* 1111 代表宽横线/T 口，不能直接当作普通回线；否则会在
           * 障碍物附近偷吃八字路线指令。真正重新捕线只接受部分黑线。 */
          /* 只在正常 TRACK 且误差较小的直道触发；圆弧和路口不参与避障。
           * 近距离回波使用新的确认状态机，避免超声波单次跳变误动作。 */
          if ((app_obstacle_state == OBSTACLE_CLEAR) &&
              (app_obstacle_ready != 0U) &&
              ((int32_t)(now_ms - app_obstacle_rearm_ms) >= 0) &&
              (new_ultrasonic_sample != 0U) &&
              (line_follower.route_state == LINE_FOLLOWER_TRACK) &&
              (line_follower.cross_active == 0U) &&
              (line_output.state == LINE_FOLLOWER_TRACK) &&
              (line_output.position == LINE_POSITION_CENTER) &&
              (line_output.error <= OBSTACLE_AVOIDANCE_STRAIGHT_MAX_ERROR) &&
              (line_output.error >= -OBSTACLE_AVOIDANCE_STRAIGHT_MAX_ERROR) &&
              (distance_valid != 0U) &&
              (distance <= OBSTACLE_AVOIDANCE_TRIGGER_DISTANCE_MM))
          {
            App_ObstacleStartState(OBSTACLE_CONFIRM, now_ms);
            app_obstacle_hits = 1U;
            app_obstacle_reacquire_hits = 0U;
            app_obstacle_approach_slowdown = 0U;
            /* 本次近距离回波已经作为第 1 个样本计入，确认分支从下一帧
             * 超声波结果开始计数，避免同一帧被重复计算成 2 个样本。 */
            new_ultrasonic_sample = 0U;
          }
          else if ((app_obstacle_state == OBSTACLE_CLEAR) &&
                   (app_obstacle_ready != 0U) &&
                   ((int32_t)(now_ms - app_obstacle_rearm_ms) >= 0) &&
                   (line_follower.route_state == LINE_FOLLOWER_TRACK) &&
                   (line_follower.cross_active == 0U) &&
                   (line_output.state == LINE_FOLLOWER_TRACK) &&
                   (line_output.position == LINE_POSITION_CENTER) &&
                   (line_output.error <= OBSTACLE_AVOIDANCE_STRAIGHT_MAX_ERROR) &&
                   (line_output.error >= -OBSTACLE_AVOIDANCE_STRAIGHT_MAX_ERROR) &&
                   (distance_valid != 0U) &&
                   (distance < OBSTACLE_AVOIDANCE_SLOWDOWN_DISTANCE_MM))
          {
            /* 450 -> 200 mm 连续预减速，避免到 200 mm 时仍以巡航速度
             * 冲到瓶子前；本车 target_speed 与速度环标定值同量级。 */
            line_output.target_speed = App_ObstacleApproachSpeed(
              line_output.target_speed, distance);
            app_obstacle_approach_slowdown = 1U;
          }
          else if (app_obstacle_state == OBSTACLE_CLEAR)
          {
            /* 无效回波只跳过本次采样，不立刻清零：HC-SR04 在一次测量
             * 期间会短暂把 valid 拉低，立即清零会导致“永远凑不齐 3 次”。
             * 只有线路离开直道，或下一次有效距离明确变远时才取消确认。 */
            if ((line_output.state != LINE_FOLLOWER_TRACK) ||
                (line_output.position != LINE_POSITION_CENTER) ||
                (line_output.error > OBSTACLE_AVOIDANCE_STRAIGHT_MAX_ERROR) ||
                (line_output.error < -OBSTACLE_AVOIDANCE_STRAIGHT_MAX_ERROR) ||
                 ((distance_valid != 0U) &&
                  (distance > (OBSTACLE_AVOIDANCE_TRIGGER_DISTANCE_MM + 40U))))
            {
              app_obstacle_hits = 0U;
              app_obstacle_approach_slowdown = 0U;
            }
          }

          /* 记录本周期巡线给出的目标速度；当处于预减速窗口时，
           * 在最终速度合成处再次按当前距离限速，保证不会被普通巡线
           * 的速度斜坡覆盖。 */
          if ((app_obstacle_state == OBSTACLE_CLEAR) &&
              (app_obstacle_approach_slowdown != 0U) &&
              (distance_valid != 0U))
          {
            line_output.target_speed = App_ObstacleApproachSpeed(
              line_output.target_speed, distance);
          }

          if (app_obstacle_state == OBSTACLE_CONFIRM)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            line_output.target_speed = OBSTACLE_AVOIDANCE_CONFIRM_CREEP_SPEED_MM_S;
            line_output.target_turn = 0;
            if ((new_ultrasonic_sample != 0U) &&
                ((distance_valid != 0U) &&
                 (distance <= OBSTACLE_AVOIDANCE_TRIGGER_DISTANCE_MM)))
            {
              if (app_obstacle_hits < OBSTACLE_AVOIDANCE_CONFIRM_SAMPLES)
              {
                app_obstacle_hits++;
              }
              if (app_obstacle_hits >= OBSTACLE_AVOIDANCE_CONFIRM_SAMPLES)
              {
                App_ObstacleStartState(OBSTACLE_BRAKE, now_ms);
                app_obstacle_hits = 0U;
                app_speed_command = 0;
                BalanceControl_Reset(&controller);
              }
            }
            else if ((distance_valid != 0U) &&
                     (distance > OBSTACLE_AVOIDANCE_TRIGGER_DISTANCE_MM))
            {
              App_ResetObstacleState();
            }
            else if (App_ObstacleElapsed(now_ms) >=
                     OBSTACLE_AVOIDANCE_CONFIRM_TIMEOUT_MS)
            {
              App_ResetObstacleState();
            }
          }
          else if (app_obstacle_state == OBSTACLE_BRAKE)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            line_output.target_speed = 0;
            line_output.target_turn = 0;
            if ((now_ms - app_obstacle_since_ms) >=
                OBSTACLE_AVOIDANCE_BRAKE_SETTLE_MS)
            {
              App_ObstacleStartState(OBSTACLE_TURN_RIGHT_OUT, now_ms);
            }
          }
          else if (app_obstacle_state == OBSTACLE_TURN_RIGHT_OUT)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            line_output.target_speed = OBSTACLE_AVOIDANCE_TURN_FORWARD_SPEED_MM_S;
            line_output.target_turn = OBSTACLE_AVOIDANCE_RIGHT_TURN_PWM;
            App_ObstacleAccumulateTurn(&sample, loop_dt_ms);
            if (App_ObstacleTurnComplete(now_ms,
                                         OBSTACLE_AVOIDANCE_RIGHT_OUT_TURN_DEG) != 0U)
            {
              App_ObstacleStartState(OBSTACLE_SHIFT_RIGHT, now_ms);
            }
          }
          else if (app_obstacle_state == OBSTACLE_SHIFT_RIGHT)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            line_output.target_speed = OBSTACLE_AVOIDANCE_BYPASS_SPEED_MM_S;
            line_output.target_turn = 0;
            if ((now_ms - app_obstacle_since_ms) >=
                OBSTACLE_AVOIDANCE_SHIFT_OUT_MS)
            {
              App_ObstacleStartState(OBSTACLE_TURN_LEFT_FORWARD, now_ms);
            }
          }
          else if (app_obstacle_state == OBSTACLE_TURN_LEFT_FORWARD)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            line_output.target_speed = OBSTACLE_AVOIDANCE_TURN_FORWARD_SPEED_MM_S;
            line_output.target_turn = OBSTACLE_AVOIDANCE_LEFT_TURN_PWM;
            App_ObstacleAccumulateTurn(&sample, loop_dt_ms);
            if (App_ObstacleTurnComplete(now_ms,
                                         OBSTACLE_AVOIDANCE_TURN_TARGET_DEG) != 0U)
            {
              App_ObstacleStartState(OBSTACLE_PASS_FORWARD, now_ms);
            }
          }
          else if (app_obstacle_state == OBSTACLE_PASS_FORWARD)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            line_output.target_speed = OBSTACLE_AVOIDANCE_BYPASS_SPEED_MM_S;
            line_output.target_turn = 0;
            if ((now_ms - app_obstacle_since_ms) >=
                OBSTACLE_AVOIDANCE_PASS_FORWARD_MS)
            {
              App_ObstacleStartState(OBSTACLE_TURN_LEFT_RETURN, now_ms);
            }
          }
          else if (app_obstacle_state == OBSTACLE_TURN_LEFT_RETURN)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            line_output.target_speed = OBSTACLE_AVOIDANCE_TURN_FORWARD_SPEED_MM_S;
            line_output.target_turn = OBSTACLE_AVOIDANCE_LEFT_TURN_PWM;
            App_ObstacleAccumulateTurn(&sample, loop_dt_ms);
            if (App_ObstacleTurnComplete(now_ms,
                                         OBSTACLE_AVOIDANCE_RETURN_TURN_DEG) != 0U)
            {
              App_ObstacleStartState(OBSTACLE_RETURN_TO_LINE, now_ms);
              app_obstacle_reacquire_hits = 0U;
            }
          }
          else if (app_obstacle_state == OBSTACLE_RETURN_TO_LINE)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            /* 最后一次左转完成后，车辆理论上朝原黑线前进；但实际
             * 轮胎打滑、左右轮速度差会让车头与黑线平行并横向偏开。
             * 保留一个很小的左偏置，使回线轨迹成为缓弧，而不是
             * “转完后直着走”卡在黑线外。检测到有效黑线后，下面的
             * 连续确认仍会立即交回八字巡线。 */
            line_output.target_speed = OBSTACLE_AVOIDANCE_RETURN_SPEED_MM_S;
            line_output.target_turn = OBSTACLE_AVOIDANCE_RETURN_SEEK_TURN_PWM;
            if (App_ObstacleLineSeen(line_output.filtered_bitmap) != 0U)
            {
              if (app_obstacle_reacquire_hits <
                  OBSTACLE_AVOIDANCE_LINE_CONFIRM_CYCLES)
                app_obstacle_reacquire_hits++;
              if (app_obstacle_reacquire_hits >=
                  OBSTACLE_AVOIDANCE_LINE_CONFIRM_CYCLES)
              {
                line_follower.line_lost_active = 0U;
                line_follower.cross_active = 2U;
                line_follower.last_route_ms = now_ms;
                app_obstacle_state = OBSTACLE_CLEAR;
                app_obstacle_rearm_ms = now_ms + OBSTACLE_AVOIDANCE_REARM_DELAY_MS;
                app_obstacle_reacquire_hits = 0U;
                app_obstacle_approach_slowdown = 0U;
                app_obstacle_ready = 0U;
                app_obstacle_clear_hits = 0U;
              }
            }
            else if ((now_ms - app_obstacle_since_ms) >=
                     OBSTACLE_AVOIDANCE_RETURN_TIMEOUT_MS)
            {
              App_ObstacleStartState(OBSTACLE_SEARCH_LINE, now_ms);
              app_obstacle_search_direction = 1;
            }
          }
          else if (app_obstacle_state == OBSTACLE_SEARCH_LINE)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            line_output.target_speed = OBSTACLE_AVOIDANCE_SEARCH_SPEED_MM_S;
            line_output.target_turn = (app_obstacle_search_direction > 0) ?
                                      OBSTACLE_AVOIDANCE_SEARCH_RIGHT_PWM :
                                      OBSTACLE_AVOIDANCE_TURN_PWM_MAG;
            if (App_ObstacleLineSeen(line_output.filtered_bitmap) != 0U)
            {
              App_ObstacleStartState(OBSTACLE_REACQUIRE_RIGHT, now_ms);
              app_obstacle_reacquire_hits = 0U;
            }
            else if ((now_ms - app_obstacle_since_ms) >=
                     OBSTACLE_AVOIDANCE_SEARCH_SWEEP_MS)
            {
              app_obstacle_since_ms = now_ms;
              app_obstacle_search_direction =
                (int8_t)-app_obstacle_search_direction;
            }
          }
          else if (app_obstacle_state == OBSTACLE_REACQUIRE_RIGHT)
          {
            line_output.state = LINE_FOLLOWER_TRACK;
            line_output.target_speed = OBSTACLE_AVOIDANCE_REACQUIRE_SPEED_MM_S;
            line_output.target_turn = OBSTACLE_AVOIDANCE_SEARCH_RIGHT_PWM;
            if ((now_ms - app_obstacle_since_ms) >=
                OBSTACLE_AVOIDANCE_REACQUIRE_RIGHT_MS)
            {
              app_obstacle_state = OBSTACLE_CLEAR;
              app_obstacle_rearm_ms = now_ms + OBSTACLE_AVOIDANCE_REARM_DELAY_MS;
              app_obstacle_approach_slowdown = 0U;
              app_obstacle_ready = 0U;
              app_obstacle_clear_hits = 0U;
            }
          }
        }
        /* 普通巡线的 target_speed 已在 LineFollower_Update() 内完成斜坡。
         * 避障阶段再经过一层小斜坡，避免速度瞬变；转向/搜索阶段直接
         * 清零速度，防止平衡环的旧速度积分把车辆拉回障碍物。 */
          if ((app_obstacle_state == OBSTACLE_BRAKE) ||
            (app_obstacle_state == OBSTACLE_SEARCH_LINE) ||
            ((app_obstacle_state == OBSTACLE_CLEAR) &&
             (line_output.state == LINE_FOLLOWER_LOST)))
        {
          app_speed_command = 0;
        }
        else if (app_obstacle_state == OBSTACLE_CLEAR)
        {
          app_speed_command = line_output.target_speed;
        }
        else
        {
          app_speed_command = App_SlewSpeed(app_speed_command,
                                             line_output.target_speed,
                                             14, 28);
        }
        line_output.target_speed = app_speed_command;
        output = BalanceControl_Update(&controller, &sample,
                                       telemetry->left_delta,
                                       telemetry->right_delta,
                                       line_output.target_speed,
                                       line_output.target_turn);
        if (output.state == BALANCE_STATE_RUNNING)
        {
          Motor_SetPWM(output.left_pwm, output.right_pwm);
        }
        else
        {
          app_obstacle_state = OBSTACLE_CLEAR;
          app_obstacle_hits = 0U;
          app_obstacle_rearm_ms = now_ms + OBSTACLE_AVOIDANCE_REARM_DELAY_MS;
          app_obstacle_reacquire_hits = 0U;
          app_obstacle_approach_slowdown = 0U;
          app_obstacle_turn_progress_deg = 0.0f;
          app_obstacle_search_direction = 1;
          LineFollower_Init(&line_follower);
          line_output.target_speed = 0;
          line_output.target_turn = 0;
          line_output.state = LINE_FOLLOWER_LOST;
          Motor_Brake();
        }
        telemetry = Motor_GetTelemetry();
      }
    }

    if ((now_ms - last_telemetry_ms) >= BALANCE_TELEMETRY_PERIOD_MS)
    {
      App_QueueTelemetry(now_ms, loop_dt_ms, data_ready_missed,
                         &sample, telemetry, &output, &line_output);
      last_telemetry_ms = now_ms;
    }

  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
  * @brief Low-priority asynchronous UART telemetry task.
  * @param argument Not used.
  */
/* USER CODE END Header_StartTelemetryTask */
/**
 * @brief 串口遥测输出任务。
 *
 * 从 TelemetryQueue 中取出控制任务打包好的数据帧，
 * 按 CSV 格式通过 USART1 打印到上位机，方便观察曲线和调参。
 */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  AppTelemetryFrame_t frame;
  uint32_t last_oled_ms = 0U;

  (void)argument;
  (void)memset(&frame, 0, sizeof(frame));
  for (;;)
  {
    if (osMessageQueueGet(TelemetryQueueHandle, &frame, NULL,
                          osWaitForever) == osOK)
    {
      App_SendTelemetry(&frame);
      if ((frame.now_ms - last_oled_ms) >= OLED_STATUS_PERIOD_MS)
      {
        OledStatus_Update(&frame.sample, &frame.motor,
                          &frame.output, &frame.line);
        last_oled_ms = frame.now_ms;
      }
    }
  }
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Application */
/**
 * @brief GPIO 外部中断回调。
 *
 * PB9 接 MPU6050 的数据就绪中断；每次 MPU6050 有新数据时，
 * 该回调给控制任务发送 FreeRTOS 线程标志，触发一次 IMU 更新。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT_Pin)
  {
    app_imu_data_ready_sequence++;
    if (ControlTaskHandle != NULL)
    {
      (void)osThreadFlagsSet(ControlTaskHandle, IMU_DATA_READY_THREAD_FLAG);
    }
  }
}

/**
 * @brief 通过 USART1 发送调试文本。
 *
 * 所有上位机日志、CSV 表头和运行状态都通过这个函数输出。
 */
static void App_SendText(const char *text)
{
  if (text == NULL)
  {
    return;
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text,
                          (uint16_t)strlen(text), 100U);
}

/**
 * @brief 输出 FreeRTOS 剩余堆空间。
 *
 * 用于确认系统动态内存是否足够，避免队列或任务创建失败。
 */
static void App_SendRtosStats(void)
{
  char line[72];

  (void)snprintf(line, sizeof(line),
                 "#rtos,free_heap_after_init=%lu\r\n",
                 (unsigned long)xPortGetFreeHeapSize());
  App_SendText(line);
}

/**
 * @brief 输出系统启动说明和串口 CSV 表头。
 *
 * 包括工程名称、控制周期、硬件映射、蓝牙协议和遥测字段名称，
 * 方便打开串口后直接知道每一列数据的含义。
 */
static void App_SendBanner(void)
{
  App_SendText("#balance,ByCarStable,wheeltec_cascade_equivalent\r\n");
  App_SendText("#rtos,timing=PB9_data_ready,imu_period_ms=5,control_period_ms=10\r\n");
  App_SendText("#map,pwm_l=TIM3_CH1_CH2,pwm_r=TIM3_CH3_CH4,enc_l=TIM4,enc_r=TIM8,imu=PB14_PB15,imu_int=PB9\r\n");
  App_SendText("#map,debug_uart=USART1_PA9_PA10,bluetooth_uart=USART3_PB10_PB11\r\n");
  App_SendText("#map,line=PB8_PC9_PC4_PC8,active=low,oled_spi=PC14_PB5_PB4_PB3\r\n");
  App_SendText("#map,ultrasonic=echo_PA1_trig_PC15,obstacle_bypass=right\r\n");
  App_SendText("#map,pwm_columns=applied_after_deadzone\r\n");
  App_SendText("#bt,protocol=wheeltec_app,0x41=fwd,0x42=right_fwd,0x43=right,0x44=right_back,0x45=back,0x46=left_back,0x47=left,0x48=left_fwd,0x58=speed_up,0x59=speed_down\r\n");
  App_SendText("ms,pitch,gyro,enc_l,enc_r,speed_f,speed_i,balance_pwm,velocity_pwm,requested_pwm,turn_pwm,pwm_l,pwm_r,line_raw,line_filtered,line_error,target_speed,target_turn,line_state,route_index,route_total,loop_dt,drdy_missed,state,oled_status\r\n");
}

/**
 * @brief 输出当前控制参数配置。
 *
 * 打印角度环 Kp/Kd、速度环 Kp/Ki、卡尔曼参数、死区补偿、
 * 蓝牙默认速度/转向等，用于记录调参版本。
 */
static void App_SendControlConfig(void)
{
  char line[512];
  char middle[18];
  char balance_kp[18];
  char balance_kd[18];
  char velocity_kp[18];
  char velocity_ki[18];
  char q_angle[18];
  char q_bias[18];
  char r_angle[18];

  App_FormatFloat(middle, sizeof(middle), BALANCE_MIDDLE_ANGLE_DEG, 2U);
  App_FormatFloat(balance_kp, sizeof(balance_kp), BALANCE_KP, 2U);
  App_FormatFloat(balance_kd, sizeof(balance_kd), BALANCE_KD, 2U);
  App_FormatFloat(velocity_kp, sizeof(velocity_kp), BALANCE_VELOCITY_KP, 3U);
  App_FormatFloat(velocity_ki, sizeof(velocity_ki), BALANCE_VELOCITY_KI, 3U);
  App_FormatFloat(q_angle, sizeof(q_angle), IMU_KALMAN_Q_ANGLE, 3U);
  App_FormatFloat(q_bias, sizeof(q_bias), IMU_KALMAN_Q_BIAS, 3U);
  App_FormatFloat(r_angle, sizeof(r_angle), IMU_KALMAN_R_ANGLE, 3U);

  (void)snprintf(line, sizeof(line),
                 "#config,mode=%s,filter=KALMAN,middle=%s,"
                 "balance_kp=%s,balance_kd=%s,velocity_kp=%s,"
                 "velocity_ki=%s,speed_filter=0.84_0.16,"
                 "integral_limit=380000,kalman_q_angle=%s,"
                 "kalman_q_bias=%s,kalman_r_angle=%s,"
                 "sample_period_ms=5,control_period_ms=10,"
                 "pwm_map=%s,pwm_deadband=%d,pwm_deadzone_l=%d,"
                 "pwm_deadzone_r=%d,pwm_limit=%d,"
                 "line_kp=%d,line_kd=%d,line_turn_limit=%d,"
                 "line_speed_straight=%d,line_speed_curve=%d,"
                 "line_speed_search=%d,line_lost_ms=%u\r\n",
                 BALANCE_CONTROL_MODE_TEXT, middle, balance_kp, balance_kd,
                 velocity_kp, velocity_ki, q_angle, q_bias, r_angle,
                 (MOTOR_PWM_DEADZONE_ENABLE != 0U) ? "linear" : "off",
                 MOTOR_PWM_COMMAND_DEADBAND_COUNTS,
                 MOTOR_LEFT_DEADZONE_COUNTS,
                 MOTOR_RIGHT_DEADZONE_COUNTS,
                 MOTOR_PWM_LIMIT_COUNTS,
                 LINE_TURN_KP_PWM,
                 LINE_TURN_KD_PWM,
                 LINE_TURN_LIMIT_PWM,
                 LINE_SPEED_STRAIGHT_COUNTS,
                 LINE_SPEED_CURVE_COUNTS,
                 LINE_SPEED_SEARCH_COUNTS,
                 (unsigned int)LINE_LOST_TIMEOUT_MS);
  App_SendText(line);
}

/**
 * @brief 设置停止状态下的输出帧。
 *
 * 当小车倒地、IMU 超时、数据异常时使用，
 * 将 PWM 清零并记录当前状态，便于串口显示 TILT/TIMEOUT 等原因。
 */
static void App_SetStoppedOutput(BalanceOutput_t *output,
                                 BalanceState_t state)
{
  if (output == NULL)
  {
    return;
  }
  (void)memset(output, 0, sizeof(*output));
  output->state = state;
}

static void App_SetLineStoppedOutput(LineFollowerOutput_t *output)
{
  uint8_t reset_generation = 0U;

  if (output == NULL)
  {
    return;
  }
  /* 故障停车时保留诊断代号 X，避免清零后无法判断是否发生过
   * LineFollower_Init()。 */
  reset_generation = output->reset_generation;
  (void)memset(output, 0, sizeof(*output));
  output->state = LINE_FOLLOWER_LOST;
  output->position = LINE_POSITION_CENTER;
  output->reset_generation = reset_generation;
}

/**
 * @brief 将一次控制循环的数据打包进遥测队列。
 *
 * 控制任务实时性要求高，不直接慢速打印串口；
 * 而是把姿态、编码器、PWM、蓝牙状态等数据放入队列，
 * 交给遥测任务异步输出。
 */
static void App_QueueTelemetry(uint32_t now_ms,
                               uint16_t loop_dt_ms,
                               uint32_t data_ready_missed,
                               const BalanceSample_t *sample,
                               const MotorTelemetry_t *telemetry,
                               const BalanceOutput_t *output,
                               const LineFollowerOutput_t *line)
{
  AppTelemetryFrame_t frame;
  AppTelemetryFrame_t dropped_frame;

  if ((TelemetryQueueHandle == NULL) || (sample == NULL) ||
      (telemetry == NULL) || (output == NULL) || (line == NULL))
  {
    return;
  }

  frame.now_ms = now_ms;
  frame.loop_dt_ms = loop_dt_ms;
  frame.data_ready_missed = data_ready_missed;
  frame.bt_target_speed = App_BluetoothGetTargetSpeed();
  frame.bt_target_turn = App_BluetoothGetTargetTurn();
  frame.bt_rx_count = app_bt_rx_count;
  frame.bt_last_byte = app_bt_last_byte;
  frame.oled_status = OledStatus_GetStatus();
  frame.sample = *sample;
  frame.motor = *telemetry;
  frame.output = *output;
  frame.line = *line;
  if (osMessageQueuePut(TelemetryQueueHandle, &frame, 0U, 0U) != osOK)
  {
    (void)osMessageQueueGet(TelemetryQueueHandle, &dropped_frame, NULL, 0U);
    (void)osMessageQueuePut(TelemetryQueueHandle, &frame, 0U, 0U);
  }
}

/**
 * @brief 发送一帧 CSV 遥测数据。
 *
 * 将遥测结构体中的角度、角速度、编码器、速度环状态、
 * PWM 输出、蓝牙目标和运行状态格式化为一行文本。
 */
static void App_SendTelemetry(const AppTelemetryFrame_t *frame)
{
  char line[384];
  char pitch[18];
  char gyro[18];
  char speed_filtered[22];
  char speed_integral[22];

  if (frame == NULL)
  {
    return;
  }
  App_FormatFloat(pitch, sizeof(pitch), frame->sample.pitch_deg, 2U);
  App_FormatFloat(gyro, sizeof(gyro), frame->sample.gyro_pitch_dps, 2U);
  App_FormatFloat(speed_filtered, sizeof(speed_filtered),
                  frame->output.speed_filtered, 2U);
  App_FormatFloat(speed_integral, sizeof(speed_integral),
                  frame->output.speed_integral, 2U);

  (void)snprintf(line, sizeof(line),
                 "%lu,%s,%s,%d,%d,%s,%s,%ld,%ld,%ld,%d,%d,%d,%u,%u,%d,%d,%d,%s,%u,%u,%u,%lu,%s,%u\r\n",
                 (unsigned long)frame->now_ms,
                 pitch,
                 gyro,
                 frame->motor.left_delta,
                 frame->motor.right_delta,
                 speed_filtered,
                 speed_integral,
                 (long)frame->output.balance_pwm,
                 (long)frame->output.velocity_pwm,
                 (long)frame->output.requested_pwm,
                 frame->output.turn_pwm,
                 frame->motor.left_pwm,
                 frame->motor.right_pwm,
                 (unsigned int)frame->line.raw_bitmap,
                 (unsigned int)frame->line.filtered_bitmap,
                 frame->line.error,
                 frame->line.target_speed,
                 frame->line.target_turn,
                 LineFollower_StateText(frame->line.state),
                 (unsigned int)frame->line.route_index,
                 (unsigned int)frame->line.route_total,
                 (unsigned int)frame->loop_dt_ms,
                 (unsigned long)frame->data_ready_missed,
                 App_StateText(frame->output.state),
                 (unsigned int)frame->oled_status);
  App_SendText(line);
}

static uint8_t App_ReadLineSensorLevels(void)
{
  uint8_t levels = 0U;

  if (HAL_GPIO_ReadPin(LINE_LEFT_OUTER_GPIO_Port,
                       LINE_LEFT_OUTER_Pin) == GPIO_PIN_SET)
  {
    levels |= LINE_SENSOR_LEFT_OUTER_BIT;
  }
  if (HAL_GPIO_ReadPin(LINE_LEFT_INNER_GPIO_Port,
                       LINE_LEFT_INNER_Pin) == GPIO_PIN_SET)
  {
    levels |= LINE_SENSOR_LEFT_INNER_BIT;
  }
  if (HAL_GPIO_ReadPin(LINE_RIGHT_INNER_GPIO_Port,
                       LINE_RIGHT_INNER_Pin) == GPIO_PIN_SET)
  {
    levels |= LINE_SENSOR_RIGHT_INNER_BIT;
  }
  if (HAL_GPIO_ReadPin(LINE_RIGHT_OUTER_GPIO_Port,
                       LINE_RIGHT_OUTER_Pin) == GPIO_PIN_SET)
  {
    levels |= LINE_SENSOR_RIGHT_OUTER_BIT;
  }
  return levels;
}

/**
 * @brief 将浮点数格式化为固定小数位字符串。
 *
 * 为了减小嵌入式 printf 浮点输出的不确定性，
 * 这里手动完成浮点数四舍五入和小数位拼接。
 */
static void App_FormatFloat(char *out, size_t out_size, float value,
                            uint8_t decimals)
{
  int32_t scale = 1;
  int32_t scaled;
  int32_t whole;
  int32_t fraction;
  uint8_t i;

  if ((out == NULL) || (out_size == 0U))
  {
    return;
  }
  for (i = 0U; i < decimals; i++)
  {
    scale *= 10;
  }
  scaled = App_RoundFloat(value * (float)scale);
  if (scaled < 0)
  {
    whole = (-scaled) / scale;
    fraction = (-scaled) % scale;
    (void)snprintf(out, out_size, "-%ld.%0*ld",
                   (long)whole, decimals, (long)fraction);
  }
  else
  {
    whole = scaled / scale;
    fraction = scaled % scale;
    (void)snprintf(out, out_size, "%ld.%0*ld",
                   (long)whole, decimals, (long)fraction);
  }
}

/**
 * @brief 对浮点数四舍五入成整数。
 *
 * 供 App_FormatFloat() 使用，正数和负数都按常规四舍五入处理。
 */
static int32_t App_RoundFloat(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value + 0.5f);
  }
  return (int32_t)(value - 0.5f);
}

/**
 * @brief 轮询蓝牙串口并处理超时。
 *
 * 从 USART3 非阻塞读取 APP 发来的单字节命令；
 * 如果超过 BT_CMD_TIMEOUT_MS 没收到新指令，就清空目标速度和转向，
 * 防止 APP 断开后小车继续跑。
 */
static void App_BluetoothPoll(uint32_t now_ms)
{
  while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
  {
    uint8_t byte = (uint8_t)(huart3.Instance->DR & 0xFFU);
    app_bt_rx_count++;
    app_bt_last_byte = byte;
    App_BluetoothParseByte(byte, now_ms);
  }

  if ((app_bt_last_cmd_ms != 0U) &&
      ((now_ms - app_bt_last_cmd_ms) > BT_CMD_TIMEOUT_MS))
  {
    App_BluetoothClearTarget();
  }
}

/**
 * @brief 解析一个蓝牙 APP 指令字节。
 *
 * 按协议把 0x41~0x48 转换成前进、后退、左转、右转或斜向运动；
 * 0x58/0x59 用于加速和减速；解析结果保存为目标速度和目标转向量。
 */
static void App_BluetoothParseByte(uint8_t byte, uint32_t now_ms)
{
  int16_t speed = 0;
  int16_t turn = 0;

  switch (byte)
  {
    case BT_CMD_FORWARD:
      speed = app_bt_speed_level;
      turn = 0;
      break;

    case BT_CMD_RIGHT_FRONT:
      speed = app_bt_speed_level;
      turn = app_bt_turn_level;
      break;

    case BT_CMD_RIGHT:
      speed = 0;
      turn = app_bt_turn_level;
      break;

    case BT_CMD_RIGHT_BACK:
      speed = (int16_t)-app_bt_speed_level;
      turn = app_bt_turn_level;
      break;

    case BT_CMD_BACK:
      speed = (int16_t)-app_bt_speed_level;
      turn = 0;
      break;

    case BT_CMD_LEFT_BACK:
      speed = (int16_t)-app_bt_speed_level;
      turn = (int16_t)-app_bt_turn_level;
      break;

    case BT_CMD_LEFT:
      speed = 0;
      turn = (int16_t)-app_bt_turn_level;
      break;

    case BT_CMD_LEFT_FRONT:
      speed = app_bt_speed_level;
      turn = (int16_t)-app_bt_turn_level;
      break;

    case BT_CMD_STOP:
      App_BluetoothClearTarget();
      app_bt_last_cmd_ms = now_ms;
      return;

    case BT_CMD_GRAVITY_MODE:
    case BT_CMD_JOYSTICK_MODE:
    case BT_CMD_BUTTON_MODE:
      app_bt_mode = byte;
      App_BluetoothClearTarget();
      app_bt_last_cmd_ms = now_ms;
      return;

    case BT_CMD_SPEED_UP:
      app_bt_speed_level = App_BluetoothClampInt16(
        (int16_t)(app_bt_speed_level + BT_SPEED_STEP_COUNTS),
        BT_SPEED_MIN_COUNTS,
        BT_SPEED_MAX_COUNTS);
      app_bt_turn_level = App_BluetoothClampInt16(
        (int16_t)(app_bt_turn_level + BT_TURN_STEP_PWM),
        BT_TURN_MIN_PWM,
        BT_TURN_MAX_PWM);
      app_bt_last_cmd_ms = now_ms;
      return;

    case BT_CMD_SPEED_DOWN:
      app_bt_speed_level = App_BluetoothClampInt16(
        (int16_t)(app_bt_speed_level - BT_SPEED_STEP_COUNTS),
        BT_SPEED_MIN_COUNTS,
        BT_SPEED_MAX_COUNTS);
      app_bt_turn_level = App_BluetoothClampInt16(
        (int16_t)(app_bt_turn_level - BT_TURN_STEP_PWM),
        BT_TURN_MIN_PWM,
        BT_TURN_MAX_PWM);
      app_bt_last_cmd_ms = now_ms;
      return;

    default:
      return;
  }

  app_bt_target_speed = (int16_t)(BT_TARGET_SPEED_SIGN * speed);
  app_bt_target_turn = (int16_t)(BT_TARGET_TURN_SIGN * turn);
  app_bt_last_cmd_ms = now_ms;
}

/**
 * @brief 清空蓝牙遥控目标。
 *
 * 停止、模式切换或蓝牙超时时调用，
 * 让目标速度和目标转向都回到 0。
 */
static void App_BluetoothClearTarget(void)
{
  app_bt_target_speed = 0;
  app_bt_target_turn = 0;
}

/**
 * @brief 对蓝牙速度/转向等级做上下限限制。
 *
 * APP 加速和减速时使用，保证速度等级和转向等级不会小于最小值，
 * 也不会超过设定最大值。
 */
static int16_t App_BluetoothClampInt16(int16_t value, int16_t min_value,
                                       int16_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

/**
 * @brief 获取当前蓝牙目标速度。
 *
 * 返回值送入速度环，正值表示前进，负值表示后退，0 表示无前后目标。
 */
static int16_t App_BluetoothGetTargetSpeed(void)
{
  return app_bt_target_speed;
}

/**
 * @brief 获取当前蓝牙目标转向量。
 *
 * 返回值作为左右轮差速，正负号决定左右转方向，数值大小决定转向强度。
 */
static int16_t App_BluetoothGetTargetTurn(void)
{
  return app_bt_target_turn;
}

static int16_t App_SlewSpeed(int16_t current, int16_t target,
                             int16_t accel_step, int16_t decel_step)
{
  int32_t delta = (int32_t)target - (int32_t)current;
  int32_t step = (delta >= 0) ? accel_step : decel_step;

  if (delta > step)
    delta = step;
  else if (delta < -step)
    delta = -step;
  return (int16_t)((int32_t)current + delta);
}

/**
 * @brief 将运行状态枚举转换成字符串。
 *
 * 用于串口最后一列显示 RUN、TILT、TIMEOUT、IMU_ERROR 等状态。
 */
static const char *App_StateText(BalanceState_t state)
{
  switch (state)
  {
    case BALANCE_STATE_RUNNING:
      return "RUN";
    case BALANCE_STATE_IMU_ERROR:
      return "IMU_ERROR";
    case BALANCE_STATE_TILT:
      return "TILT";
    case BALANCE_STATE_TIMEOUT:
      return "TIMEOUT";
    case BALANCE_STATE_OVERRUN:
      return "OVERRUN";
    case BALANCE_STATE_STOPPED:
    default:
      return "STOPPED";
  }
}
/* USER CODE END Application */



