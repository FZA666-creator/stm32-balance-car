#include "ultrasonic.h"
#include "main.h"
#include "cmsis_os.h"
#include "core_cm3.h"

static volatile uint16_t g_distance_mm;
static volatile uint8_t g_valid;
static volatile uint32_t g_sequence;

static uint32_t Us_Now(void)
{
  return DWT->CYCCNT;
}

static uint8_t Us_Expired(uint32_t start, uint32_t us)
{
  return ((uint32_t)(Us_Now() - start) >= (SystemCoreClock / 1000000U) * us) ? 1U : 0U;
}

static uint16_t Ultrasonic_MeasureOnce(void)
{
  uint32_t start;
  uint32_t rise;
  uint32_t width_cycles;
  uint32_t width_us;

  HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin,
                    GPIO_PIN_RESET);
  start = Us_Now();
  while (!Us_Expired(start, 2U)) { }
  HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin,
                    GPIO_PIN_SET);
  start = Us_Now();
  while (!Us_Expired(start, 12U)) { }
  HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin,
                    GPIO_PIN_RESET);

  /* 等待回波上升沿，最大等待 25 ms（约 4 m）。 */
  start = Us_Now();
  while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) == GPIO_PIN_RESET)
  {
    if (Us_Expired(start, 25000U)) return 0U;
  }
  rise = Us_Now();
  while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) != GPIO_PIN_RESET)
  {
    if (Us_Expired(rise, 25000U)) return 0U;
  }
  width_cycles = Us_Now() - rise;
  width_us = width_cycles / (SystemCoreClock / 1000000U);
  if ((width_us < 100U) || (width_us > 25000U)) return 0U;
  /* 声速约 0.343 mm/us，往返需除以 2。 */
  return (uint16_t)((width_us * 343U) / 2000U);
}

void Ultrasonic_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  g_distance_mm = 0U;
  g_valid = 0U;
  g_sequence = 0U;
}

uint16_t Ultrasonic_GetDistanceMm(void) { return g_distance_mm; }
uint8_t Ultrasonic_IsValid(void) { return g_valid; }
uint32_t Ultrasonic_GetSequence(void) { return g_sequence; }

void Ultrasonic_Task(void *argument)
{
  uint16_t distance;
  (void)argument;
  for (;;)
  {
    distance = Ultrasonic_MeasureOnce();
    if (distance != 0U)
    {
      g_distance_mm = distance;
      g_valid = 1U;
    }
    else
    {
      g_valid = 0U;
    }
    g_sequence++;
    osDelay(60U);
  }
}
