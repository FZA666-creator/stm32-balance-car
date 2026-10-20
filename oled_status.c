#include "oled_status.h"
#include "GUI.h"
#include "ultrasonic.h"
#include <stddef.h>
#include <stdio.h>

static uint8_t oled_ready;
static uint8_t oled_status;

static const char *Oled_BalanceStateText(BalanceState_t state)
{
  switch (state)
  {
    case BALANCE_STATE_RUNNING:
      return "RUN";
    case BALANCE_STATE_IMU_ERROR:
      return "IMU";
    case BALANCE_STATE_TILT:
      return "TILT";
    case BALANCE_STATE_TIMEOUT:
      return "TIME";
    case BALANCE_STATE_OVERRUN:
      return "OVER";
    case BALANCE_STATE_STOPPED:
    default:
      return "STOP";
  }
}

static void Oled_FormatBitmap(char out[5], uint8_t bitmap)
{
  uint8_t i;

  for (i = 0U; i < 4U; i++)
  {
    out[i] = ((bitmap & (1U << i)) != 0U) ? '1' : '0';
  }
  out[4] = '\0';
}

static int32_t Oled_RoundFloat10(float value)
{
  return (value >= 0.0f) ? (int32_t)(value * 10.0f + 0.5f) :
                           (int32_t)(value * 10.0f - 0.5f);
}

static const char *Oled_ObstacleText(void)
{
  uint16_t distance = Ultrasonic_GetDistanceMm();
  if (Ultrasonic_IsValid() == 0U) return "--";
  if (distance <= 160U) return "STOP";
  if (distance <= 280U) return "NEAR";
  return "CLR";
}

static void Oled_FormatFixed1(char *out, size_t out_size, float value)
{
  int32_t scaled = Oled_RoundFloat10(value);
  int32_t whole;
  int32_t fraction;

  if ((out == NULL) || (out_size == 0U))
  {
    return;
  }
  if (scaled < 0)
  {
    scaled = -scaled;
    whole = scaled / 10;
    fraction = scaled % 10;
    (void)snprintf(out, out_size, "-%ld.%ld",
                   (long)whole, (long)fraction);
  }
  else
  {
    whole = scaled / 10;
    fraction = scaled % 10;
    (void)snprintf(out, out_size, "%ld.%ld",
                   (long)whole, (long)fraction);
  }
}

void OledStatus_Init(void)
{
  oled_ready = 0U;
  oled_status = 0U;
  (void)GUI_Init();
  GUI_Clear();
  GUI_DispStringAt("LINE FOLLOWER", 0, 0);
  GUI_DispStringAt("OLED SPI READY", 0, 12);
  GUI_Update();
  oled_ready = 1U;
  oled_status = 1U;
  printf("#oled,spi=PC14_PB5_PB4_PB3,no_cs\r\n");
}

uint8_t OledStatus_GetStatus(void)
{
  return oled_status;
}

void OledStatus_Update(const BalanceSample_t *sample,
                       const MotorTelemetry_t *motor,
                       const BalanceOutput_t *output,
                       const LineFollowerOutput_t *line_output)
{
  char text[32];
  char raw_bitmap[5];
  char bitmap[5];
  char pitch[12];

  if ((oled_ready == 0U) || (sample == NULL) || (motor == NULL) ||
      (output == NULL) || (line_output == NULL))
  {
    return;
  }

  Oled_FormatBitmap(raw_bitmap, line_output->raw_bitmap);
  Oled_FormatBitmap(bitmap, line_output->filtered_bitmap);
  Oled_FormatFixed1(pitch, sizeof(pitch), sample->pitch_deg);
  GUI_Clear();

  (void)snprintf(text, sizeof(text), "B:%s/%s %s", raw_bitmap, bitmap,
                 LineFollower_StateText(line_output->state));
  GUI_DispStringAt(text, 0, 0);

  (void)snprintf(text, sizeof(text), "R:%u/%u X:%u",
                 (unsigned int)line_output->route_index,
                 (unsigned int)line_output->route_total,
                 (unsigned int)line_output->reset_generation);
  GUI_DispStringAt(text, 0, 12);

  (void)snprintf(text, sizeof(text), "P:%s U:%3umm",
                 LineFollower_PositionText(line_output->position),
                 (unsigned int)Ultrasonic_GetDistanceMm());
  GUI_DispStringAt(text, 0, 24);

  (void)snprintf(text, sizeof(text), "S:%d T:%d %s",
                 line_output->target_speed, line_output->target_turn,
                 Oled_ObstacleText());
  GUI_DispStringAt(text, 0, 36);

  (void)snprintf(text, sizeof(text), "SYS:%s P:%s",
                 Oled_BalanceStateText(output->state), pitch);
  GUI_DispStringAt(text, 0, 48);

  /* OLED 高度有限，PWM 仍通过 USART1 遥测输出，避免第六行越界。 */

  GUI_Update();
}
