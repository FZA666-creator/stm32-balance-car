/**
 * @file line_follower.c
 * @brief 四路灰度巡线、PD 转向和匀加速阻尼变速的实现。
 *
 * 数据流：低电平转换 -> 三次多数滤波 -> 位置误差 -> PD 转向 ->
 * 直弯速度规划 -> 匀加速斜坡与末端阻尼 -> 输出给原平衡控制器。
 */
#include "line_follower.h"
#include <stddef.h>
#include <string.h>

/*
 * 今日路线配置：每识别到一个普通路口，依次执行一条指令。
 * 按你们实际赛道修改这里即可，例如全左转改为 { LINE_ROUTE_LEFT, ... }。
 * 最后一条指令完成后，终点横线将被识别为终点并停车。
 */
static const LineRouteCommand_t g_route_commands[] =
{
  /* 八字路线：从红圈起点按实车车头方向依次执行右、左、左、右。
   * 第四次动作结束并重新捕获普通黑线后，才允许将红圈横线识别为终点。
   * 若实车起步方向相反，只改这个数组，不改平衡环和编码器。 */
  LINE_ROUTE_RIGHT,
  LINE_ROUTE_LEFT,
  LINE_ROUTE_LEFT,
  LINE_ROUTE_RIGHT
};

#define LINE_ROUTE_COMMAND_COUNT \
  ((uint8_t)(sizeof(g_route_commands) / sizeof(g_route_commands[0])))

/* 不参与路线控制，仅用于确认是否发生过 LineFollower_Init()。 */
static uint8_t g_line_reset_generation;

/** @brief 将转向量限制在允许的 PWM 范围内。 */
static int16_t LineFollower_ClampTurn(int32_t turn)
{
  if (turn > LINE_TURN_LIMIT_PWM)
  {
    return LINE_TURN_LIMIT_PWM;
  }
  if (turn < -LINE_TURN_LIMIT_PWM)
  {
    return -LINE_TURN_LIMIT_PWM;
  }
  return (int16_t)turn;
}

/**
 * @brief 将速度档位转换为带末端阻尼的匀加速斜坡。
 *
 * difference 较大时，step 被限制为固定的最大步长，因此目标速度
 * 每 10 ms 等量变化；接近目标后，step 随剩余差值减小，避免到达
 * 新档位时速度指令突然折断。最小保留 1 count，保证最终能够到达。
 */
static int16_t LineFollower_RampSpeed(int16_t current, int16_t desired)
{
  int32_t difference = (int32_t)desired - (int32_t)current;
  int32_t step;

  if (difference == 0)
  {
    return current;
  }

  step = difference / LINE_SPEED_DAMPING_DIVISOR;
  if (difference > 0)
  {
    if (step < 1)
    {
      step = 1;
    }
    if (step > LINE_SPEED_ACCEL_STEP_COUNTS)
    {
      step = LINE_SPEED_ACCEL_STEP_COUNTS;
    }
  }
  else
  {
    if (step > -1)
    {
      step = -1;
    }
    if (step < -LINE_SPEED_DECEL_STEP_COUNTS)
    {
      step = -LINE_SPEED_DECEL_STEP_COUNTS;
    }
  }

  return (int16_t)((int32_t)current + step);
}

/** @brief 按线路偏差在直道速度和弯道速度之间连续插值。 */
static int16_t LineFollower_CalculateTrackSpeed(int16_t error)
{
  int32_t absolute_error = error;
  int32_t speed_range = (int32_t)LINE_SPEED_STRAIGHT_COUNTS -
                        (int32_t)LINE_SPEED_CURVE_COUNTS;
  int32_t desired_speed;

  if (absolute_error < 0)
  {
    absolute_error = -absolute_error;
  }
  if (absolute_error > LINE_ERROR_SCALE)
  {
    absolute_error = LINE_ERROR_SCALE;
  }

  desired_speed = (int32_t)LINE_SPEED_STRAIGHT_COUNTS -
                  ((speed_range * absolute_error) / LINE_ERROR_SCALE);
  if (desired_speed < LINE_SPEED_CURVE_COUNTS)
  {
    desired_speed = LINE_SPEED_CURVE_COUNTS;
  }
  return (int16_t)desired_speed;
}

/** @brief 更新弯道保持计时，防止短暂居中后立即恢复高速。 */
static void LineFollower_UpdateCurveHold(LineFollower_t *follower,
                                         uint8_t curve_detected,
                                         uint32_t now_ms)
{
  if (curve_detected != 0U)
  {
    follower->curve_hold_active = 1U;
    follower->curve_last_seen_ms = now_ms;
  }
  else if ((follower->curve_hold_active != 0U) &&
           ((now_ms - follower->curve_last_seen_ms) >= LINE_CURVE_HOLD_MS))
  {
    follower->curve_hold_active = 0U;
  }
}

/** @brief 弯道保持期间限制最高速度，不让车辆突然追速。 */
static int16_t LineFollower_ApplyCurveHold(const LineFollower_t *follower,
                                           int16_t desired_speed)
{
  if ((follower->curve_hold_active != 0U) &&
      (desired_speed > LINE_SPEED_CURVE_HOLD_COUNTS))
  {
    return LINE_SPEED_CURVE_HOLD_COUNTS;
  }
  return desired_speed;
}

/** @brief 设置当前路口动作，统一清除动作计时和重新捕线计数。 */
static void LineFollower_StartRouteAction(LineFollower_t *follower,
                                          LineFollowerState_t state,
                                          LineRouteCommand_t command,
                                          uint32_t now_ms)
{
  follower->route_state = state;
  follower->active_command = command;
  follower->action_since_ms = now_ms;
  follower->last_route_ms = now_ms;
  follower->turn_reacquire_steps = 0U;
  follower->intersection_armed = 0U;
  follower->center_track_steps = 0U;
  /* 锁住当前 T 口，必须驶离路口区域后才能识别下一个，
   * 防止宽黑胶带在车下持续存在时重复计数。 */
  follower->cross_active = 2U;
}

/**
 * @brief 识别 T 口候选。
 *
 * 正常直线通常是中间两路黑；车头经过本赛道 T 口时，主线加上左/右
 * 支路会形成三路黑，正压到 T 的横向胶带时也可能形成四路黑。故所有
 * 三路/四路黑都先作为路口候选；终点由路线完成后的 end_armed 区分。
 */
static uint8_t LineFollower_IsIntersectionCandidate(uint8_t filtered)
{
  return ((filtered == 0x07U) || (filtered == 0x0EU) ||
          (filtered == LINE_SENSOR_MASK)) ? 1U : 0U;
}

/** @brief 终点横线横跨传感器阵列，表现为四路同时检测到黑线。 */
static uint8_t LineFollower_IsEndMarker(uint8_t filtered)
{
  return (filtered == LINE_SENSOR_MASK) ? 1U : 0U;
}

/** @brief 仅接受与下一条路线指令位于同一侧的 T 口灯态。 */
static uint8_t LineFollower_IsExpectedIntersection(const LineFollower_t *follower,
                                                    uint8_t raw)
{
  if (follower->route_index >= LINE_ROUTE_COMMAND_COUNT)
  {
    return 0U;
  }

  /* 本组八字赛道实测的四个路口灯态：第 1 个右转和第 2 个左转
   * 为单侧三灯；第 1 个左转和第 2 个右转为四路全黑。按
   * route_index 固定匹配，终点全黑仅在四次动作完成后才停车。 */
  switch (follower->route_index)
  {
    case 0U:  /* 右转，OLED 0111，内部位图 0x0E。 */
      return (raw == 0x0EU) ? 1U : 0U;
    case 1U:  /* 第 1 个左转，四灯全黑。 */
      return (raw == LINE_SENSOR_MASK) ? 1U : 0U;
    case 2U:  /* 第 2 个左转，OLED 1110，内部位图 0x07。 */
      return (raw == 0x07U) ? 1U : 0U;
    case 3U:  /* 第 2 个右转，四灯全黑。 */
      return (raw == LINE_SENSOR_MASK) ? 1U : 0U;
    default:
      return 0U;
  }
}

/** @brief 转弯结束只接受目标侧线路，避免回到原直线就提前结束。 */
static uint8_t LineFollower_IsTurnReacquiredPattern(uint8_t filtered,
                                                     int16_t turn_direction)
{
  if (turn_direction == LINE_ROUTE_LEFT_SIGN)
  {
    return (((filtered & (LINE_SENSOR_LEFT_OUTER_BIT |
                          LINE_SENSOR_LEFT_INNER_BIT)) != 0U) &&
            ((filtered & (LINE_SENSOR_RIGHT_INNER_BIT |
                          LINE_SENSOR_RIGHT_OUTER_BIT)) == 0U)) ? 1U : 0U;
  }
  return (((filtered & (LINE_SENSOR_RIGHT_INNER_BIT |
                        LINE_SENSOR_RIGHT_OUTER_BIT)) != 0U) &&
          ((filtered & (LINE_SENSOR_LEFT_OUTER_BIT |
                        LINE_SENSOR_LEFT_INNER_BIT)) == 0U)) ? 1U : 0U;
}

/**
 * @brief 只在“先稳定走中间线、再遇到三/四路黑”时开放 T 口动作。
 *
 * 圆弧弯道也可能短暂形成 0111 或 1110。进入弯道前传感器通常会先压到
 * 外侧，下面会取消 armed；而真正 T 口前是一段中心直线，连续 80 ms 的
 * 中间传感器有效后才置 armed。因此不能仅凭单帧灯态消耗路线指令。
 */
static void LineFollower_UpdateIntersectionArm(LineFollower_t *follower,
                                                uint8_t filtered,
                                                uint32_t now_ms)
{
  uint8_t required_center_steps = LINE_INTERSECTION_APPROACH_STEPS;
  uint32_t cooldown_ms = LINE_INTERSECTION_COOLDOWN_MS;

  if ((follower->route_state != LINE_FOLLOWER_TRACK) ||
      (follower->cross_active != 0U))
  {
    return;
  }

  /* 第四条是回终点前的最后一次左转，入口紧接圆弧，难以获得完整 80 ms
   * 的居中读数；单独缩短其预备与冷却，但仍要求看到左侧三灯 T 口。 */
  if (follower->route_index == (LINE_ROUTE_COMMAND_COUNT - 1U))
  {
    required_center_steps = 2U;
    cooldown_ms = 600U;
  }

  if ((follower->route_index != 0U) &&
      ((now_ms - follower->last_route_ms) < cooldown_ms))
  {
    follower->center_track_steps = 0U;
    follower->intersection_armed = 0U;
    return;
  }

  /* 候选路口这一拍不要清除上一段已经建立的 armed 标志；它表示
   * 车辆在进入 T 口之前确实经过了稳定中线。动作开始后该标志会由
   * StartRouteAction 清零，离开路口后重新建立。 */
  if (LineFollower_IsIntersectionCandidate(filtered) != 0U)
  {
    return;
  }

  /* 只有“恰好中间两路黑”才计入直线接近；一侧内探头、外探头或全白
   * 都说明车辆处于弯道/修正中，不能开放下一次路线动作。 */
  if (filtered == (LINE_SENSOR_LEFT_INNER_BIT | LINE_SENSOR_RIGHT_INNER_BIT))
  {
    if (follower->center_track_steps < required_center_steps)
    {
      follower->center_track_steps++;
    }
    if (follower->center_track_steps >= required_center_steps)
    {
      follower->intersection_armed = 1U;
    }
  }
  else
  {
    follower->center_track_steps = 0U;
    follower->intersection_armed = 0U;
  }
}

/** @brief 最后一路口动作已结束且已重新捕线后，才可将横线识别为终点。 */
static void LineFollower_ArmEndAfterLastAction(LineFollower_t *follower)
{
  if (follower->route_index >= LINE_ROUTE_COMMAND_COUNT)
  {
    follower->end_armed = 1U;
  }
}

/** @brief 处理单次路口识别；返回 1 表示本拍已经输出路口动作。 */
static uint8_t LineFollower_HandleIntersection(LineFollower_t *follower,
                                               uint8_t filtered,
                                               uint32_t now_ms,
                                               LineFollowerOutput_t *output,
                                               int16_t *desired_speed)
{
  uint32_t cross_ms;
  uint32_t confirm_ms = LINE_INTERSECTION_CONFIRM_MS;

  /* 红圈终点是横向短线：只有完整一圈的四个路口动作完成后才允许
   * 进入这里。第一次经过终点只重新开始下一圈，第二次才停车。 */
  if ((follower->end_armed != 0U) &&
      (follower->route_state == LINE_FOLLOWER_TRACK) &&
      ((now_ms - follower->last_route_ms) >= LINE_END_GUARD_MS) &&
      (LineFollower_IsEndMarker(follower->sample_history[
        (uint8_t)((follower->history_index + 2U) % 3U)]) != 0U))
  {
    if (follower->cross_active != 1U)
    {
      follower->cross_active = 1U;
      follower->cross_since_ms = now_ms;
    }
    cross_ms = now_ms - follower->cross_since_ms;
    output->position = LINE_POSITION_CROSS;
    output->error = 0;
    output->target_turn = 0;
    if (cross_ms >= LINE_END_CONFIRM_MS)
    {
      if (follower->lap_count + 1U >= LINE_ROUTE_LAPS)
      {
        follower->lap_count = LINE_ROUTE_LAPS;
        follower->route_state = LINE_FOLLOWER_END_STOP;
      }
      else
      {
        /* 第一圈结束：终点横线不停车。清除本圈路口进度，
         * 锁住当前横线，待传感器离开后再识别第二圈第一个 T 口。 */
        follower->lap_count++;
        follower->route_index = 0U;
        follower->end_armed = 0U;
        follower->route_state = LINE_FOLLOWER_TRACK;
        follower->active_command = LINE_ROUTE_STRAIGHT;
        follower->last_route_ms = now_ms;
        follower->cross_active = 2U;
      }
    }
    output->state = (follower->route_state == LINE_FOLLOWER_END_STOP) ?
                    LINE_FOLLOWER_END_STOP : LINE_FOLLOWER_TRACK;
    *desired_speed = (follower->route_state == LINE_FOLLOWER_END_STOP) ?
                     0 : LINE_INTERSECTION_SPEED_COUNTS;
    return 1U;
  }

  /* cross_active=2 表示上一处路口动作已经确认，但传感器仍可能
   * 压在宽黑胶带上。必须等位图完全离开所有三灯/四灯候选区域后
   * 才释放锁，不能拿“下一处路口的期望灯态”来判断，否则同一个
   * T 口从 0111 变成 1111 时会被重复计数。 */
  if (follower->cross_active == 2U)
  {
    if (LineFollower_IsIntersectionCandidate(filtered) == 0U)
    {
      follower->cross_active = 0U;
    }
    else
    {
      output->position = LINE_POSITION_CROSS;
      output->error = 0;
      output->state = follower->route_state;
      output->target_turn = 0;
      *desired_speed = LINE_INTERSECTION_SPEED_COUNTS;
      return 1U;
    }
  }

  if (LineFollower_IsIntersectionCandidate(filtered) == 0U)
  {
    follower->cross_active = 0U;
    return 0U;
  }

  /* 每次确认 T 口时 last_route_ms 已记录。动作期间 cross_active 会锁住
   * 当前路口；动作结束后，补足到 1 s 的冷却窗口，避免车头刚转过路口
   * 又立即压到邻近黑线时，把同一组胶带或紧邻结构误判成下一路口。
   * 计时从“确认路口”而非“转弯结束”开始，故不会额外阻塞完整 1 s。 */
  {
    uint32_t cooldown_ms = LINE_INTERSECTION_COOLDOWN_MS;

    /* 第一个 LEFT 完成后，下一段弯道较容易残留三/四灯黑态；
     * route_index=2 表示已经完成 RIGHT、LEFT，当前等待第二个 LEFT。
     * 这一段单独使用约 3 s 保护锁，避免刚出第一个 LEFT 就误触发。 */
    if (follower->route_index == 2U)
    {
      cooldown_ms = LINE_AFTER_FIRST_LEFT_COOLDOWN_MS;
    }

    if ((follower->route_index != 0U) &&
        ((now_ms - follower->last_route_ms) < cooldown_ms))
    {
      return 0U;
    }
  }

  /* 四个 T 口使用已实测的“路线阶段 + 专属灯态”确认。这里不再要求
   * 额外的中心线预备或 1000 ms 冷却：两个全黑 T 口间距较近时，这些
   * 条件会把真实 LEFT/RIGHT 动作挡掉，导致小车虽被普通 PD 带进支路，
   * route_index 却不递增。抗误判由阶段专属位图、20 ms 持续确认和
   * cross_active 路口锁共同完成。 */
  if (LineFollower_IsExpectedIntersection(follower, filtered) == 0U)
  {
    return 0U;
  }

  if (follower->cross_active == 0U)
  {
    follower->cross_active = 1U;
    follower->cross_since_ms = now_ms;
  }
  cross_ms = now_ms - follower->cross_since_ms;
  output->position = LINE_POSITION_CROSS;
  output->error = 0;

  /* 最后一项已开始执行但尚未离开路口，不能把当前 T 口当终点。 */
  if (follower->route_index >= LINE_ROUTE_COMMAND_COUNT)
  {
    output->state = LINE_FOLLOWER_TRACK;
    output->target_turn = 0;
    *desired_speed = LINE_INTERSECTION_SPEED_COUNTS;
    return 1U;
  }

  /* T 口由原始灯态直接触发：高速经过时也不会在确认等待期间漏掉。 */
  /* 最后一个 RIGHT 是四路全黑 T 口，弯道中也可能短暂出现 1111；
   * 仅此阶段延长确认到 50 ms，压掉偶发全黑而不影响前三个 T 口。 */
  if (follower->route_index == (LINE_ROUTE_COMMAND_COUNT - 1U))
  {
    confirm_ms = LINE_SECOND_RIGHT_CONFIRM_MS;
  }
  if (cross_ms >= confirm_ms)
  {
    LineRouteCommand_t command = g_route_commands[follower->route_index];

    follower->route_index++;
    if (command == LINE_ROUTE_LEFT)
    {
      LineFollower_StartRouteAction(follower, LINE_FOLLOWER_TURN_LEFT,
                                    command, now_ms);
    }
    else if (command == LINE_ROUTE_RIGHT)
    {
      LineFollower_StartRouteAction(follower, LINE_FOLLOWER_TURN_RIGHT,
                                    command, now_ms);
    }
    else
    {
      LineFollower_StartRouteAction(follower, LINE_FOLLOWER_STRAIGHT,
                                    command, now_ms);
    }
    output->state = follower->route_state;
    *desired_speed = LINE_INTERSECTION_SPEED_COUNTS;
    return 1U;
  }

  output->state = LINE_FOLLOWER_TRACK;
  output->target_turn = 0;
  *desired_speed = LINE_INTERSECTION_SPEED_COUNTS;
  return 1U;
}

/** @brief 执行已确认的直行/转弯动作；返回 1 表示动作仍在接管输出。 */
static uint8_t LineFollower_ExecuteRouteAction(LineFollower_t *follower,
                                                uint8_t filtered,
                                                uint32_t now_ms,
                                                LineFollowerOutput_t *output,
                                                int16_t *desired_speed)
{
  uint32_t action_ms = now_ms - follower->action_since_ms;

  if (follower->route_state == LINE_FOLLOWER_END_STOP)
  {
    output->state = LINE_FOLLOWER_END_STOP;
    output->target_turn = 0;
    *desired_speed = 0;
    return 1U;
  }

  if (follower->route_state == LINE_FOLLOWER_STRAIGHT)
  {
    if ((action_ms >= LINE_STRAIGHT_PASS_MS) && (filtered != LINE_SENSOR_MASK))
    {
      follower->route_state = LINE_FOLLOWER_TRACK;
      /* 动作已经驶离当前路口，立即释放路口锁。
       * 若下一个路口紧接出现，不能因为上一动作遗留的 cross_active=2
       * 而把新路口误认为旧路口，导致 route_index 不递增。 */
      follower->cross_active = 2U;
      LineFollower_ArmEndAfterLastAction(follower);
      return 0U;
    }
    output->state = LINE_FOLLOWER_STRAIGHT;
    output->target_turn = 0;
    *desired_speed = LINE_INTERSECTION_SPEED_COUNTS;
    return 1U;
  }

  if ((follower->route_state == LINE_FOLLOWER_TURN_LEFT) ||
      (follower->route_state == LINE_FOLLOWER_TURN_RIGHT))
  {
    int16_t turn_direction = (follower->route_state == LINE_FOLLOWER_TURN_LEFT) ?
                             LINE_ROUTE_LEFT_SIGN : LINE_ROUTE_RIGHT_SIGN;
    uint8_t is_first_left =
      ((follower->active_command == LINE_ROUTE_LEFT) &&
       (follower->route_index == 2U)) ? 1U : 0U;
    uint8_t is_second_left =
      ((follower->active_command == LINE_ROUTE_LEFT) &&
       (follower->route_index == 3U)) ? 1U : 0U;
    uint8_t is_first_right =
      ((follower->active_command == LINE_ROUTE_RIGHT) &&
       (follower->route_index == 1U)) ? 1U : 0U;
    uint8_t is_second_right =
      ((follower->active_command == LINE_ROUTE_RIGHT) &&
       (follower->route_index == 4U)) ? 1U : 0U;
    uint32_t turn_min_ms = LINE_TURN_MIN_MS;
    int16_t turn_pwm = LINE_ROUTE_TURN_PWM;
    uint8_t reacquire_steps = LINE_TURN_REACQUIRE_STEPS;

    if (is_first_right != 0U)
    {
      turn_min_ms = LINE_FIRST_RIGHT_TURN_MIN_MS;
      turn_pwm = LINE_FIRST_RIGHT_TURN_PWM;
    }
    else if (is_first_left != 0U)
    {
      turn_min_ms = LINE_FIRST_LEFT_TURN_MIN_MS;
      turn_pwm = LINE_FIRST_LEFT_TURN_PWM;
      reacquire_steps = LINE_FIRST_LEFT_REACQUIRE_STEPS;
    }
    else if (is_second_left != 0U)
    {
      turn_min_ms = LINE_SECOND_LEFT_TURN_MIN_MS;
      turn_pwm = LINE_SECOND_LEFT_TURN_PWM;
      reacquire_steps = LINE_SECOND_LEFT_REACQUIRE_STEPS;
    }
    else if (is_second_right != 0U)
    {
      /* 第 4 个动作是第二个 RIGHT，靠近终点时实车容易继续旋转。
       * 采用与第一个 LEFT 相同的柔和退出参数，防止转成掉头。 */
      turn_min_ms = LINE_SECOND_RIGHT_TURN_MIN_MS;
      turn_pwm = LINE_SECOND_RIGHT_TURN_PWM;
      reacquire_steps = LINE_SECOND_RIGHT_REACQUIRE_STEPS;
    }

    output->state = follower->route_state;
    /* 特定路口使用单独的强制转向参数，降低转过量风险。 */
    output->target_turn = (int16_t)(turn_direction * turn_pwm);
    *desired_speed = LINE_TURN_SPEED_COUNTS;

    /* 先离开 T 口的宽黑区，再连续看到普通单线，才允许结束强制转向。
     * 不能只判断“不是全黑”，否则刚掠过路口边缘时会过早松开转向。 */
    if ((action_ms >= turn_min_ms) &&
        (LineFollower_IsTurnReacquiredPattern(filtered, turn_direction) != 0U))
    {
      if (follower->turn_reacquire_steps < reacquire_steps)
      {
        follower->turn_reacquire_steps++;
      }
      if (follower->turn_reacquire_steps >= reacquire_steps)
      {
        follower->route_state = LINE_FOLLOWER_TRACK;
        /* 已经重新捕获目标线路，当前路口动作结束；
         * 清除动作锁，允许下一拍识别新的 T 型口。 */
        /* 保持路口锁，直到传感器离开所有三灯/四灯候选区域。 */
        follower->cross_active = 2U;
        /* 路口保护锁从“转弯真正结束”开始计时，防止传感器仍压在
         * 上一个 T 口时重复计数。第一个 LEFT 仍由专用 3 s 参数保护。 */
        follower->last_route_ms = now_ms;
        LineFollower_ArmEndAfterLastAction(follower);
        return 0U;
      }
    }
    else if (filtered == 0U)
    {
      follower->turn_reacquire_steps = 0U;
    }

    /* 转向超过合理时间仍未重新捕线，切回 SEARCH 沿转向方向继续找线。 */
    if (action_ms >= LINE_TURN_TIMEOUT_MS)
    {
      follower->last_search_direction = turn_direction;
      follower->route_state = LINE_FOLLOWER_SEARCH;
      follower->cross_active = 0U;
      follower->line_lost_active = 1U;
      follower->lost_since_ms = now_ms;
    }
    return 1U;
  }

  return 0U;
}

/** @brief 对最近三次四路采样逐位进行多数判定。 */
static uint8_t LineFollower_Majority3(const uint8_t history[3])
{
  return (uint8_t)(((history[0] & history[1]) |
                    (history[0] & history[2]) |
                    (history[1] & history[2])) & LINE_SENSOR_MASK);
}

/** @brief 计算所有压线传感器位置权重的平均值。 */
static int16_t LineFollower_CalculateError(uint8_t bitmap)
{
  int16_t sum = 0;
  uint8_t count = 0U;

  if ((bitmap & LINE_SENSOR_LEFT_OUTER_BIT) != 0U)
  {
    sum = (int16_t)(sum + LINE_WEIGHT_LEFT_OUTER);
    count++;
  }
  if ((bitmap & LINE_SENSOR_LEFT_INNER_BIT) != 0U)
  {
    sum = (int16_t)(sum + LINE_WEIGHT_LEFT_INNER);
    count++;
  }
  if ((bitmap & LINE_SENSOR_RIGHT_INNER_BIT) != 0U)
  {
    sum = (int16_t)(sum + LINE_WEIGHT_RIGHT_INNER);
    count++;
  }
  if ((bitmap & LINE_SENSOR_RIGHT_OUTER_BIT) != 0U)
  {
    sum = (int16_t)(sum + LINE_WEIGHT_RIGHT_OUTER);
    count++;
  }

  return (count == 0U) ? 0 : (int16_t)(sum / (int16_t)count);
}

/** @brief 清空巡线历史状态，并设置默认搜索方向。 */
void LineFollower_Init(LineFollower_t *follower)
{
  if (follower == NULL)
  {
    return;
  }

  g_line_reset_generation++;
  (void)memset(follower, 0, sizeof(*follower));
  follower->last_search_direction = LINE_DEFAULT_SEARCH_DIRECTION;
}

void LineFollower_ClearRouteState(LineFollower_t *follower)
{
  if (follower == NULL)
  {
    return;
  }

  follower->route_state = LINE_FOLLOWER_TRACK;
  follower->active_command = LINE_ROUTE_STRAIGHT;
  follower->route_index = 0U;
  follower->cross_active = 0U;
  follower->turn_reacquire_steps = 0U;
  follower->intersection_armed = 0U;
  follower->center_track_steps = 0U;
  follower->end_armed = 0U;
  follower->cross_since_ms = 0U;
  follower->action_since_ms = 0U;
  follower->last_route_ms = 0U;
}

/**
 * @brief 执行一次完整巡线计算，设计调用周期为 10 ms。
 */
LineFollowerOutput_t LineFollower_Update(LineFollower_t *follower,
                                         uint8_t pin_level_bitmap,
                                         uint32_t now_ms)
{
  LineFollowerOutput_t output;
  uint8_t filtered;
  int16_t desired_speed;

  (void)memset(&output, 0, sizeof(output));
  output.state = LINE_FOLLOWER_LOST;
  output.position = LINE_POSITION_CENTER;
  output.reset_generation = g_line_reset_generation;
  if (follower == NULL)
  {
    return output;
  }

  /* 数字灰度低电平有效：GPIO 电平取反后，1 表示检测到黑线。 */
  output.raw_bitmap = (uint8_t)((~pin_level_bitmap) & LINE_SENSOR_MASK);
  follower->sample_history[follower->history_index] = output.raw_bitmap;
  follower->history_index = (uint8_t)((follower->history_index + 1U) % 3U);
  filtered = LineFollower_Majority3(follower->sample_history);
  output.filtered_bitmap = filtered;

  /* 八字版本才启用路口动作和终点停车；普通避障模式完全按黑线巡线。 */
#if LINE_ENABLE_ROUTE_MODE
  /* 已确认的路口动作、终点停车拥有高于普通 PD 巡线的优先级。 */
  if (LineFollower_ExecuteRouteAction(follower, filtered, now_ms, &output,
                                      &desired_speed) != 0U)
  {
    follower->ramped_speed = LineFollower_RampSpeed(follower->ramped_speed,
                                                     desired_speed);
    output.target_speed = follower->ramped_speed;
    output.route_index = follower->route_index;
    output.route_total = LINE_ROUTE_COMMAND_COUNT;
    return output;
  }
#endif

  /* 先基于滤波后正常巡线状态更新“路口预备”；T 口候选、方向匹配和
   * 持续确认统一使用 filtered，避免“候选来自滤波位图、方向却来自
   * 原始位图”造成第三/第四个路口转了但 route_index 不递增。 */
#if LINE_ENABLE_ROUTE_MODE
  LineFollower_UpdateIntersectionArm(follower, filtered, now_ms);
  if (LineFollower_HandleIntersection(follower, filtered, now_ms, &output,
                                      &desired_speed) != 0U)
  {
    follower->ramped_speed = LineFollower_RampSpeed(follower->ramped_speed,
                                                     desired_speed);
    output.target_speed = follower->ramped_speed;
    output.route_index = follower->route_index;
    output.route_total = LINE_ROUTE_COMMAND_COUNT;
    return output;
  }
#endif

  /* 至少一路检测到黑线：进入 TRACK，计算位置误差和 PD 转向。 */
  if (filtered != 0U)
  {
    int16_t error = LineFollower_CalculateError(filtered);
    int16_t error_delta = (int16_t)(error - follower->previous_error);
    int16_t absolute_error = (error < 0) ? (int16_t)(-error) : error;
    uint8_t curve_detected =
      ((absolute_error >= LINE_CURVE_HOLD_ERROR) ||
       ((filtered & (LINE_SENSOR_LEFT_OUTER_BIT |
                     LINE_SENSOR_RIGHT_OUTER_BIT)) != 0U)) ? 1U : 0U;
    /* P 项提供回线力度，D 项抑制误差快速变化造成的左右摆动。 */
    int32_t turn = (int32_t)LINE_ERROR_TO_TURN_SIGN *
      (((int32_t)LINE_TURN_KP_PWM * error) +
       ((int32_t)LINE_TURN_KD_PWM * error_delta)) /
      LINE_ERROR_SCALE;

    follower->line_lost_active = 0U;
    follower->previous_error = error;
    follower->last_line_error = error;
    if (error > 0)
    {
      follower->last_search_direction = LINE_TURN_RIGHT_SIGN;
      output.position = LINE_POSITION_RIGHT;
    }
    else if (error < 0)
    {
      follower->last_search_direction = LINE_TURN_LEFT_SIGN;
      output.position = LINE_POSITION_LEFT;
    }
    else
    {
      output.position = LINE_POSITION_CENTER;
    }

    output.error = error;
    output.target_turn = LineFollower_ClampTurn(turn);
    output.state = LINE_FOLLOWER_TRACK;
    /* 偏差越大速度越低；弯中短暂居中时保持低速，不立即追速。 */
    LineFollower_UpdateCurveHold(follower, curve_detected, now_ms);
    desired_speed = LineFollower_ApplyCurveHold(
      follower, LineFollower_CalculateTrackSpeed(error));
  }
  /* 四路全白：先给直线短暂保持窗口，再搜索；超过 800 ms 才进入 LOST。 */
  else
  {
    uint32_t lost_ms;

    if (follower->line_lost_active == 0U)
    {
      follower->line_lost_active = 1U;
      follower->lost_since_ms = now_ms;
    }
    LineFollower_UpdateCurveHold(follower, 1U, now_ms);
    lost_ms = now_ms - follower->lost_since_ms;
    output.error = follower->last_line_error;
    output.position = (follower->last_search_direction == LINE_TURN_LEFT_SIGN) ?
                      LINE_POSITION_LEFT : LINE_POSITION_RIGHT;

    /* 直道上偶尔会因车身轻微点头、胶带接缝或灰度阈值造成短暂
     * 全白。如果丢线前误差接近 0，先保持低速直行，不立即按最后
     * 方向大幅搜索，避免“短暂丢线 -> 打舵 -> 真正偏出直线”。 */
    {
      int16_t hold_error = follower->last_line_error;
      if (hold_error < 0)
      {
        hold_error = (int16_t)-hold_error;
      }
      if ((lost_ms <= LINE_STRAIGHT_HOLD_LOST_MS) &&
          (hold_error <= LINE_STRAIGHT_HOLD_ERROR))
      {
        output.state = LINE_FOLLOWER_TRACK;
        output.position = LINE_POSITION_CENTER;
        output.target_turn = 0;
        desired_speed = LINE_SPEED_CURVE_COUNTS;
      }
      else if (lost_ms <= LINE_LOST_TIMEOUT_MS)
      {
        /* 沿丢线前最后方向低速画弧，争取重新检测到黑线。 */
        output.state = LINE_FOLLOWER_SEARCH;
        desired_speed = LINE_SPEED_SEARCH_COUNTS;
        output.target_turn = (int16_t)(LINE_ERROR_TO_TURN_SIGN *
          follower->last_search_direction * LINE_SEARCH_TURN_PWM);
      }
      else
      {
        /* 安全停车优先级最高，因此超时后立即清零斜坡速度。 */
        output.state = LINE_FOLLOWER_LOST;
        output.target_turn = 0;
        follower->ramped_speed = 0;
        desired_speed = 0;
      }
    }
  }

  /* 正常速度档位切换统一经过匀加速和末端阻尼后再送入速度环。 */
  follower->ramped_speed = LineFollower_RampSpeed(follower->ramped_speed,
                                                   desired_speed);
  output.target_speed = follower->ramped_speed;
  output.route_index = follower->route_index;
  output.route_total = LINE_ROUTE_COMMAND_COUNT;
  output.reset_generation = g_line_reset_generation;
  return output;
}

/** @brief 返回巡线状态的 OLED/串口显示文本。 */
const char *LineFollower_StateText(LineFollowerState_t state)
{
  switch (state)
  {
    case LINE_FOLLOWER_TRACK:
      return "TRACK";
    case LINE_FOLLOWER_SEARCH:
      return "SEARCH";
    case LINE_FOLLOWER_STRAIGHT:
      return "GO";
    case LINE_FOLLOWER_TURN_LEFT:
      return "LEFT";
    case LINE_FOLLOWER_TURN_RIGHT:
      return "RIGHT";
    case LINE_FOLLOWER_END_STOP:
      return "END";
    case LINE_FOLLOWER_LOST:
    default:
      return "LOST";
  }
}

/** @brief 返回线路位置的 OLED/串口显示文本。 */
const char *LineFollower_PositionText(LinePosition_t position)
{
  switch (position)
  {
    case LINE_POSITION_LEFT:
      return "LEFT";
    case LINE_POSITION_RIGHT:
      return "RIGHT";
    case LINE_POSITION_CROSS:
      return "CROSS";
    case LINE_POSITION_CENTER:
    default:
      return "CENTER";
  }
}
