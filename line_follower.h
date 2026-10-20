/**
 * @file line_follower.h
 * @brief 四路数字灰度巡线模块的参数、状态及对外接口。
 *
 * 模块以 10 ms 为调用周期，只生成 target_speed 和 target_turn，
 * 不直接修改原有角度环、速度环和电机驱动逻辑。
 */
#ifndef __LINE_FOLLOWER_H
#define __LINE_FOLLOWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 八字避障模式：保留右、左、左、右路口动作，并连续跑两圈。 */
#define LINE_ENABLE_ROUTE_MODE 1U
#define LINE_ROUTE_LAPS         2U

/*
 * 四路灰度位图定义（从车头向前看，由左到右）。
 * LineFollower_Update() 的输入位为 GPIO 实际电平：1=高，0=低；
 * 输出 raw_bitmap/filtered_bitmap 统一转换为：1=检测到黑线。
 */
#define LINE_SENSOR_LEFT_OUTER_BIT       (1U << 0)
#define LINE_SENSOR_LEFT_INNER_BIT       (1U << 1)
#define LINE_SENSOR_RIGHT_INNER_BIT      (1U << 2)
#define LINE_SENSOR_RIGHT_OUTER_BIT      (1U << 3)
#define LINE_SENSOR_MASK                 0x0FU

/* 传感器物理位置归一化权重，中心为 0，最外侧绝对值为 300。 */
#define LINE_WEIGHT_LEFT_OUTER           (-300)
#define LINE_WEIGHT_LEFT_INNER           (-54)
#define LINE_WEIGHT_RIGHT_INNER          54
#define LINE_WEIGHT_RIGHT_OUTER          300
#define LINE_ERROR_SCALE                 300

/*
 * 巡线 PD 参数，单位为 PWM。
 * turn = LINE_ERROR_TO_TURN_SIGN *
 *        (Kp * error + Kd * error_delta) / LINE_ERROR_SCALE
 * 误差量程约 -300..+300，Kp=650 时稳态外侧压线（error=300, 差分为 0）输出
 * 恰为 -650，即转向限幅；该值同时等于 LINE_SEARCH_TURN_PWM，因此从外侧压线
 * 切到丢线搜索时转向不跳变。Kd 按固定 10 ms 周期的一拍误差差分计算。
 */
#define LINE_TURN_KP_PWM                 650
#define LINE_TURN_KD_PWM                 90
#define LINE_TURN_LIMIT_PWM              650

/* 当前电机混控中 target_turn>0 为右转，左右方向符号集中在这里校准。 */
#define LINE_TURN_RIGHT_SIGN             1
#define LINE_TURN_LEFT_SIGN              (-1)
#define LINE_ERROR_TO_TURN_SIGN          (-1)

/*
 * 路口强制转向单独校准：实测本车的路线 RIGHT 状态若使用 +PWM 会先向左偏，
 * 因此这里与通用搜索方向分开定义。只影响 T 口预设动作，不改正常 PD 巡线。
 */
#define LINE_ROUTE_RIGHT_SIGN            (-1)
#define LINE_ROUTE_LEFT_SIGN             1

/* 分级速度及每 10 ms 的最大斜坡步长，单位与速度环 target_speed 一致。 */
#define LINE_SPEED_STRAIGHT_COUNTS       430  /* 采用已验证八字版的直道速度。 */
#define LINE_SPEED_CURVE_COUNTS          260  /* 采用已验证八字版的弯道速度。 */
#define LINE_SPEED_SEARCH_COUNTS         60   /* 短时丢线后的向前搜索速度。 */
#define LINE_SPEED_ACCEL_STEP_COUNTS     10   /* 每周期最大加速步长。 */
#define LINE_SPEED_DECEL_STEP_COUNTS     10   /* 每周期最大减速步长。 */

/*
 * 速度切换采用“匀加速斜坡 + 末端阻尼”。
 * 速度差较大时每周期固定变化 10，形成近似匀加速/匀减速；
 * 接近目标档位时，步长按剩余速度差的 1/4 缩小，减轻到档冲击。
 * 数值越大，末端越柔和，但完全到达目标速度所需时间越长。
 */
#define LINE_SPEED_DAMPING_DIVISOR       4

/*
 * 弯道保速参数：一旦检测到弯道/偏离（|error| 超过阈值或压到最外侧传感器），
 * 速度被钳制在 LINE_SPEED_CURVE_HOLD_COUNTS；离开弯道后继续维持该低速
 * LINE_CURVE_HOLD_MS，再按斜坡加回直道高速，避免出弯瞬间提速造成抖动。
 */
#define LINE_SPEED_CURVE_HOLD_COUNTS     335
#define LINE_CURVE_HOLD_MS               300U
#define LINE_CURVE_HOLD_ERROR            100

#if LINE_SPEED_DAMPING_DIVISOR < 1
#error "LINE_SPEED_DAMPING_DIVISOR must be at least 1"
#endif

#if (LINE_SPEED_CURVE_HOLD_COUNTS < LINE_SPEED_CURVE_COUNTS) || \
    (LINE_SPEED_CURVE_HOLD_COUNTS > LINE_SPEED_STRAIGHT_COUNTS)
#error "Curve hold speed must be between curve and straight speed"
#endif

/* 丢线搜索参数；超过 500 ms 后置零停车，重新检测到线后自动恢复。 */
#define LINE_SEARCH_TURN_PWM             650
#define LINE_LOST_TIMEOUT_MS             500U
#define LINE_STRAIGHT_HOLD_LOST_MS        0U  /* 成功八字版不在全白时强行直行。 */
#define LINE_STRAIGHT_HOLD_ERROR         70   /* 仅在丢线前接近中心时启用。 */
#define LINE_DEFAULT_SEARCH_DIRECTION    LINE_TURN_RIGHT_SIGN

/*
 * 路口任务参数。T 口可能表现为三路或四路黑；终点横线也可能是四路黑。
 * 因此终点不能只靠一帧传感器判断，必须在全部路线动作完成后才允许识别。
 */
#define LINE_INTERSECTION_CONFIRM_MS      20U
#define LINE_SECOND_RIGHT_CONFIRM_MS      50U
#define LINE_INTERSECTION_APPROACH_STEPS  8U
#define LINE_INTERSECTION_COOLDOWN_MS     1000U
#define LINE_AFTER_FIRST_LEFT_COOLDOWN_MS 3000U
#define LINE_INTERSECTION_SPEED_COUNTS    180
#define LINE_TURN_SPEED_COUNTS            90
#define LINE_ROUTE_TURN_PWM               650
#define LINE_STRAIGHT_PASS_MS             160U
#define LINE_TURN_MIN_MS                  900U
#define LINE_FIRST_RIGHT_TURN_MIN_MS      750U  /* 第一处 RIGHT 实测过转，缩短约 100 ms。 */
#define LINE_FIRST_RIGHT_TURN_PWM         560   /* 与已验证的柔和 LEFT 相同，避免内侧轮反转。 */
#define LINE_TURN_TIMEOUT_MS              1500U
#define LINE_TURN_REACQUIRE_STEPS         5U
#define LINE_FIRST_LEFT_TURN_MIN_MS       700U
#define LINE_FIRST_LEFT_TURN_PWM          560
#define LINE_FIRST_LEFT_REACQUIRE_STEPS   3U
#define LINE_SECOND_LEFT_TURN_MIN_MS      700U  /* 与已验证八字版一致：第三个动作 LEFT 至少保持 700 ms。 */
#define LINE_SECOND_LEFT_TURN_PWM         560
#define LINE_SECOND_LEFT_REACQUIRE_STEPS  3U
#define LINE_SECOND_RIGHT_TURN_MIN_MS     650U
#define LINE_SECOND_RIGHT_TURN_PWM        560
#define LINE_SECOND_RIGHT_REACQUIRE_STEPS 3U
#define LINE_END_CONFIRM_MS               20U
#define LINE_END_GUARD_MS                 300U

typedef enum
{
  LINE_FOLLOWER_TRACK = 0,   /* 已检测到黑线，正常巡线。 */
  LINE_FOLLOWER_SEARCH = 1,  /* 丢线未超时，按最后方向搜索。 */
  LINE_FOLLOWER_LOST = 2,    /* 丢线超时，速度和转向清零。 */
  LINE_FOLLOWER_STRAIGHT = 3,/* 正在通过已确认的直行路口。 */
  LINE_FOLLOWER_TURN_LEFT = 4, /* 正在执行预存左转指令。 */
  LINE_FOLLOWER_TURN_RIGHT = 5,/* 正在执行预存右转指令。 */
  LINE_FOLLOWER_END_STOP = 6 /* 已到终点，速度斜坡降到零。 */
} LineFollowerState_t;

/* 预存路线指令类型；实际路线顺序在 line_follower.c 顶部配置。 */
typedef enum
{
  LINE_ROUTE_STRAIGHT = 0,
  LINE_ROUTE_LEFT = 1,
  LINE_ROUTE_RIGHT = 2
} LineRouteCommand_t;

typedef enum
{
  LINE_POSITION_LEFT = 0,    /* 黑线位于车体左侧。 */
  LINE_POSITION_CENTER = 1,  /* 黑线位于车体中部。 */
  LINE_POSITION_RIGHT = 2,   /* 黑线位于车体右侧。 */
  LINE_POSITION_CROSS = 3    /* 四路全黑，识别为路口。 */
} LinePosition_t;

typedef struct
{
  uint8_t raw_bitmap;       /* 未滤波黑线位图，bit0..3 对应从左到右传感器。 */
  uint8_t filtered_bitmap;  /* 连续三次采样逐位多数判定后的黑线位图。 */
  int16_t error;            /* 所有有效传感器权重的平均值，范围 -300..+300。 */
  int16_t target_speed;     /* 已经过匀加速和阻尼处理的速度环目标。 */
  int16_t target_turn;      /* 已限幅到 +/-650 的左右轮差速目标。 */
  LineFollowerState_t state; /* TRACK、SEARCH 或 LOST。 */
  LinePosition_t position;   /* LEFT、CENTER、RIGHT 或 CROSS。 */
  uint8_t route_index;       /* 已经开始执行的路线指令数。 */
  uint8_t route_total;       /* 预存路线指令总数。 */
  uint8_t reset_generation;  /* 巡线模块初始化次数，用于诊断途中安全复位。 */
} LineFollowerOutput_t;

/* 巡线模块内部历史状态，由 LineFollower_Init() 统一复位。 */
typedef struct
{
  uint8_t sample_history[3];   /* 最近三次黑线位图，用于多数滤波。 */
  uint8_t history_index;       /* 环形采样缓冲区的下一写入位置。 */
  uint8_t line_lost_active;    /* 是否已经开始计算连续丢线时间。 */
  int16_t previous_error;      /* 上一拍误差，用于 PD 微分项。 */
  int16_t last_line_error;     /* 丢线前最后误差，用于搜索方向。 */
  int16_t ramped_speed;        /* 当前已平滑的 target_speed。 */
  int8_t last_search_direction;/* 最近一次有效线路方向，取 +/-1。 */
  uint32_t lost_since_ms;      /* 本次连续丢线开始时间。 */
  uint8_t curve_hold_active;   /* 是否处于出弯限速保持阶段。 */
  uint32_t curve_last_seen_ms; /* 最近一次检测到明显弯道的时间。 */
  LineFollowerState_t route_state; /* 路口动作子状态。 */
  LineRouteCommand_t active_command;/* 当前执行中的路口指令。 */
  uint8_t route_index;         /* 下一条待执行指令的下标。 */
  uint8_t cross_active;        /* 是否已经开始确认本次全黑路口。 */
  uint8_t turn_reacquire_steps;/* 转向后连续重新捕线次数。 */
  uint8_t intersection_armed;  /* 已稳定沿中间线行驶，允许识别下一次 T 口。 */
  uint8_t center_track_steps;  /* 连续中心巡线计数，用于区分弯道和 T 口。 */
  uint8_t end_armed;           /* 最后一次路口动作完成并离开路口后，才允许终点停车。 */
  uint8_t lap_count;           /* 已经完整通过终点的圈数，范围 0..LINE_ROUTE_LAPS。 */
  uint32_t cross_since_ms;     /* 本次四路全黑起始时间。 */
  uint32_t action_since_ms;    /* 当前直行/转弯动作的起始时间。 */
  uint32_t last_route_ms;      /* 最近一次路口动作确认时刻，用于抑制弯道误触发。 */
} LineFollower_t;

/** @brief 初始化或复位巡线模块内部状态。 */
void LineFollower_Init(LineFollower_t *follower);

/**
 * @brief 仅用于“普通巡线 + 避障”任务的重新捕线。
 * 保留速度斜坡和传感器滤波历史，清除八字路口动作、终点与计数状态，
 * 使当前黑线重新成为一条普通巡线，不会在避障后因路线状态停车。
 */
void LineFollower_ClearRouteState(LineFollower_t *follower);

/**
 * @brief 执行一次灰度采样、滤波、转向计算和速度规划。
 * @param follower 巡线控制器对象。
 * @param pin_level_bitmap GPIO 实际电平位图，bit0..3 从最左到最右。
 * @param now_ms 当前系统毫秒时间。
 * @return 本周期的巡线状态、目标速度和目标转向。
 */
LineFollowerOutput_t LineFollower_Update(LineFollower_t *follower,
                                         uint8_t pin_level_bitmap,
                                         uint32_t now_ms);
/* 以下接口用于 OLED 和串口显示简短状态文本。 */
const char *LineFollower_StateText(LineFollowerState_t state);
const char *LineFollower_PositionText(LinePosition_t position);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_FOLLOWER_H */
