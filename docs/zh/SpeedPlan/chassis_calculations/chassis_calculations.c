/**
 * @file    chassis_calculations.c
 * @author  Jackrainman
 * @brief   底盘平滑速度规划模块（1.0接口兼容实现）
 * @version 2.0
 * @date    2026-03-02
 */

#include "chassis_calculations.h"
#include "my_math/my_math.h"
#include "steering_wheel/steering_wheel.h"
#include <math.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

/* ======================== 默认参数 ======================== */

#define DEFAULT_TICK_FREQ_HZ     1000u
#define DEFAULT_ACCEL_X          40.0f
#define DEFAULT_ACCEL_Y          40.0f
#define DEFAULT_ACCEL_W          20.0f
#define DEFAULT_JERK_RATIO       10.0f
#define DEFAULT_SPEED_LIMIT_X    5000.0f
#define DEFAULT_SPEED_LIMIT_Y    5000.0f
#define DEFAULT_SPEED_LIMIT_W    10.0f
#define DEFAULT_ROLLOVER_BASE    30.0f
#define DEFAULT_ROLLOVER_MAX     40.0f
#define DEFAULT_ROLLOVER_THRESH  30.0f

#define MIN_ACCEL                0.001f
#define SPEED_EPS                0.001f
#define ACCEL_EPS                0.001f
#define MAX_DT_SEC               0.1f

/* ======================== 全局兼容变量 ======================== */

ChassisSpeedPlanType chassis_speed_plan_type = CHASSIS_SPEED_PLAN_COSINE;

/* ======================== 内部状态结构 ======================== */

typedef struct {
    float start_speed;
    float target_speed;
    float duration;
    float elapsed;
    int active;
} CosineAxisState;

typedef struct {
    /* 配置 */
    float max_accel_x;
    float max_accel_y;
    float max_accel_w;

    float max_jerk_x;
    float max_jerk_y;
    float max_jerk_w;

    float max_speed_x;
    float max_speed_y;
    float max_speed_w;

    float rollover_base_limit;
    float rollover_max_limit;
    float rollover_threshold;
    int   enable_rollover;

    uint32_t tick_freq_hz;

    /* 状态 */
    float speed_x;
    float speed_y;
    float speed_w;

    float accel_x;
    float accel_y;
    float accel_w;

    CosineAxisState cos_x;
    CosineAxisState cos_y;
    CosineAxisState cos_w;

    float last_vx;
    float last_vy;

    uint32_t last_tick;
    int first_update;
} ChassisPlannerCtx;

static ChassisPlannerCtx g_planner = {
    .max_accel_x = DEFAULT_ACCEL_X,
    .max_accel_y = DEFAULT_ACCEL_Y,
    .max_accel_w = DEFAULT_ACCEL_W,
    .max_jerk_x = DEFAULT_ACCEL_X * DEFAULT_JERK_RATIO,
    .max_jerk_y = DEFAULT_ACCEL_Y * DEFAULT_JERK_RATIO,
    .max_jerk_w = DEFAULT_ACCEL_W * DEFAULT_JERK_RATIO,
    .max_speed_x = DEFAULT_SPEED_LIMIT_X,
    .max_speed_y = DEFAULT_SPEED_LIMIT_Y,
    .max_speed_w = DEFAULT_SPEED_LIMIT_W,
    .rollover_base_limit = DEFAULT_ROLLOVER_BASE,
    .rollover_max_limit = DEFAULT_ROLLOVER_MAX,
    .rollover_threshold = DEFAULT_ROLLOVER_THRESH,
    .enable_rollover = 1,
    .tick_freq_hz = DEFAULT_TICK_FREQ_HZ,
    .first_update = 1
};

/* ======================== 内部工具函数 ======================== */

static float absf_local(float v) {
    return (v >= 0.0f) ? v : -v;
}

static float clampf_local(float v, float low, float high) {
    if (v < low) return low;
    if (v > high) return high;
    return v;
}

static float clamp_speed(float speed, float max_speed) {
    if (max_speed <= 0.0f) {
        return speed;
    }
    return clampf_local(speed, -max_speed, max_speed);
}

static void apply_auto_jerk_if_needed(void) {
    if (g_planner.max_jerk_x <= 0.0f) {
        g_planner.max_jerk_x = g_planner.max_accel_x * DEFAULT_JERK_RATIO;
    }
    if (g_planner.max_jerk_y <= 0.0f) {
        g_planner.max_jerk_y = g_planner.max_accel_y * DEFAULT_JERK_RATIO;
    }
    if (g_planner.max_jerk_w <= 0.0f) {
        g_planner.max_jerk_w = g_planner.max_accel_w * DEFAULT_JERK_RATIO;
    }
}

static float get_time_delta(uint32_t current_tick) {
    if (g_planner.first_update) {
        g_planner.last_tick = current_tick;
        g_planner.first_update = 0;
        return 0.0f;
    }

    if (g_planner.tick_freq_hz == 0u) {
        g_planner.tick_freq_hz = DEFAULT_TICK_FREQ_HZ;
    }

    {
        uint32_t delta_ticks = current_tick - g_planner.last_tick;
        float dt = (float)delta_ticks / (float)g_planner.tick_freq_hz;
        g_planner.last_tick = current_tick;

        if (dt <= 0.0f || dt > MAX_DT_SEC) {
            return 0.0f;
        }
        return dt;
    }
}

static float trapezoid_axis_update(float current, float target, float max_change) {
    float diff = target - current;
    if (diff > max_change) {
        return current + max_change;
    }
    if (diff < -max_change) {
        return current - max_change;
    }
    return target;
}

static void cosine_axis_update(CosineAxisState *cs, float target,
                               float max_accel, float dt,
                               float *current_speed) {
    if (absf_local(target - cs->target_speed) > SPEED_EPS || !cs->active) {
        float dv;
        cs->start_speed = *current_speed;
        cs->target_speed = target;
        dv = absf_local(target - *current_speed);
        cs->duration = (dv > SPEED_EPS) ? (PI * dv) / (2.0f * max_accel) : 0.0f;
        cs->elapsed = 0.0f;
        cs->active = 1;
    }

    if (!cs->active || cs->duration <= 0.0f) {
        *current_speed = target;
        cs->active = 0;
        return;
    }

    cs->elapsed += dt;
    if (cs->elapsed >= cs->duration) {
        *current_speed = cs->target_speed;
        cs->active = 0;
    } else {
        float dv = cs->target_speed - cs->start_speed;
        float progress = cs->elapsed / cs->duration;
        *current_speed = cs->start_speed + dv * 0.5f * (1.0f - cosf(PI * progress));
    }
}

/*
 * jerk 限制 S 曲线速度平滑 + 减速前瞻
 * 前瞻逻辑：若当前加速度以最大 jerk 回收至 0 所产生的速度增量已接近误差，
 * 则提前将期望加速度切换到 0，减少速度过冲。
 */
static float scurve_axis_update(float current_speed, float target_speed,
                                float *current_accel,
                                float max_accel, float max_jerk, float dt) {
    float speed_error = target_speed - current_speed;
    float desired_accel;
    float accel_error;
    float max_accel_change;
    float new_speed;

    if (max_jerk <= 0.0f) {
        *current_accel = 0.0f;
        return trapezoid_axis_update(current_speed, target_speed, max_accel * dt);
    }

    if (absf_local(speed_error) <= SPEED_EPS && absf_local(*current_accel) <= ACCEL_EPS) {
        *current_accel = 0.0f;
        return target_speed;
    }

    {
        float abs_accel = absf_local(*current_accel);
        float t_to_zero = abs_accel / max_jerk;
        float dv_to_zero = 0.5f * abs_accel * t_to_zero;
        if (absf_local(speed_error) <= (dv_to_zero + SPEED_EPS)) {
            desired_accel = 0.0f;
        } else {
            desired_accel = (speed_error > 0.0f) ? max_accel : -max_accel;
        }
    }

    accel_error = desired_accel - *current_accel;
    max_accel_change = max_jerk * dt;
    if (accel_error > max_accel_change) {
        *current_accel += max_accel_change;
    } else if (accel_error < -max_accel_change) {
        *current_accel -= max_accel_change;
    } else {
        *current_accel = desired_accel;
    }

    *current_accel = clampf_local(*current_accel, -max_accel, max_accel);

    new_speed = current_speed + (*current_accel) * dt;

    if ((speed_error > 0.0f && new_speed > target_speed) ||
        (speed_error < 0.0f && new_speed < target_speed)) {
        new_speed = target_speed;
        *current_accel = 0.0f;
    }

    if (absf_local(target_speed - new_speed) <= SPEED_EPS && absf_local(*current_accel) <= ACCEL_EPS) {
        new_speed = target_speed;
        *current_accel = 0.0f;
    }

    return new_speed;
}

static void apply_rollover_limit(float *vx, float *vy) {
    float dx;
    float dy;
    float distance;
    float actual_limit;

    if (!g_planner.enable_rollover) {
        g_planner.last_vx = *vx;
        g_planner.last_vy = *vy;
        return;
    }

    dx = *vx - g_planner.last_vx;
    dy = *vy - g_planner.last_vy;
    distance = sqrtf(dx * dx + dy * dy);

    if (distance <= g_planner.rollover_threshold) {
        actual_limit = g_planner.rollover_threshold;
    } else if (distance <= g_planner.rollover_threshold +
                          (g_planner.rollover_max_limit - g_planner.rollover_base_limit) * 2.0f) {
        actual_limit = g_planner.rollover_base_limit + (distance - g_planner.rollover_threshold) / 2.0f;
    } else {
        actual_limit = g_planner.rollover_max_limit;
    }

    if (distance > actual_limit && distance > 0.0f) {
        float ratio = actual_limit / distance;
        *vx = g_planner.last_vx + dx * ratio;
        *vy = g_planner.last_vy + dy * ratio;
    }

    g_planner.last_vx = *vx;
    g_planner.last_vy = *vy;
}

static void clamp_all_speed(void) {
    g_planner.speed_x = clamp_speed(g_planner.speed_x, g_planner.max_speed_x);
    g_planner.speed_y = clamp_speed(g_planner.speed_y, g_planner.max_speed_y);
    g_planner.speed_w = clamp_speed(g_planner.speed_w, g_planner.max_speed_w);
}

/* ======================== 兼容 API 实现 ======================== */

void chassis_speed_plan_init(ChassisSpeedPlanType type) {
    if (type >= CHASSIS_SPEED_PLAN_NUM) {
        type = CHASSIS_SPEED_PLAN_COSINE;
    }
    chassis_speed_plan_type = type;

    g_planner.speed_x = 0.0f;
    g_planner.speed_y = 0.0f;
    g_planner.speed_w = 0.0f;
    g_planner.accel_x = 0.0f;
    g_planner.accel_y = 0.0f;
    g_planner.accel_w = 0.0f;
    g_planner.last_vx = 0.0f;
    g_planner.last_vy = 0.0f;
    g_planner.last_tick = 0u;
    g_planner.first_update = 1;

    memset(&g_planner.cos_x, 0, sizeof(g_planner.cos_x));
    memset(&g_planner.cos_y, 0, sizeof(g_planner.cos_y));
    memset(&g_planner.cos_w, 0, sizeof(g_planner.cos_w));

    apply_auto_jerk_if_needed();
}

ChassisSpeedPlanType chassis_get_speed_plan(void) {
    return chassis_speed_plan_type;
}

void easy_speed_calc_handle(float target_x, float target_y, float target_yaw) {
    float dt = get_time_delta(xTaskGetTickCount());
    if (dt <= 0.0f) {
        return;
    }

    g_planner.speed_x = trapezoid_axis_update(g_planner.speed_x, target_x, g_planner.max_accel_x * dt);
    g_planner.speed_y = trapezoid_axis_update(g_planner.speed_y, target_y, g_planner.max_accel_y * dt);
    g_planner.speed_w = trapezoid_axis_update(g_planner.speed_w, target_yaw, g_planner.max_accel_w * dt);
    clamp_all_speed();
}

void cosine_speed_calc_handle(float target_x, float target_y, float target_yaw) {
    float dt = get_time_delta(xTaskGetTickCount());
    if (dt <= 0.0f) {
        return;
    }

    cosine_axis_update(&g_planner.cos_x, target_x, g_planner.max_accel_x, dt, &g_planner.speed_x);
    cosine_axis_update(&g_planner.cos_y, target_y, g_planner.max_accel_y, dt, &g_planner.speed_y);
    cosine_axis_update(&g_planner.cos_w, target_yaw, g_planner.max_accel_w, dt, &g_planner.speed_w);
    clamp_all_speed();
}

void scurve_speed_plan_handle(float target_x, float target_y, float target_yaw) {
    float dt = get_time_delta(xTaskGetTickCount());
    if (dt <= 0.0f) {
        return;
    }

    g_planner.speed_x = scurve_axis_update(g_planner.speed_x, target_x, &g_planner.accel_x,
                                           g_planner.max_accel_x, g_planner.max_jerk_x, dt);
    g_planner.speed_y = scurve_axis_update(g_planner.speed_y, target_y, &g_planner.accel_y,
                                           g_planner.max_accel_y, g_planner.max_jerk_y, dt);
    g_planner.speed_w = scurve_axis_update(g_planner.speed_w, target_yaw, &g_planner.accel_w,
                                           g_planner.max_accel_w, g_planner.max_jerk_w, dt);
    clamp_all_speed();
}

void chassis_speed_plan(float input_x, float input_y, float input_yaw,
                        float *output_x, float *output_y, float *output_yaw) {
    switch (chassis_speed_plan_type) {
    case CHASSIS_SPEED_PLAN_EASY:
        easy_speed_calc_handle(input_x, input_y, input_yaw);
        break;
    case CHASSIS_SPEED_PLAN_COSINE:
        cosine_speed_calc_handle(input_x, input_y, input_yaw);
        break;
    case CHASSIS_SPEED_PLAN_S_CURVE:
        scurve_speed_plan_handle(input_x, input_y, input_yaw);
        break;
    default:
        cosine_speed_calc_handle(input_x, input_y, input_yaw);
        break;
    }

    speed_point_limit(&g_planner.speed_x, &g_planner.speed_y);

    *output_x = g_planner.speed_x;
    *output_y = g_planner.speed_y;
    *output_yaw = g_planner.speed_w;
}

void speed_point_limit(float *vx, float *vy) {
    apply_rollover_limit(vx, vy);
}

/* ======================== 新增配置接口 ======================== */

void chassis_speed_plan_set_tick_freq(uint32_t tick_freq_hz) {
    if (tick_freq_hz > 0u) {
        g_planner.tick_freq_hz = tick_freq_hz;
    }
}

void chassis_speed_plan_set_accel(float accel_x, float accel_y, float accel_w) {
    g_planner.max_accel_x = (accel_x < MIN_ACCEL) ? MIN_ACCEL : accel_x;
    g_planner.max_accel_y = (accel_y < MIN_ACCEL) ? MIN_ACCEL : accel_y;
    g_planner.max_accel_w = (accel_w < MIN_ACCEL) ? MIN_ACCEL : accel_w;
    apply_auto_jerk_if_needed();
}

void chassis_speed_plan_set_jerk(float jerk_x, float jerk_y, float jerk_w) {
    g_planner.max_jerk_x = jerk_x;
    g_planner.max_jerk_y = jerk_y;
    g_planner.max_jerk_w = jerk_w;
    apply_auto_jerk_if_needed();
}

void chassis_speed_plan_set_speed_limit(float speed_x, float speed_y, float speed_w) {
    g_planner.max_speed_x = speed_x;
    g_planner.max_speed_y = speed_y;
    g_planner.max_speed_w = speed_w;
}

void chassis_speed_plan_set_rollover(int enable, float base_limit, float max_limit, float threshold) {
    g_planner.enable_rollover = (enable != 0);

    if (base_limit > 0.0f) {
        g_planner.rollover_base_limit = base_limit;
    }
    if (max_limit > 0.0f) {
        g_planner.rollover_max_limit = max_limit;
    }
    if (threshold >= 0.0f) {
        g_planner.rollover_threshold = threshold;
    }

    if (g_planner.rollover_max_limit < g_planner.rollover_base_limit) {
        g_planner.rollover_max_limit = g_planner.rollover_base_limit;
    }
}
