/**
 * @file    chassis_speed_planner.c
 * @author  Jackrainman
 * @brief   底盘速度规划模块（实例化版本）
 *          重构自 chassis_calculations，支持多实例、物理参数配置
 * @version 2.0
 * @date    2025-11-22
 *
 * 改动说明:
 *   1. 全部状态封装进 SpeedPlanner_t，消除全局/静态变量
 *   2. get_time_delta 正确返回秒 (tick差 / tick_freq_hz)
 *   3. 余弦模式重写为基于时间的余弦过渡 v(t)=v0+Δv·0.5·(1-cos(πt/T))
 *   4. 所有配置参数具有明确物理单位，无经验乘数
 *   5. 删除 ~150 行 #if 0 死代码
 */

#include "chassis_speed_planner.h"
#include "my_math/my_math.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

/* ================================================================
 *  内部工具函数
 * ================================================================ */

/**
 * @brief 计算速度差值（保持原函数不变）
 */
static float calculate_speed_diff(float target, float current) {
    return target - current;
}

/* ================================================================
 *  梯形加减速
 * ================================================================ */

/**
 * @brief 梯形算法更新单轴速度（保持原逻辑不变）
 * @param diff          速度差值 (mm/s 或 rad/s)
 * @param max_change    本周期最大允许速度变化量 = a_max × dt
 * @param current_speed 当前速度指针
 * @param target_speed  目标速度
 */
static void trapezoid_update_speed(float diff, float max_change,
                                   float *current_speed, float target_speed) {
    if (diff > max_change) {
        *current_speed = *current_speed + max_change;
    } else if (diff < -max_change) {
        *current_speed = *current_speed - max_change;
    } else {
        *current_speed = target_speed;
    }
}

/**
 * @brief 梯形速度规划三轴处理
 */
static void trapezoid_handle(SpeedPlanner_t *inst,
                             float target_x, float target_y, float target_w,
                             float dt) {
    float max_change_x = inst->cfg.max_accel_x * dt;
    float max_change_y = inst->cfg.max_accel_y * dt;
    float max_change_w = inst->cfg.max_accel_w * dt;

    float diff_x = calculate_speed_diff(target_x, inst->speed_x);
    float diff_y = calculate_speed_diff(target_y, inst->speed_y);
    float diff_w = calculate_speed_diff(target_w, inst->speed_w);

    trapezoid_update_speed(diff_x, max_change_x, &inst->speed_x, target_x);
    trapezoid_update_speed(diff_y, max_change_y, &inst->speed_y, target_y);
    trapezoid_update_speed(diff_w, max_change_w, &inst->speed_w, target_w);
}

/* ================================================================
 *  余弦平滑加减速（修正实现）
 *
 *  速度过渡公式:
 *    v(t) = v_start + Δv × 0.5 × (1 - cos(π × t / T))
 *
 *  过渡时间由物理最大加速度决定:
 *    a(t) = dv/dt = Δv × π/(2T) × sin(πt/T)
 *    a_peak = |Δv| × π / (2T)
 *    令 a_peak = a_max  =>  T = π × |Δv| / (2 × a_max)
 *
 *  特性:
 *    - t=0 时 v=v_start, a=0（平滑启动）
 *    - t=T 时 v=v_target, a=0（平滑停止）
 *    - 峰值加速度恰好等于配置的 a_max
 * ================================================================ */

/**
 * @brief 余弦过渡单轴更新
 * @param cs           该轴的余弦过渡状态
 * @param target       目标速度
 * @param max_accel    该轴最大加速度
 * @param dt           时间步长 (s)
 * @param current_speed 当前速度指针
 */
static void cosine_axis_update(CosineAxisState *cs, float target,
                               float max_accel, float dt,
                               float *current_speed) {
    /* 检测目标变化 → 从当前速度重新规划 */
    if (my_fabs(target - cs->target_speed) > 0.001f || !cs->active) {
        cs->start_speed  = *current_speed;
        cs->target_speed = target;
        float dv = my_fabs(target - *current_speed);
        /* T = π|Δv| / (2·a_max)，当 dv 很小时直接跳到目标 */
        cs->duration = (dv > 0.001f) ? (PI * dv) / (2.0f * max_accel) : 0.0f;
        cs->elapsed  = 0.0f;
        cs->active   = 1;
    }

    /* 无需过渡 */
    if (!cs->active || cs->duration <= 0.0f) {
        *current_speed = target;
        cs->active = 0;
        return;
    }

    /* 推进时间 */
    cs->elapsed += dt;

    if (cs->elapsed >= cs->duration) {
        /* 过渡完成 */
        *current_speed = cs->target_speed;
        cs->active = 0;
    } else {
        /* 余弦插值 */
        float dv = cs->target_speed - cs->start_speed;
        float progress = cs->elapsed / cs->duration;
        *current_speed = cs->start_speed + dv * 0.5f * (1.0f - cosf(PI * progress));
    }
}

/**
 * @brief 余弦速度规划三轴处理
 */
static void cosine_handle(SpeedPlanner_t *inst,
                          float target_x, float target_y, float target_w,
                          float dt) {
    cosine_axis_update(&inst->cos_x, target_x, inst->cfg.max_accel_x,
                       dt, &inst->speed_x);
    cosine_axis_update(&inst->cos_y, target_y, inst->cfg.max_accel_y,
                       dt, &inst->speed_y);
    cosine_axis_update(&inst->cos_w, target_w, inst->cfg.max_accel_w,
                       dt, &inst->speed_w);
}

/* ================================================================
 *  S型曲线（带 jerk 限制）
 *
 *  通过限制加加速度(jerk)使加速度连续变化:
 *    jerk → 加速度线性变化 → 速度平滑过渡
 *
 *  每步更新逻辑:
 *    1. 根据速度误差方向，用 jerk 调节加速度
 *    2. 加速度限幅到 ±a_max
 *    3. 用加速度更新速度
 *    4. 防超调
 * ================================================================ */

/**
 * @brief S型曲线单轴更新
 * @param diff          速度差值
 * @param max_accel     该轴最大加速度
 * @param max_jerk      该轴最大 jerk
 * @param dt            时间步长 (s)
 * @param current_accel 当前加速度指针（状态量）
 * @param current_speed 当前速度指针
 * @param target_speed  目标速度
 */
static void scurve_axis_update(float diff, float max_accel, float max_jerk,
                               float dt, float *current_accel,
                               float *current_speed, float target_speed) {
    if (my_fabs(diff) <= 0.001f) {
        *current_speed = target_speed;
        *current_accel = 0.0f;
        return;
    }

    float max_acc_change = max_jerk * dt;

    /* 根据速度误差方向调节加速度 */
    if (diff > 0.0f) {
        /* 需要加速 */
        *current_accel += max_acc_change;
        if (*current_accel > max_accel) {
            *current_accel = max_accel;
        }
    } else {
        /* 需要减速 */
        *current_accel -= max_acc_change;
        if (*current_accel < -max_accel) {
            *current_accel = -max_accel;
        }
    }

    /* 接近目标时的减速规划:
     * 制动距离估算: v²/(2a) 对应到速度域为 v_brake = sqrt(2·a·|diff|)
     * 当 |current_accel·dt| 已接近 |diff| 时，应开始收敛加速度 */
    float brake_speed = sqrtf(2.0f * max_accel * my_fabs(diff));
    if (my_fabs(*current_speed - target_speed) > brake_speed * 0.5f) {
        /* 在制动区域内，开始降低加速度 */
        if (diff > 0.0f && *current_accel > 0.0f) {
            float desired_accel = diff / dt * 0.5f;
            if (desired_accel < *current_accel) {
                *current_accel -= max_acc_change;
            }
        } else if (diff < 0.0f && *current_accel < 0.0f) {
            float desired_accel = diff / dt * 0.5f;
            if (desired_accel > *current_accel) {
                *current_accel += max_acc_change;
            }
        }
    }

    /* 更新速度 */
    *current_speed += (*current_accel) * dt;

    /* 防超调 */
    float new_diff = target_speed - *current_speed;
    if ((diff > 0.0f && new_diff < 0.0f) ||
        (diff < 0.0f && new_diff > 0.0f)) {
        *current_speed = target_speed;
        *current_accel = 0.0f;
    }
}

/**
 * @brief S型速度规划三轴处理
 */
static void scurve_handle(SpeedPlanner_t *inst,
                          float target_x, float target_y, float target_w,
                          float dt) {
    float diff_x = calculate_speed_diff(target_x, inst->speed_x);
    float diff_y = calculate_speed_diff(target_y, inst->speed_y);
    float diff_w = calculate_speed_diff(target_w, inst->speed_w);

    scurve_axis_update(diff_x, inst->cfg.max_accel_x, inst->cfg.max_jerk_x,
                       dt, &inst->accel_x, &inst->speed_x, target_x);
    scurve_axis_update(diff_y, inst->cfg.max_accel_y, inst->cfg.max_jerk_y,
                       dt, &inst->accel_y, &inst->speed_y, target_y);
    scurve_axis_update(diff_w, inst->cfg.max_accel_w, inst->cfg.max_jerk_w,
                       dt, &inst->accel_w, &inst->speed_w, target_w);
}

/* ================================================================
 *  防侧翻速度向量限幅（保持原算法逻辑，状态移入实例）
 *
 *  原理: 不限制速度绝对值，而是限制相邻周期速度向量的变化量。
 *  使用梯形函数平滑过渡限制值。
 * ================================================================ */

/**
 * @brief 防侧翻速度向量限幅
 * @param inst 规划器实例（读写 last_vx/last_vy 状态）
 * @param vx   X速度指针（可能被限幅修改）
 * @param vy   Y速度指针（可能被限幅修改）
 */
static void rollover_speed_limit(SpeedPlanner_t *inst, float *vx, float *vy) {
    float dx = *vx - inst->last_vx;
    float dy = *vy - inst->last_vy;
    float distance = sqrtf(dx * dx + dy * dy);

    float threshold  = inst->cfg.rollover_threshold;
    float base_limit = inst->cfg.rollover_base_limit;
    float max_limit  = inst->cfg.rollover_max_limit;

    /* 梯形函数计算实际速度限制 */
    float actual_limit;
    if (distance <= threshold) {
        /* 低于阈值: 不限制 */
        actual_limit = threshold;
    } else if (distance <= threshold + (max_limit - base_limit) * 2.0f) {
        /* 梯形斜坡段 */
        actual_limit = base_limit + (distance - threshold) / 2.0f;
    } else {
        /* 超出梯形顶部: 使用最大限制 */
        actual_limit = max_limit;
    }

    if (distance > actual_limit) {
        /* 沿变化向量方向缩放到限制值 */
        float ratio = actual_limit / distance;
        *vx = inst->last_vx + dx * ratio;
        *vy = inst->last_vy + dy * ratio;
    }

    /* 保存本周期输出 */
    inst->last_vx = *vx;
    inst->last_vy = *vy;
}

/* ================================================================
 *  公共 API
 * ================================================================ */

void speed_planner_init(SpeedPlan`ner_t *inst, const SpeedPlannerCfg *cfg) {
    /* 清零全部状态 */
    memset(inst, 0, sizeof(SpeedPlanner_t));
    /* 拷贝配置 */
    inst->cfg = *cfg;
    /* 默认 tick 频率 */
    if (inst->cfg.tick_freq_hz == 0) {
        inst->cfg.tick_freq_hz = 1000;
    }
    inst->first_update = 1;
}

void speed_planner_update(SpeedPlanner_t *inst,
                          float target_x, float target_y, float target_w,
                          float *out_vx, float *out_vy, float *out_vw) {
    uint32_t current_tick = xTaskGetTickCount();

    /* 首次调用: 仅记录时间戳，输出零速 */
    if (inst->first_update) {
        inst->last_tick    = current_tick;
        inst->first_update = 0;
        *out_vx  = inst->speed_x;
        *out_vy  = inst->speed_y;
        *out_vw  = inst->speed_w;
        return;
    }

    /* 计算时间步长（秒） */
    float dt = (float)(current_tick - inst->last_tick) / (float)inst->cfg.tick_freq_hz;
    inst->last_tick = current_tick;

    /* 时间步长保护: 过小或过大均跳过 */
    if (dt <= 0.0f || dt > 0.1f) {
        *out_vx = inst->speed_x;
        *out_vy = inst->speed_y;
        *out_vw = inst->speed_w;
        return;
    }

    /* 根据算法类型分发 */
    switch (inst->cfg.plan_type) {
        case SPEED_PLAN_TRAPEZOID:
            trapezoid_handle(inst, target_x, target_y, target_w, dt);
            break;
        case SPEED_PLAN_COSINE:
            cosine_handle(inst, target_x, target_y, target_w, dt);
            break;
        case SPEED_PLAN_SCURVE:
            scurve_handle(inst, target_x, target_y, target_w, dt);
            break;
        default:
            trapezoid_handle(inst, target_x, target_y, target_w, dt);
            break;
    }

    /* 防侧翻限速 */
    if (inst->cfg.enable_rollover) {
        rollover_speed_limit(inst, &inst->speed_x, &inst->speed_y);
    }

    /* 输出 */
    *out_vx = inst->speed_x;
    *out_vy = inst->speed_y;
    *out_vw = inst->speed_w;
}

void speed_planner_reset(SpeedPlanner_t *inst) {
    SpeedPlannerCfg cfg_backup = inst->cfg;
    memset(inst, 0, sizeof(SpeedPlanner_t));
    inst->cfg = cfg_backup;
    inst->first_update = 1;
}

void speed_planner_set_type(SpeedPlanner_t *inst, SpeedPlanType type) {
    if (type < SPEED_PLAN_NUM) {
        inst->cfg.plan_type = type;
    }
}

SpeedPlanType speed_planner_get_type(const SpeedPlanner_t *inst) {
    return inst->cfg.plan_type;
}
