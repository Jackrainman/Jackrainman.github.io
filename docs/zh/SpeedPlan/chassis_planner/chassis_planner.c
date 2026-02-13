/**
 * @file    chassis_planner.c
 * @author  Jackrainman
 * @brief   底盘速度规划模块 -- 纯速度规划 (梯形/余弦/S曲线)
 * @version 3.3
 * @date    2026-02-13
 *
 * 物理单位约定:
 *   线速度       : mm/s
 *   线加速度     : mm/s^2
 *   角速度       : rad/s
 *   角加速度     : rad/s^2
 *   时间         : 秒
 *
 * v3.3 改进:
 *   - 移除轨迹规划模式 (TRAJECTORY), 仅保留速度规划
 *   - 余弦模式替换为基于时间的完整余弦过渡 (移植自 v2.0)
 *     v(t) = v_start + Δv × 0.5 × (1 - cos(π × t / T))
 *     峰值加速度恰好等于配置的 a_max
 *
 * v3.2 改进:
 *   - 新增 chassis_planner_default_cfg() 默认配置函数
 *   - 新增 jerk 自动推算: jerk=0 时按 DEFAULT_JERK_RATIO(10) 倍 accel 自动填充
 *   - 新增 jerk 最低倍率校验: 手动指定的 jerk 必须 >= MIN_JERK_RATIO(1) 倍 accel
 *   - 新增 XY 对称配置辅助宏 (CHASSIS_ACCEL_SYM 等)
 *
 * v3.1 改进:
 *   - 修复 get_dt_seconds 时间不一致 bug
 *   - 增加配置参数校验和除零保护
 *   - 实现 interp_scurve_axis 真正的 S 曲线 (带 jerk 限制)
 *   - 速度模式增加速度限幅
 */

#include "chassis_planner.h"
#include "my_math/my_math.h"
#include <math.h>
#include <string.h>

/* ======================== 内部常量 ======================== */

#define MIN_ACCEL           0.001f    /* 最小加速度, 防止除零 (mm/s^2 或 rad/s^2) */
#define DEFAULT_JERK_RATIO  10.0f    /* jerk 自动推算比例: jerk = 10 * accel, t_ramp = 100ms */
#define MIN_JERK_RATIO       1.0f    /* jerk 最低倍率: jerk >= 1 * accel, t_ramp <= 1s */

#ifndef PI
#define PI 3.14159265358979f
#endif

/* ======================== 内部工具函数 ======================== */

/**
 * @brief 校验配置参数合法性
 *
 * @param cfg  配置指针
 * @return true: 合法, false: 存在非法参数
 */
static bool validate_config(const ChassisPlannerCfg *cfg) {
    if (cfg->tick_rate_hz == 0) return false;

    if (cfg->accel.max_accel_x < MIN_ACCEL) return false;
    if (cfg->accel.max_accel_y < MIN_ACCEL) return false;
    if (cfg->accel.max_accel_w < MIN_ACCEL) return false;

    if (cfg->enable_rollover_limit) {
        if (cfg->rollover.base_limit <= 0.0f) return false;
        if (cfg->rollover.max_limit < cfg->rollover.base_limit) return false;
        if (cfg->rollover.threshold < 0.0f) return false;
    }

    /* jerk 校验: >0 时必须 >= accel * MIN_JERK_RATIO (即 k >= 1, t_ramp <= 1s) */
    /* jerk == 0 不报错, 表示"自动推算" */
    if (cfg->jerk.max_jerk_x > 0.0f &&
        cfg->jerk.max_jerk_x < cfg->accel.max_accel_x * MIN_JERK_RATIO)
        return false;
    if (cfg->jerk.max_jerk_y > 0.0f &&
        cfg->jerk.max_jerk_y < cfg->accel.max_accel_y * MIN_JERK_RATIO)
        return false;
    if (cfg->jerk.max_jerk_w > 0.0f &&
        cfg->jerk.max_jerk_w < cfg->accel.max_accel_w * MIN_JERK_RATIO)
        return false;

    return true;
}

/**
 * @brief 将 FreeRTOS tick 差值转换为秒
 *
 * @param last_tick     上一次 tick 值
 * @param current_tick  当前 tick 值 (由调用者传入, 避免二次获取)
 * @param tick_rate     configTICK_RATE_HZ (通常 1000)
 * @return 时间差 (秒), 钳位于 [0, 0.1]
 */
static float get_dt_seconds(uint32_t last_tick, uint32_t current_tick, uint32_t tick_rate) {
    uint32_t delta_ticks = current_tick - last_tick; /* uint32 溢出安全 */
    float dt = (float)delta_ticks / (float)tick_rate;

    /* 安全钳位: dt > 100ms 说明任务被饿死或初始化异常 */
    if (dt > 0.1f) dt = 0.1f;
    if (dt <= 0.0f) dt = 0.0f;

    return dt;
}

/**
 * @brief 对单轴速度做限幅
 */
static float clamp_speed(float speed, float max_speed) {
    if (max_speed <= 0.0f) return speed; /* max_speed <= 0 表示不限制 */
    if (speed > max_speed)  return max_speed;
    if (speed < -max_speed) return -max_speed;
    return speed;
}

/* ======================== 单轴插值函数 ======================== */

/**
 * @brief 梯形 (线性斜坡) 单轴速度更新
 *
 * @param current     当前速度 (mm/s 或 rad/s)
 * @param target      目标速度 (mm/s 或 rad/s)
 * @param max_change  本周期最大允许变化量 = max_accel * dt (mm/s 或 rad/s)
 * @return 更新后的速度
 */
static float interp_trapezoid_axis(float current, float target, float max_change) {
    float diff = target - current;
    if (diff > max_change) {
        return current + max_change;
    } else if (diff < -max_change) {
        return current - max_change;
    } else {
        return target;
    }
}

/**
 * @brief 余弦过渡单轴更新 (移植自 v2.0 chassis_speed_planner)
 *
 * 基于时间的完整余弦速度过渡:
 *   v(t) = v_start + Δv × 0.5 × (1 - cos(π × t / T))
 *
 * 过渡时间由物理最大加速度决定:
 *   a(t) = dv/dt = Δv × π/(2T) × sin(πt/T)
 *   a_peak = |Δv| × π / (2T)
 *   令 a_peak = a_max  =>  T = π × |Δv| / (2 × a_max)
 *
 * 特性:
 *   - t=0 时 v=v_start, a=0 (平滑启动)
 *   - t=T 时 v=v_target, a=0 (平滑停止)
 *   - 峰值加速度恰好等于配置的 a_max
 *
 * @param cs            该轴的余弦过渡状态
 * @param target        目标速度
 * @param max_accel     该轴最大加速度
 * @param dt            时间步长 (s)
 * @param current_speed 当前速度指针
 */
static void interp_cosine_axis(CosineAxisState *cs, float target,
                                float max_accel, float dt,
                                float *current_speed) {
    /* 检测目标变化 → 从当前速度重新规划 */
    if (my_fabs(target - cs->target_speed) > 0.001f || !cs->active) {
        cs->start_speed  = *current_speed;
        cs->target_speed = target;
        float dv = my_fabs(target - *current_speed);
        /* T = π|Δv| / (2·a_max), 当 dv 很小时直接跳到目标 */
        cs->duration = (dv > 0.001f) ? (PI * dv) / (2.0f * max_accel) : 0.0f;
        cs->elapsed  = 0.0f;
        cs->active   = true;
    }

    /* 无需过渡 */
    if (!cs->active || cs->duration <= 0.0f) {
        *current_speed = target;
        cs->active = false;
        return;
    }

    /* 推进时间 */
    cs->elapsed += dt;

    if (cs->elapsed >= cs->duration) {
        /* 过渡完成 */
        *current_speed = cs->target_speed;
        cs->active = false;
    } else {
        /* 余弦插值 */
        float dv = cs->target_speed - cs->start_speed;
        float progress = cs->elapsed / cs->duration;
        *current_speed = cs->start_speed + dv * 0.5f * (1.0f - cosf(PI * progress));
    }
}

/**
 * @brief S型曲线单轴速度更新 (带 jerk 限制)
 *
 * 通过限制加速度的变化率 (jerk), 使加速度连续变化,
 * 产生真正的 S 形速度曲线. 加速度从 0 平滑升到 max_accel, 再平滑降到 0.
 *
 * @param current_speed  当前速度 (mm/s 或 rad/s)
 * @param target_speed   目标速度 (mm/s 或 rad/s)
 * @param current_accel  当前加速度指针 (mm/s^2 或 rad/s^2), 原地修改
 * @param max_accel      最大加速度 (mm/s^2 或 rad/s^2)
 * @param max_jerk       最大 jerk (mm/s^3 或 rad/s^3), <=0 时退化为梯形
 * @param dt             时间步长 (s)
 * @return 更新后的速度
 */
static float interp_scurve_axis(float current_speed, float target_speed,
                                float *current_accel,
                                float max_accel, float max_jerk, float dt) {
    /* jerk 未配置时退化为梯形 */
    if (max_jerk <= 0.0f) {
        float max_change = max_accel * dt;
        *current_accel = 0.0f;
        return interp_trapezoid_axis(current_speed, target_speed, max_change);
    }

    float speed_error = target_speed - current_speed;

    /* 计算期望加速度方向 */
    float desired_accel;
    if (my_fabs(speed_error) < 0.001f) {
        desired_accel = 0.0f;
    } else {
        desired_accel = (speed_error > 0.0f) ? max_accel : -max_accel;
    }

    /* 限制加速度变化率 (jerk 限制) */
    float accel_error = desired_accel - *current_accel;
    float max_accel_change = max_jerk * dt;
    if (accel_error > max_accel_change) {
        *current_accel += max_accel_change;
    } else if (accel_error < -max_accel_change) {
        *current_accel -= max_accel_change;
    } else {
        *current_accel = desired_accel;
    }

    /* 限制加速度幅值 */
    if (*current_accel > max_accel)  *current_accel = max_accel;
    if (*current_accel < -max_accel) *current_accel = -max_accel;

    /* 更新速度 */
    float new_speed = current_speed + *current_accel * dt;

    /* 防超调: 速度越过目标时钳位 */
    if ((speed_error > 0.0f && new_speed > target_speed) ||
        (speed_error < 0.0f && new_speed < target_speed)) {
        new_speed = target_speed;
        *current_accel = 0.0f;
    }

    return new_speed;
}

/* ======================== 速度模式分发 ======================== */

/**
 * @brief 对三轴执行插值
 *
 * target_x/y 为目标速度 (mm/s), target_w 为目标角速度 (rad/s).
 * 根据 cfg.interp_type 选择插值算法, 并对输出做速度限幅.
 *
 * @param inst      规划器实例
 * @param target_x  目标 vx (mm/s)
 * @param target_y  目标 vy (mm/s)
 * @param target_w  目标 omega (rad/s)
 * @param dt        时间步长 (s)
 */
static void update_speed_mode(ChassisPlannerInst *inst,
                              float target_x, float target_y, float target_w,
                              float dt) {
    switch (inst->cfg.interp_type) {
    case CHASSIS_INTERP_TRAPEZOID: {
        float max_dx = inst->cfg.accel.max_accel_x * dt;
        float max_dy = inst->cfg.accel.max_accel_y * dt;
        float max_dw = inst->cfg.accel.max_accel_w * dt;
        inst->speed_x = interp_trapezoid_axis(inst->speed_x, target_x, max_dx);
        inst->speed_y = interp_trapezoid_axis(inst->speed_y, target_y, max_dy);
        inst->speed_w = interp_trapezoid_axis(inst->speed_w, target_w, max_dw);
        break;
    }

    case CHASSIS_INTERP_COSINE:
        interp_cosine_axis(&inst->cos_x, target_x,
                           inst->cfg.accel.max_accel_x, dt, &inst->speed_x);
        interp_cosine_axis(&inst->cos_y, target_y,
                           inst->cfg.accel.max_accel_y, dt, &inst->speed_y);
        interp_cosine_axis(&inst->cos_w, target_w,
                           inst->cfg.accel.max_accel_w, dt, &inst->speed_w);
        break;

    case CHASSIS_INTERP_SCURVE:
        inst->speed_x = interp_scurve_axis(inst->speed_x, target_x,
                                           &inst->accel_x,
                                           inst->cfg.accel.max_accel_x,
                                           inst->cfg.jerk.max_jerk_x, dt);
        inst->speed_y = interp_scurve_axis(inst->speed_y, target_y,
                                           &inst->accel_y,
                                           inst->cfg.accel.max_accel_y,
                                           inst->cfg.jerk.max_jerk_y, dt);
        inst->speed_w = interp_scurve_axis(inst->speed_w, target_w,
                                           &inst->accel_w,
                                           inst->cfg.accel.max_accel_w,
                                           inst->cfg.jerk.max_jerk_w, dt);
        break;

    default:
        /* 未知类型: 回退到梯形 (最安全) */
        {
            float max_dx = inst->cfg.accel.max_accel_x * dt;
            float max_dy = inst->cfg.accel.max_accel_y * dt;
            float max_dw = inst->cfg.accel.max_accel_w * dt;
            inst->speed_x = interp_trapezoid_axis(inst->speed_x, target_x, max_dx);
            inst->speed_y = interp_trapezoid_axis(inst->speed_y, target_y, max_dy);
            inst->speed_w = interp_trapezoid_axis(inst->speed_w, target_w, max_dw);
        }
        break;
    }

    /* 速度限幅 */
    inst->speed_x = clamp_speed(inst->speed_x, inst->cfg.speed_limit.max_speed_x);
    inst->speed_y = clamp_speed(inst->speed_y, inst->cfg.speed_limit.max_speed_y);
    inst->speed_w = clamp_speed(inst->speed_w, inst->cfg.speed_limit.max_speed_w);
}

/* ======================== 防侧翻限速 ======================== */

/**
 * @brief 防侧翻速度向量限制
 *
 * 使用梯形约束函数限制 2D 速度向量 (vx, vy) 的变化率.
 * 防止突然的方向变化导致机器人侧翻.
 *
 * 约束逻辑:
 *   - 速度变化量 distance = sqrt(dvx^2 + dvy^2) (mm/s)
 *   - distance <= threshold: 不限制
 *   - threshold < distance <= threshold + (max_limit - base_limit) * 2: 线性斜坡限制
 *   - distance > 斜坡上限: 钳位到 max_limit
 *
 * @param inst  规划器实例 (读写 rollover 状态)
 * @param vx    X 速度指针 (mm/s), 原地修改
 * @param vy    Y 速度指针 (mm/s), 原地修改
 */
static void apply_rollover_limit(ChassisPlannerInst *inst, float *vx, float *vy) {
    const ChassisRolloverCfg *cfg = &inst->cfg.rollover;

    float dx = *vx - inst->rollover.last_vx;
    float dy = *vy - inst->rollover.last_vy;
    float distance = sqrtf(dx * dx + dy * dy); /* 速度变化量 (mm/s) */

    float actual_limit;
    if (distance <= cfg->threshold) {
        actual_limit = cfg->threshold;
    } else if (distance <= cfg->threshold + (cfg->max_limit - cfg->base_limit) * 2.0f) {
        actual_limit = cfg->base_limit + (distance - cfg->threshold) / 2.0f;
    } else {
        actual_limit = cfg->max_limit;
    }

    if (distance > actual_limit) {
        float ratio = actual_limit / distance;
        *vx = inst->rollover.last_vx + dx * ratio;
        *vy = inst->rollover.last_vy + dy * ratio;
    }

    inst->rollover.last_vx = *vx;
    inst->rollover.last_vy = *vy;
}

/* ======================== 公共 API 实现 ======================== */

ChassisPlannerCfg chassis_planner_default_cfg(ChassisInterpType type) {
    ChassisPlannerCfg cfg = {
        .interp_type           = type,
        .accel                 = { .max_accel_x = 40.0f,
                                   .max_accel_y = 40.0f,
                                   .max_accel_w = 20.0f },
        .speed_limit           = { .max_speed_x = 5000.0f,
                                   .max_speed_y = 5000.0f,
                                   .max_speed_w = 10.0f },
        .jerk                  = { 0.0f, 0.0f, 0.0f },  /* 0 = 自动推算 */
        .rollover              = { .base_limit = 30.0f,
                                   .max_limit  = 40.0f,
                                   .threshold  = 30.0f },
        .enable_rollover_limit = false,
        .tick_rate_hz          = 1000,
    };
    return cfg;
}

bool chassis_planner_init(ChassisPlannerInst *inst, const ChassisPlannerCfg *cfg) {
    if (!validate_config(cfg)) return false;

    memset(inst, 0, sizeof(ChassisPlannerInst));
    inst->cfg = *cfg;

    /* jerk 自动推算: jerk <= 0 时按 DEFAULT_JERK_RATIO * accel 填充 */
    if (inst->cfg.jerk.max_jerk_x <= 0.0f)
        inst->cfg.jerk.max_jerk_x = inst->cfg.accel.max_accel_x * DEFAULT_JERK_RATIO;
    if (inst->cfg.jerk.max_jerk_y <= 0.0f)
        inst->cfg.jerk.max_jerk_y = inst->cfg.accel.max_accel_y * DEFAULT_JERK_RATIO;
    if (inst->cfg.jerk.max_jerk_w <= 0.0f)
        inst->cfg.jerk.max_jerk_w = inst->cfg.accel.max_accel_w * DEFAULT_JERK_RATIO;

    inst->first_update = true;
    inst->last_tick = xTaskGetTickCount();
    return true;
}

void chassis_planner_update(ChassisPlannerInst *inst,
                            float target_x, float target_y, float target_w,
                            float *out_vx, float *out_vy, float *out_vw) {
    uint32_t current_tick = xTaskGetTickCount();
    float dt = get_dt_seconds(inst->last_tick, current_tick, inst->cfg.tick_rate_hz);
    inst->last_tick = current_tick;

    /* dt 为零 (同一 tick 内被调用两次) 则直接输出当前值 */
    if (dt <= 0.0f) {
        *out_vx = inst->speed_x;
        *out_vy = inst->speed_y;
        *out_vw = inst->speed_w;
        return;
    }

    /* 速度规划 */
    update_speed_mode(inst, target_x, target_y, target_w, dt);

    /* 后处理: 防侧翻限速 */
    if (inst->cfg.enable_rollover_limit) {
        apply_rollover_limit(inst, &inst->speed_x, &inst->speed_y);
    }

    /* 输出 */
    *out_vx = inst->speed_x;
    *out_vy = inst->speed_y;
    *out_vw = inst->speed_w;
}

void chassis_planner_reset(ChassisPlannerInst *inst) {
    ChassisPlannerCfg cfg_backup = inst->cfg;
    memset(inst, 0, sizeof(ChassisPlannerInst));
    inst->cfg = cfg_backup;
    inst->first_update = true;
    inst->last_tick = xTaskGetTickCount();
}

void chassis_planner_set_interp(ChassisPlannerInst *inst, ChassisInterpType type) {
    inst->cfg.interp_type = type;
    chassis_planner_reset(inst);
}
