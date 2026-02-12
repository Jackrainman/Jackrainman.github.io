/**
 * @file    chassis_planner.c
 * @author  Jackrainman
 * @brief   底盘速度规划模块 -- 合并 v1.0 速度规划 + v2.0 轨迹规划
 * @version 3.1
 * @date    2026-02-10
 *
 * 物理单位约定:
 *   线速度       : mm/s
 *   线加速度     : mm/s^2
 *   角速度       : rad/s
 *   角加速度     : rad/s^2
 *   时间         : 秒
 *
 * v3.1 改进:
 *   - 修复 get_dt_seconds 时间不一致 bug
 *   - 增加配置参数校验和除零保护
 *   - 修正 interp_cosine_axis 余弦平滑算法
 *   - 实现 interp_scurve_axis 真正的 S 曲线 (带 jerk 限制)
 *   - 轨迹重规划时考虑当前速度, 避免速度跳变
 *   - 轨迹模式三轴时间同步, 保证直线路径
 *   - 速度模式增加速度限幅
 *   - 增加起始位置可配置 API
 *   - 修正 is_finished 在非轨迹模式下的语义
 */

#include "chassis_planner.h"
#include "my_math/my_math.h"
#include <math.h>
#include <string.h>

/* ======================== 内部常量 ======================== */

#define MIN_ACCEL       0.001f    /* 最小加速度, 防止除零 (mm/s^2 或 rad/s^2) */
#define MIN_DISTANCE    0.001f    /* 最小距离, 低于此值视为到达 (mm 或 rad) */
#define TARGET_CHANGE_THRESHOLD  0.1f  /* 目标变化检测阈值 (mm 或 rad) */

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

    if (cfg->interp_type == CHASSIS_INTERP_TRAJECTORY) {
        if (cfg->speed_limit.max_speed_x < MIN_ACCEL) return false;
        if (cfg->speed_limit.max_speed_y < MIN_ACCEL) return false;
        if (cfg->speed_limit.max_speed_w < MIN_ACCEL) return false;
    }

    if (cfg->enable_rollover_limit) {
        if (cfg->rollover.base_limit <= 0.0f) return false;
        if (cfg->rollover.max_limit < cfg->rollover.base_limit) return false;
        if (cfg->rollover.threshold < 0.0f) return false;
    }

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
 * @brief 余弦平滑单轴速度更新
 *
 * 在远离目标时按线性斜坡（梯形）变化, 在接近目标时用余弦函数平滑减速.
 * 平滑区间为 smooth_zone = 2 * max_change, 在此区间内加速度连续趋近于零.
 *
 * @param current     当前速度 (mm/s 或 rad/s)
 * @param target      目标速度 (mm/s 或 rad/s)
 * @param max_change  本周期最大允许变化量 = max_accel * dt (mm/s 或 rad/s)
 * @return 更新后的速度
 */
static float interp_cosine_axis(float current, float target, float max_change) {
    float diff = target - current;
    float abs_diff = my_fabs(diff);

    if (abs_diff < 0.001f) return target;

    /* 平滑区间: 当距离目标小于 2 倍步长时, 用余弦平滑减速 */
    float smooth_zone = 2.0f * max_change;

    float step;
    if (abs_diff <= smooth_zone) {
        /* 在平滑区间内: t 从 1(刚进入) 到 0(到达目标) */
        float t = abs_diff / smooth_zone;
        /* 余弦映射: t=1 时 step ≈ max_change, t→0 时 step→0 */
        step = abs_diff * 0.5f * (1.0f - cosf(PI * t));
        /* 确保 step 不超过 abs_diff */
        if (step > abs_diff) step = abs_diff;
    } else {
        /* 远离目标: 使用线性斜坡 (与梯形相同) */
        step = max_change;
    }

    if (diff > 0.0f) {
        return current + step;
    } else {
        return current - step;
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
 * @brief 速度模式: 对三轴执行插值
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
    /* max_accel (mm/s^2) * dt (s) = max_change (mm/s) */
    float max_dx = inst->cfg.accel.max_accel_x * dt;
    float max_dy = inst->cfg.accel.max_accel_y * dt;
    float max_dw = inst->cfg.accel.max_accel_w * dt;

    switch (inst->cfg.interp_type) {
    case CHASSIS_INTERP_TRAPEZOID:
        inst->speed_x = interp_trapezoid_axis(inst->speed_x, target_x, max_dx);
        inst->speed_y = interp_trapezoid_axis(inst->speed_y, target_y, max_dy);
        inst->speed_w = interp_trapezoid_axis(inst->speed_w, target_w, max_dw);
        break;

    case CHASSIS_INTERP_COSINE:
        inst->speed_x = interp_cosine_axis(inst->speed_x, target_x, max_dx);
        inst->speed_y = interp_cosine_axis(inst->speed_y, target_y, max_dy);
        inst->speed_w = interp_cosine_axis(inst->speed_w, target_w, max_dw);
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
        inst->speed_x = interp_trapezoid_axis(inst->speed_x, target_x, max_dx);
        inst->speed_y = interp_trapezoid_axis(inst->speed_y, target_y, max_dy);
        inst->speed_w = interp_trapezoid_axis(inst->speed_w, target_w, max_dw);
        break;
    }

    /* 速度限幅 */
    inst->speed_x = clamp_speed(inst->speed_x, inst->cfg.speed_limit.max_speed_x);
    inst->speed_y = clamp_speed(inst->speed_y, inst->cfg.speed_limit.max_speed_y);
    inst->speed_w = clamp_speed(inst->speed_w, inst->cfg.speed_limit.max_speed_w);
}

/* ======================== 轨迹规划 (来自 v2.0) ======================== */

/**
 * @brief 初始化单轴梯形轨迹
 *
 * 预计算加速时间、匀速时间、总时间.
 * 自动判断梯形/三角形轨迹 (距离不足时降低最大速度).
 * 支持非零初始速度的重规划: 如果初速方向一致且不超过 max_speed,
 * 则缩短加速段; 否则按零初速处理.
 *
 * @param axis          轴轨迹结构体
 * @param current_pos   当前位置 (mm 或 rad)
 * @param current_speed 当前速度 (mm/s 或 rad/s), 用于重规划时保持速度连续
 * @param target_pos    目标位置 (mm 或 rad)
 * @param max_speed     最大速度 (mm/s 或 rad/s)
 * @param max_accel     最大加速度 (mm/s^2 或 rad/s^2)
 */
static void traj_axis_init(AxisTrajectory *axis,
                           float current_pos,
                           float current_speed,
                           float target_pos,
                           float max_speed,
                           float max_accel) {
    axis->start_pos = current_pos;
    axis->current_pos = current_pos;
    axis->target_pos = target_pos;
    axis->max_speed = fabsf(max_speed);
    axis->max_accel = fabsf(max_accel);
    axis->current_time = 0.0f;

    /* 除零保护 */
    if (axis->max_accel < MIN_ACCEL) {
        axis->state = CHASSIS_STATE_FINISHED;
        axis->t_total = 0.0f;
        axis->t_accel = 0.0f;
        axis->t_uniform = 0.0f;
        axis->current_speed = 0.0f;
        axis->start_speed = 0.0f;
        return;
    }

    /* 计算距离和方向 */
    float distance = target_pos - current_pos;
    axis->direction = (distance >= 0) ? 1 : -1;
    float D = fabsf(distance);

    /* 距离太小, 直接完成 */
    if (D < MIN_DISTANCE) {
        axis->state = CHASSIS_STATE_FINISHED;
        axis->t_total = 0.0f;
        axis->t_accel = 0.0f;
        axis->t_uniform = 0.0f;
        axis->current_speed = 0.0f;
        axis->start_speed = 0.0f;
        return;
    }

    /* 处理初始速度: 只有方向一致且不超过 max_speed 时才使用 */
    float v0 = 0.0f;
    float speed_along_dir = current_speed * axis->direction; /* 沿运动方向的速度分量 */
    if (speed_along_dir > 0.0f && speed_along_dir <= axis->max_speed) {
        v0 = speed_along_dir;
    }
    axis->start_speed = v0 * axis->direction;
    axis->current_speed = axis->start_speed;

    /* 计算从 v0 加速到 v_max 的时间和距离 */
    float t_accel_up = (axis->max_speed - v0) / axis->max_accel;
    float p_accel_up = v0 * t_accel_up + 0.5f * axis->max_accel * t_accel_up * t_accel_up;

    /* 减速段: 从 v_max 减速到 0 */
    float t_decel = axis->max_speed / axis->max_accel;
    float p_decel = 0.5f * axis->max_accel * t_decel * t_decel;

    if (D < p_accel_up + p_decel) {
        /* 三角形轨迹: 距离不足以达到最大速度 */
        /* 求解峰值速度 v_peak: p_up(v0→vp) + p_down(vp→0) = D */
        /* (vp^2 - v0^2)/(2a) + vp^2/(2a) = D */
        /* 2*vp^2 - v0^2 = 2*a*D */
        /* vp = sqrt((v0^2 + 2*a*D) / 2) */
        float v_peak_sq = (v0 * v0 + 2.0f * axis->max_accel * D) / 2.0f;
        if (v_peak_sq < 0.0f) v_peak_sq = 0.0f;
        float v_peak = sqrtf(v_peak_sq);

        if (v_peak < v0) {
            /* 需要先减速: 初速度已经太高, 直接以当前加速度减速 */
            v_peak = v0;
        }

        axis->max_speed = v_peak;
        axis->t_accel = (axis->max_speed - v0) / axis->max_accel;
        axis->t_uniform = 0.0f;
        axis->p_accel = v0 * axis->t_accel +
                        0.5f * axis->max_accel * axis->t_accel * axis->t_accel;

        /* 减速段时间 */
        t_decel = axis->max_speed / axis->max_accel;
    } else {
        /* 梯形轨迹: 有匀速段 */
        axis->t_accel = t_accel_up;
        axis->p_accel = p_accel_up;
        float p_uniform = D - p_accel_up - p_decel;
        axis->t_uniform = p_uniform / axis->max_speed;
        t_decel = axis->max_speed / axis->max_accel;
    }

    /* 总时间 = 加速 + 匀速 + 减速 */
    axis->t_total = axis->t_accel + axis->t_uniform + t_decel;

    axis->state = (axis->t_accel > 0.001f) ? CHASSIS_STATE_ACCELERATING : CHASSIS_STATE_UNIFORM;
}

/**
 * @brief 将单轴轨迹时间拉伸到指定总时间 (用于三轴同步)
 *
 * 在保持加速度不变的前提下, 降低 max_speed 使总时间等于 t_target.
 * 公式: v^2 - T*a*v + a*D = 0 (其中 T = t_target, a = max_accel, D = 距离)
 * 简化处理: 假设初速为 0 的对称梯形/三角形.
 *
 * @param axis      轴轨迹结构体
 * @param t_target  目标总时间 (s)
 */
static void traj_axis_rescale_time(AxisTrajectory *axis, float t_target) {
    if (axis->state == CHASSIS_STATE_FINISHED) return;
    if (t_target <= 0.001f) return;

    float D = fabsf(axis->target_pos - axis->start_pos);
    if (D < MIN_DISTANCE) return;

    float a = axis->max_accel;
    float v0 = fabsf(axis->start_speed);

    /* 求解新的 max_speed 使总时间 = t_target */
    /* 对于 v0=0 的对称梯形: v^2 - T*a*v + a*D = 0 */
    /* 对于 v0≠0, 近似处理: 用 v0=0 的公式作为初始估计 */
    float T = t_target;
    float discriminant = T * T * a * a - 4.0f * a * D;

    float v_new;
    if (discriminant >= 0.0f) {
        /* 取较小根 (梯形解) */
        v_new = (T * a - sqrtf(discriminant)) / 2.0f;
        if (v_new < v0) v_new = v0 + 0.001f; /* 新速度不能低于初速 */
    } else {
        /* 无解: t_target 过短, 保持原轨迹 */
        return;
    }

    /* 用新的 max_speed 重新初始化 */
    float saved_start_speed = axis->start_speed;
    traj_axis_init(axis, axis->start_pos, saved_start_speed,
                   axis->target_pos, v_new, a);
}

/**
 * @brief 更新单轴轨迹 (每个控制周期调用)
 *
 * 三阶段执行: 加速 → 匀速 → 减速.
 * 支持非零初速: 加速段从 v0 加速到 v_max.
 * 减速段从终点反算, 防止超调.
 *
 * @param axis  轴轨迹结构体
 * @param dt    时间步长 (s)
 * @return 1: 运动中, 0: 已完成
 */
static int traj_axis_update(AxisTrajectory *axis, float dt) {
    if (axis->state == CHASSIS_STATE_FINISHED) {
        axis->current_pos = axis->target_pos;
        axis->current_speed = 0.0f;
        return 0;
    }

    axis->current_time += dt;

    /* 超过总时间, 强制完成 */
    if (axis->current_time >= axis->t_total) {
        axis->current_time = axis->t_total;
        axis->current_pos = axis->target_pos;
        axis->current_speed = 0.0f;
        axis->state = CHASSIS_STATE_FINISHED;
        return 0;
    }

    float t  = axis->current_time;
    float a  = axis->max_accel;
    float v  = axis->max_speed;
    float v0 = fabsf(axis->start_speed);
    float Ta = axis->t_accel;
    float Tv = axis->t_uniform;

    /* 减速段起始时间 */
    float t_decel_start = Ta + Tv;
    /* 减速段时间 = v_max / a */
    float t_decel = (a > MIN_ACCEL) ? (v / a) : 0.0f;

    if (t <= Ta) {
        /* 加速阶段: p = p_start + v0*t + 0.5*a*t^2, v = v0 + a*t */
        axis->state = CHASSIS_STATE_ACCELERATING;
        axis->current_pos = axis->start_pos +
                            (v0 * t + 0.5f * a * t * t) * axis->direction;
        axis->current_speed = (v0 + a * t) * axis->direction;

    } else if (t <= t_decel_start) {
        /* 匀速阶段: p = p_start + p_accel + v_max * (t - Ta) */
        axis->state = CHASSIS_STATE_UNIFORM;
        axis->current_pos = axis->start_pos +
                            (axis->p_accel + v * (t - Ta)) * axis->direction;
        axis->current_speed = v * axis->direction;

    } else {
        /* 减速阶段: 从终点反算, p = p_goal - 0.5 * a * T_rem^2 */
        axis->state = CHASSIS_STATE_DECELERATING;
        float T_rem = axis->t_total - t;
        if (T_rem < 0.0f) T_rem = 0.0f;
        float p_rem = 0.5f * a * T_rem * T_rem;
        axis->current_pos = axis->target_pos - p_rem * axis->direction;
        axis->current_speed = (a * T_rem) * axis->direction;
    }

    return 1;
}

/**
 * @brief 对三轴轨迹做时间同步
 *
 * 取三轴中最长的 t_total, 将较快轴的速度降低使其在相同时间内完成.
 * 这样三轴同步运动, XY 平面运动轨迹为直线.
 *
 * @param inst  规划器实例
 */
static void sync_trajectory_axes(ChassisPlannerInst *inst) {
    float t_max = inst->traj_x.t_total;
    if (inst->traj_y.t_total > t_max) t_max = inst->traj_y.t_total;
    if (inst->traj_w.t_total > t_max) t_max = inst->traj_w.t_total;

    if (t_max < 0.001f) return;

    /* 将较快轴拉伸到 t_max */
    if (inst->traj_x.t_total < t_max && inst->traj_x.state != CHASSIS_STATE_FINISHED) {
        traj_axis_rescale_time(&inst->traj_x, t_max);
    }
    if (inst->traj_y.t_total < t_max && inst->traj_y.state != CHASSIS_STATE_FINISHED) {
        traj_axis_rescale_time(&inst->traj_y, t_max);
    }
    if (inst->traj_w.t_total < t_max && inst->traj_w.state != CHASSIS_STATE_FINISHED) {
        traj_axis_rescale_time(&inst->traj_w, t_max);
    }
}

/**
 * @brief 轨迹模式: 管理三轴轨迹规划
 *
 * target_x/y 为目标位置 (mm), target_w 为目标角度 (rad).
 * 首次调用时初始化轨迹; 目标变化时自动重新规划.
 * 三轴做时间同步, 保证 XY 平面运动轨迹为直线.
 *
 * @param inst      规划器实例
 * @param target_x  目标 X 位置 (mm)
 * @param target_y  目标 Y 位置 (mm)
 * @param target_w  目标偏航角 (rad)
 * @param dt        时间步长 (s)
 */
static void update_trajectory_mode(ChassisPlannerInst *inst,
                                   float target_x, float target_y, float target_w,
                                   float dt) {
    /* 首次更新: 初始化轨迹 */
    if (inst->first_update) {
        traj_axis_init(&inst->traj_x, inst->init_pos_x, 0.0f, target_x,
                       inst->cfg.speed_limit.max_speed_x, inst->cfg.accel.max_accel_x);
        traj_axis_init(&inst->traj_y, inst->init_pos_y, 0.0f, target_y,
                       inst->cfg.speed_limit.max_speed_y, inst->cfg.accel.max_accel_y);
        traj_axis_init(&inst->traj_w, inst->init_pos_w, 0.0f, target_w,
                       inst->cfg.speed_limit.max_speed_w, inst->cfg.accel.max_accel_w);
        sync_trajectory_axes(inst);
        inst->last_target_x = target_x;
        inst->last_target_y = target_y;
        inst->last_target_w = target_w;
        inst->first_update = false;
        return;
    }

    /* 目标变化检测 → 从当前位置和当前速度重新规划 */
    if (fabsf(target_x - inst->last_target_x) > TARGET_CHANGE_THRESHOLD ||
        fabsf(target_y - inst->last_target_y) > TARGET_CHANGE_THRESHOLD ||
        fabsf(target_w - inst->last_target_w) > TARGET_CHANGE_THRESHOLD) {

        traj_axis_init(&inst->traj_x, inst->traj_x.current_pos,
                       inst->traj_x.current_speed, target_x,
                       inst->cfg.speed_limit.max_speed_x, inst->cfg.accel.max_accel_x);
        traj_axis_init(&inst->traj_y, inst->traj_y.current_pos,
                       inst->traj_y.current_speed, target_y,
                       inst->cfg.speed_limit.max_speed_y, inst->cfg.accel.max_accel_y);
        traj_axis_init(&inst->traj_w, inst->traj_w.current_pos,
                       inst->traj_w.current_speed, target_w,
                       inst->cfg.speed_limit.max_speed_w, inst->cfg.accel.max_accel_w);
        sync_trajectory_axes(inst);
        inst->last_target_x = target_x;
        inst->last_target_y = target_y;
        inst->last_target_w = target_w;
    }

    /* 时间步长异常保护 */
    if (dt <= 0.0f || dt > 0.1f) return;

    /* 更新三轴轨迹 */
    traj_axis_update(&inst->traj_x, dt);
    traj_axis_update(&inst->traj_y, dt);
    traj_axis_update(&inst->traj_w, dt);

    /* 将轨迹速度输出到实例 */
    inst->speed_x = inst->traj_x.current_speed;
    inst->speed_y = inst->traj_y.current_speed;
    inst->speed_w = inst->traj_w.current_speed;
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

bool chassis_planner_init(ChassisPlannerInst *inst, const ChassisPlannerCfg *cfg) {
    if (!validate_config(cfg)) return false;

    memset(inst, 0, sizeof(ChassisPlannerInst));
    inst->cfg = *cfg;
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

    /* 根据模式分发 */
    if (inst->cfg.interp_type == CHASSIS_INTERP_TRAJECTORY) {
        update_trajectory_mode(inst, target_x, target_y, target_w, dt);
    } else {
        update_speed_mode(inst, target_x, target_y, target_w, dt);
    }

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
    inst->speed_x = 0.0f;
    inst->speed_y = 0.0f;
    inst->speed_w = 0.0f;
    inst->accel_x = 0.0f;
    inst->accel_y = 0.0f;
    inst->accel_w = 0.0f;
    inst->first_update = true;
    inst->last_tick = xTaskGetTickCount();
    inst->rollover.last_vx = 0.0f;
    inst->rollover.last_vy = 0.0f;
    memset(&inst->traj_x, 0, sizeof(AxisTrajectory));
    memset(&inst->traj_y, 0, sizeof(AxisTrajectory));
    memset(&inst->traj_w, 0, sizeof(AxisTrajectory));
}

void chassis_planner_set_interp(ChassisPlannerInst *inst, ChassisInterpType type) {
    inst->cfg.interp_type = type;
    chassis_planner_reset(inst);
}

void chassis_planner_set_position(ChassisPlannerInst *inst,
                                  float pos_x, float pos_y, float pos_w) {
    inst->init_pos_x = pos_x;
    inst->init_pos_y = pos_y;
    inst->init_pos_w = pos_w;
}

bool chassis_planner_is_finished(const ChassisPlannerInst *inst) {
    if (inst->cfg.interp_type != CHASSIS_INTERP_TRAJECTORY) {
        return false; /* 速度模式无"完成"概念 */
    }
    return (inst->traj_x.state == CHASSIS_STATE_FINISHED &&
            inst->traj_y.state == CHASSIS_STATE_FINISHED &&
            inst->traj_w.state == CHASSIS_STATE_FINISHED);
}

void chassis_planner_get_pos(const ChassisPlannerInst *inst,
                             float *pos_x, float *pos_y, float *pos_w) {
    if (pos_x) *pos_x = inst->traj_x.current_pos;
    if (pos_y) *pos_y = inst->traj_y.current_pos;
    if (pos_w) *pos_w = inst->traj_w.current_pos;
}
