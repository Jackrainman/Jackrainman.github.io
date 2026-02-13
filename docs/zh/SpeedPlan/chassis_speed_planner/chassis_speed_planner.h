/**
 * @file    chassis_speed_planner.h
 * @author  Jackrainman
 * @brief   底盘速度规划模块（实例化版本）
 *          重构自 chassis_calculations，支持多实例、物理参数配置
 * @version 2.0
 * @date    2025-11-22
 *
 * 单位约定:
 *   线速度: mm/s      线加速度: mm/s²     线 jerk: mm/s³
 *   角速度: rad/s     角加速度: rad/s²    角 jerk: rad/s³
 *   时间:   s (内部自动从 FreeRTOS tick 换算)
 */

#ifndef CHASSIS_SPEED_PLANNER_H
#define CHASSIS_SPEED_PLANNER_H

#include "includes.h"

/* ========================== 枚举 ========================== */

/** @brief 速度规划算法类型 */
typedef enum {
    SPEED_PLAN_TRAPEZOID,   /**< 梯形加减速 */
    SPEED_PLAN_COSINE,      /**< 余弦平滑加减速 */
    SPEED_PLAN_SCURVE,      /**< S型曲线（带 jerk 限制） */
    SPEED_PLAN_NUM
} SpeedPlanType;

/* ========================== 配置 ========================== */

/**
 * @brief 速度规划器配置（所有参数均有明确物理含义）
 */
typedef struct {
    /* ---------- 加速度限制（所有模式通用） ---------- */
    float max_accel_x;          /**< X轴最大加速度 (mm/s²) */
    float max_accel_y;          /**< Y轴最大加速度 (mm/s²) */
    float max_accel_w;          /**< 偏航轴最大角加速度 (rad/s²) */

    /* ---------- S曲线专用: 加加速度限制 ---------- */
    float max_jerk_x;           /**< X轴最大 jerk (mm/s³) */
    float max_jerk_y;           /**< Y轴最大 jerk (mm/s³) */
    float max_jerk_w;           /**< 偏航轴最大 jerk (rad/s³) */

    /* ---------- 防侧翻限速参数 ---------- */
    float rollover_base_limit;  /**< 基础速度变化限制 (mm/s) */
    float rollover_max_limit;   /**< 最大速度变化限制 (mm/s) */
    float rollover_threshold;   /**< 速度变化阈值 (mm/s)，低于此值不限制 */
    int   enable_rollover;      /**< 是否启用防侧翻限速 (0=关, 1=开) */

    /* ---------- 系统参数 ---------- */
    uint32_t tick_freq_hz;      /**< FreeRTOS tick 频率 (Hz)，通常为 1000 */
    SpeedPlanType plan_type;    /**< 默认规划算法类型 */
} SpeedPlannerCfg;

/* ========================== 内部状态 ========================== */

/**
 * @brief 余弦过渡状态（每轴独立）
 *
 * 余弦速度过渡公式:
 *   v(t) = v_start + Δv × 0.5 × (1 - cos(π × t / T))
 * 其中过渡时间:
 *   T = π × |Δv| / (2 × a_max)
 * 峰值加速度恰好等于 a_max，加速度在起止点为 0。
 */
typedef struct {
    float start_speed;      /**< 过渡起始速度 */
    float target_speed;     /**< 过渡目标速度 */
    float duration;         /**< 过渡总时间 T (s) */
    float elapsed;          /**< 已运行时间 (s) */
    int   active;           /**< 是否正在过渡中 */
} CosineAxisState;

/* ========================== 规划器实例 ========================== */

/**
 * @brief 速度规划器实例
 *
 * 使用方法:
 *   SpeedPlanner_t planner;
 *   SpeedPlannerCfg  cfg = { ... };
 *   speed_planner_init(&planner, &cfg);
 *
 *   // 每个控制周期调用:
 *   float vx, vy, vw;
 *   speed_planner_update(&planner, target_x, target_y, target_w, &vx, &vy, &vw);
 */
typedef struct {
    SpeedPlannerCfg cfg;        /**< 配置副本 */

    /* 当前输出速度 */
    float speed_x;              /**< 当前规划 vx (mm/s) */
    float speed_y;              /**< 当前规划 vy (mm/s) */
    float speed_w;              /**< 当前规划 ω  (rad/s) */

    /* S曲线模式: 当前加速度状态 */
    float accel_x;              /**< X轴当前加速度 (mm/s²) */
    float accel_y;              /**< Y轴当前加速度 (mm/s²) */
    float accel_w;              /**< 偏航轴当前角加速度 (rad/s²) */

    /* 余弦模式: 每轴过渡状态 */
    CosineAxisState cos_x;      /**< X轴余弦过渡 */
    CosineAxisState cos_y;      /**< Y轴余弦过渡 */
    CosineAxisState cos_w;      /**< 偏航轴余弦过渡 */

    /* 防侧翻: 上周期输出（用于计算速度变化向量） */
    float last_vx;              /**< 上周期输出 vx (mm/s) */
    float last_vy;              /**< 上周期输出 vy (mm/s) */

    /* 时间管理 */
    uint32_t last_tick;         /**< 上次 xTaskGetTickCount() 值 */
    int first_update;           /**< 首次更新标志 */
} SpeedPlanner_t;

/* ========================== 公共 API ========================== */

/**
 * @brief 初始化速度规划器
 * @param inst 规划器实例指针
 * @param cfg  配置参数指针（会被拷贝到实例内部）
 */
void speed_planner_init(SpeedPlanner_t *inst, const SpeedPlannerCfg *cfg);

/**
 * @brief 速度规划主更新函数（每控制周期调用一次）
 * @param inst     规划器实例
 * @param target_x 目标 X 速度 (mm/s)
 * @param target_y 目标 Y 速度 (mm/s)
 * @param target_w 目标角速度  (rad/s)
 * @param out_vx   输出 X 速度 (mm/s)
 * @param out_vy   输出 Y 速度 (mm/s)
 * @param out_vw   输出角速度  (rad/s)
 */
void speed_planner_update(SpeedPlanner_t *inst,
                          float target_x, float target_y, float target_w,
                          float *out_vx, float *out_vy, float *out_vw);

/**
 * @brief 重置规划器状态（速度归零）
 * @param inst 规划器实例
 */
void speed_planner_reset(SpeedPlanner_t *inst);

/**
 * @brief 运行时切换规划算法类型
 * @param inst 规划器实例
 * @param type 新的规划类型
 */
void speed_planner_set_type(SpeedPlanner_t *inst, SpeedPlanType type);

/**
 * @brief 获取当前规划算法类型
 * @param inst 规划器实例
 * @return 当前规划类型
 */
SpeedPlanType speed_planner_get_type(const SpeedPlanner_t *inst);

#endif /* CHASSIS_SPEED_PLANNER_H */
