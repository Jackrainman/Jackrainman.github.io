/**
 * @file    chassis_planner.h
 * @author  Jackrainman
 * @brief   底盘速度规划模块 -- 实例化, 无全局变量
 * @version 3.3
 * @date    2026-02-13
 *
 * 物理单位约定 (所有参数与输入输出均遵循):
 *   线速度       : mm/s
 *   线加速度     : mm/s^2
 *   角速度       : rad/s
 *   角加速度     : rad/s^2
 *   时间         : 秒 (内部自动将 FreeRTOS tick 转换为秒)
 */

#ifndef CHASSIS_PLANNER_H
#define CHASSIS_PLANNER_H

#include "includes.h"
#include <stdbool.h>

/* ======================== 枚举定义 ======================== */

/** 插值策略 */
typedef enum {
    CHASSIS_INTERP_TRAPEZOID,   /**< 梯形加减速 (线性斜坡) */
    CHASSIS_INTERP_COSINE,      /**< 余弦平滑加减速 */
    CHASSIS_INTERP_SCURVE,      /**< S型曲线加减速 (带 jerk 限制) */
    CHASSIS_INTERP_NUM
} ChassisInterpType;

/* ======================== 配置结构体 ======================== */

/** 各轴加速度限制 */
typedef struct {
    float max_accel_x;   /**< X轴最大加速度 (mm/s^2) */
    float max_accel_y;   /**< Y轴最大加速度 (mm/s^2) */
    float max_accel_w;   /**< 偏航轴最大角加速度 (rad/s^2) */
} ChassisAccelCfg;

/** 各轴最大速度限制 */
typedef struct {
    float max_speed_x;   /**< X轴最大速度 (mm/s) */
    float max_speed_y;   /**< Y轴最大速度 (mm/s) */
    float max_speed_w;   /**< 偏航轴最大角速度 (rad/s) */
} ChassisSpeedLimitCfg;

/** 各轴 jerk (加加速度) 限制 -- 仅 SCURVE 模式使用 */
typedef struct {
    float max_jerk_x;    /**< X轴最大 jerk (mm/s^3), 0 = 自动按 10*accel 推算 */
    float max_jerk_y;    /**< Y轴最大 jerk (mm/s^3), 0 = 自动按 10*accel 推算 */
    float max_jerk_w;    /**< 偏航轴最大 jerk (rad/s^3), 0 = 自动按 10*accel 推算 */
} ChassisJerkCfg;

/** 防侧翻速度向量限制参数 */
typedef struct {
    float base_limit;    /**< 基础速度限制 (mm/s) */
    float max_limit;     /**< 最大速度限制 (mm/s) */
    float threshold;     /**< 阈值速度 (mm/s) */
} ChassisRolloverCfg;

/** 规划器顶层配置 */
typedef struct {
    ChassisInterpType interp_type;       /**< 插值策略 */
    ChassisAccelCfg accel;               /**< 加速度限制 (必填) */
    ChassisSpeedLimitCfg speed_limit;    /**< 最大速度限制 (必填, 所有模式生效) */
    ChassisJerkCfg jerk;                 /**< jerk 限制 (仅 SCURVE 模式, 0 = 自动按 10×accel 推算) */
    ChassisRolloverCfg rollover;         /**< 防侧翻参数 */
    bool enable_rollover_limit;          /**< 是否启用防侧翻限速 */
    uint32_t tick_rate_hz;               /**< FreeRTOS tick 频率 (通常为 1000) */
} ChassisPlannerCfg;

/* ======================== 配置辅助宏 ======================== */

/** XY 对称加速度配置 (XY 相同, W 轴单独指定) */
#define CHASSIS_ACCEL_SYM(xy, w) \
    (ChassisAccelCfg){ (xy), (xy), (w) }

/** XY 对称速度限制配置 */
#define CHASSIS_SPEED_LIMIT_SYM(xy, w) \
    (ChassisSpeedLimitCfg){ (xy), (xy), (w) }

/** XY 对称 jerk 配置 (全部置 0 表示由 init 自动推算) */
#define CHASSIS_JERK_SYM(xy, w) \
    (ChassisJerkCfg){ (xy), (xy), (w) }

/** 全轴 jerk 置零 -> init 时自动从 accel 推算 (jerk = 10 * accel) */
#define CHASSIS_JERK_AUTO \
    (ChassisJerkCfg){ 0.0f, 0.0f, 0.0f }

/* ======================== 余弦过渡状态 ======================== */

/**
 * 余弦速度过渡状态 (每轴独立)
 *
 * 速度过渡公式:
 *   v(t) = v_start + Δv × 0.5 × (1 - cos(π × t / T))
 * 过渡时间:
 *   T = π × |Δv| / (2 × a_max)
 * 峰值加速度恰好等于 a_max, 起止加速度为 0.
 */
typedef struct {
    float start_speed;      /**< 过渡起始速度 */
    float target_speed;     /**< 过渡目标速度 */
    float duration;         /**< 过渡总时间 T (s) */
    float elapsed;          /**< 已运行时间 (s) */
    bool  active;           /**< 是否正在过渡中 */
} CosineAxisState;

/* ======================== 规划器实例 ======================== */

/** 防侧翻状态 (替代原 speed_point_limit 中的 static 变量) */
typedef struct {
    float last_vx;           /**< 上一周期输出 vx (mm/s) */
    float last_vy;           /**< 上一周期输出 vy (mm/s) */
} RolloverState;

/** 规划器实例 -- 每个底盘一个 */
typedef struct {
    ChassisPlannerCfg cfg;   /**< 配置 (初始化时复制) */

    /* 当前输出速度 */
    float speed_x;           /**< 当前规划 vx (mm/s) */
    float speed_y;           /**< 当前规划 vy (mm/s) */
    float speed_w;           /**< 当前规划 omega (rad/s) */

    /* S 曲线模式: 当前加速度状态 */
    float accel_x;           /**< 当前 X 轴加速度 (mm/s^2) */
    float accel_y;           /**< 当前 Y 轴加速度 (mm/s^2) */
    float accel_w;           /**< 当前 W 轴角加速度 (rad/s^2) */

    /* 时间管理 */
    uint32_t last_tick;      /**< 上次 xTaskGetTickCount() 值 */
    bool first_update;       /**< 首次更新标志 */

    /* 余弦模式: 每轴过渡状态 */
    CosineAxisState cos_x;   /**< X轴余弦过渡 */
    CosineAxisState cos_y;   /**< Y轴余弦过渡 */
    CosineAxisState cos_w;   /**< 偏航轴余弦过渡 */

    /* 防侧翻状态 */
    RolloverState rollover;
} ChassisPlannerInst;

/* ======================== 公共 API ======================== */

/**
 * @brief 获取默认配置
 *
 * 返回一组经过验证的合理默认值, 调用者只需覆盖关心的参数.
 * jerk 默认为 0 (CHASSIS_JERK_AUTO), init 时自动按 10 倍 accel 推算.
 *
 * @param type  期望的插值策略
 * @return 填充好默认值的配置结构体
 *
 * 默认值:
 *   accel       : x/y = 40 mm/s^2,  w = 20 rad/s^2
 *   speed_limit : x/y = 5000 mm/s,  w = 10 rad/s
 *   jerk        : 全部 0 (自动推算为 10 * accel)
 *   rollover    : threshold=30, base=30, max=40, 默认关闭
 *   tick_rate   : 1000 Hz
 *
 * 用法:
 *   ChassisPlannerCfg cfg = chassis_planner_default_cfg(CHASSIS_INTERP_COSINE);
 *   cfg.accel = CHASSIS_ACCEL_SYM(60.0f, 30.0f);
 *   chassis_planner_init(&planner, &cfg);
 */
ChassisPlannerCfg chassis_planner_default_cfg(ChassisInterpType type);

/**
 * @brief 初始化规划器实例
 * @param inst  规划器实例指针 (由调用者分配内存)
 * @param cfg   配置 (复制到实例中)
 * @return true: 初始化成功, false: 配置参数非法
 */
bool chassis_planner_init(ChassisPlannerInst *inst, const ChassisPlannerCfg *cfg);

/**
 * @brief 执行一个规划周期
 *
 * @param inst      规划器实例
 * @param target_x  目标 vx (mm/s)
 * @param target_y  目标 vy (mm/s)
 * @param target_w  目标角速度 omega (rad/s)
 * @param out_vx    输出规划速度 X (mm/s)
 * @param out_vy    输出规划速度 Y (mm/s)
 * @param out_vw    输出规划角速度 (rad/s)
 */
void chassis_planner_update(ChassisPlannerInst *inst,
                            float target_x, float target_y, float target_w,
                            float *out_vx, float *out_vy, float *out_vw);

/**
 * @brief 重置规划器状态为零
 * @param inst  规划器实例
 */
void chassis_planner_reset(ChassisPlannerInst *inst);

/**
 * @brief 运行时切换插值策略
 * @note  内部会重置速度状态以避免不连续
 * @param inst  规划器实例
 * @param type  新的插值策略
 */
void chassis_planner_set_interp(ChassisPlannerInst *inst, ChassisInterpType type);

#endif /* CHASSIS_PLANNER_H */
