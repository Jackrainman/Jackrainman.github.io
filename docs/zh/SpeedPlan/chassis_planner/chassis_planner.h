/**
 * @file    chassis_planner.h
 * @author  Jackrainman
 * @brief   底盘速度规划模块 -- 实例化, 无全局变量
 * @version 3.1
 * @date    2026-02-10
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
    CHASSIS_INTERP_TRAJECTORY,  /**< 完整梯形轨迹规划 (含位置跟踪) */
    CHASSIS_INTERP_NUM
} ChassisInterpType;

/** 运动状态 (仅 TRAJECTORY 模式使用) */
typedef enum {
    CHASSIS_STATE_IDLE,          /**< 空闲 */
    CHASSIS_STATE_ACCELERATING,  /**< 加速阶段 */
    CHASSIS_STATE_UNIFORM,       /**< 匀速阶段 */
    CHASSIS_STATE_DECELERATING,  /**< 减速阶段 */
    CHASSIS_STATE_FINISHED       /**< 运动完成 */
} ChassisMotionState;

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
    float max_jerk_x;    /**< X轴最大 jerk (mm/s^3), 0 表示不限制 */
    float max_jerk_y;    /**< Y轴最大 jerk (mm/s^3), 0 表示不限制 */
    float max_jerk_w;    /**< 偏航轴最大 jerk (rad/s^3), 0 表示不限制 */
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
    ChassisAccelCfg accel;               /**< 加速度限制 */
    ChassisSpeedLimitCfg speed_limit;    /**< 最大速度限制 (所有模式生效) */
    ChassisJerkCfg jerk;                 /**< jerk 限制 (仅 SCURVE 模式) */
    ChassisRolloverCfg rollover;         /**< 防侧翻参数 */
    bool enable_rollover_limit;          /**< 是否启用防侧翻限速 */
    uint32_t tick_rate_hz;               /**< FreeRTOS tick 频率 (通常为 1000) */
} ChassisPlannerCfg;

/* ======================== 单轴轨迹状态 ======================== */

/** 单轴轨迹参数 (仅 TRAJECTORY 模式使用) */
typedef struct {
    float current_pos;       /**< 当前位置 (mm 或 rad) */
    float current_speed;     /**< 当前速度 (mm/s 或 rad/s) */
    ChassisMotionState state;/**< 运动状态 */

    /* 轨迹参数 (初始化时预计算) */
    float target_pos;        /**< 目标位置 (mm 或 rad) */
    float start_pos;         /**< 起始位置 (mm 或 rad) */
    float start_speed;       /**< 起始速度 (mm/s 或 rad/s), 用于非零初速重规划 */
    float max_speed;         /**< 本段最大速度 (mm/s 或 rad/s) */
    float max_accel;         /**< 最大加速度 (mm/s^2 或 rad/s^2) */
    float t_accel;           /**< 加速段时长 (s) */
    float t_uniform;         /**< 匀速段时长 (s) */
    float t_total;           /**< 总运动时长 (s) */
    float p_accel;           /**< 加速段距离 (mm 或 rad) */
    float current_time;      /**< 已运行时间 (s) */
    int direction;           /**< 运动方向 (+1 或 -1) */
} AxisTrajectory;

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

    /* 轨迹模式状态 (其他模式不使用) */
    AxisTrajectory traj_x;
    AxisTrajectory traj_y;
    AxisTrajectory traj_w;
    float last_target_x;     /**< 用于目标变化检测 (mm 或 mm/s) */
    float last_target_y;
    float last_target_w;

    /* 轨迹模式: 初始位置 (由 chassis_planner_set_position 设置) */
    float init_pos_x;        /**< 初始 X 位置 (mm) */
    float init_pos_y;        /**< 初始 Y 位置 (mm) */
    float init_pos_w;        /**< 初始偏航角 (rad) */

    /* 防侧翻状态 */
    RolloverState rollover;
} ChassisPlannerInst;

/* ======================== 公共 API ======================== */

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
 * TRAPEZOID/COSINE/SCURVE 模式:
 *   target_x/y 为目标速度 (mm/s), target_w 为目标角速度 (rad/s)
 *
 * TRAJECTORY 模式:
 *   target_x/y 为目标位置 (mm), target_w 为目标角度 (rad)
 *
 * @param inst      规划器实例
 * @param target_x  目标 X (mm/s 或 mm, 取决于模式)
 * @param target_y  目标 Y (mm/s 或 mm, 取决于模式)
 * @param target_w  目标偏航 (rad/s 或 rad, 取决于模式)
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

/**
 * @brief 设置轨迹模式的初始位置 (在 init 之后、第一次 update 之前调用)
 * @param inst   规划器实例
 * @param pos_x  初始 X 位置 (mm)
 * @param pos_y  初始 Y 位置 (mm)
 * @param pos_w  初始偏航角 (rad)
 */
void chassis_planner_set_position(ChassisPlannerInst *inst,
                                  float pos_x, float pos_y, float pos_w);

/**
 * @brief 检查轨迹是否完成 (仅 TRAJECTORY 模式有意义)
 * @param inst  规划器实例
 * @return TRAJECTORY 模式: true 表示三轴全部完成;
 *         其他模式: 始终返回 false (速度模式无"完成"概念)
 */
bool chassis_planner_is_finished(const ChassisPlannerInst *inst);

/**
 * @brief 获取轨迹位置 (仅 TRAJECTORY 模式, 调试用)
 * @param inst   规划器实例
 * @param pos_x  输出 X 位置 (mm), 可传 NULL 跳过
 * @param pos_y  输出 Y 位置 (mm), 可传 NULL 跳过
 * @param pos_w  输出偏航角 (rad), 可传 NULL 跳过
 */
void chassis_planner_get_pos(const ChassisPlannerInst *inst,
                             float *pos_x, float *pos_y, float *pos_w);

#endif /* CHASSIS_PLANNER_H */
