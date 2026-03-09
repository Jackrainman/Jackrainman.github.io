/**
 * @file    chassis_calculations.h
 * @author  Jackrainman
 * @brief   底盘平滑速度规划模块（兼容1.0接口）
 * @version 2.0
 * @date    2026-03-02
 *
 * 说明：
 * 1. 保留 1.0 对外 API，支持无缝替换；
 * 2. 内核升级为实例化平滑规划（梯形/余弦/S曲线）；
 * 3. 修复 dt 单位错误，增加 jerk 与速度限幅能力。
 */

#ifndef _CHASSIS_CALCULATIONS_H_
#define _CHASSIS_CALCULATIONS_H_

#include "includes.h"

/**
 * @brief 速度规划类型定义
 */
typedef enum {
    CHASSIS_SPEED_PLAN_EASY = 0,   /**< 梯形加减速 */
    CHASSIS_SPEED_PLAN_COSINE,     /**< 余弦平滑加减速 */
    CHASSIS_SPEED_PLAN_S_CURVE,    /**< jerk 限制 S 曲线 */
    CHASSIS_SPEED_PLAN_NUM
} ChassisSpeedPlanType;

/* 速度规划类型全局变量 */
extern ChassisSpeedPlanType chassis_speed_plan_type;

/* 函数声明 */

/**
 * @brief 初始化速度规划方式（也可以用来更新）
 * @param type 速度规划类型
 */
void chassis_speed_plan_init(ChassisSpeedPlanType type);

/**
 * @brief 获取当前使用的速度规划方式
 * @return 当前速度规划类型
 */
ChassisSpeedPlanType chassis_get_speed_plan(void);

/**
 * @brief 统一的速度规划函数
 * @param input_x 输入的x方向速度
 * @param input_y 输入的y方向速度
 * @param input_yaw 输入的角速度
 * @param output_x 输出处理后的x方向速度
 * @param output_y 输出处理后的y方向速度
 * @param output_yaw 输出处理后的角速度
 */
void chassis_speed_plan(float input_x, float input_y, float input_yaw,
                        float *output_x, float *output_y, float *output_yaw);

/**
 * @brief 简单梯形速度规划
 */
void easy_speed_calc_handle(float target_x, float target_y, float target_yaw);

/**
 * @brief 余弦速度规划
 */
void cosine_speed_calc_handle(float target_x, float target_y, float target_yaw);

/**
 * @brief S型速度规划
 */
void scurve_speed_plan_handle(float target_x, float target_y, float target_yaw);

/**
 * @brief 速度点限制
 */
void speed_point_limit(float *vx, float *vy);

/**
 * @brief 设置控制周期 tick 频率（Hz）
 * @param tick_freq_hz FreeRTOS tick 频率，通常为 1000
 */
void chassis_speed_plan_set_tick_freq(uint32_t tick_freq_hz);

/**
 * @brief 设置三轴最大加速度
 * @param accel_x X 轴最大加速度（mm/s^2）
 * @param accel_y Y 轴最大加速度（mm/s^2）
 * @param accel_w W 轴最大角加速度（rad/s^2）
 */
void chassis_speed_plan_set_accel(float accel_x, float accel_y, float accel_w);

/**
 * @brief 设置三轴最大 jerk（<=0 表示自动按 10*accel 推算）
 * @param jerk_x X 轴最大 jerk（mm/s^3）
 * @param jerk_y Y 轴最大 jerk（mm/s^3）
 * @param jerk_w W 轴最大 jerk（rad/s^3）
 */
void chassis_speed_plan_set_jerk(float jerk_x, float jerk_y, float jerk_w);

/**
 * @brief 设置三轴最大速度限制（<=0 表示不限制）
 * @param speed_x X 轴最大速度（mm/s）
 * @param speed_y Y 轴最大速度（mm/s）
 * @param speed_w W 轴最大角速度（rad/s）
 */
void chassis_speed_plan_set_speed_limit(float speed_x, float speed_y, float speed_w);

/**
 * @brief 设置防侧翻速度向量限制参数
 * @param enable 是否启用（0=关，非0=开）
 * @param base_limit 基础限制（mm/s）
 * @param max_limit 最大限制（mm/s）
 * @param threshold 变化阈值（mm/s）
 */
void chassis_speed_plan_set_rollover(int enable, float base_limit, float max_limit, float threshold);

#endif /* _CHASSIS_CALCULATIONS_H_ */
