/**
 * @file    chassis_calculations.h
 * @author  Jackrainman
 * @brief   侧翻抑制模块
 * @version 1.0
 * @date    2025-11-22
 */

#ifndef _CHASSIS_CALCULATIONS_H_
#define _CHASSIS_CALCULATIONS_H_

#include "includes.h"

/**
 * @brief 速度规划类型定义
 */
typedef enum{
    CHASSIS_SPEED_PLAN_EASY,       /**< 简单梯形速度规划 */
    CHASSIS_SPEED_PLAN_COSINE,     /**< 余弦速度规划 */
    CHASSIS_SPEED_PLAN_S_CURVE,    /**< S型速度规划 */

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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/**
 * @brief 获取时间差
 * @param last_tick 上次时间戳
 * @return 时间差（秒）
 */
static float get_time_delta(uint32_t last_tick);

/**
 * @brief 计算速度差值
 * @param target 目标速度
 * @param current 当前速度
 * @return 速度差值
 */
static float calculate_speed_diff(float target, float current);

/**
 * @brief 更新速度值（梯形算法）
 * @param diff 速度差值
 * @param max_change 最大速度变化量
 * @param current_speed 当前速度指针
 * @param target_speed 目标速度
 */
static void easy_calc_update_speed(float diff, float max_change, float* current_speed, float target_speed);

/**
 * @brief 更新速度值（使用余弦平滑算法）
 * @param diff 速度差值
 * @param max_change 最大速度变化量
 * @param current_speed 当前速度指针
 * @param target_speed 目标速度
 */
static void cosine_update_speed(float diff, float max_change, float* current_speed, float target_speed);

/**
 * @brief 更新速度值（使用S型曲线平滑算法）
 * @param diff 速度差值
 * @param max_change 最大速度变化量
 * @param current_speed 当前速度指针
 * @param target_speed 目标速度
 */
static void scurve_update_speed(float diff, float max_change, float* current_speed, float target_speed);

/* Restore warning settings */
#pragma GCC diagnostic pop

#endif /* _CHASSIS_CALCULATIONS_H_ */
