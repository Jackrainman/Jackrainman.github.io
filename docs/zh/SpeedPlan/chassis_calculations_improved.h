/**
 * @file    chassis_calculations_improved.h
 * @author  Jackrainman (改进版)
 * @brief   改进的轨迹规划模块头文件
 * @version 2.0
 * @date    2025-11-29
 */

#ifndef _CHASSIS_CALCULATIONS_IMPROVED_H_
#define _CHASSIS_CALCULATIONS_IMPROVED_H_

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

/**
 * @brief 运动状态枚举（新增）
 */
typedef enum {
    TRAJ_IDLE,           // 空闲
    TRAJ_ACCELERATING,   // 加速阶段
    TRAJ_UNIFORM,        // 匀速阶段
    TRAJ_DECELERATING,   // 减速阶段
    TRAJ_FINISHED        // 完成
} TrajectoryState;

/* 速度规划类型全局变量 */
extern ChassisSpeedPlanType chassis_speed_plan_type;

/* ==================== 基础接口（兼容原代码） ==================== */

/**
 * @brief 初始化速度规划方式
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
 * @param input_x 输入的x方向目标位置
 * @param input_y 输入的y方向目标位置
 * @param input_yaw 输入的目标角度
 * @param output_x 输出处理后的x方向速度
 * @param output_y 输出处理后的y方向速度
 * @param output_yaw 输出处理后的角速度
 */
void chassis_speed_plan(float input_x, float input_y, float input_yaw,
                        float *output_x, float *output_y, float *output_yaw);

/* ==================== 新增接口（改进功能） ==================== */

/**
 * @brief 改进的梯形轨迹规划
 * @param target_x 目标x位置
 * @param target_y 目标y位置
 * @param target_yaw 目标角度
 */
void improved_trajectory_plan(float target_x, float target_y, float target_yaw);

/**
 * @brief 获取当前规划的速度和位置
 * @param output_x 输出x速度
 * @param output_y 输出y速度
 * @param output_yaw 输出角速度
 * @param pos_x 输出x位置（可选，传NULL跳过）
 * @param pos_y 输出y位置（可选，传NULL跳过）
 * @param pos_yaw 输出角度位置（可选，传NULL跳过）
 */
void get_trajectory_output(float *output_x, float *output_y, float *output_yaw,
                          float *pos_x, float *pos_y, float *pos_yaw);

/**
 * @brief 检查轨迹是否完成
 * @return 1: 全部完成, 0: 还在运动
 */
int is_trajectory_finished(void);

/**
 * @brief 获取当前运动状态（用于调试）
 * @param state_x 输出x轴状态（可选，传NULL跳过）
 * @param state_y 输出y轴状态（可选，传NULL跳过）
 * @param state_w 输出角度轴状态（可选，传NULL跳过）
 */
void get_trajectory_state(TrajectoryState *state_x,
                         TrajectoryState *state_y,
                         TrajectoryState *state_w);

#endif /* _CHASSIS_CALCULATIONS_IMPROVED_H_ */
