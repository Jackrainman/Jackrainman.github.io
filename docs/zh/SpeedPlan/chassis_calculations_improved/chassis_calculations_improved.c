/**
 * @file    chassis_calculations_improved.c
 * @author  Jackrainman (改进版)
 * @brief   改进的轨迹规划模块 - 完整梯形速度规划实现
 * @version 2.0
 * @date    2025-11-29
 */

#include "chassis_calculations.h"
#include "my_math/my_math.h"
#include "steering_wheel/steering_wheel.h"
#include <math.h>

/* 全局变量定义 */
#define MAX_ACCEL_X 40.0f  /* x方向最大加速度 */
#define MAX_ACCEL_Y 40.0f  /* y方向最大加速度 */
#define MAX_ACCEL_W 20.0f  /* 旋转最大角加速度 */

#define MAX_SPEED_X 100.0f /* x方向最大速度 */
#define MAX_SPEED_Y 100.0f /* y方向最大速度 */
#define MAX_SPEED_W 50.0f  /* 最大角速度 */

/* ==================== 新增：轨迹规划数据结构 ==================== */

/**
 * @brief 运动状态枚举
 */
typedef enum {
    TRAJ_IDLE,           // 空闲
    TRAJ_ACCELERATING,   // 加速阶段
    TRAJ_UNIFORM,        // 匀速阶段
    TRAJ_DECELERATING,   // 减速阶段
    TRAJ_FINISHED        // 完成
} TrajectoryState;

/**
 * @brief 单轴轨迹参数结构体
 */
typedef struct {
    // 当前状态
    float current_pos;      /**< 当前位置 */
    float current_speed;    /**< 当前速度 */
    TrajectoryState state;  /**< 运动状态 */

    // 目标参数
    float target_pos;       /**< 目标位置 */
    float start_pos;        /**< 起始位置 */
    float max_speed;        /**< 最大速度 */
    float max_accel;        /**< 最大加速度 */

    // 轨迹规划参数（在初始化时计算）
    float t_accel;          /**< 加速时间 (s) */
    float t_uniform;        /**< 匀速时间 (s) */
    float t_total;          /**< 总时间 (s) */
    float p_accel;          /**< 加速段距离 */
    float current_time;     /**< 当前已运行时间 (s) */
    int direction;          /**< 运动方向 (+1 或 -1) */
} AxisTrajectory;

/**
 * @brief 改进的轨迹规划句柄
 */
static struct {
    AxisTrajectory axis_x;
    AxisTrajectory axis_y;
    AxisTrajectory axis_w;
    uint32_t last_tick;
} chassis_trajectory_handle = {0};

/* 全局速度规划类型变量 */
ChassisSpeedPlanType chassis_speed_plan_type;

/* ==================== 核心改进：轨迹规划算法 ==================== */

/**
 * @brief 为单个轴初始化轨迹（核心改进）
 * @param axis 轴轨迹结构体指针
 * @param current_pos 当前位置
 * @param target_pos 目标位置
 * @param max_speed 最大速度
 * @param max_accel 最大加速度
 */
static void axis_trajectory_init(AxisTrajectory *axis,
                                  float current_pos,
                                  float target_pos,
                                  float max_speed,
                                  float max_accel) {
    // 1. 保存基本参数
    axis->start_pos = current_pos;
    axis->current_pos = current_pos;
    axis->target_pos = target_pos;
    axis->max_speed = fabsf(max_speed);
    axis->max_accel = fabsf(max_accel);
    axis->current_time = 0.0f;

    // 保持当前速度作为初速度（支持非零初速度）
    // 如果需要从静止开始，可以设置为 0.0f
    // axis->current_speed = 0.0f;

    // 2. 计算距离和方向
    float distance = target_pos - current_pos;
    axis->direction = (distance >= 0) ? 1 : -1;
    float D = fabsf(distance);

    // 3. 如果距离太小，直接标记为完成
    if (D < 0.001f) {
        axis->state = TRAJ_FINISHED;
        axis->t_total = 0.0f;
        axis->t_accel = 0.0f;
        axis->t_uniform = 0.0f;
        axis->current_speed = 0.0f;
        return;
    }

    // 4. 计算理想的加速时间（假设能达到最大速度）
    axis->t_accel = axis->max_speed / axis->max_accel;

    // 5. 计算加速段距离: s = 1/2 * a * t^2
    axis->p_accel = 0.5f * axis->max_accel * axis->t_accel * axis->t_accel;

    // 6. 判断是梯形还是三角形轨迹（关键！）
    if (D < 2.0f * axis->p_accel) {
        // 【三角形轨迹】距离太短，无法达到最大速度

        // 重新计算实际最大速度: v = sqrt(a * D)
        float v_actual = sqrtf(axis->max_accel * D);
        axis->max_speed = v_actual;

        // 重新计算加速时间
        axis->t_accel = axis->max_speed / axis->max_accel;

        // 匀速时间为0
        axis->t_uniform = 0.0f;

        // 更新加速段距离
        axis->p_accel = 0.5f * axis->max_accel * axis->t_accel * axis->t_accel;

    } else {
        // 【梯形轨迹】可以达到最大速度

        // 计算匀速段距离
        float p_uniform = D - 2.0f * axis->p_accel;

        // 计算匀速时间
        axis->t_uniform = p_uniform / axis->max_speed;
    }

    // 7. 计算总时间
    axis->t_total = 2.0f * axis->t_accel + axis->t_uniform;

    // 8. 设置初始状态
    axis->state = TRAJ_ACCELERATING;
}

/**
 * @brief 更新单个轴的轨迹（每个控制周期调用）
 * @param axis 轴轨迹结构体指针
 * @param dt 时间增量（秒）
 * @return 1: 运动中, 0: 已完成
 */
static int axis_trajectory_update(AxisTrajectory *axis, float dt) {

    // 如果已完成，直接返回
    if (axis->state == TRAJ_FINISHED) {
        axis->current_pos = axis->target_pos;
        axis->current_speed = 0.0f;
        return 0;
    }

    // 更新时间
    axis->current_time += dt;

    // 检查是否到达总时间
    if (axis->current_time >= axis->t_total) {
        axis->current_time = axis->t_total;
        axis->current_pos = axis->target_pos;
        axis->current_speed = 0.0f;
        axis->state = TRAJ_FINISHED;
        return 0;
    }

    float t = axis->current_time;
    float a = axis->max_accel;
    float v = axis->max_speed;
    float Ta = axis->t_accel;
    float Tv = axis->t_uniform;

    // 【关键改进】根据时间判断当前处于哪个阶段

    if (t <= Ta) {
        // ====== 加速阶段 ======
        axis->state = TRAJ_ACCELERATING;

        // 位置: p = p_start + 0.5 * a * t^2
        axis->current_pos = axis->start_pos +
                           (0.5f * a * t * t) * axis->direction;

        // 速度: v = a * t
        axis->current_speed = (a * t) * axis->direction;

    } else if (t <= (Ta + Tv)) {
        // ====== 匀速阶段 ======
        axis->state = TRAJ_UNIFORM;

        // 位置: p = p_start + p_accel + v_max * (t - Ta)
        axis->current_pos = axis->start_pos +
                           (axis->p_accel + v * (t - Ta)) * axis->direction;

        // 速度: v = v_max (恒定)
        axis->current_speed = v * axis->direction;

    } else {
        // ====== 减速阶段 ======
        axis->state = TRAJ_DECELERATING;

        // 计算剩余时间（关键！从终点反算）
        float T_rem = axis->t_total - t;

        // 位置: p = p_goal - 0.5 * a * T_rem^2
        float p_rem = 0.5f * a * T_rem * T_rem;
        axis->current_pos = axis->target_pos - p_rem * axis->direction;

        // 速度: v = a * T_rem (逐渐减小)
        axis->current_speed = (a * T_rem) * axis->direction;
    }

    return 1;  // 运动未完成
}

/* ==================== 改进的对外接口函数 ==================== */

/**
 * @brief 改进的梯形速度规划（替代 easy_speed_calc_handle）
 * @param target_x 目标x位置（注意：不是速度！）
 * @param target_y 目标y位置
 * @param target_yaw 目标角度
 */
void improved_trajectory_plan(float target_x, float target_y, float target_yaw) {
    uint32_t current_tick = xTaskGetTickCount();
    float dt = (current_tick - chassis_trajectory_handle.last_tick) / 1000.0f; // 转换为秒

    // 初始化last_tick
    if (chassis_trajectory_handle.last_tick == 0) {
        chassis_trajectory_handle.last_tick = current_tick;

        // 初始化三个轴的轨迹（假设当前位置为0）
        axis_trajectory_init(&chassis_trajectory_handle.axis_x,
                            0.0f, target_x, MAX_SPEED_X, MAX_ACCEL_X);
        axis_trajectory_init(&chassis_trajectory_handle.axis_y,
                            0.0f, target_y, MAX_SPEED_Y, MAX_ACCEL_Y);
        axis_trajectory_init(&chassis_trajectory_handle.axis_w,
                            0.0f, target_yaw, MAX_SPEED_W, MAX_ACCEL_W);
        return;
    }

    // 检测目标是否改变（如果改变需要重新规划）
    static float last_target_x = 0.0f;
    static float last_target_y = 0.0f;
    static float last_target_yaw = 0.0f;

    if (fabsf(target_x - last_target_x) > 0.01f ||
        fabsf(target_y - last_target_y) > 0.01f ||
        fabsf(target_yaw - last_target_yaw) > 0.01f) {

        // 目标改变，重新初始化轨迹
        axis_trajectory_init(&chassis_trajectory_handle.axis_x,
                            chassis_trajectory_handle.axis_x.current_pos,
                            target_x, MAX_SPEED_X, MAX_ACCEL_X);
        axis_trajectory_init(&chassis_trajectory_handle.axis_y,
                            chassis_trajectory_handle.axis_y.current_pos,
                            target_y, MAX_SPEED_Y, MAX_ACCEL_Y);
        axis_trajectory_init(&chassis_trajectory_handle.axis_w,
                            chassis_trajectory_handle.axis_w.current_pos,
                            target_yaw, MAX_SPEED_W, MAX_ACCEL_W);

        last_target_x = target_x;
        last_target_y = target_y;
        last_target_yaw = target_yaw;
    }

    // 时间步长保护
    if (dt <= 0.0f || dt > 0.1f) {
        chassis_trajectory_handle.last_tick = current_tick;
        return;
    }

    // 更新三个轴的轨迹
    axis_trajectory_update(&chassis_trajectory_handle.axis_x, dt);
    axis_trajectory_update(&chassis_trajectory_handle.axis_y, dt);
    axis_trajectory_update(&chassis_trajectory_handle.axis_w, dt);

    // 更新时间戳
    chassis_trajectory_handle.last_tick = current_tick;
}

/**
 * @brief 获取当前规划的速度和位置
 * @param output_x 输出x速度
 * @param output_y 输出y速度
 * @param output_yaw 输出角速度
 * @param pos_x 输出x位置
 * @param pos_y 输出y位置
 * @param pos_yaw 输出角度位置
 */
void get_trajectory_output(float *output_x, float *output_y, float *output_yaw,
                          float *pos_x, float *pos_y, float *pos_yaw) {
    // 输出速度
    *output_x = chassis_trajectory_handle.axis_x.current_speed;
    *output_y = chassis_trajectory_handle.axis_y.current_speed;
    *output_yaw = chassis_trajectory_handle.axis_w.current_speed;

    // 输出位置
    if (pos_x) *pos_x = chassis_trajectory_handle.axis_x.current_pos;
    if (pos_y) *pos_y = chassis_trajectory_handle.axis_y.current_pos;
    if (pos_yaw) *pos_yaw = chassis_trajectory_handle.axis_w.current_pos;
}

/**
 * @brief 检查轨迹是否完成
 * @return 1: 全部完成, 0: 还在运动
 */
int is_trajectory_finished(void) {
    return (chassis_trajectory_handle.axis_x.state == TRAJ_FINISHED &&
            chassis_trajectory_handle.axis_y.state == TRAJ_FINISHED &&
            chassis_trajectory_handle.axis_w.state == TRAJ_FINISHED);
}

/**
 * @brief 获取当前运动状态（用于调试）
 */
void get_trajectory_state(TrajectoryState *state_x,
                         TrajectoryState *state_y,
                         TrajectoryState *state_w) {
    if (state_x) *state_x = chassis_trajectory_handle.axis_x.state;
    if (state_y) *state_y = chassis_trajectory_handle.axis_y.state;
    if (state_w) *state_w = chassis_trajectory_handle.axis_w.state;
}

/* ==================== 保留原有接口的兼容层 ==================== */

/**
 * @brief 初始化速度规划方式
 */
void chassis_speed_plan_init(ChassisSpeedPlanType type) {
    chassis_speed_plan_type = type;
}

/**
 * @brief 获取当前使用的速度规划方式
 */
ChassisSpeedPlanType chassis_get_speed_plan(void) {
    return chassis_speed_plan_type;
}

/**
 * @brief 统一的速度规划函数（修改为调用改进版）
 */
void chassis_speed_plan(float input_x, float input_y, float input_yaw,
                        float *output_x, float *output_y, float *output_yaw) {

    // 使用改进的轨迹规划
    improved_trajectory_plan(input_x, input_y, input_yaw);

    // 获取输出
    get_trajectory_output(output_x, output_y, output_yaw, NULL, NULL, NULL);

    // 可选：速度点映射限速
    // speed_point_limit(output_x, output_y);
}
