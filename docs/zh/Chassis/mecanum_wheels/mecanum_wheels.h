/**
 * @file    mecanum_wheels.h
 * @authors Jackrainman
 * @brief   麦克纳姆轮底盘控制
 * @version 1.0
 * @date    2026-2-12
 * @note    底盘局部坐标系
 */

#ifndef __MECANUM_WHEELS_H
#define __MECANUM_WHEELS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef USE_ARM_MATH
#include "arm_math.h"
#define SIN_F32(x)  arm_sin_f32(x)
#define COS_F32(x)  arm_cos_f32(x)
#else /* USE_ARM_MATH */
#include <math.h>
#define SIN_F32(x)  sinf(x)
#define COS_F32(x)  cosf(x)
#endif /* USE_ARM_MATH */

#define MECANUM_WHEEL_NUM 4 /*!< 麦轮底盘固定为四轮 */

/**
 * @brief 麦轮装配模式
 *
 * 俯视图辊子方向示意 (A轮: \, B轮: /):
 *
 *   X型:  \  /       O型:  /  \
 *         /  \             \  /
 *
 *   X型: 1A 2B       O型: 1B 2A
 *        3B 4A            3A 4B
 */
typedef enum {
    MECANUM_X_TYPE = 0, /*!< X型装配: 对角线辊子同向 */
    MECANUM_O_TYPE = 1, /*!< O型装配: 同侧辊子同向 */
} mecanum_assembly_t;

/* 麦轮底盘电机控制函数指针 */
typedef void (*mecanum_wheel_ctrl_t)(float /* speed */[]);

void mecanum_wheel_init(float wheel_radius, float half_wheelbase,
                        float half_track, float max_wheel_speed,
                        mecanum_wheel_ctrl_t wheel_ctrl,
                        float *yaw_angle, bool coordinate,
                        mecanum_assembly_t assembly);

void mecanum_set_halt(bool halt);
bool mecanum_get_halt(void);

void mecanum_wheel_ctrl(float target_speedx, float target_speedy,
                        float target_speedw);

void mecanum_set_world_coordinate(void);
void mecanum_set_self_coordinate(void);
bool mecanum_is_world_coordinate(void);

void mecanum_set_assembly(mecanum_assembly_t assembly);
mecanum_assembly_t mecanum_get_assembly(void);

void mecanum_forward_kinematics(const float wheel_speed[], float *vx,
                                float *vy, float *vw);

#endif /* __MECANUM_WHEELS_H */
