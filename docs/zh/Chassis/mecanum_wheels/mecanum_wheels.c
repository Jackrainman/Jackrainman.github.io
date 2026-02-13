/**
 * @file    mecanum_wheels.c
 * @authors CV_Engineer_CHEN
 * @brief   麦克纳姆轮底盘解算
 * @version 2.0
 * @date    2026-2-12
 * @note    底盘局部坐标系
 *                 y
 *                 |
 *                 |           z轴逆时针正
 *                 |
 *                 |
 *                (0)—-——-——-——-——>x
 *          四轮底盘由前轮为一号, 以横向定义序号 (Z 字型), 如下图:
 *                (1)------------------(2)
 *                 |                    |
 *                 |                    |
 *                 |                    |
 *                 |                    |
 *                 |                    |
 *                 |                    |
 *                (3)------------------(4)
 *          角度相关都为逆时针为正 (角速度, theta 分解)
 ******************************************************************************
 * 麦克纳姆轮运动学模型:
 *   每个麦轮的辊子与轮轴呈 45 度角, 通过四个轮子不同转速组合,
 *   可实现全向移动 (前后, 左右平移, 自转及其任意组合).
 *
 * 麦轮分为 A 轮和 B 轮 (辊子方向互为镜像), 有两种可行的装配方式:
 *
 *   X型装配 (辊子俯视呈 X 形):    O型装配 (辊子俯视呈 O 形):
 *       1A  2B                        1B  2A
 *       3B  4A                        3A  4B
 *       \  /                          /  \
 *       /  \                          \  /
 *
 * 逆运动学公式 (世界速度 -> 轮子转速), s 为符号因子 (X型: s=+1, O型: s=-1):
 *   w1 = (1/r) * ( Vx - s*Vy + s*(a+b)*w )
 *   w2 = (1/r) * ( Vx + s*Vy - s*(a+b)*w )
 *   w3 = (1/r) * ( Vx + s*Vy + s*(a+b)*w )
 *   w4 = (1/r) * ( Vx - s*Vy - s*(a+b)*w )
 *
 * 正运动学公式 (轮子转速 -> 世界速度):
 *   Vx =     (r/4) * ( w1 + w2 + w3 + w4 )
 *   Vy = s * (r/4) * (-w1 + w2 + w3 - w4 )
 *   w  = s * (r/4) * ( w1 - w2 + w3 - w4 ) / (a + b)
 *
 * 其中:
 *   r     - 轮子半径 (m)
 *   a     - 轮子中心到底盘中心的纵向距离 (半轴距, m)
 *   b     - 轮子中心到底盘中心的横向距离 (半轮距, m)
 *   Vx,Vy - 底盘在自身坐标系下 x,y 方向的线速度 (m/s)
 *   w     - 底盘绕自身中心的角速度 (rad/s), 逆时针为正
 *   s     - 辊子符号因子, X型装配 s=+1, O型装配 s=-1
 ******************************************************************************
 */

#include "mecanum_wheels.h"
#include "my_math/my_math.h"

#ifndef PI
#define PI 3.1415926
#endif /* PI */

/* 底盘解算句柄 */
static struct {
    float wheel_radius;             /*!< 轮子半径, 单位 m */
    float half_wheelbase;           /*!< 半轴距 (a), 单位 m */
    float half_track;               /*!< 半轮距 (b), 单位 m */
    float max_wheel_speed;          /*!< 单轮最大转速, 用于归一化 */
    float *world_angle;             /*!< 世界坐标的角度指针 */
    bool world_coordinate;          /*!< 是否使用世界坐标系 */
    bool halt;                      /*!< 是否驻停 */
    mecanum_assembly_t assembly;    /*!< 装配模式: X型或O型 */
    float roller_sign;              /*!< 辊子符号因子: X型=+1, O型=-1 */
    mecanum_wheel_ctrl_t wheel_ctrl; /*!< 电机控制函数 */
} mecanum_ctrl_handle;

/**
 * @brief 设置驻停
 *
 * @param halt 是否开启驻停
 */
void mecanum_set_halt(bool halt) {
    mecanum_ctrl_handle.halt = halt;
}

/**
 * @brief 查看驻停状态
 *
 * @return 是否开启驻停
 */
bool mecanum_get_halt(void) {
    return mecanum_ctrl_handle.halt;
}

/**
 * @brief 设置世界坐标系
 */
void mecanum_set_world_coordinate(void) {
    mecanum_ctrl_handle.world_coordinate = true;
}

/**
 * @brief 设置自身坐标系
 */
void mecanum_set_self_coordinate(void) {
    mecanum_ctrl_handle.world_coordinate = false;
}

/**
 * @brief 获取当前是否是世界坐标
 *
 * @return 是否为世界坐标系
 */
bool mecanum_is_world_coordinate(void) {
    return mecanum_ctrl_handle.world_coordinate;
}

/**
 * @brief 设置装配模式
 *
 * @param assembly 装配模式: MECANUM_X_TYPE 或 MECANUM_O_TYPE
 */
void mecanum_set_assembly(mecanum_assembly_t assembly) {
    mecanum_ctrl_handle.assembly = assembly;
    mecanum_ctrl_handle.roller_sign =
        (assembly == MECANUM_X_TYPE) ? 1.0f : -1.0f;
}

/**
 * @brief 获取当前装配模式
 *
 * @return 当前装配模式
 */
mecanum_assembly_t mecanum_get_assembly(void) {
    return mecanum_ctrl_handle.assembly;
}

/**
 * @brief 麦轮底盘初始化
 *
 * @param wheel_radius 轮子半径, 单位 m
 * @param half_wheelbase 半轴距 (轮子中心到底盘中心纵向距离), 单位 m
 * @param half_track 半轮距 (轮子中心到底盘中心横向距离), 单位 m
 * @param max_wheel_speed 单轮最大转速, 用于轮速归一化
 * @param wheel_ctrl 电机控制函数指针
 * @param yaw_angle 世界坐标系下偏航角指针, 范围 [-180, +180] DEG
 * @param coordinate 坐标系选择: 世界坐标[1], 局部坐标[0]
 * @param assembly 装配模式: MECANUM_X_TYPE 或 MECANUM_O_TYPE
 */
void mecanum_wheel_init(float wheel_radius, float half_wheelbase,
                        float half_track, float max_wheel_speed,
                        mecanum_wheel_ctrl_t wheel_ctrl,
                        float *yaw_angle, bool coordinate,
                        mecanum_assembly_t assembly) {
    mecanum_ctrl_handle.wheel_radius = wheel_radius;
    mecanum_ctrl_handle.half_wheelbase = half_wheelbase;
    mecanum_ctrl_handle.half_track = half_track;
    mecanum_ctrl_handle.max_wheel_speed = max_wheel_speed;
    mecanum_ctrl_handle.wheel_ctrl = wheel_ctrl;
    mecanum_ctrl_handle.world_angle = yaw_angle;
    mecanum_ctrl_handle.world_coordinate = coordinate;
    mecanum_ctrl_handle.halt = true;
    mecanum_set_assembly(assembly);
}

/**
 * @brief 麦轮底盘运动解算 (逆运动学)
 *
 * @param target_speedx x 方向目标速度 (m/s)
 * @param target_speedy y 方向目标速度 (m/s)
 * @param target_speedw 自转角速度 (rad/s), 逆时针为正
 */
void mecanum_wheel_ctrl(float target_speedx, float target_speedy,
                        float target_speedw) {
    float speedx, speedy;                       /* 车身自身坐标系下的速度 */
    float speed_wheel[MECANUM_WHEEL_NUM];       /* 每个轮子的转速 */
    float r = mecanum_ctrl_handle.wheel_radius;
    float k = mecanum_ctrl_handle.half_wheelbase +
              mecanum_ctrl_handle.half_track;    /* a + b */

    /* 驻停: 四轮速度置零 */
    if (mecanum_ctrl_handle.halt) {
        for (uint32_t i = 0; i < MECANUM_WHEEL_NUM; i++) {
            speed_wheel[i] = 0.0f;
        }
        mecanum_ctrl_handle.wheel_ctrl(speed_wheel);
        return;
    }

    float world_angle_rad = DEG2RAD(*mecanum_ctrl_handle.world_angle);

    /* 判定世界坐标开启或关闭 */
    if (mecanum_ctrl_handle.world_coordinate) {
        /* 世界坐标系转换到自身坐标系 */
        speedx = COS_F32(world_angle_rad) * target_speedx +
                 SIN_F32(world_angle_rad) * target_speedy;
        speedy = -SIN_F32(world_angle_rad) * target_speedx +
                 COS_F32(world_angle_rad) * target_speedy;
    } else {
        /* 自身坐标系, 无需转换 */
        speedx = target_speedx;
        speedy = target_speedy;
    }

    /* 逆运动学解算: 世界速度 -> 各轮转速
     * X型: w1=(Vx-Vy+kω)/r, w2=(Vx+Vy-kω)/r, w3=(Vx+Vy+kω)/r, w4=(Vx-Vy-kω)/r
     * O型: Vy和ω符号取反 (等价于 roller_sign = -1)
     */
    float s = mecanum_ctrl_handle.roller_sign;
    speed_wheel[0] = (1.0f / r) * (speedx - s * speedy + s * k * target_speedw);
    speed_wheel[1] = (1.0f / r) * (speedx + s * speedy - s * k * target_speedw);
    speed_wheel[2] = (1.0f / r) * (speedx + s * speedy + s * k * target_speedw);
    speed_wheel[3] = (1.0f / r) * (speedx - s * speedy - s * k * target_speedw);

    /* 轮速归一化: 若最大轮速超限, 等比例缩放保持运动方向 */
    float max_speed = 0.0f;
    for (uint32_t i = 0; i < MECANUM_WHEEL_NUM; i++) {
        float abs_speed = speed_wheel[i] > 0 ? speed_wheel[i] : -speed_wheel[i];
        if (abs_speed > max_speed) {
            max_speed = abs_speed;
        }
    }
    if (max_speed > mecanum_ctrl_handle.max_wheel_speed) {
        float ratio = mecanum_ctrl_handle.max_wheel_speed / max_speed;
        for (uint32_t i = 0; i < MECANUM_WHEEL_NUM; i++) {
            speed_wheel[i] *= ratio;
        }
    }

    /* 调用电机控制函数 */
    mecanum_ctrl_handle.wheel_ctrl(speed_wheel);
}

/**
 * @brief 麦轮底盘正运动学解算
 *
 * @param wheel_speed 四个轮子的转速数组
 * @param vx 解算输出: x 方向速度 (m/s)
 * @param vy 解算输出: y 方向速度 (m/s)
 * @param vw 解算输出: 自转角速度 (rad/s)
 */
void mecanum_forward_kinematics(const float wheel_speed[], float *vx,
                                float *vy, float *vw) {
    float r = mecanum_ctrl_handle.wheel_radius;
    float k = mecanum_ctrl_handle.half_wheelbase +
              mecanum_ctrl_handle.half_track;
    float s = mecanum_ctrl_handle.roller_sign;

    *vx = (r / 4.0f) * (wheel_speed[0] + wheel_speed[1] +
                         wheel_speed[2] + wheel_speed[3]);
    *vy = s * (r / 4.0f) * (-wheel_speed[0] + wheel_speed[1] +
                              wheel_speed[2] - wheel_speed[3]);
    *vw = s * (r / (4.0f * k)) * (wheel_speed[0] - wheel_speed[1] +
                                   wheel_speed[2] - wheel_speed[3]);
}
