  /**
   * @file    ladrc.c
   * @author  Jackrainman
   * @brief   LADRC算法库实现（包含一阶和二阶LADRC）
   * @version 2.2
   * @date    2026-02-05
   */

#include "ladrc.h"
#include <math.h>

/* ============================================================================
 *                          通用工具函数
 * ============================================================================ */

/**
 * @brief 输出限幅
 *
 * @param a 传入的值
 * @param abs_max 限制值
 */
static inline void abs_limit(float *a, float abs_max) {
    if (*a > abs_max) {
        *a = abs_max;
    } else if (*a < -abs_max) {
        *a = -abs_max;
    }
}

/* ============================================================================
 *                          一阶 LADRC 实现
 * ============================================================================ */

/**
 * @brief 一阶LADRC初始化
 */
void first_order_ladrc_init(first_order_ladrc_t *fladrc, float max_output,
                           float beta1, float beta2, float kp, float b,
                           float dt, float k_aw) {

    /* 初始化ESO参数 */
    fladrc->beta1 = beta1;
    fladrc->beta2 = beta2;

    /* 初始化控制器参数 */
    fladrc->kp = kp;
    fladrc->b = b;

    /* 初始化状态估计值 */
    fladrc->x1 = 0.0f;
    fladrc->x2 = 0.0f;

    /* 初始化输出限制 */
    fladrc->max_output = max_output;

    /* 初始化抗积分饱和参数 */
    fladrc->k_aw = k_aw;
    fladrc->pre_out = 0.0f;

    /* 初始化采样周期 */
    fladrc->dt = dt;

    /* 初始化输出 */
    fladrc->out = 0.0f;
}

/**
 * @brief 一阶LADRC参数重置
 */
void first_order_ladrc_reset(first_order_ladrc_t *fladrc,
                             float beta1, float beta2, float kp, float b,
                             float dt, float k_aw) {
    fladrc->beta1 = beta1;
    fladrc->beta2 = beta2;
    fladrc->kp = kp;
    fladrc->b = b;
    fladrc->dt = dt;
    fladrc->k_aw = k_aw;
}

/**
 * @brief 一阶LADRC计算函数
 */
float first_order_ladrc_calc(first_order_ladrc_t *fladrc,
                             float target, float measure) {
    /*
     * 一阶LADRC原理：
     * 被控对象：ẋ = f(x, w, t) + b*u  (一阶系统)
     * 其中f(x,w,t)为总扰动，包含模型不确定性和外部扰动
     */

    /* 步骤1: 执行二阶扩张状态观测器(ESO) */
    /*
     * 一阶系统ESO公式 (二阶观测器):
     * dx1 = x2 + b*u + beta1 * (measure - x1)
     * dx2 = beta2 * (measure - x1)
     *
     * 状态含义:
     * x1 - 系统输出估计
     * x2 - 总扰动估计 (包含内部动态和外部扰动)
     *
     * 注意：使用上一时刻的实际输出(限幅后的值)进行ESO更新，防止积分饱和
     */

    /* 计算ESO微分方程 */
    float dx1 = fladrc->x2 + fladrc->b * fladrc->pre_out +
                fladrc->beta1 * (measure - fladrc->x1);
    float dx2 = fladrc->beta2 * (measure - fladrc->x1);

    /* 更新状态估计值(欧拉积分，乘以dt) */
    fladrc->x1 += dx1 * fladrc->dt;
    fladrc->x2 += dx2 * fladrc->dt;

    /* 步骤2: 计算控制量 */
    /*
     * 一阶LADRC控制律:
     * u0 = kp * (target - x1)        // 比例控制
     * u = (u0 - x2) / b              // 扰动补偿
     */

    /* 计算名义控制量u0 */
    float u0 = fladrc->kp * (target - fladrc->x1);

    /* 计算理论控制输出u */
    float out_temp;
    if (fabsf(fladrc->b) < 0.0001f) {
        out_temp = 0.0f;
    } else {
        out_temp = (u0 - fladrc->x2) / fladrc->b;
    }

    /* 输出限幅 */
    abs_limit(&out_temp, fladrc->max_output);

    /* 抗积分饱和 */
    float aw_term = 0.0f;
    if (fladrc->k_aw > 0.0f) {
        float saturation_error = out_temp - fladrc->pre_out;
        aw_term = fladrc->k_aw * saturation_error;
    }

    /* 保存输出用于下一次ESO计算 */
    fladrc->out = out_temp;
    fladrc->pre_out = out_temp;

    /* 更新扰动估计（包含抗积分饱和） */
    fladrc->x2 += aw_term * fladrc->dt;

    return fladrc->out;
}

/* ============================================================================
 *                          二阶 LADRC 实现
 * ============================================================================ */

/**
 * @brief LADRC 初始化
 *
 * @param ladrc LADRC 结构体指针
 * @param max_output 输出限幅
 * @param beta1 ESO增益beta1
 * @param beta2 ESO增益beta2
 * @param beta3 ESO增益beta3
 * @param kp 控制器比例增益
 * @param kd 控制器微分增益
 * @param b 控制增益
 * @param dt 采样周期(秒)
 * @param k_aw 抗积分饱和增益(0表示不使用)
 */
void ladrc_init(ladrc_t *ladrc, float max_output,
                float beta1, float beta2, float beta3,
                float kp, float kd, float b,
                float dt, float k_aw) {
    /* 初始化ESO参数 */
    ladrc->beta1 = beta1;
    ladrc->beta2 = beta2;
    ladrc->beta3 = beta3;

    /* 初始化控制器参数 */
    ladrc->kp = kp;
    ladrc->kd = kd;
    ladrc->b = b;

    /* 初始化状态估计值 */
    ladrc->x1 = 0.0f;
    ladrc->x2 = 0.0f;
    ladrc->x3 = 0.0f;

    /* 初始化输出限制 */
    ladrc->max_output = max_output;

    /* 初始化抗积分饱和参数 */
    ladrc->k_aw = k_aw;
    ladrc->pre_out = 0.0f;

    /* 初始化采样周期 */
    ladrc->dt = dt;

    /* 初始化输出 */
    ladrc->out = 0.0f;
}

/**
 * @brief LADRC 参数调整
 *
 * @param ladrc LADRC 结构体指针
 * @param beta1 ESO增益beta1
 * @param beta2 ESO增益beta2
 * @param beta3 ESO增益beta3
 * @param kp 控制器比例增益
 * @param kd 控制器微分增益
 * @param b 控制增益
 * @param dt 采样周期(秒)
 * @param k_aw 抗积分饱和增益
 */
void ladrc_reset(ladrc_t *ladrc, float beta1, float beta2, float beta3,
                 float kp, float kd, float b,
                 float dt, float k_aw) {
    ladrc->beta1 = beta1;
    ladrc->beta2 = beta2;
    ladrc->beta3 = beta3;
    ladrc->kp = kp;
    ladrc->kd = kd;
    ladrc->b = b;
    ladrc->dt = dt;
    ladrc->k_aw = k_aw;
}

/**
 * @brief LADRC 计算
 *
 * @param ladrc LADRC 结构体指针
 * @param target 目标值
 * @param measure 电机测量值
 * @return LADRC 计算的结果
 */
float ladrc_calc(ladrc_t *ladrc, float target, float measure) {
    /* 步骤1: 执行三阶扩张状态观测器(ESO) */
    /*
     * 二阶LADRC被控对象: ẍ = f + b*u
     * 三阶ESO公式:
     * dx1 = x2 + β1*(y - x1)         <- ẋ1 = x2 (速度)
     * dx2 = x3 + b*u + β2*(y - x1)   <- ẋ2 = x3 + b*u (加速度=扰动+控制)
     * dx3 = β3*(y - x1)              <- ẋ3 = df/dt (扰动变化率)
     *
     * 状态含义:
     * x1 = y (位置)
     * x2 = ẏ = v (速度)
     * x3 = f(x,ẋ,d) (总扰动)
     *
     * 注意：使用上一时刻的实际输出(限幅后的值)进行ESO更新
     */

    /* 计算ESO微分方程 */
    float dx1 = ladrc->x2 +
                ladrc->beta1 * (measure - ladrc->x1);
    float dx2 = ladrc->x3 + ladrc->b * ladrc->pre_out +
                ladrc->beta2 * (measure - ladrc->x1);
    float dx3 = ladrc->beta3 * (measure - ladrc->x1);

    /* 更新状态估计值(欧拉积分，乘以dt) */
    ladrc->x1 += dx1 * ladrc->dt;
    ladrc->x2 += dx2 * ladrc->dt;
    ladrc->x3 += dx3 * ladrc->dt;

    /* 步骤2: 计算控制量u0 */
    /*
     * 控制律公式:
     * u0 = kp * (r - x1) - kd * x2
     * u = (u0 - x3) / b
     */

    /* 计算名义控制量u0 */
    float u0 = ladrc->kp * (target - ladrc->x1) - ladrc->kd * ladrc->x2;

    /* 计算理论控制输出u */
    float out_temp;
    if (fabsf(ladrc->b) < 0.0001f) {
        out_temp = 0.0f;
    } else {
        out_temp = (u0 - ladrc->x3) / ladrc->b;
    }

    /* 输出限幅 */
    abs_limit(&out_temp, ladrc->max_output);

    /* 抗积分饱和 */
    float aw_term = 0.0f;
    if (ladrc->k_aw > 0.0f) {
        float saturation_error = out_temp - ladrc->pre_out;
        aw_term = ladrc->k_aw * saturation_error;
    }

    /* 保存输出用于下一次ESO计算 */
    ladrc->out = out_temp;
    ladrc->pre_out = out_temp;

    /* 更新扰动估计（包含抗积分饱和） */
    ladrc->x3 += aw_term * ladrc->dt;

    return ladrc->out;
}

/* ============================================================================
 *                          TD（跟踪微分器）实现
 * ============================================================================ */

/**
 * @brief TD 初始化
 */
void td_init(td_t *td, float r, float h, float h0, float max_x2) {
    td->r = r;
    td->h = h;
    td->h0 = h0;
    td->max_x2 = max_x2;
    td->x1 = 0.0f;
    td->x2 = 0.0f;
}

/**
 * @brief TD 重置状态
 */
void td_reset(td_t *td, float init_value) {
    td->x1 = init_value;
    td->x2 = 0.0f;
}

/**
 * @brief fhan 最速控制综合函数（梯形加速度曲线）
 *
 * 这是韩京清教授提出的最速控制综合函数，用于实现时间最优控制。
 * 产生的加速度曲线是梯形的（加速-匀速-减速）。
 *
 * @param x1 位置误差
 * @param x2 速度
 * @param r 快速跟踪因子（相当于最大加速度）
 * @param h 积分步长
 * @param h0 滤波因子（用于输入滤波，典型值为h的5-10倍）
 * @return 加速度输出
 */
float td_fhan(float x1, float x2, float r, float h, float h0) {
    float d = r * h;                                // 单步速度变化量 (delta v)
    float d0 = h0 * d;                              // 线性区宽度

    float y = x1 + h * x2;                          /* 预测：当前位置 + h*速度 = 不加控制时的未来位置 */
    float a0 = sqrtf(d * d + 8.0f * r * fabsf(y));  /* 离散时间下的刹车轨迹方程的逆运算 */

    float a;
    if (fabsf(y) <= d0) {
        a = x2 + y / h;
    } else {
        a = x2 + 0.5f * (a0 - d) * ((y > 0) ? 1.0f : -1.0f);
    }

    float fhan_out;
    if (fabsf(a) <= d) {
        fhan_out = -r * a / d;
    } else {
        fhan_out = -r * ((a > 0) ? 1.0f : -1.0f);
    }

    return fhan_out;
}

/**
 * @brief TD 更新计算
 *
 * @param td TD 结构体指针
 * @param target 目标值
 * @param dt 采样周期(秒)
 * @return 跟踪输出 x1（平滑后的目标）
 */
float td_update(td_t *td, float target, float dt) {
    float x1_error = td->x1 - target;

    float fh = td_fhan(x1_error, td->x2, td->r, dt, td->h0);

    float new_x2 = td->x2 + dt * fh;

    if (td->max_x2 > 0.0f) {
        if (new_x2 > td->max_x2) {
            new_x2 = td->max_x2;
        } else if (new_x2 < -td->max_x2) {
            new_x2 = -td->max_x2;
        }
    }

    td->x2 = new_x2;
    td->x1 = td->x1 + dt * td->x2;

    return td->x1;
}
