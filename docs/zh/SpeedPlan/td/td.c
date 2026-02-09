/**
 * @file    td.c
 * @brief   跟踪微分器 (Tracking Differentiator) 库实现
 * @version 1.0
 * @date    2026-02-09
 */

#include "td.h"

/**
 * @brief 输出限幅辅助函数
 */
static inline void abs_limit(float *a, float abs_max) {
    if (*a > abs_max) {
        *a = abs_max;
    } else if (*a < -abs_max) {
        *a = -abs_max;
    }
}

/**
 * @brief TD初始化
 * @param td TD实例指针
 * @param dt 采样周期（秒）
 * @param cfg 配置参数
 */
void td_init(TD* td, float dt, const TD_Config* cfg) {
    /* 安全性检查 */
    float n = cfg->n;
    if (n < 1.0f) {
        n = 1.0f;
    }

    td->r = cfg->r;
    td->h = dt;
    td->h0 = n * dt;
    td->max_x2 = cfg->max_v;
    td->x1 = 0.0f;
    td->x2 = 0.0f;
}

/**
 * @brief fhan最速控制综合函数（梯形加速度曲线）
 *
 * 这是韩京清教授提出的最速控制综合函数，用于实现时间最优控制。
 * 产生的加速度曲线是梯形的（加速-匀速-减速）。
 *
 * @param x1 位置误差
 * @param x2 速度
 * @param r 快速跟踪因子（相当于最大加速度）
 * @param h 积分步长（采样周期）
 * @param h0 滤波因子
 * @return 加速度输出
 */
float td_fhan(float x1, float x2, float r, float h, float h0) {
    float d = r * h;                                /* 单步速度变化量 (delta v) */
    float d0 = h0 * d;                              /* 线性区宽度 */

    float y = x1 + h * x2;                          /* 预测：当前位置 + h*速度 */

    /* 安全检查：限制 y 的范围防止 sqrtf 溢出 */
    const float MAX_Y = 1e15f;
    if (fabsf(y) > MAX_Y) {
        y = (y > 0.0f) ? MAX_Y : -MAX_Y;
    }

    float sqrt_arg = d * d + 8.0f * r * fabsf(y);

    /* 安全检查：确保 sqrtf 参数非负且有限 */
    if (sqrt_arg < 0.0f || !isfinite(sqrt_arg)) {
        sqrt_arg = 0.0f;
    }

    float a0 = sqrtf(sqrt_arg);                     /* 离散时间下的刹车轨迹方程的逆运算 */

    float a;
    if (fabsf(y) <= d0) {                           /* 线性区 */
        a = x2 + y / h;
    } else {                                        /* 非线性区 */
        a = x2 + 0.5f * (a0 - d) * ((y > 0) ? 1.0f : -1.0f);
    }

    float fhan_out;                                 /* 输出 */
    if (fabsf(a) <= d) {                            /* 积分区 */
        fhan_out = -r * a / d;
    } else {                                        /* 非积分区 */
        fhan_out = -r * ((a > 0) ? 1.0f : -1.0f);
    }

    return fhan_out;
}

/**
 * @brief TD更新计算
 *
 * @param td TD实例指针
 * @param target 目标值
 * @return 跟踪输出x1（平滑后的目标值）
 *
 * @note 使用结构体中固化的采样周期td->h
 */
float td_update(TD* td, float target) {
    float x1_error = td->x1 - target;

    /* 使用固化的采样周期td->h进行计算 */
    float fh = td_fhan(x1_error, td->x2, td->r, td->h, td->h0);

    /* 使用固化的td->h进行积分 */
    float new_x2 = td->x2 + td->h * fh;

    /* 速度限制检查 */
    if (td->max_x2 > 0.0f) {
        abs_limit(&new_x2, td->max_x2);
    }

    td->x2 = new_x2;
    /* 使用固化的td->h进行位置积分 */
    td->x1 = td->x1 + td->h * td->x2;

    return td->x1;
}

/**
 * @brief TD重置状态
 * @param td TD实例指针
 * @param init_val 初始值
 */
void td_reset(TD* td, float init_val) {
    td->x1 = init_val;
    td->x2 = 0.0f;
}
