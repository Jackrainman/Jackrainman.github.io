/**
 * @file    ladrc.c
 * @author  Jackrainman
 * @brief   LADRC算法库实现（包含一阶和二阶LADRC）
 * @version 3.1
 * @date    2026-02-07
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
 *                          TD（跟踪微分器）实现
 * ============================================================================ */

/**
 * @brief TD 初始化 - 参数固化模式
 *
 * @param td      TD结构体指针
 * @param r       快速跟踪因子，决定跟踪速度
 * @param dt      固定采样周期(秒) - RTOS任务周期，将固化到结构体中
 * @param n       滤波因子(无量纲) - 建议值1~5，值越大滤波越强但延迟越大
 * @param max_x2  最大速度限制（0表示不限制）
 *
 * @note 参数固化模式的核心逻辑：
 *       1. 固化采样周期：td->h = dt
 *       2. 自动计算内部滤波参数：td->h0 = n * dt
 *       3. 安全检查：如果 n < 1.0，强制设为 1.0，防止数学模型崩溃
 */
void td_init(td_t *td, float r, float dt, float n, float max_x2) {
    /* 安全性检查：滤波因子N不能小于1.0，否则数学模型会崩溃 */
    if (n < 1.0f) {
        n = 1.0f;
    }

    td->r = r;
    td->h = dt;         /* 固化采样周期到结构体中 */
    td->h0 = n * dt;    /* 自动计算内部滤波参数 h0 = N * dt */
    td->max_x2 = max_x2;
    td->x1 = 0.0f;
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
 * @param h 积分步长（采样周期）
 * @param h0 滤波因子（用于输入滤波，典型值为h的5-10倍）
 * @return 加速度输出
 */
float td_fhan(float x1, float x2, float r, float h, float h0) {
    float d = r * h;                                // 单步速度变化量 (delta v)
    float d0 = h0 * d;                              // 线性区宽度

    float y = x1 + h * x2;                          /* 预测：当前位置 + h*速度 = 不加控制时的未来位置 */

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
    if (fabsf(y) <= d0) {                           // 线性区
        a = x2 + y / h;
    } else {                                        // 非线性区
        a = x2 + 0.5f * (a0 - d) * ((y > 0) ? 1.0f : -1.0f);
    }

    float fhan_out;                                 // 输出
    if (fabsf(a) <= d) {                            // 积分区
        fhan_out = -r * a / d;
    } else {                                        //非积分区
        fhan_out = -r * ((a > 0) ? 1.0f : -1.0f);
    }

    return fhan_out;
}

/**
 * @brief TD 更新计算 - 参数固化模式
 *
 * @param td      TD结构体指针
 * @param target  目标值
 * @return        跟踪输出x1（平滑后的目标值）
 *
 * @note 参数固化模式：此函数使用结构体中固化的采样周期td->h，
 *       不再接受外部传入的dt。这适用于RTOS固定频率调用的场景，
 *       可以消除时间抖动对积分的影响。
 */
float td_update(td_t *td, float target) {
    float x1_error = td->x1 - target;

    /* 使用固化的采样周期td->h进行计算，不再依赖外部传入的dt */
    float fh = td_fhan(x1_error, td->x2, td->r, td->h, td->h0);

    /* 使用固化的td->h进行积分，确保时间一致性 */
    float new_x2 = td->x2 + td->h * fh;

    /* 速度限制检查 */
    if (td->max_x2 > 0.0f) {
        if (new_x2 > td->max_x2) {
            new_x2 = td->max_x2;
        } else if (new_x2 < -td->max_x2) {
            new_x2 = -td->max_x2;
        }
    }

    td->x2 = new_x2;
    /* 使用固化的td->h进行位置积分 */
    td->x1 = td->x1 + td->h * td->x2;

    return td->x1;
}

/**
 * @brief TD 重置状态
 */
void td_reset(td_t *td, float init_value) {
    td->x1 = init_value;
    td->x2 = 0.0f;
}

/* ============================================================================
 *                          一阶 LADRC 实现
 * ============================================================================ */

/**
 * @brief 一阶LADRC初始化
 */
void first_order_ladrc_init(first_order_ladrc_t *fladrc, float max_output,
                           float beta1, float beta2, float kp, float b,
                           float dt, float k_aw,
                           float td_r, float td_n, float td_max_x2) {

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

    /* 初始化采样周期 - 固化到结构体中 */
    fladrc->dt = dt;

    /* 初始化输出 */
    fladrc->out = 0.0f;

    /* 初始化TD（组合模式）- 使用新的参数固化模式 */
    if (td_r > 0.0f) {
        td_init(&fladrc->td, td_r, dt, td_n, td_max_x2);
        fladrc->use_td = true;
    } else {
        fladrc->use_td = false;
    }
}

/**
 * @brief 一阶LADRC参数重置
 */
void first_order_ladrc_reset(first_order_ladrc_t *fladrc,
                             float beta1, float beta2, float kp, float b,
                             float dt, float k_aw,
                             float td_r, float td_n, float td_max_x2) {
    fladrc->beta1 = beta1;
    fladrc->beta2 = beta2;
    fladrc->kp = kp;
    fladrc->b = b;
    fladrc->dt = dt;
    fladrc->k_aw = k_aw;

    /* 重置 ESO 状态，避免参数切换时的瞬态问题 */
    fladrc->x1 = 0.0f;
    fladrc->x2 = 0.0f;
    fladrc->pre_out = 0.0f;

    /* 重新配置TD参数 - 使用新的参数固化模式 */
    if (td_r > 0.0f) {
        /* 安全性检查：滤波因子N不能小于1.0 */
        float n = td_n;
        if (n < 1.0f) {
            n = 1.0f;
        }
        fladrc->td.r = td_r;
        fladrc->td.h = dt;          /* 固化采样周期 */
        fladrc->td.h0 = n * dt;     /* 计算内部滤波参数 h0 = N * dt */
        fladrc->td.max_x2 = td_max_x2;
        fladrc->use_td = true;
    } else {
        fladrc->use_td = false;
    }
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

    /* 步骤0: TD（跟踪微分器）处理目标值 - 使用组合模式调用TD模块（参数固化模式） */
    float td_target = target;
    if (fladrc->use_td) {
        /* 参数固化模式：不再传入dt，使用结构体中固化的td->h */
        td_target = td_update(&fladrc->td, target);
    }

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

    /* 计算ESO微分方程 - 优化：只计算一次误差 */
    float error = measure - fladrc->x1;
    float dx1 = fladrc->x2 + fladrc->b * fladrc->pre_out + fladrc->beta1 * error;
    float dx2 = fladrc->beta2 * error;

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
    float u0 = fladrc->kp * (td_target - fladrc->x1);

    /* 抗积分饱和：当处于饱和状态时，减小 u0 以退出饱和 */
    if (fladrc->k_aw > 0.0f && fabsf(fladrc->pre_out) >= fladrc->max_output * 0.99f) {
        float u_ideal = (u0 - fladrc->x2) / fladrc->b;
        float saturation_error = fladrc->pre_out - u_ideal;
        u0 -= fladrc->k_aw * fladrc->b * saturation_error;
    }

    /* 计算理论控制输出u */
    float out_temp;
    if (fabsf(fladrc->b) < 0.0001f) {
        out_temp = 0.0f;
    } else {
        out_temp = (u0 - fladrc->x2) / fladrc->b;
    }

    /* 输出限幅 */
    abs_limit(&out_temp, fladrc->max_output);

    /* 保存输出用于下一次ESO计算 */
    fladrc->out = out_temp;
    fladrc->pre_out = out_temp;

    return fladrc->out;
}

/* ============================================================================
 *                          二阶 LADRC 实现
 * ============================================================================ */

/**
 * @brief LADRC 初始化 (参数固化模式)
 *
 * @note 此函数将物理参数 dt 固化到结构体中，后续 update 调用不再需要传 dt。
 *
 * @param ladrc      LADRC 结构体句柄
 * @param max_output 控制量输出限幅 (例如 PWM 最大值)
 * @param beta1      ESO 状态观测器增益 1 (位置观测带宽)
 * @param beta2      ESO 状态观测器增益 2 (速度观测带宽)
 * @param beta3      ESO 状态观测器增益 3 (扰动观测带宽)
 * @param kp         控制器比例增益 (刚度)
 * @param kd         控制器微分增益 (阻尼) - 注意：LADRC中通常设为0，由 b0 处理，除非需要额外的 PD
 * @param b          系统增益估计值 (b0) - 决定控制量的缩放比例
 * @param dt         RTOS 固定采样周期 (秒) - [关键] 必须与实际任务频率一致，将固化到结构体
 * @param k_aw       抗积分饱和增益 (建议值: 1.0 ~ 3.0，0 表示不启用)
 * @param td_r       TD 快速跟踪因子 (r) - 决定目标值响应速度。0 表示禁用 TD (直接透传)
 * @param td_n       TD 滤波因子 (无量纲，建议值 1~5) - 决定噪声过滤能力。
 *                   [物理意义]: N=1: 滤波最弱，响应最快；
 *                             N=3~5: 典型推荐值，平衡滤波效果与响应速度；
 *                             N>10: 强滤波但有明显滞后。
 * @param td_max_x2  TD 最大速度限制 (0 表示不限制) - 防止设定值跳变过大导致系统冲击
 */
void ladrc_init(ladrc_t *ladrc, float max_output,
                float beta1, float beta2, float beta3,
                float kp, float kd, float b,
                float dt, float k_aw,
                float td_r, float td_n, float td_max_x2) {
    /* 1. 初始化 ESO (扩张状态观测器) 参数 */
    ladrc->beta1 = beta1;
    ladrc->beta2 = beta2;
    ladrc->beta3 = beta3;

    /* 2. 初始化 控制器 (State Error Feedback) 参数 */
    ladrc->kp = kp;
    ladrc->kd = kd;
    ladrc->b = b;

    /* 3. 清空 ESO 内部状态 */
    ladrc->x1 = 0.0f; // 估计位置
    ladrc->x2 = 0.0f; // 估计速度
    ladrc->x3 = 0.0f; // 估计总扰动

    /* 4. 输出与抗饱和设置 */
    ladrc->max_output = max_output;

    /* 初始化抗积分饱和参数 */
    ladrc->k_aw = k_aw;
    ladrc->pre_out = 0.0f; // 上一时刻的输出（用于计算饱和程度）
    ladrc->out = 0.0f;

    /* 5. 固化采样周期 (核心修改) */
    // 这个 dt 将被 ESO 和 TD 共同使用，作为时间基准
    ladrc->dt = dt;

    /* 6. 初始化 TD (参数固化模式) */
    if (td_r > 0.0f) {
        // 调用 td_init - 使用新的参数固化模式
        // 传入滤波因子 N (td_n)，内部自动计算 h0 = N * dt
        td_init(&ladrc->td, td_r, dt, td_n, td_max_x2);
        ladrc->use_td = true;
    } else {
        // 禁用 TD 模式
        ladrc->use_td = false;
        // 为了安全，将 TD 状态清零
        td_reset(&ladrc->td, 0.0f);
    }
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
 * @param td_r TD快速跟踪因子(0表示不使用TD)
 * @param td_n TD滤波因子(无量纲，建议值1~5)
 * @param td_max_x2 TD最大速度限制
 */
void ladrc_reset(ladrc_t *ladrc, float beta1, float beta2, float beta3,
                 float kp, float kd, float b,
                 float dt, float k_aw,
                 float td_r, float td_n, float td_max_x2) {
    ladrc->beta1 = beta1;
    ladrc->beta2 = beta2;
    ladrc->beta3 = beta3;
    ladrc->kp = kp;
    ladrc->kd = kd;
    ladrc->b = b;
    ladrc->dt = dt;
    ladrc->k_aw = k_aw;

    /* 重置 ESO 状态，避免参数切换时的瞬态问题 */
    ladrc->x1 = 0.0f;
    ladrc->x2 = 0.0f;
    ladrc->x3 = 0.0f;
    ladrc->pre_out = 0.0f;

    /* 重新配置TD参数 - 使用新的参数固化模式 */
    if (td_r > 0.0f) {
        /* 安全性检查：滤波因子N不能小于1.0 */
        float n = td_n;
        if (n < 1.0f) {
            n = 1.0f;
        }
        ladrc->td.r = td_r;
        ladrc->td.h = dt;           /* 固化采样周期 */
        ladrc->td.h0 = n * dt;      /* 计算内部滤波参数 h0 = N * dt */
        ladrc->td.max_x2 = td_max_x2;
        ladrc->use_td = true;
    } else {
        ladrc->use_td = false;
    }
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
    /* 步骤0: TD（跟踪微分器）处理目标值 - 使用组合模式调用TD模块（参数固化模式） */
    float td_target = target;
    if (ladrc->use_td) {
        /* 参数固化模式：不再传入dt，使用结构体中固化的td->h */
        td_target = td_update(&ladrc->td, target);
    }

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

    /* 计算ESO微分方程 - 优化：只计算一次误差 */
    float error = measure - ladrc->x1;
    float dx1 = ladrc->x2 + ladrc->beta1 * error;
    float dx2 = ladrc->x3 + ladrc->b * ladrc->pre_out + ladrc->beta2 * error;
    float dx3 = ladrc->beta3 * error;

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
    float u0 = ladrc->kp * (td_target - ladrc->x1) - ladrc->kd * ladrc->x2;

    /* 抗积分饱和：当处于饱和状态时，减小 u0 以退出饱和 */
    if (ladrc->k_aw > 0.0f && fabsf(ladrc->pre_out) >= ladrc->max_output * 0.99f) {
        float u_ideal = (u0 - ladrc->x3) / ladrc->b;
        float saturation_error = ladrc->pre_out - u_ideal;
        u0 -= ladrc->k_aw * ladrc->b * saturation_error;
    }

    /* 计算理论控制输出u */
    float out_temp;
    if (fabsf(ladrc->b) < 0.0001f) {
        out_temp = 0.0f;
    } else {
        out_temp = (u0 - ladrc->x3) / ladrc->b;
    }

    /* 输出限幅 */
    abs_limit(&out_temp, ladrc->max_output);

    /* 保存输出用于下一次ESO计算 */
    ladrc->out = out_temp;
    ladrc->pre_out = out_temp;

    return ladrc->out;
}
