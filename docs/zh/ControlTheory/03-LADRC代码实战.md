# LADRC代码实战

> **最近修改日期**：2026-02-07  
> **参与者**：Jackrainman  
> **前置知识**：[02-LADRC算法详解.md](./02-LADRC算法详解.md)

---

## 目录

1. [前置知识](#前置知识)
2. [第一部分：跟踪微分器(TD)实现](#第一部分跟踪微分器td实现)
3. [第二部分：二阶LADRC实现](#第二部分二阶ladrc实现)
4. [第三部分：一阶LADRC实现](#第三部分一阶ladrc实现)
5. [第四部分：完整使用示例](#第四部分完整使用示例)
6. [第五部分：调试技巧与常见问题](#第五部分调试技巧与常见问题)

---

## 前置知识

在开始编写代码之前，请确保你已经理解以下内容：

| 知识点 | 说明 | 参考文档 |
|--------|------|----------|
| **LADRC基本原理** | 总扰动、LESO、LSEF的概念 | [02-LADRC算法详解.md](./02-LADRC算法详解.md) |
| **离散化方法** | 零阶保持法、欧拉法 | 本文第六章 |
| **C语言基础** | 结构体、指针、基本运算 | - |

**本文代码特点**：
- 使用 **C语言** 编写（适用于嵌入式平台如STM32、DSP等）
- 代码风格清晰，注释详细
- 浮点数使用 `float` 类型（可根据需要改为 `double`）

---

## 第一部分：跟踪微分器(TD)实现

### 1.1 TD的作用

跟踪微分器（Tracking Differentiator, TD）有两个核心功能：

1. **安排过渡过程**：将跳变的输入信号平滑化，避免控制量突变
2. **提取微分信号**：从有噪声的信号中提取可靠的微分信号

**为什么需要TD？**

假设输入信号从0跳变到100，如果不加处理直接输入控制器，会产生巨大的控制量突变，可能导致：
- 执行机构（如电机）过流损坏
- 系统振荡甚至失稳
- 机械结构受到冲击

TD通过安排合理的过渡过程，让信号"平滑地"到达目标值。

### 1.2 代码实现

以下是二阶跟踪微分器的实现：

```c
/**
 * @brief  跟踪微分器结构体
 * @note   二阶TD，用于安排过渡过程和提取微分信号
 */
typedef struct {
    float x1;       // 跟踪信号（位置）
    float x2;       // 微分信号（速度）
    float r;        // 速度因子（调节跟踪快慢，相当于最大加速度）
    float h;        // 积分步长（采样周期）
    float h0;       // 滤波因子（用于输入滤波，典型值为h的5-10倍）
    float max_x2;   // 最大速度限制（0表示不限制）
} td_t;

/**
 * @brief  fhan最速控制综合函数（梯形加速度曲线）
 * @param  x1: 位置误差
 * @param  x2: 速度
 * @param  r: 快速跟踪因子（相当于最大加速度）
 * @param  h: 积分步长
 * @param  h0: 滤波因子
 * @retval 加速度输出
 * @note   韩京清教授提出的最速控制综合函数，实现时间最优控制
 */
float td_fhan(float x1, float x2, float r, float h, float h0)
{
    float d = r * h;                                // 单步速度变化量
    float d0 = h0 * d;                              // 线性区宽度

    float y = x1 + h * x2;                          // 预测：当前位置 + h*速度
    float a0 = sqrtf(d * d + 8.0f * r * fabsf(y));  // 离散时间下的刹车轨迹方程

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
 * @brief  TD初始化
 * @param  td: TD结构体指针
 * @param  r: 速度因子（越大跟踪越快）
 * @param  h: 积分步长（采样周期）
 * @param  h0: 滤波因子（典型值为h的5-10倍）
 * @param  max_x2: 最大速度限制（0表示不限制）
 */
void td_init(td_t *td, float r, float h, float h0, float max_x2)
{
    td->r = r;
    td->h = h;
    td->h0 = h0;
    td->max_x2 = max_x2;
    td->x1 = 0.0f;
    td->x2 = 0.0f;
}

/**
 * @brief  TD重置状态
 * @param  td: TD结构体指针
 * @param  init_value: 初始值
 */
void td_reset(td_t *td, float init_value)
{
    td->x1 = init_value;
    td->x2 = 0.0f;
}

/**
 * @brief  TD更新计算
 * @param  td: TD结构体指针
 * @param  target: 目标值
 * @param  dt: 采样周期(秒)
 * @retval 跟踪输出x1（平滑后的目标）
 */
float td_update(td_t *td, float target, float dt)
{
    float x1_error = td->x1 - target;
    
    float fh = td_fhan(x1_error, td->x2, td->r, dt, td->h0);
    
    float new_x2 = td->x2 + dt * fh;
    
    // 速度限制
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
```

### 1.3 参数说明

| 参数 | 符号 | 作用 | 调节建议 |
|------|------|------|----------|
| **速度因子** | $r$ | 决定跟踪速度 | $r$ 越大，跟踪越快，但可能超调。建议初始值：$r = 1/T_{settle}$ |
| **滤波因子** | $h_0$ | 决定平滑程度 | 典型值为采样周期的5-10倍，越大滤波效果越好但相位滞后越大 |
| **最大速度** | $max\_x2$ | 限制输出速度 | 根据执行器最大速度设置，防止目标变化过快 |

**离散积分说明**：

代码中使用了**欧拉法**进行离散积分：

```c
x[k+1] = x[k] + h * dx/dt
```

即：新的位置 = 旧的位置 + 采样周期 × 速度

这是一种简单但有效的离散化方法，当采样周期足够小时（通常 $h < 0.01s$），精度可以满足大多数工程需求。

### 1.4 使用示例

```c
// 定义TD结构体
td_t my_td;

// 初始化：速度因子r=100，采样周期dt=0.001s，滤波因子h0=0.005s，最大速度限制50
// h0 = 5 * dt 是典型设置
td_init(&my_td, 100.0f, 0.001f, 0.005f, 50.0f);

// 主循环（每1ms执行一次）
while(1) {
    float target = Get_Target();                    // 获取目标值
    float smooth_target = td_update(&my_td, target, 0.001f);  // 更新TD
    float target_speed = my_td.x2;                  // 目标值的变化率
    
    // 将smooth_target和target_speed用于后续控制...
    
    Delay_ms(1);                                    // 等待下一个周期
}

// 如果需要重置TD状态（如系统复位时）
td_reset(&my_td, 0.0f);  // 将位置重置为0，速度重置为0
```

---

## 第二部分：二阶LADRC实现

### 2.1 二阶LADRC原理

二阶LADRC适用于被控对象为二阶的系统（如位置控制、角度控制等）。其核心组成：

1. **三阶ESO（扩张状态观测器）**：估计位置、速度和总扰动
2. **PD控制器**：根据误差生成虚拟控制量
3. **扰动补偿**：利用估计的总扰动进行补偿

**被控对象模型**：

$$
\ddot{y} = f(x, \dot{x}, d, t) + b \cdot u
$$

其中 $f$ 为总扰动（包含模型不确定性和外部扰动）。

### 2.2 ESO连续时间方程

ESO使用连续时间微分方程形式，在代码中通过欧拉积分离散化：

$$
\begin{cases}
\dot{x}_1 = x_2 + \beta_1 \cdot (y - x_1) \\
\dot{x}_2 = x_3 + b \cdot u + \beta_2 \cdot (y - x_1) \\
\dot{x}_3 = \beta_3 \cdot (y - x_1)
\end{cases}
$$

**状态含义**：
- $x_1$：估计的位置
- $x_2$：估计的速度  
- $x_3$：估计的总扰动

### 2.3 代码实现

```c
/**
 * @brief  二阶LADRC结构体
 * @note   适用于位置/角度控制等二阶系统
 */
typedef struct {
    // ESO增益
    float beta1;        // ESO增益beta1
    float beta2;        // ESO增益beta2
    float beta3;        // ESO增益beta3
    
    // 控制器增益
    float kp;           // 比例增益
    float kd;           // 微分增益
    
    // 系统参数
    float b;            // 控制增益
    float dt;           // 采样周期(秒)
    
    // 状态估计值
    float x1;           // 估计的位置
    float x2;           // 估计的速度
    float x3;           // 估计的总扰动
    
    // 输出限制
    float max_output;   // 输出限幅值
    
    // 抗积分饱和
    float k_aw;         // 抗积分饱和增益（0表示不使用）
    float pre_out;      // 上一时刻输出（用于抗饱和）
    
    // 输出
    float out;          // 当前输出
} ladrc_t;
```

### 2.4 初始化函数

```c
/**
 * @brief  二阶LADRC初始化
 * @param  ladrc: LADRC结构体指针
 * @param  max_output: 输出限幅值
 * @param  beta1: ESO增益beta1
 * @param  beta2: ESO增益beta2
 * @param  beta3: ESO增益beta3
 * @param  kp: 控制器比例增益
 * @param  kd: 控制器微分增益
 * @param  b: 控制增益
 * @param  dt: 采样周期(秒)
 * @param  k_aw: 抗积分饱和增益(0表示不使用)
 */
void ladrc_init(ladrc_t *ladrc, float max_output,
                float beta1, float beta2, float beta3,
                float kp, float kd, float b,
                float dt, float k_aw)
{
    // 初始化ESO参数
    ladrc->beta1 = beta1;
    ladrc->beta2 = beta2;
    ladrc->beta3 = beta3;

    // 初始化控制器参数
    ladrc->kp = kp;
    ladrc->kd = kd;
    ladrc->b = b;

    // 初始化状态估计值
    ladrc->x1 = 0.0f;
    ladrc->x2 = 0.0f;
    ladrc->x3 = 0.0f;

    // 初始化输出限制
    ladrc->max_output = max_output;

    // 初始化抗积分饱和参数
    ladrc->k_aw = k_aw;
    ladrc->pre_out = 0.0f;

    // 初始化采样周期
    ladrc->dt = dt;

    // 初始化输出
    ladrc->out = 0.0f;
}

/**
 * @brief  二阶LADRC参数调整
 * @param  ladrc: LADRC结构体指针
 * @param  beta1: ESO增益beta1
 * @param  beta2: ESO增益beta2
 * @param  beta3: ESO增益beta3
 * @param  kp: 控制器比例增益
 * @param  kd: 控制器微分增益
 * @param  b: 控制增益
 * @param  dt: 采样周期(秒)
 * @param  k_aw: 抗积分饱和增益
 */
void ladrc_reset(ladrc_t *ladrc, float beta1, float beta2, float beta3,
                 float kp, float kd, float b,
                 float dt, float k_aw)
{
    ladrc->beta1 = beta1;
    ladrc->beta2 = beta2;
    ladrc->beta3 = beta3;
    ladrc->kp = kp;
    ladrc->kd = kd;
    ladrc->b = b;
    ladrc->dt = dt;
    ladrc->k_aw = k_aw;
}
```

### 2.5 核心计算函数

```c
/**
 * @brief  二阶LADRC计算
 * @param  ladrc: LADRC结构体指针
 * @param  target: 目标值（经过TD平滑后的参考输入）
 * @param  measure: 系统实际输出（测量值）
 * @retval LADRC计算结果
 * @note   在每个控制周期调用一次
 * 
 * 控制流程：
 * 1. ESO估计系统状态和总扰动
 * 2. PD控制器计算虚拟控制量u0
 * 3. 扰动补偿得到实际控制量
 * 4. 输出限幅和抗积分饱和处理
 */
float ladrc_calc(ladrc_t *ladrc, float target, float measure)
{
    /* 步骤1: 执行三阶扩张状态观测器(ESO) */
    // 计算ESO微分方程
    float dx1 = ladrc->x2 +
                ladrc->beta1 * (measure - ladrc->x1);
    float dx2 = ladrc->x3 + ladrc->b * ladrc->pre_out +
                ladrc->beta2 * (measure - ladrc->x1);
    float dx3 = ladrc->beta3 * (measure - ladrc->x1);

    // 更新状态估计值(欧拉积分，乘以dt)
    ladrc->x1 += dx1 * ladrc->dt;
    ladrc->x2 += dx2 * ladrc->dt;
    ladrc->x3 += dx3 * ladrc->dt;

    /* 步骤2: 计算控制量u0 */
    // u0 = kp * (target - x1) - kd * x2
    float u0 = ladrc->kp * (target - ladrc->x1) - ladrc->kd * ladrc->x2;

    /* 步骤3: 扰动补偿得到实际控制量 */
    float out_temp;
    if (fabsf(ladrc->b) < 0.0001f) {
        out_temp = 0.0f;
    } else {
        out_temp = (u0 - ladrc->x3) / ladrc->b;
    }

    /* 步骤4: 输出限幅 */
    // 限幅函数
    if (out_temp > ladrc->max_output) {
        out_temp = ladrc->max_output;
    } else if (out_temp < -ladrc->max_output) {
        out_temp = -ladrc->max_output;
    }

    /* 步骤5: 抗积分饱和 */
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
```

### 2.6 参数说明

| 参数 | 符号 | 作用 | 整定建议 |
|------|------|------|----------|
| **ESO增益** | $\beta_1, \beta_2, \beta_3$ | 决定观测器跟踪速度和稳定性 | 基于带宽法：$\beta_1=3\omega_o, \beta_2=3\omega_o^2, \beta_3=\omega_o^3$ |
| **比例增益** | $k_p$ | 位置误差响应 | 基于带宽法：$k_p = \omega_c^2$ |
| **微分增益** | $k_d$ | 速度阻尼 | 基于带宽法：$k_d = 2\omega_c$ |
| **控制增益** | $b$ | 控制效率 | 从阶跃响应估计，需与实际系统匹配 |
| **输出限幅** | $max\_output$ | 执行器输出限制 | 根据执行器能力设置 |
| **抗饱和增益** | $k\_{aw}$ | 抑制积分饱和 | 0表示不使用，建议值0.1-1.0 |

**理解抗积分饱和**：

当控制量达到限幅值时，ESO中的扰动估计可能会持续累积（积分饱和），导致系统退出饱和时出现大的超调。抗积分饱和通过检测饱和误差，将误差反馈到扰动估计中，防止过度累积。

---

## 第三部分：一阶LADRC实现

### 3.1 一阶LADRC原理

一阶LADRC适用于被控对象为一阶的系统（如电流控制、简单速度控制等）：

**被控对象模型**：

$$
\dot{x} = f(x, w, t) + b \cdot u
$$

一阶LADRC使用**二阶ESO**（估计状态和一阶总扰动）。

### 3.2 代码实现

```c
/**
 * @brief  一阶LADRC结构体
 * @note   适用于电流/简单速度控制等一阶系统
 */
typedef struct {
    // ESO增益（二阶观测器）
    float beta1;        // ESO增益beta1
    float beta2;        // ESO增益beta2
    
    // 控制器增益
    float kp;           // 比例增益
    
    // 系统参数
    float b;            // 控制增益
    float dt;           // 采样周期(秒)
    
    // 状态估计值
    float x1;           // 估计的系统输出
    float x2;           // 估计的总扰动
    
    // 输出限制
    float max_output;   // 输出限幅值
    
    // 抗积分饱和
    float k_aw;         // 抗积分饱和增益（0表示不使用）
    float pre_out;      // 上一时刻输出
    
    // 输出
    float out;          // 当前输出
} first_order_ladrc_t;

/**
 * @brief  一阶LADRC初始化
 */
void first_order_ladrc_init(first_order_ladrc_t *fladrc, float max_output,
                           float beta1, float beta2, float kp, float b,
                           float dt, float k_aw)
{
    fladrc->beta1 = beta1;
    fladrc->beta2 = beta2;
    fladrc->kp = kp;
    fladrc->b = b;
    fladrc->x1 = 0.0f;
    fladrc->x2 = 0.0f;
    fladrc->max_output = max_output;
    fladrc->k_aw = k_aw;
    fladrc->pre_out = 0.0f;
    fladrc->dt = dt;
    fladrc->out = 0.0f;
}

/**
 * @brief  一阶LADRC计算函数
 */
float first_order_ladrc_calc(first_order_ladrc_t *fladrc,
                              float target, float measure)
{
    // 步骤1: 执行二阶ESO
    float dx1 = fladrc->x2 + fladrc->b * fladrc->pre_out +
                fladrc->beta1 * (measure - fladrc->x1);
    float dx2 = fladrc->beta2 * (measure - fladrc->x1);

    // 更新状态
    fladrc->x1 += dx1 * fladrc->dt;
    fladrc->x2 += dx2 * fladrc->dt;

    // 步骤2: 计算控制量
    float u0 = fladrc->kp * (target - fladrc->x1);

    // 步骤3: 扰动补偿
    float out_temp;
    if (fabsf(fladrc->b) < 0.0001f) {
        out_temp = 0.0f;
    } else {
        out_temp = (u0 - fladrc->x2) / fladrc->b;
    }

    // 步骤4: 输出限幅
    if (out_temp > fladrc->max_output) {
        out_temp = fladrc->max_output;
    } else if (out_temp < -fladrc->max_output) {
        out_temp = -fladrc->max_output;
    }

    // 步骤5: 抗积分饱和
    float aw_term = 0.0f;
    if (fladrc->k_aw > 0.0f) {
        float saturation_error = out_temp - fladrc->pre_out;
        aw_term = fladrc->k_aw * saturation_error;
    }

    // 保存输出
    fladrc->out = out_temp;
    fladrc->pre_out = out_temp;

    // 更新扰动估计
    fladrc->x2 += aw_term * fladrc->dt;

    return fladrc->out;
}
```

### 3.3 一阶LADRC参数说明

| 参数 | 符号 | 作用 | 整定建议 |
|------|------|------|----------|
| **ESO增益** | $\beta_1, \beta_2$ | 观测器增益 | 基于带宽法：$\beta_1=2\omega_o, \beta_2=\omega_o^2$ |
| **比例增益** | $k_p$ | 误差响应 | 基于带宽法：$k_p = \omega_c$ |

---

## 第四部分：完整使用示例

```c
#include <stdio.h>
#include <math.h>

// 假设上述所有代码已包含（ladrc.h和ladrc.c）

// 模拟被控对象（二阶系统）
float Plant_Update(float u, float h) {
    static float x1 = 0, x2 = 0;    // 位置和速度状态
    static float d = 0;              // 外部扰动
    
    // 系统动力学：ddot(y) = -a1*dot(y) - a0*y + b*u + d
    float a1 = 2.0f, a0 = 1.0f, b = 10.0f;
    float dx2 = -a1 * x2 - a0 * x1 + b * u + d;
    
    // 离散积分（欧拉法）
    x2 += h * dx2;      // 更新速度
    x1 += h * x2;       // 更新位置
    
    return x1;          // 返回输出（位置）
}

int main(void)
{
    // 定义TD和LADRC控制器
    td_t my_td;
    ladrc_t controller;
    
    // 系统参数
    float dt = 0.001f;              // 1ms采样周期
    float wo = 100.0f;              // 观测器带宽
    float wc = 25.0f;               // 控制器带宽
    float b = 10.0f;                // 控制增益
    
    // 计算LADRC参数（带宽法）
    float beta1 = 3.0f * wo;                // 3*wo
    float beta2 = 3.0f * wo * wo;           // 3*wo^2
    float beta3 = wo * wo * wo;             // wo^3
    float kp = wc * wc;                     // wc^2
    float kd = 2.0f * wc;                   // 2*wc
    
    // 初始化TD
    td_init(&my_td, 100.0f, dt, 5.0f * dt, 0.0f);
    
    // 初始化LADRC：max_output=50, k_aw=0.5（启用抗积分饱和）
    ladrc_init(&controller, 50.0f, beta1, beta2, beta3, 
               kp, kd, b, dt, 0.5f);
    
    printf("LADRC控制仿真开始...\n");
    printf("时间\t目标\t输出\t控制量\t扰动估计\n");
    
    // 仿真循环
    float y = 0.0f;                  // 系统输出
    for (int k = 0; k < 2000; k++) {
        float t = k * dt;            // 当前时间
        
        // 目标值：0-0.5s为0，0.5s后跳变到10
        float ref = (t > 0.5f) ? 10.0f : 0.0f;
        
        // 步骤1：TD处理目标值（平滑过渡）
        float smooth_ref = td_update(&my_td, ref, dt);
        
        // 步骤2：LADRC控制计算
        float u = ladrc_calc(&controller, smooth_ref, y);
        
        // 更新被控对象（实际系统中，这是物理过程）
        y = Plant_Update(u, dt);
        
        // 每100ms打印一次
        if (k % 100 == 0) {
            printf("%.3f\t%.2f\t%.2f\t%.2f\t%.2f\n", 
                   t, ref, y, u, controller.x3);
        }
    }
    
    printf("仿真结束\n");
    return 0;
}
```

---

## 第五部分：调试技巧与常见问题

### 5.1 调试步骤

LADRC参数整定的一般流程：

```
步骤1: 确定系统阶数
    └─→ 观察被控对象：输入到输出需要几次积分
    
步骤2: 估计b
    ├─→ 施加阶跃输入，记录输出响应
    ├─→ 测量初始加速度：a0 = (y[2] - 2*y[1] + y[0]) / h^2
    └─→ 计算：b = a0 / U（U为阶跃幅值）
    
步骤3: 设置带宽参数
    ├─→ wc = 期望控制器带宽（如：希望0.1s响应，wc≈10*2π/0.1≈60）
    ├─→ wo = 4 * wc（初始比例，观测器带宽）
    └─→ 计算LADRC增益：
        - β1 = 3*wo, β2 = 3*wo^2, β3 = wo^3
        - kp = wc^2, kd = 2*wc
    
步骤4: 设置TD参数
    ├─→ r = 100（TD速度因子，初始值）
    └─→ h0 = 5*dt（滤波因子）
    
步骤5: 逐步调试wc
    ├─→ 先设较小的wc（如目标值的50%）
    ├─→ 测试响应，如过慢则增大20%
    └─→ 直到响应速度满足要求
    
步骤6: 调试wo
    ├─→ 固定wc，调整wo/wc比例
    ├─→ 如果估计滞后明显：增大wo
    └─→ 如果噪声敏感：减小wo
    
步骤7: 微调b
    ├─→ 如果系统振荡：减小b（如减10%）
    └─→ 如果响应迟缓：增大b（如增10%）
    
步骤8: 优化TD参数
    ├─→ 如果目标跳变时超调大：减小r或增大h0
    └─→ 如果过渡过程过慢：增大r
```

### 5.2 常见问题排查

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| **系统振荡** | wc过大、b过大、wo过大 | 逐步减小参数，每次减10-20% |
| **响应迟缓** | wc过小、b过小 | 增大wc或b |
| **估计滞后** | wo过小 | 增大wo，但不要超过5wc |
| **噪声敏感** | wo过大、h0过小 | 减小wo，或增大h0 |
| **超调严重** | wc过大、r过大 | 减小wc或r |
| **稳态误差** | b估计不准 | 微调b，或检查是否有积分作用 |
| **启动冲击** | TD参数不当 | 减小r或增大h0 |
| **饱和后超调** | 未启用抗积分饱和 | 增大k_aw（建议0.1-1.0） |

### 5.3 参数整定实操

#### 案例1：电机位置控制

**系统特性**：
- 类型：直流电机+编码器
- 采样周期：1ms
- 期望响应：0.1s内到达目标位置

**整定过程**：

```c
// 步骤1：估计b
// 施加5V电压，测量初始加速度约为 500 rad/s^2
// b = 500 / 5 = 100
float b = 100.0f;

// 步骤2：设置带宽
// 期望0.1s响应，wc ≈ 10 / 0.1 = 100 rad/s
// 先取保守值 50
float wc = 50.0f;
float wo = 4.0f * wc;  // 200 rad/s

// 步骤3：计算LADRC增益
float beta1 = 3.0f * wo;
float beta2 = 3.0f * wo * wo;
float beta3 = wo * wo * wo;
float kp = wc * wc;
float kd = 2.0f * wc;

// 步骤4：初始化TD和LADRC
td_init(&my_td, 100.0f, 0.001f, 0.005f, 0.0f);
ladrc_init(&controller, 50.0f, beta1, beta2, beta3, 
           kp, kd, b, 0.001f, 0.5f);

// 测试结果：响应0.15s，略慢
// 调整：wc = 70，重新计算kp、kd，wo = 280
// 测试结果：响应0.08s，有轻微超调
// 调整：wc = 60，wo = 240
// 测试结果：响应0.1s，超调<5%，满意！
```

#### 案例2：温度控制

**系统特性**：
- 类型：加热器+温度传感器
- 采样周期：100ms
- 特性：大惯性、大延迟

**整定要点**：

```c
// 温度系统响应慢，wc不宜过大
float wc = 0.5f;        // 很慢的控制器
float wo = 3.0f * wc;   // wo=1.5，保守设计（噪声环境）

// b估计：从阶跃响应估算升温速率
// 施加100%功率，初始升温速率约 2°C/s
// b = 2 / 1 = 2
float b = 2.0f;

// 计算增益
float beta1 = 3.0f * wo;
float beta2 = 3.0f * wo * wo;
float beta3 = wo * wo * wo;
float kp = wc * wc;
float kd = 2.0f * wc;

// TD参数：温度系统不宜突变，TD参数要保守
float r = 10.0f;        // 较慢的跟踪速度
float h0 = 0.5f;        // 较强的滤波

td_init(&my_td, r, 0.1f, h0, 0.0f);
ladrc_init(&controller, 100.0f, beta1, beta2, beta3,
           kp, kd, b, 0.1f, 0.0f);
```

### 5.4 调试技巧

**1. 实时监控关键变量**

在调试阶段，建议实时输出以下变量：
- 目标值 `ref`
- 实际输出 `y`
- 估计的扰动 `z3`
- 控制量 `u`
- 虚拟控制量 `u0`

通过观察这些变量的波形，可以快速定位问题：

```c
// 调试输出示例（通过串口或日志）
void LADRC_DebugOutput(ladrc_t *ladrc, td_t *td, float ref, float y)
{
    printf("ref=%.2f, y=%.2f, x1=%.2f, x2=%.2f, x3=%.2f, out=%.2f\n",
           ref, y, ladrc->x1, ladrc->x2, ladrc->x3, ladrc->out);
    printf("TD: x1=%.2f, x2=%.2f\n", td->x1, td->x2);
}
```

**2. 参数调整口诀**

```
响应慢 → 增wc
有振荡 → 减wc或减b0
估计慢 → 增wo
噪声大 → 减wo
有超调 → 减r（TD速度）
```

**3. 安全限幅**

在实际系统中，控制量通常有限制（如电机电压、PWM占空比）：

```c
// ladrc_calc内部已集成输出限幅，只需设置max_output参数
// 如需动态调整限幅值，可直接修改结构体

void LADRC_SetOutputLimit(ladrc_t *ladrc, float max_output)
{
    ladrc->max_output = max_output;
}

// 实际使用时，ladrc_calc自动处理限幅
// 抗积分饱和通过k_aw参数启用
void LADRC_SetAntiWindup(ladrc_t *ladrc, float k_aw)
{
    ladrc->k_aw = k_aw;  // 建议值0.1-1.0，0表示禁用
}
```

**4. 在线参数调整**

在调试阶段，可以设计通信接口实现参数在线调整：

```c
// 通过串口接收命令调整参数
void LADRC_AdjustParameter(ladrc_t *ladrc, char param, float value)
{
    switch(param) {
        case 'w':  // wo - 重新计算beta增益
            ladrc->beta1 = 3.0f * value;
            ladrc->beta2 = 3.0f * value * value;
            ladrc->beta3 = value * value * value;
            break;
        case 'c':  // wc - 重新计算PD增益
            ladrc->kp = value * value;
            ladrc->kd = 2.0f * value;
            break;
        case 'b':  // b - 控制增益
            ladrc->b = value;
            break;
    }
}
```

---

> 💡 **总结**：
> - **TD模块**：负责目标值平滑和微分提取，关键参数r（速度因子）和h0（滤波因子）
> - **二阶LADRC**：适用于位置/角度控制，包含三阶ESO和PD控制器，关键参数β1/β2/β3（观测器增益）、kp/kd（控制器增益）、b（控制增益）
> - **一阶LADRC**：适用于电流/速度控制，包含二阶ESO和P控制器
> - **抗积分饱和**：通过k_aw参数启用，防止饱和后超调
> 
> 建议初学者按照文档顺序先理解每个模块的作用，再逐步调试参数，最终你会体会到LADRC"以简御繁"的设计哲学。
