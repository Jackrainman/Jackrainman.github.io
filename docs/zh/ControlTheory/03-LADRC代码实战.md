# LADRC代码实战

> **最近修改日期**：2026-02-06  
> **参与者**：Jackrainman  
> **前置知识**：[02-LADRC算法详解.md](./02-LADRC算法详解.md)

---

## 目录

1. [前置知识](#前置知识)
2. [第一部分：跟踪微分器(TD)实现](#第一部分跟踪微分器td实现)
3. [第二部分：线性扩张状态观测器(LESO)离散实现](#第二部分线性扩张状态观测器leso离散实现)
4. [第三部分：线性误差反馈控制律(LSEF)](#第三部分线性误差反馈控制律lsef)
5. [第四部分：完整LADRC控制器](#第四部分完整ladrc控制器)
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
    float r;        // 速度因子（调节跟踪快慢）
    float h;        // 滤波因子（调节平滑程度）
    float h0;       // 计算步长（一般等于或略大于采样周期）
} TD_TypeDef;

/**
 * @brief  最速跟踪控制函数（fhan函数）
 * @param  x1: 当前位置误差
 * @param  x2: 当前速度
 * @param  r: 速度因子
 * @param  h: 滤波因子
 * @retval 控制量u
 * @note   这是TD的核心算法，实现非线性最速控制
 */
float TD_fhan(float x1, float x2, float r, float h)
{
    float d, d0, y, a0, a, u;
    
    // 计算中间变量
    d = r * h;          // 边界层厚度
    d0 = h * d;         // 边界层厚度平方
    y = x1 + h * x2;    // 预测下一时刻位置
    a0 = sqrtf(d * d + 8 * r * fabsf(y));  // 计算加速度边界
    
    // 根据位置误差选择控制策略
    if (fabsf(y) > d0) {
        // 远离目标，使用最大加速度
        a = x2 + (a0 - d) / 2 * sign(y);
    } else {
        // 接近目标，减小加速度
        a = x2 + y / h;
    }
    
    // 计算最终控制量
    if (fabsf(a) > d) {
        u = -r * sign(a);
    } else {
        u = -r * a / d;
    }
    
    return u;
}

/**
 * @brief  符号函数
 * @param  x: 输入值
 * @retval 符号（-1, 0, 1）
 */
float sign(float x)
{
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

/**
 * @brief  跟踪微分器更新函数
 * @param  td: TD结构体指针
 * @param  input: 输入信号（给定值）
 * @note   在每个控制周期调用一次
 */
void TD_Update(TD_TypeDef *td, float input)
{
    float u;
    
    // 计算跟踪误差
    float error = td->x1 - input;
    
    // 使用fhan函数计算控制量
    u = TD_fhan(error, td->x2, td->r, td->h0);
    
    // 离散积分更新状态（欧拉法）
    // x2是速度，对其积分得到位置x1
    td->x1 += td->h * td->x2;
    
    // x2的导数是控制量u
    td->x2 += td->h * u;
}

/**
 * @brief  跟踪微分器初始化
 * @param  td: TD结构体指针
 * @param  r: 速度因子（越大跟踪越快）
 * @param  h: 滤波因子（越大滤波效果越好）
 * @param  sample_time: 采样周期
 */
void TD_Init(TD_TypeDef *td, float r, float h, float sample_time)
{
    td->x1 = 0.0f;
    td->x2 = 0.0f;
    td->r = r;
    td->h = sample_time;
    td->h0 = h;
}
```

### 1.3 参数说明

| 参数       | 符号  | 作用     | 调节建议                                                                |
| -------- | --- | ------ | ------------------------------------------------------------------- |
| **速度因子** | $r$ | 决定跟踪速度 | $r$ 越大，跟踪越快，但可能超调。建议初始值：$r = 1/T_{settle}$，其中 $T_{settle}$ 是期望的过渡时间 |
| **滤波因子** | $h$ | 决定平滑程度 | $h$ 越大，滤波效果越好，但相位滞后越大。通常 $h = (2/sim5) 	imes sample\_time$          |

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
TD_TypeDef my_td;

// 初始化：速度因子r=100，滤波因子h=0.01，采样周期0.001s（1ms）
TD_Init(&my_td, 100.0f, 0.01f, 0.001f);

// 主循环（每1ms执行一次）
while(1) {
    float target = Get_Target();        // 获取目标值
    TD_Update(&my_td, target);          // 更新TD
    
    float smooth_target = my_td.x1;     // 平滑后的目标值
    float target_speed = my_td.x2;      // 目标值的变化率
    
    // 将smooth_target和target_speed用于后续控制...
    
    Delay_ms(1);                        // 等待下一个周期
}
```

---

## 第二部分：线性扩张状态观测器(LESO)离散实现

### 2.1 离散ESO原理

在 [02-LADRC算法详解.md](./02-LADRC算法详解.md) 的第六章中，我们详细推导了零阶保持法的离散化过程。这里回顾一下二阶系统LESO的离散方程：

$$
\begin{cases}
z_1(k+1) = z_1(k) + h \cdot z_2(k) + \frac{h^2}{2} \cdot z_3(k) + \frac{b_0 h^2}{2} \cdot u(k) + l_{c1} \cdot (y(k) - z_1(k)) \\
z_2(k+1) = z_2(k) + h \cdot z_3(k) + b_0 h \cdot u(k) + l_{c2} \cdot (y(k) - z_1(k)) \\
z_3(k+1) = z_3(k) + l_{c3} \cdot (y(k) - z_1(k))
\end{cases}
$$

其中：
- $z_1, z_2$：估计的状态（位置、速度）
- $z_3$：估计的总扰动
- $l_{c1}, l_{c2}, l_{c3}$：离散观测器增益

### 2.2 代码实现

```c
/**
 * @brief  LESO结构体（二阶系统）
 * @note   估计位置、速度和总扰动
 */
typedef struct {
    // 观测器状态
    float z1;       // 估计的位置
    float z2;       // 估计的速度
    float z3;       // 估计的总扰动
    
    // 观测器增益（离散）
    float lc1;      // z1的增益
    float lc2;      // z2的增益
    float lc3;      // z3的增益
    
    // 系统参数
    float b0;       // 控制量系数
    float h;        // 采样周期
} LESO_TypeDef;

/**
 * @brief  LESO初始化
 * @param  leso: LESO结构体指针
 * @param  wo: 观测器带宽
 * @param  b0: 控制量系数
 * @param  sample_time: 采样周期
 * @note   根据带宽自动计算离散增益
 */
void LESO_Init(LESO_TypeDef *leso, float wo, float b0, float sample_time)
{
    float h = sample_time;
    
    // 初始化状态
    leso->z1 = 0.0f;
    leso->z2 = 0.0f;
    leso->z3 = 0.0f;
    
    // 保存参数
    leso->b0 = b0;
    leso->h = h;
    
    // 计算离散观测器增益（基于零阶保持法）
    // lc1 = 3*wo*h, lc2 = 3*wo^2*h, lc3 = wo^3*h
    leso->lc1 = 3.0f * wo * h;
    leso->lc2 = 3.0f * wo * wo * h;
    leso->lc3 = wo * wo * wo * h;
}

/**
 * @brief  LESO更新函数
 * @param  leso: LESO结构体指针
 * @param  y: 系统实际输出（测量值）
 * @param  u: 控制量（用于计算控制作用）
 * @note   在每个控制周期调用，更新状态估计
 */
void LESO_Update(LESO_TypeDef *leso, float y, float u)
{
    float h = leso->h;
    float b0 = leso->b0;
    
    // 计算估计误差
    float e = y - leso->z1;
    
    // 临时保存当前状态（用于计算）
    float z1_old = leso->z1;
    float z2_old = leso->z2;
    float z3_old = leso->z3;
    
    // 更新z1（位置估计）
    // z1(k+1) = z1(k) + h*z2(k) + (h^2/2)*z3(k) + (b0*h^2/2)*u(k) + lc1*e
    leso->z1 = z1_old + h * z2_old + (h * h / 2.0f) * z3_old 
               + (b0 * h * h / 2.0f) * u + leso->lc1 * e;
    
    // 更新z2（速度估计）
    // z2(k+1) = z2(k) + h*z3(k) + b0*h*u(k) + lc2*e
    leso->z2 = z2_old + h * z3_old + b0 * h * u + leso->lc2 * e;
    
    // 更新z3（扰动估计）
    // z3(k+1) = z3(k) + lc3*e
    leso->z3 = z3_old + leso->lc3 * e;
}

/**
 * @brief  获取LESO估计的状态
 * @param  leso: LESO结构体指针
 * @param  z1: 位置估计值输出指针（可为NULL）
 * @param  z2: 速度估计值输出指针（可为NULL）
 * @param  z3: 扰动估计值输出指针（可为NULL）
 */
void LESO_GetState(LESO_TypeDef *leso, float *z1, float *z2, float *z3)
{
    if (z1 != NULL) *z1 = leso->z1;
    if (z2 != NULL) *z2 = leso->z2;
    if (z3 != NULL) *z3 = leso->z3;
}
```

### 2.3 参数说明

| 参数 | 符号 | 作用 | 整定建议 |
|------|------|------|----------|
| **观测器带宽** | $\omega_o$ | 决定观测器响应速度 | $\omega_o = (3\sim5) \times \omega_c$，初始可取 $4\omega_c$ |
| **控制量系数** | $b_0$ | 控制效率 | 从阶跃响应估计：$b_0 = \ddot{y}(0) / U$ |
| **采样周期** | $h$ | 离散化时间步长 | 通常 $h < \frac{1}{10\omega_o}$ |

**为什么离散增益是 $3\omega_o h, 3\omega_o^2 h, \omega_o^3 h$？**

这是零阶保持法离散化的结果。直观理解：
- 采样周期 $h$ 越小，每次修正的增益应该越小（因为更新更频繁）
- 因此离散增益与 $h$ 成正比
- 系数 $3, 3, 1$ 是二项式展开系数，保证极点配置正确

---

## 第三部分：线性误差反馈控制律(LSEF)

### 3.1 控制律实现

LSEF的作用是根据误差生成虚拟控制量 $u_0$，采用PD控制形式：

$$
u_0 = k_p(r - z_1) - k_d z_2
$$

```c
/**
 * @brief  LSEF结构体
 * @note   线性误差反馈控制律
 */
typedef struct {
    float kp;       // 比例增益
    float kd;       // 微分增益
    float wc;       // 控制器带宽（用于自动计算增益）
} LSEF_TypeDef;

/**
 * @brief  LSEF初始化（带宽法）
 * @param  lsef: LSEF结构体指针
 * @param  wc: 控制器带宽
 * @note   自动计算kp和kd：kp=wc^2, kd=2*wc
 */
void LSEF_Init(LSEF_TypeDef *lsef, float wc)
{
    lsef->wc = wc;
    lsef->kp = wc * wc;      // kp = wc^2
    lsef->kd = 2.0f * wc;    // kd = 2*wc
}

/**
 * @brief  LSEF计算
 * @param  lsef: LSEF结构体指针
 * @param  ref: 参考输入（经过TD平滑后的目标值）
 * @param  z1: 估计的位置
 * @param  z2: 估计的速度
 * @retval 虚拟控制量u0
 */
float LSEF_Calculate(LSEF_TypeDef *lsef, float ref, float z1, float z2)
{
    float error = ref - z1;          // 位置误差
    float u0 = lsef->kp * error - lsef->kd * z2;  // PD控制
    return u0;
}

/**
 * @brief  LSEF初始化（直接设置增益）
 * @param  lsef: LSEF结构体指针
 * @param  kp: 比例增益
 * @param  kd: 微分增益
 * @note   如需手动整定，使用此函数
 */
void LSEF_InitDirect(LSEF_TypeDef *lsef, float kp, float kd)
{
    lsef->kp = kp;
    lsef->kd = kd;
    lsef->wc = sqrtf(kp);  // 反推wc（仅作记录）
}
```

### 3.2 完整控制律（含扰动补偿）

完整的LADRC控制律包括LSEF生成的虚拟控制量 $u_0$ 和对扰动的补偿：

$$
u = \frac{u_0 - z_3}{b_0}
$$

```c
/**
 * @brief  完整LADRC控制律计算
 * @param  u0: 虚拟控制量（来自LSEF）
 * @param  z3: 估计的总扰动（来自LESO）
 * @param  b0: 控制量系数
 * @retval 实际控制量u
 * @note   u = (u0 - z3) / b0
 */
float LADRC_ControlLaw(float u0, float z3, float b0)
{
    // 扰动补偿：抵消估计的总扰动
    float u = (u0 - z3) / b0;
    return u;
}
```

**理解扰动补偿**：

- $z_3$ 是LESO估计的"总扰动"，包括内部不确定性和外部干扰
- $u_0$ 是我们期望的控制作用（假设没有扰动）
- $(u_0 - z_3)$ 表示我们需要额外的控制量来抵消扰动
- 除以 $b_0$ 是因为控制量需要转换为实际的执行器输入

---

## 第四部分：完整LADRC控制器

### 4.1 结构体定义

将TD、LESO、LSEF整合为一个完整的LADRC控制器：

```c
/**
 * @brief  完整LADRC控制器结构体（二阶系统）
 */
typedef struct {
    // 三个核心组件
    TD_TypeDef td;          // 跟踪微分器
    LESO_TypeDef leso;      // 线性扩张状态观测器
    LSEF_TypeDef lsef;      // 线性误差反馈控制律
    
    // 系统参数
    float b0;               // 控制量系数
    float h;                // 采样周期
    
    // 内部状态（用于调试和监视）
    float u0;               // 虚拟控制量
    float u;                // 实际控制量
    float y;                // 系统输出（记录）
    float ref;              // 参考输入（记录）
} LADRC_TypeDef;
```

### 4.2 初始化函数

```c
/**
 * @brief  LADRC控制器初始化
 * @param  ladrc: LADRC结构体指针
 * @param  wo: 观测器带宽
 * @param  wc: 控制器带宽
 * @param  b0: 控制量系数
 * @param  r: TD速度因子
 * @param  h_td: TD滤波因子
 * @param  sample_time: 采样周期
 * @retval 0: 成功, -1: 参数错误
 */
int LADRC_Init(LADRC_TypeDef *ladrc, 
               float wo, float wc, float b0,
               float r, float h_td,
               float sample_time)
{
    // 参数检查
    if (wo <= 0 || wc <= 0 || b0 <= 0 || sample_time <= 0) {
        return -1;  // 参数错误
    }
    
    // 保存参数
    ladrc->b0 = b0;
    ladrc->h = sample_time;
    
    // 初始化三个组件
    TD_Init(&ladrc->td, r, h_td, sample_time);
    LESO_Init(&ladrc->leso, wo, b0, sample_time);
    LSEF_Init(&ladrc->lsef, wc);
    
    // 初始化内部状态
    ladrc->u0 = 0.0f;
    ladrc->u = 0.0f;
    ladrc->y = 0.0f;
    ladrc->ref = 0.0f;
    
    return 0;
}

/**
 * @brief  LADRC快速初始化（简化版）
 * @param  ladrc: LADRC结构体指针
 * @param  wo: 观测器带宽
 * @param  wc: 控制器带宽
 * @param  b0: 控制量系数
 * @param  sample_time: 采样周期
 * @note   使用默认TD参数，适合大多数场景
 */
void LADRC_InitSimple(LADRC_TypeDef *ladrc, 
                      float wo, float wc, float b0,
                      float sample_time)
{
    // 默认TD参数：r=100, h=5*sample_time
    float r = 100.0f;
    float h_td = 5.0f * sample_time;
    
    LADRC_Init(ladrc, wo, wc, b0, r, h_td, sample_time);
}
```

### 4.3 核心计算函数

```c
/**
 * @brief  LADRC控制器更新（核心函数）
 * @param  ladrc: LADRC结构体指针
 * @param  ref: 参考输入（目标值）
 * @param  y: 系统实际输出（测量值）
 * @retval 实际控制量u
 * @note   在每个控制周期调用一次
 * 
 * 控制流程：
 * 1. TD处理参考输入，得到平滑的目标值和微分
 * 2. LESO估计系统状态和总扰动
 * 3. LSEF计算虚拟控制量
 * 4. 扰动补偿得到实际控制量
 */
float LADRC_Update(LADRC_TypeDef *ladrc, float ref, float y)
{
    // 步骤1：TD处理参考输入
    TD_Update(&ladrc->td, ref);
    float ref_smooth = ladrc->td.x1;    // 平滑后的目标值
    // float ref_dot = ladrc->td.x2;    // 目标值变化率（如需前馈可使用）
    
    // 步骤2：LESO更新（使用上一步的控制量u）
    LESO_Update(&ladrc->leso, y, ladrc->u);
    
    float z1, z2, z3;
    LESO_GetState(&ladrc->leso, &z1, &z2, &z3);
    
    // 步骤3：LSEF计算虚拟控制量
    ladrc->u0 = LSEF_Calculate(&ladrc->lsef, ref_smooth, z1, z2);
    
    // 步骤4：扰动补偿，得到实际控制量
    ladrc->u = LADRC_ControlLaw(ladrc->u0, z3, ladrc->b0);
    
    // 记录状态（用于调试）
    ladrc->y = y;
    ladrc->ref = ref;
    
    return ladrc->u;
}

/**
 * @brief  获取LADRC内部状态（用于调试和监视）
 * @param  ladrc: LADRC结构体指针
 * @param  u0: 虚拟控制量输出指针（可为NULL）
 * @param  z1: 位置估计输出指针（可为NULL）
 * @param  z2: 速度估计输出指针（可为NULL）
 * @param  z3: 扰动估计输出指针（可为NULL）
 */
void LADRC_GetInternalState(LADRC_TypeDef *ladrc, 
                            float *u0, float *z1, float *z2, float *z3)
{
    if (u0 != NULL) *u0 = ladrc->u0;
    if (z1 != NULL) LESO_GetState(&ladrc->leso, z1, NULL, NULL);
    if (z2 != NULL) LESO_GetState(&ladrc->leso, NULL, z2, NULL);
    if (z3 != NULL) LESO_GetState(&ladrc->leso, NULL, NULL, z3);
}

/**
 * @brief  重置LADRC控制器
 * @param  ladrc: LADRC结构体指针
 * @note   清除所有状态，保持参数不变
 */
void LADRC_Reset(LADRC_TypeDef *ladrc)
{
    // 重置TD
    ladrc->td.x1 = 0.0f;
    ladrc->td.x2 = 0.0f;
    
    // 重置LESO
    ladrc->leso.z1 = 0.0f;
    ladrc->leso.z2 = 0.0f;
    ladrc->leso.z3 = 0.0f;
    
    // 重置内部状态
    ladrc->u0 = 0.0f;
    ladrc->u = 0.0f;
    ladrc->y = 0.0f;
    ladrc->ref = 0.0f;
}
```

### 4.4 完整使用示例

```c
#include <stdio.h>
#include <math.h>

// 假设上述所有代码已包含

// 模拟被控对象（二阶系统）
float Plant_Update(float u, float h) {
    static float x1 = 0, x2 = 0;    // 位置和速度状态
    static float d = 0;              // 外部扰动
    
    // 模拟外部扰动（如负载变化）
    // 在实际系统中，这部分是真实的物理过程
    
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
    // 定义LADRC控制器
    LADRC_TypeDef controller;
    
    // 系统参数
    float sample_time = 0.001f;     // 1ms采样周期
    float wo = 100.0f;               // 观测器带宽
    float wc = 25.0f;                // 控制器带宽（wo/wc = 4）
    float b0 = 10.0f;                // 控制量系数（应与实际b接近）
    
    // 初始化
    LADRC_InitSimple(&controller, wo, wc, b0, sample_time);
    
    // 或者使用完整初始化自定义TD参数
    // LADRC_Init(&controller, wo, wc, b0, 200.0f, 0.005f, sample_time);
    
    printf("LADRC控制仿真开始...\n");
    printf("时间\t目标\t输出\t控制量\t扰动估计\n");
    
    // 仿真循环
    float y = 0.0f;                  // 系统输出
    for (int k = 0; k < 2000; k++) {
        float t = k * sample_time;   // 当前时间
        
        // 目标值：0-0.5s为0，0.5s后跳变到10
        float ref = (t > 0.5f) ? 10.0f : 0.0f;
        
        // LADRC控制计算
        float u = LADRC_Update(&controller, ref, y);
        
        // 控制量限幅（实际执行器有输出限制）
        if (u > 50.0f) u = 50.0f;
        if (u < -50.0f) u = -50.0f;
        
        // 更新被控对象（实际系统中，这是物理过程）
        y = Plant_Update(u, sample_time);
        
        // 每100ms打印一次
        if (k % 100 == 0) {
            float z3;
            LESO_GetState(&controller.leso, NULL, NULL, &z3);
            printf("%.3f\t%.2f\t%.2f\t%.2f\t%.2f\n", t, ref, y, u, z3);
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
    
步骤2: 估计b0
    ├─→ 施加阶跃输入，记录输出响应
    ├─→ 测量初始加速度：a0 = (y[2] - 2*y[1] + y[0]) / h^2
    └─→ 计算：b0 = a0 / U（U为阶跃幅值）
    
步骤3: 初始参数设置
    ├─→ wc = 期望带宽（如：希望0.1s响应，wc≈10*2π/0.1≈60）
    ├─→ wo = 4 * wc（初始比例）
    └─→ r = 100（TD速度因子，初始值）
    
步骤4: 逐步调试wc
    ├─→ 先设较小的wc（如目标值的50%）
    ├─→ 测试响应，如过慢则增大20%
    └─→ 直到响应速度满足要求
    
步骤5: 调试wo
    ├─→ 固定wc，调整wo/wc比例
    ├─→ 如果估计滞后明显：增大wo
    └─→ 如果噪声敏感：减小wo
    
步骤6: 微调b0
    ├─→ 如果系统振荡：减小b0（如减10%）
    └─→ 如果响应迟缓：增大b0（如增10%）
    
步骤7: 优化TD参数
    ├─→ 如果目标跳变时超调大：减小r或增大h
    └─→ 如果过渡过程过慢：增大r
```

### 5.2 常见问题排查

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| **系统振荡** | wc过大、b0过大、wo过大 | 逐步减小参数，每次减10-20% |
| **响应迟缓** | wc过小、b0过小 | 增大wc或b0 |
| **估计滞后** | wo过小 | 增大wo，但不要超过5wc |
| **噪声敏感** | wo过大、h过小 | 减小wo，或增大滤波因子 |
| **超调严重** | wc过大、r过大 | 减小wc或r |
| **稳态误差** | b0估计不准 | 微调b0，或检查是否有积分作用 |
| **启动冲击** | TD参数不当 | 减小r或增大h |

### 5.3 参数整定实操

#### 案例1：电机位置控制

**系统特性**：
- 类型：直流电机+编码器
- 采样周期：1ms
- 期望响应：0.1s内到达目标位置

**整定过程**：

```c
// 步骤1：估计b0
// 施加5V电压，测量初始加速度约为 500 rad/s^2
// b0 = 500 / 5 = 100
float b0 = 100.0f;

// 步骤2：设置wc
// 期望0.1s响应，wc ≈ 10 / 0.1 = 100 rad/s
// 先取保守值 50
float wc = 50.0f;

// 步骤3：设置wo
float wo = 4.0f * wc;  // 200 rad/s

// 步骤4：初始化并测试
LADRC_InitSimple(&controller, wo, wc, b0, 0.001f);

// 测试结果：响应0.15s，略慢
// 调整：wc = 70，wo = 280
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

// b0估计：从阶跃响应估算升温速率
// 施加100%功率，初始升温速率约 2°C/s
// b0 = 2 / 1 = 2
float b0 = 2.0f;

// TD参数：温度系统不宜突变，TD参数要保守
float r = 10.0f;        // 较慢的跟踪速度
float h_td = 0.5f;      // 较强的滤波

LADRC_Init(&controller, wo, wc, b0, r, h_td, 0.1f);
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
void LADRC_DebugOutput(LADRC_TypeDef *ladrc)
{
    float z1, z2, z3;
    LESO_GetState(&ladrc->leso, &z1, &z2, &z3);
    
    printf("ref=%.2f, y=%.2f, z1=%.2f, z2=%.2f, z3=%.2f, u0=%.2f, u=%.2f\n",
           ladrc->ref, ladrc->y, z1, z2, z3, ladrc->u0, ladrc->u);
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
float LADRC_UpdateWithLimit(LADRC_TypeDef *ladrc, float ref, float y, 
                            float u_min, float u_max)
{
    float u = LADRC_Update(ladrc, ref, y);
    
    // 限幅
    if (u > u_max) u = u_max;
    if (u < u_min) u = u_min;
    
    // 重要：将限幅后的u更新回控制器（抗积分饱和）
    ladrc->u = u;
    
    return u;
}
```

**4. 在线参数调整**

在调试阶段，可以设计通信接口实现参数在线调整：

```c
// 通过串口接收命令调整参数
void LADRC_AdjustParameter(LADRC_TypeDef *ladrc, char param, float value)
{
    switch(param) {
        case 'w':  // wo
            ladrc->leso.lc1 = 3.0f * value * ladrc->h;
            ladrc->leso.lc2 = 3.0f * value * value * ladrc->h;
            ladrc->leso.lc3 = value * value * value * ladrc->h;
            break;
        case 'c':  // wc
            ladrc->lsef.kp = value * value;
            ladrc->lsef.kd = 2.0f * value;
            break;
        case 'b':  // b0
            ladrc->b0 = value;
            ladrc->leso.b0 = value;
            break;
    }
}
```

---

## 附录：一阶系统LADRC实现

对于一阶系统（如电流环、简单温度控制），LESO可以简化为2阶：

```c
/**
 * @brief  一阶系统LESO结构体
 */
typedef struct {
    float z1;       // 估计的输出
    float z2;       // 估计的总扰动
    float lc1;      // z1增益 = 2*wo*h
    float lc2;      // z2增益 = wo^2*h
    float b0;
    float h;
} LESO_1st_TypeDef;

void LESO_1st_Init(LESO_1st_TypeDef *leso, float wo, float b0, float h)
{
    leso->z1 = 0.0f;
    leso->z2 = 0.0f;
    leso->b0 = b0;
    leso->h = h;
    leso->lc1 = 2.0f * wo * h;
    leso->lc2 = wo * wo * h;
}

void LESO_1st_Update(LESO_1st_TypeDef *leso, float y, float u)
{
    float e = y - leso->z1;
    float z1_old = leso->z1;
    float z2_old = leso->z2;
    
    // 一阶LESO离散方程
    leso->z1 = z1_old + leso->h * (z2_old + leso->b0 * u) + leso->lc1 * e;
    leso->z2 = z2_old + leso->lc2 * e;
}
```

---

> 💡 **总结**：LADRC的代码实现虽然涉及多个模块，但每个模块都有清晰的物理意义。建议初学者先理解每个模块的作用，再逐步调试参数，最终你会体会到LADRC"以简御繁"的设计哲学。
