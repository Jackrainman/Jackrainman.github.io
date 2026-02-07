# 线性自抗扰控制（LADRC）详解

> 最近修改日期：2026-02-06
> 参与者：Jackrainman

## 概述

线性自抗扰控制（LADRC）是由高志强教授在韩京清教授原始 ADRC 基础上提出的工程化改进方案。其核心思想是将 ADRC 的参数与控制器和观测器的带宽联系起来，把复杂的参数整定问题转化为简单的带宽调节问题。

---

## 目录

1. [为什么需要 LADRC](LADRC_已废除，请勿查看.md#1-为什么需要-ladrc)
2. [核心概念回顾](LADRC_已废除，请勿查看.md#2-核心概念回顾)
   - [2.1 积分器串联型](LADRC_已废除，请勿查看.md#21-积分器串联型)
   - [2.2 总扰动](LADRC_已废除，请勿查看.md#22-总扰动)
3. [LADRC 数学模型](LADRC_已废除，请勿查看.md#3-ladrc-数学模型)
   - [3.1 系统描述](LADRC_已废除，请勿查看.md#31-系统描述)
   - [3.2 线性扩张状态观测器](LADRC_已废除，请勿查看.md#32-线性扩张状态观测器)
   - [3.3 线性误差反馈控制律](LADRC_已废除，请勿查看.md#33-线性误差反馈控制律)
4. [离散化实现](LADRC_已废除，请勿查看.md#4-离散化实现)
   - [4.1 零阶保持法](LADRC_已废除，请勿查看.md#41-零阶保持法)
   - [4.2 一阶系统离散化](LADRC_已废除，请勿查看.md#42-一阶系统离散化)
   - [4.3 二阶系统离散化](LADRC_已废除，请勿查看.md#43-二阶系统离散化)
5. [参数整定方法](LADRC_已废除，请勿查看.md#5-参数整定方法)
   - [5.1 扰动补偿系数 b₀](LADRC_已废除，请勿查看.md#51-扰动补偿系数-b₀)
   - [5.2 控制器带宽 w_c](LADRC_已废除，请勿查看.md#52-控制器带宽-w_c)
   - [5.3 观测器带宽 w_o](LADRC_已废除，请勿查看.md#53-观测器带宽-w_o)
6. [代码实现](LADRC_已废除，请勿查看.md#6-代码实现)
   - [6.1 跟踪微分器](LADRC_已废除，请勿查看.md#61-跟踪微分器)
   - [6.2 扩张状态观测器](LADRC_已废除，请勿查看.md#62-扩张状态观测器)
   - [6.3 控制器集成](LADRC_已废除，请勿查看.md#63-控制器集成)
7. [工程实践指南](LADRC_已废除，请勿查看.md#7-工程实践指南)
   - [7.1 调试步骤](LADRC_已废除，请勿查看.md#71-调试步骤)
   - [7.2 常见问题](LADRC_已废除，请勿查看.md#72-常见问题)
8. [总结](LADRC_已废除，请勿查看.md#8-总结)

---

## 1. 为什么需要 LADRC

原始 ADRC 存在以下问题：
- 非线性函数（fal）参数多，整定困难
- 极点配置需要丰富的工程经验
- 工程推广受限

LADRC 的改进：
- 采用线性 ESO，降低实现难度
- 引入带宽概念，参数整定系统化
- 保持抗扰能力，适应工程需求

---

## 2. 核心概念回顾

### 2.1 积分器串联型

积分器串联型是 LADRC 理论的基础。无论系统是否线性，在一定条件下都可以回归于积分器串联型：

$$y = \int\int...\int u$$

这意味着输出是输入的 n 次积分。

### 2.2 总扰动

总扰动将系统中所有不确定因素打包为一个整体：

$$f(y, \dot{y}, d, t) = -a_1\dot{y} - a_0y + d + \Delta a_1\dot{y} + \Delta a_0y$$

包括：
- 外部扰动 $d(t)$
- 未建模动态
- 参数不确定性

---

## 3. LADRC 数学模型

### 3.1 系统描述

以二阶系统为例：

$$\ddot{y} = f(y, \dot{y}, d, t) + bu$$

其中：
- $y$：系统输出
- $u$：控制输入
- $b$：输入系数
- $f$：总扰动

### 3.2 线性扩张状态观测器（LESO）

将总扰动 $f$ 扩张为状态变量 $x_3$，得到三阶系统：

$$\begin{cases}
\dot{x}_1 = x_2 \\
\dot{x}_2 = x_3 + bu \\
\dot{x}_3 = \dot{f} \\
y = x_1
\end{cases}$$

设计 LESO：

$$\dot{z} = (A - LC)z + (B - Ld)u + Ly$$

极点配置在 $-\omega_o$ 处，增益为：

| 增益 | 值 |
|------|------|
| $l_1$ | $3\omega_o$ |
| $l_2$ | $3\omega_o^2$ |
| $l_3$ | $\omega_o^3$ |

### 3.3 线性误差反馈控制律（LSEF）

采用 PD 控制形式：

$$u_0 = k_p(r - z_1) + k_d(\dot{r} - z_2)$$

其中 $k_p = \omega_c^2$，$k_d = 2\omega_c$。

最终控制律：

$$u = \frac{u_0 - z_3}{b_0}$$

---

## 4. 离散化实现

### 4.1 零阶保持法

将连续系统离散化，零阶保持法效果优于欧拉法：

$$G_h(s) = \frac{1 - e^{-sh}}{s}$$

### 4.2 一阶系统离散化

连续模型：

$$\dot{y} = f + bu$$

离散估计器：

$$z(k+1) = A_c z(k) + B_c u(k) + L_c(y(k) - C z(k))$$

### 4.3 二阶系统离散化

连续模型：

$$\ddot{y} = f + bu$$

离散估计器方程：

$$z_1(k+1) = ...$$
$$z_2(k+1) = ...$$
$$z_3(k+1) = ...$$

---

## 5. 参数整定方法

### 5.1 扰动补偿系数 b₀

**物理意义**: 表示控制对象的特性

**整定方法**:
1. 对控制对象输入幅值为 $U$ 的阶跃信号
2. 计算初始加速度 $a_0$
3. $b_0 = U / a_0$

**注意事项**:
- b₀ 越大，抗干扰能力越弱
- b₀ 不能过大或过小，需微调至系统稳定

### 5.2 控制器带宽 w_c

**作用**: 决定闭环响应速度

**影响**:
- w_c 越大，响应越快
- w_c 过大导致超调或不稳定
- w_c 过大引入更多噪声

**整定建议**: 先设小值，根据系统性能微调

### 5.3 观测器带宽 w_o

**作用**: 决定扰动估计速度

**影响**:
- w_o 越大，估计越快，抗干扰能力越强
- w_o 过大引入高频噪声

**经验公式**: $w_o = (3 \sim 5)w_c$，通常取 $w_o = 4w_c$

---

## 6. 代码实现

### 6.1 跟踪微分器

```c
// 跟踪微分器结构体
typedef struct {
    float r;          // 速度因子
    float h0;         // 滤波因子
    float h;          // 积分步长
    float v1_k1;      // 上一时刻的跟踪输出
    float v2_k1;      // 上一时刻的微分输出
} td_struct_t;

// TD 计算函数
float td_calc(td_struct_t *td, float input) {
    float v1 = td->v1_k1;
    float fh = td->h * td->v2_k1;
    float fv = td->v1_k1 - input + td->v2_k1 / (td->r * td->h0);
    float td_out = td->v1_k1 + td->h * td->v2_k1;

    if (fv > td->h0) {
        td->v2_k1 -= td->h * td->r * 1.0f;
    } else if (fv < -td->h0) {
        td->v2_k1 -= td->h * td->r * (-1.0f);
    } else {
        td->v2_k1 -= td->h * td->r * fv / td->h0;
    }

    td->v1_k1 = v1 + td->h * td->v2_k1;
    return td_out;
}
```

### 6.2 扩张状态观测器

```c
// 扩张状态观测器结构体
typedef struct {
    float b0;         // 扰动补偿增益
    float h;         // 积分步长
    float beta1;     // 观测器增益 1
    float beta2;     // 观测器增益 2
    float beta3;     // 观测器增益 3
    float z1_k1;     // 位置估计值
    float z2_k1;     // 速度估计值
    float z3_k1;     // 总扰动估计值
} lueso_struct_t;

void lueso_init(lueso_struct_t *eso, float init_state) {
    eso->z1_k1 = init_state;
    eso->z2_k1 = 0.0f;
    eso->z3_k1 = 0.0f;
}

void lueso_update(lueso_struct_t *eso, float y_measure, float u_control) {
    float e = eso->z1_k1 - y_measure;
    eso->z1_k1 += eso->h * (eso->z2_k1 - eso->beta1 * e);
    eso->z2_k1 += eso->h * (eso->z3_k1 - eso->beta2 * e + eso->b0 * u_control);
    eso->z3_k1 += eso->h * (-eso->beta3 * e);
}
```

### 6.3 控制器集成

```c
// LADRC 控制器总结构体
typedef struct {
    td_struct_t td;       // 跟踪微分器
    lueso_struct_t eso;   // 扩张状态观测器
    float kp;             // 比例增益
    float kd;             // 微分增益
    float h;              // 控制周期
} ladrc_struct_t;

// 初始化
void ladrc_init(ladrc_struct_t *ctl, float init_state) {
    ctl->td.r = 100.0f;
    ctl->td.h0 = ctl->h * 0.1f;
    ctl->td.h = ctl->h;
    ctl->td.v1_k1 = init_state;
    ctl->td.v2_k1 = 0.0f;

    lueso_init(&ctl->eso, init_state);
    ctl->eso.b0 = 1.0f;
    ctl->eso.h = ctl->h;
    ctl->eso.beta1 = 10.0f;
    ctl->eso.beta2 = 100.0f;
    ctl->eso.beta3 = 1000.0f;

    ctl->kp = 10.0f;
    ctl->kd = 2.0f * sqrtf(ctl->kp);  // kd = 2*sqrt(kp)
}

// 核心计算函数
float ladrc_compute(ladrc_struct_t *ctl, float ref_input, float measure_output) {
    float v1 = td_calc(&ctl->td, ref_input);
    lueso_update(&ctl->eso, measure_output, 0.0f);

    float e1 = v1 - measure_output;
    float e2 = ctl->td.v2_k1 - ctl->eso.z2_k1;

    float u0 = ctl->kp * e1 + ctl->kd * e2;
    float u = (u0 - ctl->eso.z3_k1) / ctl->eso.b0;

    return u;
}
```

---

## 7. 工程实践指南

### 7.1 调试步骤

1. **确定 b₀**: 通过阶跃响应估算
2. **设置 w_c**: 从小到大逐步调整
3. **设置 w_o**: 根据经验公式 $w_o = 4w_c$
4. **微调参数**: 观察系统响应，继续优化

### 7.2 常见问题

| 问题 | 原因 | 解决方法 |
|------|------|----------|
| 振荡 | w_c 或 w_o 过大 | 减小带宽 |
| 跟踪慢 | w_c 过小 | 适当增大 |
| 噪声大 | w_o 过大 | 减小 w_o 或增加滤波 |
| 静差 | b₀ 估计不准 | 重新估算 b₀ |

---

## 8. 总结

LADRC 的核心优势：
- **参数整定简单**: 只需调节三个带宽参数
- **抗扰能力强**: ESO 实时估计总扰动
- **工程友好**: 线性结构，易于实现

---

## 参考资料

- CSDN 原文: https://blog.csdn.net/weixin_41276397/article/details/127353049
- 高志强教授 LADRC 论文
