---
title: "首页"
hide:
  - navigation
  - toc
comments: false
---

<div class="landing-hero" style="text-align: center; padding: 3rem 1rem;">
  <p class="landing-eyebrow" style="color: var(--md-typeset-color); opacity: 0.7; font-weight: bold; text-transform: uppercase; letter-spacing: 0.1em;">Jackrainman / Robotics Control Notes</p>
  <h1 style="font-size: 2.5em; font-weight: 800; margin-bottom: 0.5em;">机器人电控与控制系统工程文档</h1>
  <p class="landing-lead" style="max-width: 42rem; margin: 0 auto 2rem; font-size: 1.2em; color: var(--md-typeset-color); opacity: 0.8; line-height: 1.6;">
    记录我在机器人电控方向的控制理论探索、底盘运动学解算、CAN 通信机制与嵌入式 RTOS 工程实践。化繁为简，连接理论与落地。
  </p>
  <div class="landing-chips" style="display: flex; justify-content: center; gap: 0.5rem; flex-wrap: wrap; margin-bottom: 2rem;">
    <span class="landing-chip" style="background: var(--md-default-bg-color); border: 1px solid var(--md-default-fg-color--lighter); padding: 0.3em 0.8em; border-radius: 20px; font-size: 0.9em;">Control Theory</span>
    <span class="landing-chip" style="background: var(--md-default-bg-color); border: 1px solid var(--md-default-fg-color--lighter); padding: 0.3em 0.8em; border-radius: 20px; font-size: 0.9em;">Speed Planning</span>
    <span class="landing-chip" style="background: var(--md-default-bg-color); border: 1px solid var(--md-default-fg-color--lighter); padding: 0.3em 0.8em; border-radius: 20px; font-size: 0.9em;">Chassis Kinematics</span>
    <span class="landing-chip" style="background: var(--md-default-bg-color); border: 1px solid var(--md-default-fg-color--lighter); padding: 0.3em 0.8em; border-radius: 20px; font-size: 0.9em;">STM32 & FreeRTOS</span>
  </div>

  <a href="zh/ControlTheory/00-学习路径指南.md" class="md-button md-button--primary" style="margin: 0.5rem;">开始阅读指引</a>
  <a href="zh/About/index.md" class="md-button" style="margin: 0.5rem;">关于作者</a>
</div>

## 核心知识体系

构建机器人底层控制的基石，建议按顺序或按需查阅。

<div class="grid cards" markdown>

-   :material-chip: __底层与系统 | STM32 & RTOS__

    ---

    以 STM32 工程为载体，整理外设驱动、FreeRTOS 任务调度和系统组织方式。

    [STM32F1 学习文档](zh/单片机/学习文档.md) · [FreeRTOS 调度](zh/FreeRTOS/01_调度.md)

-   :material-lan: __总线与驱动 | CAN & Motor__

    ---

    汇总底层 CAN 总线通信原理、电机驱动封装规范以及接口层的设计思路。

    [CAN 总线基础](zh/CAN/CAN总线基础.md) · [电机驱动接口详解](zh/Motor/电机驱动代码详细讲解.md)

-   :material-axis-arrow: __运动与解算 | Chassis & Kinematics__

    ---

    聚焦各类底盘的运动学解算模型、坐标系转换以及从期望速度到轮速的映射。

    [底盘解算入口](zh/Chassis/底盘解算.md)

-   :material-school-outline: __算法与控制 | Control Theory__

    ---

    从系统的分层认知、基础 PID 调节到自抗扰控制（ADRC）的完整主线脉络。

    [学习路径指南](zh/ControlTheory/00-学习路径指南.md)

</div>

## 实战与进阶指南

面向战队开发场景和具体工程痛点的深度专题。

<div class="grid cards" markdown>

-   :material-target: __LADRC 战队实战指南__

    ---

    突破传统 PID 的局限，面向战队实际开发场景，强调 LADRC 的调参顺序、坑点与工程上的可落地性。

    [立即阅读](zh/ControlTheory/08-LADRC战队实战指南.md)

-   :material-chart-bell-curve-cumulative: __速度规划与平滑理论__

    ---

    解决机器人运动过程中的突变与抖动问题。梳理速度规划的数学建模、加减速曲线平滑策略及代码实现。

    [立即阅读](zh/SpeedPlan/速度规划-理论.md)

</div>

> 💡 **提示**：站点还保留了代码规范与历史归档等内容。这些内容不在首页展示，可通过左侧导航栏和顶部搜索框进行访问。
