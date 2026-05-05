---
title: "首页"
hide:
  - navigation
  - toc
comments: false
---

<div class="landing-hero" markdown>

<span class="landing-eyebrow">// Jackrainman · Robotics Control Notes</span>

<h1 class="landing-title">机器人电控与<br/>控制系统工程文档</h1>

<p class="landing-lead">
记录我在机器人电控方向的控制理论探索、底盘运动学解算、CAN 通信机制与嵌入式 RTOS 工程实践。<br/>
<em>化繁为简，连接理论与落地。</em>
</p>

<div class="landing-chips">
  <span class="landing-chip">Control Theory</span>
  <span class="landing-chip">Speed Planning</span>
  <span class="landing-chip">Chassis Kinematics</span>
  <span class="landing-chip">STM32 &amp; FreeRTOS</span>
  <span class="landing-chip">CAN Bus</span>
  <span class="landing-chip">LADRC</span>
</div>

<div class="landing-actions">
  <a href="zh/ControlTheory/00-%E5%AD%A6%E4%B9%A0%E8%B7%AF%E5%BE%84%E6%8C%87%E5%8D%97/" class="md-button md-button--primary landing-cta">开始阅读指引 →</a>
  <a href="zh/About/" class="md-button landing-cta">关于作者</a>
</div>

</div>

<div class="landing-stats" markdown>
  <div class="landing-stat">
    <span class="landing-stat-value">5</span>
    <span class="landing-stat-label">核心模块</span>
  </div>
  <div class="landing-stat">
    <span class="landing-stat-value">40+</span>
    <span class="landing-stat-label">专题文档</span>
  </div>
  <div class="landing-stat">
    <span class="landing-stat-value">3</span>
    <span class="landing-stat-label">底盘类型解算</span>
  </div>
  <div class="landing-stat">
    <span class="landing-stat-value">∞</span>
    <span class="landing-stat-label">持续迭代</span>
  </div>
</div>

<div class="landing-section-label">
  <span class="landing-section-index">/* 01 */</span>
  <span class="landing-section-name">Core Stack</span>
</div>

## 核心知识体系

构建机器人底层控制的基石，建议按顺序或按需查阅。

<div class="grid cards" markdown>

-   :material-chip: __底层与系统__ · STM32 & RTOS

    ---

    以 STM32 工程为载体，整理外设驱动、FreeRTOS 任务调度和系统组织方式。

    [:material-arrow-right: STM32F1 学习文档](zh/单片机/学习文档.md) ·
    [:material-arrow-right: FreeRTOS 调度](zh/FreeRTOS/01_调度.md)

-   :material-lan-connect: __总线与驱动__ · CAN & Motor

    ---

    汇总底层 CAN 总线通信原理、电机驱动封装规范以及接口层的设计思路。

    [:material-arrow-right: CAN 总线基础](zh/CAN/CAN总线基础.md) ·
    [:material-arrow-right: 电机驱动接口详解](zh/Motor/电机驱动代码详细讲解.md)

-   :material-axis-arrow: __运动与解算__ · Chassis & Kinematics

    ---

    聚焦各类底盘的运动学解算模型、坐标系转换以及从期望速度到轮速的映射。

    [:material-arrow-right: 底盘解算入口](zh/Chassis/底盘解算.md)

-   :material-school-outline: __算法与控制__ · Control Theory

    ---

    从系统的分层认知、基础 PID 调节到自抗扰控制（ADRC）的完整主线脉络。

    [:material-arrow-right: 学习路径指南](zh/ControlTheory/00-学习路径指南.md)

</div>

<div class="landing-section-label">
  <span class="landing-section-index">/* 02 */</span>
  <span class="landing-section-name">Deep Dives</span>
</div>

## 实战与进阶指南

面向战队开发场景和具体工程痛点的深度专题。

<div class="grid cards" markdown>

-   :material-target: __LADRC 战队实战指南__

    ---

    突破传统 PID 的局限，面向战队实际开发场景，强调 LADRC 的调参顺序、坑点与工程上的可落地性。

    [:material-arrow-right-bold-circle: 立即阅读](zh/ControlTheory/08-LADRC实战指南.md)

-   :material-chart-bell-curve-cumulative: __速度规划与平滑理论__

    ---

    解决机器人运动过程中的突变与抖动问题。梳理速度规划的数学建模、加减速曲线平滑策略及代码实现。

    [:material-arrow-right-bold-circle: 立即阅读](zh/SpeedPlan/速度规划-理论.md)

</div>

<div class="landing-tip" markdown>
:material-lightbulb-on-outline: **提示** &nbsp;·&nbsp; 站点还保留了代码规范与历史归档等内容。这些内容不在首页展示，可通过左侧导航栏和顶部搜索框进行访问。
</div>
