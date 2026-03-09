---
title: "Jackrainman"
hide:
  - navigation
  - toc
comments: true
---

<div class="landing-hero">
  <p class="landing-eyebrow">Jackrainman / Robotics Control Notes</p>
  <h1>Jackrainman 的机器人电控与控制系统文档</h1>
  <p class="landing-lead">
    记录我在机器人电控方向的控制理论、速度规划、底盘解算、CAN 通信与嵌入式工程实践。
    首页只展示精选内容，其余资料保留在完整文档和搜索中。
  </p>
  <div class="landing-chips">
    <span class="landing-chip">Control Theory</span>
    <span class="landing-chip">Speed Planning</span>
    <span class="landing-chip">Chassis Kinematics</span>
    <span class="landing-chip">STM32 / FreeRTOS</span>
  </div>
</div>

[开始阅读](ControlTheory/00-学习路径指南.md){ .md-button .md-button--primary }
[查看 About](About/index.md){ .md-button }

## 精选专题

<div class="grid cards" markdown>

-   :material-school-outline: __控制理论__

    ---

    从分层认知、PID 到 LADRC 的完整主线，适合第一次进入本站的人。

    [学习路径指南](ControlTheory/00-学习路径指南.md)

    [LADRC 战队实战指南](ControlTheory/08-LADRC战队实战指南.md)

-   :material-axis-arrow: __速度规划与底盘__

    ---

    聚焦底盘运动学、平滑控制和速度规划问题，偏工程落地。

    [底盘解算](Chassis/底盘解算.md)

    [速度规划-理论](SpeedPlan/速度规划-理论.md)

-   :material-lan: __CAN 与电机通信__

    ---

    汇总总线通信、电机驱动封装和接口设计相关内容。

    [CAN 总线基础](CAN/CAN总线基础.md)

    [电机驱动代码详细讲解](Motor/电机驱动代码详细讲解.md)

-   :material-chip: __STM32 / FreeRTOS 实践__

    ---

    以 STM32 工程为载体，整理外设、任务调度和系统组织方式。

    [STM32F1 冬季作业学习文档](单片机/学习文档.md)

    [FreeRTOS 调度](FreeRTOS/01_调度.md)

</div>

## 推荐阅读

<div class="grid cards" markdown>

-   __控制理论学习路径指南__

    先建立整体认知，再决定该读 PID、LADRC 还是别的方案。

    [阅读文章](ControlTheory/00-学习路径指南.md)

-   __LADRC 战队实战指南__

    面向战队开发场景，强调调参顺序与工程上的可落地性。

    [阅读文章](ControlTheory/08-LADRC战队实战指南.md)

-   __速度规划-理论__

    梳理速度规划问题该如何建模、平滑和实现。

    [阅读文章](SpeedPlan/速度规划-理论.md)

-   __底盘解算__

    适合作为底盘运动学和坐标系转换的直接入口。

    [阅读文章](Chassis/底盘解算.md)

-   __CAN 总线基础__

    用于快速建立总线通信认知，再进入具体报文与代码实现。

    [阅读文章](CAN/CAN总线基础.md)

-   __电机驱动代码详细讲解__

    从工程代码角度看电机接口、驱动封装和调用组织。

    [阅读文章](Motor/电机驱动代码详细讲解.md)

</div>

> 站点还保留培养方案、任务资料、文档规范与历史归档；这些内容不在首页推荐，但仍可通过导航和搜索访问。

