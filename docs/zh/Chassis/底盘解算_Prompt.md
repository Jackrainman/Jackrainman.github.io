# 底盘解算文档 - AI内容生成Prompt

> 本文档为《底盘解算.md》各章节提供详细的AI内容生成Prompt
> 每个Prompt包含：内容要求、图片描述、图片生成建议

---

## 1. 理论基础

### Prompt 1.1: 坐标系定义

**内容要求：**
详细解释机器人底盘控制中使用的坐标系统，包括：
1. 世界坐标系（全局坐标系）的定义和用途
2. 机器人本体坐标系（局部坐标系）的定义
3. 两个坐标系之间的转换关系（旋转矩阵）
4. 速度分量的定义：Vx（纵向速度）、Vy（横向速度）、ω（角速度）
5. 右手坐标系约定和符号规则

**图片描述：**
一张俯视图，展示：
- 世界坐标系 Xw-Yw（固定不动，通常与场地相关）
- 机器人本体坐标系 Xr-Yr（随机器人移动和旋转）
- 机器人在世界坐标系中的位姿 (x, y, θ)
- 用箭头标注Vx（前进方向）、Vy（左侧方向）、ω（逆时针旋转）
- 两个坐标系之间的夹角θ标注清楚

**图片生成建议：**
- **MATLAB:** 使用plot绘制坐标轴、机器人矩形框、箭头和标注文字，保存为矢量图
- **AI生成:** 提示词 "Top-down view technical diagram of mobile robot coordinate systems, world frame Xw-Yw in blue, robot frame Xr-Yr in red, robot body as rectangle, velocity vectors Vx Vy omega labeled with arrows, clean engineering drawing style, white background, high precision"
- **网图:** 搜索关键词 "mobile robot coordinate system kinematics" "robot frame vs world frame"

---

### Prompt 1.2: 刚体平面运动学

**内容要求：**
1. 平面刚体运动的速度合成定理
2. 机器人上任一点的速度计算公式：V_point = V_center + ω × r
3. 轮子安装位置向量的定义（从机器人中心指向各轮中心）
4. 旋转的叉积运算解释
5. 瞬时旋转中心（ICR）概念简介

**图片1描述（速度分解）：**
机器人俯视图，中心点C标注速度矢量Vx、Vy，从中心向外辐射箭头表示不同位置点的速度方向，显示速度大小与到中心距离的关系

**图片2描述（轮子位置向量）：**
底盘俯视图，标注：
- 机器人几何中心O
- 四个轮子的位置（FL, FR, BL, BR）
- 从O到各轮中心的向量r1, r2, r3, r4
- 底盘的长度和宽度参数（a, b）

**图片生成建议：**
- **MATLAB:**
  - 图1：绘制机器人轮廓、速度矢量场（quiver函数）
  - 图2：绘制底盘矩形、轮子位置、标注线和参数
- **AI生成:**
  - 图1: "Robot velocity field diagram, top view, center velocity vector, radial velocity arrows showing rotation effect, engineering technical drawing"
  - 图2: "Robot chassis top view with four wheel positions labeled, geometric center marked, dimension lines showing wheelbase and track width, technical blueprint style"
- **网图:** "differential drive kinematics vector diagram"

---

### Prompt 1.3: 轮式移动机器人运动学建模基础

**内容要求：**
1. 纯滚动条件（无滑动假设）的数学表达
2. 无侧滑约束条件
3. 轮子速度与机器人速度的基本映射关系
4. 运动学约束方程的建立
5. 为什么要做这些假设，以及实际中的偏差

**图片需求：**
本小节无需单独配图，使用前面章节的图即可

---

## 2. 麦克纳姆轮底盘解算

### Prompt 2.1: 麦轮结构与运动原理

**内容要求：**
1. 麦克纳姆轮的结构组成（轮毂+辊子）
2. 辊子45°倾斜角的原理和作用
3. X型布局与O型布局的区别
4. 麦轮产生横向力的机理
5. 主动轮转动时辊子的被动转动分析
6. 力的分解：一部分力驱动前进，一部分产生侧向移动

**图片1描述（麦轮结构）：**
麦克纳姆轮立体图或剖面图：
- 显示轮毂和多个辊子
- 辊子与轮毂轴成45°角
- 标注辊子旋转轴方向
- 显示X型（辊子呈X状）或O型布局
- 可以展示多个视角（正视图、俯视图）

**图片2描述（单轮速度分解）：**
单个麦轮的力学分析图：
- 轮子整体旋转方向
- 辊子接触地面的接触点
- 速度矢量分解：沿辊子方向和垂直于辊子方向
- 显示有效驱动力方向和滑动方向
- 标注角度45°

**图片生成建议：**
- **MATLAB:** 难以绘制复杂3D结构，建议用2D示意图展示速度分解
- **AI生成:**
  - 图1: "Mecanum wheel 3D exploded view diagram, hub with angled rollers at 45 degrees, technical illustration, labeled components, engineering drawing style"
  - 图2: "Single mecanum wheel velocity decomposition diagram, top view, wheel rotation arrow, roller contact point, force vectors broken into components, 45-degree angle标注"
- **网图:** "mecanum wheel roller angle diagram" "mecanum wheel force analysis"
- **建议：** 麦轮结构图建议找网图或使用CAD软件渲染，AI生成可能不够精确

---

### Prompt 2.2: 逆运动学

**内容要求：**
1. 逆运动学的定义：从机器人目标速度计算各轮所需速度
2. 四轮麦轮底盘的布局参数定义（轮距a，轴距b）
3. 单个轮子速度公式的推导：V_wheel = (Vx ± Vy ± ω·(a+b)) / r
4. 公式的物理意义解释
5. 符号约定（哪个轮子是+，哪个是-）
6. 速度分配矩阵的构建

**图片1描述（四轮布局）：**
麦轮底盘俯视图：
- 矩形底盘轮廓
- 四个轮子位置：FL（前左）、FR（前右）、BL（后左）、BR（后右）
- 每个轮子标注辊子方向（45°倾斜）
- 标注轮距（左右轮间距）和轴距（前后轮间距）
- 显示X型布局的辊子方向

**图片2描述（速度合成向量）：**
向量合成示意图：
- 机器人中心速度V（分解为Vx, Vy）
- 旋转产生的线速度ω×r
- 两者向量相加得到各轮需要提供的速度
- 四个轮子分别的合速度矢量
- 用不同颜色区分不同速度分量

**图片生成建议：**
- **MATLAB:** 非常适合绘制这种几何示意图，使用plot、quiver、annotation
- **AI生成:**
  - 图1: "Mecanum wheel robot chassis top view, four wheels labeled FL FR BL BR, X-shaped roller orientation, wheelbase and track width dimensions, technical diagram"
  - 图2: "Velocity vector synthesis diagram, robot center with Vx Vy arrows, four wheel positions with resultant velocity vectors, vector addition parallelograms, color-coded components"
- **网图:** "mecanum wheel kinematics diagram" "mecanum wheel velocity vectors"

---

### Prompt 2.4: 典型运动模式分析

**内容要求：**
1. 纵向运动（前进/后退）：四轮同向同速
2. 横向运动（左右平移）：对角轮同向，另一对角轮反向
3. 原地旋转（自转）：对角轮同向，另一对角轮反向（与横移不同的组合）
4. 斜向运动：组合运动
5. 每种模式下轮速比例关系
6. 为什么麦轮能实现全向移动

**图片描述：**
表格或示意图形式：
- 4行（四种运动模式）× 4列（四个轮子）
- 每个单元格用箭头表示轮子转向（顺时针/逆时针）和速度大小（箭头长度）
- 或者用+1/-1/0数值表示
- 每行右侧配一个小示意图显示机器人运动方向

**图片生成建议：**
- **MATLAB:** 使用subplot绘制多个小图，每个小图显示一种运动模式下的轮速方向
- **AI生成:** "Mecanum wheel motion modes table, four rows for different movements (forward, strafe, rotate, diagonal), four columns for wheels, arrows showing direction and magnitude, small robot icons showing resulting motion"
- **网图:** "mecanum wheel drive modes" "omnidirectional robot movement patterns"
- **建议：** 此图适合用表格形式，MATLAB或PPT制作更清晰

---

## 3. 全向轮底盘解算

### Prompt 3.1: 全向轮结构特点

**内容要求：**
1. 全向轮的结构（轮毂+垂直辊子）
2. 与麦轮的关键区别（辊子方向不同）
3. 全向轮的运动原理
4. 为什么叫"全向轮"
5. 小辊子的排列方式（单排/双排）

**图片描述：**
全向轮结构图：
- 轮毂主体
- 多个小辊子均匀分布在轮毂外围
- 辊子轴与轮毂轴垂直
- 可以展示侧视图和正视图
- 标注关键部件名称

**图片生成建议：**
- **MATLAB:** 2D示意图即可
- **AI生成:** "Omni wheel technical diagram, hub with perpendicular rollers arranged around circumference, side view and front view, labeled components, clean engineering illustration"
- **网图:** "omni wheel structure" "omnidirectional wheel diagram"
- **建议：** 全向轮结构较简单，网图资源丰富且清晰

---

### Prompt 3.2: 三轮全向底盘（120°布局）

**内容要求：**
1. 120°布局的几何设计原理
2. 为什么选择120°（覆盖360°均匀分布）
3. 逆运动学公式推导：V_wheel_i = Vx·cos(θi) + Vy·sin(θi) + ω·R
4. θi取值的确定（0°, 120°, 240°或等效角度）
5. 3×3速度分配矩阵的构建
6. 正运动学求解（矩阵求逆）

**图片1描述（120°布局）：**
三轮底盘俯视图：
- 等边三角形或圆形容器轮廓
- 三个轮子分别位于0°、120°、240°方向
- 从中心到各轮的连线
- 标注角度和距离参数R
- 轮子方向标注（轮子前进方向与径向的关系）

**图片2描述（速度矢量分解）：**
单轮速度分析：
- 机器人坐标系X-Y
- 轮子安装角度θ
- 机器人速度V分解到轮子方向的分量
- 旋转速度ω产生的线速度
- 两者合成轮子目标速度

**图片3描述（三轮速度合成）：**
三个速度矢量从中心发出：
- 三个箭头分别代表三个轮子的速度方向和大小
- 显示如何合成为机器人的整体运动
- 可以用不同场景（纯前进、纯旋转、复合运动）展示

**图片生成建议：**
- **MATLAB:** 非常适合，使用极坐标或直角坐标绘制
- **AI生成:**
  - 图1: "Three-wheel omnidirectional robot top view, 120-degree symmetrical layout, wheels at 0 120 240 degrees, radial lines from center, equal distance R, technical diagram"
  - 图2: "Velocity projection diagram, wheel mounting angle theta, robot velocity V decomposed into wheel direction components, vector projection arrows"
  - 图3: "Three wheel velocity vectors synthesis, arrows from center showing wheel speeds, resultant robot motion direction, different motion cases"
- **网图:** "3 wheel omni robot kinematics" "120 degree omni wheel layout"

---

### Prompt 3.3: 四轮全向底盘

**内容要求：**
1. 四轮全向底盘的常见布局（矩形四角、菱形、H型）
2. 四轮布局相比三轮的优势（冗余、稳定性、承载能力）
3. 超定方程组问题（4个轮子3个自由度）
4. 最小二乘法求解最佳速度分配
5. 不同布局对运动性能的影响

**图片1描述（四轮布局）：**
对比图或选型图：
- 方案A：矩形四角布局（类似麦轮位置，但轮子方向不同）
- 方案B：菱形布局
- 标注各方案的特点和适用场景

**图片2描述（性能对比）：**
雷达图或条形图：
- 维度：承载能力、稳定性、控制复杂度、成本、机动性
- 三条线分别代表三轮、四轮矩形、四轮菱形

**图片生成建议：**
- **MATLAB:** 使用subplot对比不同布局，使用bar或radar chart做性能对比
- **AI生成:**
  - 图1: "Four wheel omni robot layout comparison, rectangular vs diamond configuration, top view diagrams, pros and cons labels"
  - 图2: "Performance comparison radar chart, three configurations compared across multiple dimensions, technical chart style"
- **网图:** "4 wheel omni robot design" "omni wheel configurations"

---

### Prompt 3.4: 运动特性分析

**内容要求：**
1. 各向同性运动能力的含义
2. 三轮与四轮在不同方向上的速度极限
3. 效率分析（能量传递效率）
4. 稳定性考虑（重心位置、支撑多边形）
5. 全向轮相比麦轮的优缺点对比

**图片需求：**
本小节可使用前面章节的图片进行对比说明，无需新图

---

## 4. 舵轮底盘解算

### Prompt 4.1: 舵轮结构与原理

**内容要求：**
1. 舵轮模块的组成（转向电机+驱动电机+编码器）
2. 双自由度设计（转向角度+驱动速度）
3. 机械结构详解（转向机构、悬挂、轮子）
4. 编码器系统（转向角度编码器+轮速编码器）
5. 转向-驱动解耦控制的意义

**图片1描述（舵轮模块结构）：**
舵轮模块爆炸图或剖面图：
- 整体外观（轮子+转向机构外壳）
- 爆炸展示内部组件：
  * 转向电机（或转向轴）
  * 驱动电机
  * 减速机构
  * 轮子本体
  * 转向角度传感器位置
- 标注各部件名称

**图片2描述（舵轮速度矢量）：**
单舵轮分析：
- 轮子中心点
- 转向角度α（相对于机器人前进方向）
- 轮子速度V（沿轮子朝向）
- 分解为Vx和Vy分量
- 显示转向后的新方向

**图片生成建议：**
- **MATLAB:** 2D示意图展示速度矢量
- **AI生成:**
  - 图1: "Swerve drive module exploded view, steering motor, drive motor, wheel, gear mechanism, encoder positions, technical cutaway diagram"
  - 图2: "Swerve wheel velocity vector diagram, steering angle alpha, wheel velocity V, decomposed into Vx and Vy components, angle标注"
- **网图:** "swerve drive module CAD" "swerve drive mechanism" "swerve module exploded view"
- **建议：** 舵轮结构较复杂，建议使用CAD渲染图或高质量网图

---

### Prompt 4.2: 逆运动学算法

**内容要求：**
1. 从机器人目标速度到各舵轮需求的映射流程
2. 目标速度分解到每个舵轮：V_i = V_robot + ω × r_i
3. 转向角度计算：α_i = atan2(V_i_y, V_i_x)
4. 轮速计算：|V_wheel_i| = sqrt(V_i_x² + V_i_y²)
5. 最优转角选择策略（最短路径原则）
6. 反向旋转方案（当最优转角>90°时）
7. 速度归一化处理（当某轮速度超过上限时）

**图片1描述（速度映射流程）：**
流程图：
- 输入：目标Vx, Vy, ω
- 模块1：计算各轮需求速度V_i
- 模块2：计算转向角度α_i
- 模块3：转角优化（选择最短路径）
- 模块4：速度归一化
- 输出：四个转向角度+四个轮速
- 显示数据流向和判断节点

**图片2描述（转角优化）：**
角度选择示意图：
- 当前轮子朝向（实线箭头）
- 目标朝向（虚线箭头）
- 显示顺时针转和逆时针转两条路径
- 标注转角大小
- 展示选择最短路径的原则
- 展示反向方案（轮子倒转+转向180°）

**图片3描述（底盘几何参数）：**
四舵轮底盘俯视图：
- 底盘矩形轮廓
- 四个舵轮位置（通常四个角）
- 标注：
  * 轮距（track width）
  * 轴距（wheelbase）
  * 从中心到各轮的向量r1, r2, r3, r4
  * 各轮的安装角度基准

**图片生成建议：**
- **MATLAB:** 非常适合流程图和几何图
- **AI生成:**
  - 图1: "Swerve drive inverse kinematics flowchart, input Vx Vy omega, calculation modules, angle optimization decision, normalization, outputs, clean flow diagram"
  - 图2: "Steering angle optimization diagram, current wheel direction, target direction, two rotation paths, shortest path selection, reverse option标注"
  - 图3: "Four swerve module chassis top view, wheel positions at corners, wheelbase and track width标注, vectors from center to wheels, coordinate system"
- **网图:** "swerve drive kinematics" "swerve module geometry"

---

### Prompt 4.3: 正运动学

**内容要求：**
1. 从编码器数据（转向角+轮速）反解机器人速度
2. 速度矢量的提取：V_i = V_wheel_i · [cos(α_i), sin(α_i)]
3. 多轮速度融合的必要性
4. 最小二乘法求解超定方程组
5. 协方差加权融合（考虑各轮可靠性差异）
6. 实际实现中的数值稳定性考虑

**图片描述（速度融合）：**
数据融合示意图：
- 四个输入框（各轮的V_i矢量）
- 融合算法模块（标注"最小二乘融合"）
- 输出：机器人速度Vx, Vy, ω
- 可以加权重输入（表示各轮可靠性）
- 显示噪声/误差的概念

**图片生成建议：**
- **MATLAB:** 流程图或框图形式
- **AI生成:** "Sensor fusion diagram for swerve drive, four wheel velocity inputs, least squares fusion block, weighted inputs, robot velocity output, error estimation"
- **网图:** "odometry fusion" "multi-sensor fusion mobile robot"

---

### Prompt 4.4: 协调控制策略

**内容要求：**
1. 为什么需要协调控制（各轮转向不同步的问题）
2. 同步转向控制策略
3. 转向优先级分配（根据速度大小）
4. 零半径旋转模式的特殊处理
5. 防止轮子之间相互干涉
6. 平滑过渡控制（避免突然变向）

**图片1描述（矢量协调）：**
四舵轮协调运动展示：
- 底盘俯视图
- 四个轮子各自的朝向（箭头表示）
- 显示转向过程中的中间状态
- 标注转向完成度或同步状态
- 可以展示不同运动模式（前进、旋转、曲线运动）

**图片生成建议：**
- **MATLAB:** 动画帧或关键帧展示
- **AI生成:** "Swerve drive coordinated motion diagram, four wheels with steering arrows, synchronization state, intermediate steering positions, different motion modes"
- **网图:** "swerve drive coordination" "swerve module synchronization"

---

## 5. 控制实现与优化

### Prompt 5.1: 软件架构设计

**内容要求：**
1. 分层控制架构的优势
2. 上层：指令输入层（遥控、导航规划、上层算法）
3. 中层：运动学解算层（本文档核心内容）
4. 下层：电机控制层（PID、FOC等）
5. 各层之间的接口定义
6. 实时性要求和通信方式

**图片描述（软件架构）：**
三层架构图：
- 顶层：应用层（遥控/导航）
- 中间层：运动学解算（逆运动学/正运动学）
- 底层：电机控制（PID控制器×8，对应4个转向+4个驱动）
- 显示层间数据流（箭头）
- 显示反馈回路（编码器数据向上传递）
- 标注通信接口（CAN bus / Ethernet / UART等）

**图片生成建议：**
- **MATLAB:** Simulink框图或使用rectangle+arrow绘制
- **AI生成:** "Robot chassis control software architecture diagram, three layers (application, kinematics, motor control), data flow arrows, feedback loops, communication interfaces labeled, clean system diagram"
- **网图:** "mobile robot control architecture" "chassis control system diagram"
- **建议：** 架构图建议使用Visio/Draw.io手动绘制更清晰

---

### Prompt 5.2: 运动控制算法

**内容要求：**
1. PID控制基础回顾
2. 速度环PID设计
3. 各轮独立PID vs 整体协调PID的对比
4. 前馈控制的作用
5. 摩擦力补偿方法
6. 加速度前馈设计
7. 参数整定建议

**图片描述（PID控制框图）：**
控制系统框图：
- 输入：目标速度
- 比较器：计算误差e
- PID控制器块：Kp, Ki, Kd
- 被控对象：电机+轮子
- 输出：实际速度
- 反馈回路：编码器测量
- 可选：前馈支路（虚线表示）

**图片生成建议：**
- **MATLAB:** 使用Simulink或手绘风格框图
- **AI生成:** "PID control block diagram for wheel speed control, reference input, error calculation, PID controller with Kp Ki Kd, plant block, feedback loop with encoder, feedforward path, clean control system diagram"
- **网图:** "PID velocity control diagram" "motor speed control block diagram"

---

### Prompt 5.3: 误差处理与鲁棒性

**内容要求：**
1. 机械误差来源分析
2. 轮子半径误差的影响与校准方法
3. 安装角度偏差的影响与补偿
4. 编码器分辨率限制
5. 打滑检测方法（速度不一致性检测）
6. 打滑恢复策略

**图片需求：**
本小节主要为文字描述，可用前面章节的示意图辅助说明

---

### Prompt 5.4: 参数整定与测试

**内容要求：**
1. 几何参数测量方法（轮距、轴距精确测量）
2. 轮子半径校准方法
3. 转向零点校准
4. 性能测试指标定义
5. 定位精度测试方法
6. 轨迹跟踪测试
7. 旋转中心稳定性测试

**图片描述（可选）：**
测试场景示意图：
- 测试场地标注（坐标点、轨迹线）
- 机器人实际轨迹vs期望轨迹对比
- 误差标注
- 或者用表格形式展示测试数据

**图片生成建议：**
- **MATLAB:** 使用plot绘制期望轨迹和实际轨迹对比
- **AI生成:** "Robot testing scenario diagram, test field with coordinate grid, desired trajectory dashed line, actual trajectory solid line, error measurements标注"
- **网图：** 如有实际测试数据，建议直接生成

---

## 附录图片清单与生成建议汇总

| 章节 | 图片编号 | 内容 | 推荐生成方式 | 难度 |
|------|----------|------|-------------|------|
| 1.1 | 图1-1 | 坐标系定义 | MATLAB / AI | 低 |
| 1.2 | 图1-2 | 速度分解 | MATLAB | 低 |
| 1.2 | 图1-3 | 轮子位置向量 | MATLAB | 低 |
| 2.1 | 图2-1 | 麦轮结构 | 网图 / CAD | 中 |
| 2.1 | 图2-2 | 单轮速度分解 | MATLAB / AI | 中 |
| 2.2 | 图2-3 | 四轮布局 | MATLAB | 低 |
| 2.2 | 图2-4 | 速度合成向量 | MATLAB | 中 |
| 2.4 | 图2-5 | 运动模式对比 | MATLAB / PPT | 低 |
| 3.1 | 图3-1 | 全向轮结构 | 网图 | 低 |
| 3.2 | 图3-2 | 120°三轮布局 | MATLAB | 低 |
| 3.2 | 图3-3 | 速度矢量分解 | MATLAB | 中 |
| 3.2 | 图3-4 | 速度合成 | MATLAB | 中 |
| 3.3 | 图3-5 | 四轮布局方案 | MATLAB | 低 |
| 4.1 | 图4-1 | 舵轮模块结构 | 网图 / CAD | 高 |
| 4.1 | 图4-2 | 舵轮速度矢量 | MATLAB / AI | 中 |
| 4.2 | 图4-3 | 速度映射流程 | Draw.io / AI | 低 |
| 4.2 | 图4-4 | 转角优化 | MATLAB | 中 |
| 4.2 | 图4-5 | 底盘几何参数 | MATLAB | 低 |
| 4.3 | 图4-6 | 速度融合 | MATLAB / AI | 低 |
| 4.4 | 图4-7 | 矢量协调 | MATLAB | 中 |
| 5.1 | 图5-1 | 软件架构 | Draw.io | 低 |
| 5.2 | 图5-2 | PID控制框图 | MATLAB / AI | 低 |

---

## 图片生成工具推荐

### MATLAB
**适用场景：** 所有几何示意图、向量图、流程图
**优势：** 精度高、可编程、可导出矢量图
**代码示例：**
```matlab
% 绘制坐标系示意图
figure;
plot([0 2], [0 0], 'b-', 'LineWidth', 2); hold on;
plot([0 0], [0 2], 'b-', 'LineWidth', 2);
quiver(0, 0, 1, 0.5, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
axis equal; grid on;
title('坐标系定义');
```

### AI图片生成（如Midjourney/DALL-E）
**适用场景：** 概念图、3D效果图、装饰性插图
**提示词技巧：**
- 加上 "technical diagram" "engineering drawing" "schematic"
- 指定风格 "clean white background" "vector illustration"
- 避免生成带文字的图（AI文字通常不准确）

### 网图搜索
**推荐来源：**
- Google Scholar / ResearchGate（学术论文配图）
- RoboMaster/RoboCup官网（竞赛机器人文档）
- GitHub开源项目文档
- 维基百科（英文版质量更高）

**搜索关键词：**
- "mecanum wheel kinematics"
- "omni wheel robot kinematic model"
- "swerve drive kinematics"
- "mobile robot coordinate systems"
- "differential drive odometry"

---

## 使用说明

1. **复制对应章节的Prompt** 发送给AI助手
2. **指定图片生成方式**（根据上表推荐）
3. **要求AI提供图片的文字描述**，方便后续配图
4. **代码示例要求** 使用MATLAB生成
5. **检查生成的内容** 是否符合技术准确性

---

*Prompt文档版本：v1.0*
*配套文档：《底盘解算.md》*
