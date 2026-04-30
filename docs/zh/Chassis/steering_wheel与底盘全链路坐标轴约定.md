# steering_wheel 与底盘全链路坐标轴约定

## 文档目的

本文只解释当前 `G4_2026/` 主线里，底盘从上层速度输入到 `steering_wheel` 四轮解算，再到舵向/驱动电机执行的整条链路中：

- 每一层到底在用什么坐标轴
- `speed_x / speed_y / speed_yaw` 分别代表什么
- 为什么 `steering_wheel.c` 里写着“角度相关都为逆时针为正”，但读代码时又会觉得方向很怪
- 为什么 1 号轮纯旋转时，代码里分到的是 `(+x, -y)`，而不是直觉里的 `(+x, +y)`
- 哪些负号是正常的坐标变换，哪些负号是安装/历史约定带来的适配层

本文不讨论底盘是否已经达到最终统一坐标约定，只讨论当前代码实际怎么跑。

## 适用范围

只针对当前默认主线：

- `G4_2026/`

重点对应文件：

- `G4_2026/User/Application/Src/chassis.c`
- `G4_2026/User/Application/Src/main_ctrl.c`
- `G4_2026/User/Application/Src/remote_link.c`
- `G4_2026/User/Modules/steering_wheel/steering_wheel.c`
- `G4_2026/User/Utils/chassis_calculations/chassis_calculations.c`

## 一句话总览

当前底盘链路里，至少同时存在下面三套“方向定义”：

1. 上层速度输入坐标
2. `steering_wheel` 内部的 `mix_x / mix_y` 分量坐标
3. `wheel_angle` 的角度编码坐标

它们不是完全同一套，因此会出现下面这种阅读体验：

- 平动速度看起来还算正常
- 纯旋转分解时正负号很怪
- 角度零点又不像常见的 `+X`
- 代码能正常跑，但读起来非常绕

这不是单一公式的问题，而是多层坐标约定叠在一起的结果。

## 全链路概览

底盘控制主链路如下：

```text
遥控/自动输入
    |
    v
chassis_ctrl_task()
    |
    | 生成底盘三参速度
    | speed_x, speed_y, speed_yaw
    v
chassis_set_speed()
    |
    v
chassis_execute_task()
    |
    | 把 speed_yaw 从角速度换成轮组圆周切向线速度
    | speedw = speed_yaw * CHASSIS_RADIUS_MM
    v
steering_wheel_ctrl(speedx, speedy, speedw)
    |
    | 做四轮向量分解
    | 得到每个轮子的:
    | - set_wheel_angle[i]
    | - set_wheel_speed[i]
    v
direction_motor_ctrl() / speed_motor_ctrl()
    |
    v
舵向电机 + 驱动电机执行
```

## 上层输入坐标

### 手动模式输入

手动模式下，`chassis_ctrl_task()` 直接从遥控摇杆生成目标速度：

- `rs[0] -> target_x`
- `rs[1] -> target_y`
- `rs[2] -> target_yaw`

关键逻辑：

```c
float target_x = joystick_normalize(g_remote_ctrl_data.rs[0]) * CHASSIS_MAX_SPEED_XY;
float target_y = joystick_normalize(g_remote_ctrl_data.rs[1]) * CHASSIS_MAX_SPEED_XY;
float target_yaw = joystick_normalize(g_remote_ctrl_data.rs[2]) * YAW_MAX_SPEED;
```

这里先按上层语义理解成：

- `speed_x`：底盘平动 x 速度
- `speed_y`：底盘平动 y 速度
- `speed_yaw`：底盘 yaw 方向角速度

此时先不要急着把它和 `steering_wheel` 内部坐标完全等同。

### 自动模式输入

自动模式下，上层先把极坐标速度拆成笛卡尔坐标：

```c
float vx = g_nuc_ctrl_data.v * cosf(g_nuc_ctrl_data.yaw);
float vy = g_nuc_ctrl_data.v * sinf(g_nuc_ctrl_data.yaw);
chassis_set_speed(vx, vy, g_nuc_ctrl_data.vw);
```

这里的语义仍然是：

- `vx`：平动 x 分量
- `vy`：平动 y 分量
- `vw`：角速度 `rad/s`

## chassis 到 steering_wheel 的接口语义

`chassis_execute_task()` 调用 `steering_wheel_ctrl()` 时，做了一件很关键的事：

```c
steering_wheel_ctrl(chassis_sub.speed_x,
                    chassis_sub.speed_y,
                    chassis_sub.speed_yaw * CHASSIS_RADIUS_MM);
```

这说明：

- `speed_x`、`speed_y`：仍是平动线速度
- `speed_yaw`：原本是角速度 `rad/s`
- 进入 `steering_wheel_ctrl()` 之前，已经先乘了底盘半径
- 所以 `steering_wheel_ctrl()` 里的第三个参数 `speedw`，本质上已经不是角速度，而是纯自旋时轮组圆周上的切向线速度

这一点必须先记住，否则后面看四轮分解会混乱。

## steering_wheel 内部到底在做什么

`steering_wheel_ctrl(speedx, speedy, speedw)` 的核心目标不是直接算角度，而是先给每个轮子算一个二维速度向量：

```text
wheel_i = (mix_x[i], mix_y[i])
```

然后：

- 向量长度 = 该轮应该跑多快
- 向量方向 = 该轮应该转向哪里

所以它本质上是在做：

```text
整车速度 = 平动 + 自旋
每个轮子的局部合速度 = 平动分量 + 该轮自旋切向分量
```

## steering_wheel 内部的坐标怪在哪里

### `world_angle` 的负号

代码里有：

```c
float steering_select_yaw_angle = DEG2RAD(-(*world_angle));
```

这一步本身并不奇怪。

如果要把“世界系速度”转成“车体系速度”，本来就应使用逆变换，也就是用 `-yaw` 去旋转。

因此：

- 这个负号本身是正常的坐标变换负号
- 它不是主要的混乱来源

### 真正更容易引起混乱的是这句

```c
Vx = speedx;
Vy = -speedy;
```

这句意味着：

```text
steering_wheel 内部使用的 Y 方向
与上层 speed_y 的正方向相反
```

也就是说，如果脑中默认：

```text
物理俯视图:
x 向右为正
y 向前/向上为正
```

那么 `steering_wheel` 内部的 `mix_x / mix_y` 更像是在用：

```text
内部 mix 坐标:
x 向右为正
y 向下为正
```

所以：

```text
物理上的“右上”
到了 steering_wheel 的内部 mix 坐标里
会变成“右 + 负Y”
```

这就是后面看纯旋转分量时最容易困惑的根源。

## 四轮编号与轮位

当前四轮编号是：

```text
            前
      (1) -------- (2)
       |            |
       |            |
       |            |
      (3) -------- (4)
            后
```

即：

- 1：左前
- 2：右前
- 3：左后
- 4：右后

## 四轮纯自旋分量是怎么来的

### 代码中的分解式

四轮纯自旋分量写成：

```c
speed_wx[0] = speedw * cos(PI / 4);
speed_wx[1] = speedw * cos(PI / 4);
speed_wx[2] = speedw * -cos(PI / 4);
speed_wx[3] = speedw * -cos(PI / 4);

speed_wy[0] = speedw * -sin(PI / 4);
speed_wy[1] = speedw * sin(PI / 4);
speed_wy[2] = speedw * -sin(PI / 4);
speed_wy[3] = speedw * sin(PI / 4);
```

记：

```text
k = speedw / sqrt(2)
```

则四轮自旋分量就是：

- 1 号轮：`(+k, -k)`
- 2 号轮：`(+k, +k)`
- 3 号轮：`(-k, -k)`
- 4 号轮：`(-k, +k)`

### 为什么这看起来和直觉不一样

如果用物理俯视图理解“左前轮顺时针转应该右上”，会期望看到：

```text
1号轮 = (+, +)
```

但代码里 1 号轮纯正 `speedw` 得到的是：

```text
1号轮 = (+, -)
```

这不是在说“1 号轮物理上不往右上”，而是在说：

```text
代码内部的 mix_y 正方向
和物理直觉里的向上方向相反
```

也就是：

- 物理俯视图“右上”
- 在当前 `mix_x / mix_y` 坐标里
- 会写成 `(+x, -y)`

### 这说明了什么

它说明当前 `steering_wheel.c` 中：

- 自旋分量的正负号定义
- 并不是直接照搬“标准数学俯视图”
- 而是已经掺进了内部安装/方向适配后的结果

所以看公式时，不能直接把 `mix_x / mix_y` 当成物理直角坐标分量。

## 平动加自旋后的四轮合速度公式

在当前主线常见情况里，`world_angle` 大多为 `0`，先只看这一种情形。

此时：

```text
Vx = speedx
Vy = -speedy
k = speedw / sqrt(2)
```

四个轮子的合速度分量可以直接写成：

- 1号轮：`mix = (Vx + k, Vy - k)`
- 2号轮：`mix = (Vx + k, Vy + k)`
- 3号轮：`mix = (Vx - k, Vy - k)`
- 4号轮：`mix = (Vx - k, Vy + k)`

这组公式不是标准物理俯视图的直接表达，而是：

- 上层速度先进入 `steering_wheel` 内部坐标
- `Y` 轴已做一次翻转
- 再叠加当前代码约定下的自旋切向分量

## 轮速是怎么从合速度向量算出来的

每个轮子的速度大小：

```c
wheel_speed[i] = sqrt(pow(mix_x[i], 2) + pow(mix_y[i], 2));
```

这一步没有争议，就是二维向量长度：

```text
轮速 = sqrt(mix_x^2 + mix_y^2)
```

## 轮角是怎么从合速度向量算出来的

代码写法：

```c
wheel_angle[i] = mix_x[i] < 0
                     ? acos(mix_y[i] / wheel_speed[i])
                     : -acos(mix_y[i] / wheel_speed[i]);
```

这段不直观，但它表达的角度空间大致是：

```text
          +Y
           ^
           | 0
           |
 +PI/2 <---+---> -PI/2
           |
           |
           v
         -Y  (±PI)
```

也就是说：

- `0` 角在 `+Y`
- 不是常见的 `+X`
- 角度按逆时针为正
- 角度范围映射到 `[-PI, PI]`

所以这里又和很多人的直觉不同：

- 很多人习惯 `atan2(y, x)`，即 `+X = 0`
- 当前代码这套角度编码更接近“以 `+Y` 为零度”的空间

这就是为什么会让人感觉：

- 分量像是某根轴翻了
- 角度又像零点转了 90 度

## 最短转角为什么又会再改一次符号

即使已经算出几何目标角 `wheel_angle[i]`，也不会直接把它原样发给舵向电机。

`steering_wheel_single_ctrl()` 会比较两种方案：

1. 直接转到 `theta`
2. 转到 `theta ± PI`，同时把轮速取反

原因是：

```text
一个轮子的几何速度向量
既可以通过“角度 theta + 正轮速”实现
也可以通过“角度 theta+PI + 负轮速”实现
```

地面效果相同，但机械转角更小的那种更优。

因此最终输出的：

- `set_wheel_angle`
- `set_wheel_speed`

不一定和中间求得的：

- `wheel_angle`
- `wheel_speed`

同号、同向。

这又是实际调试时“看起来方向怪”的另一来源。

## 当前代码里哪些负号是正常的，哪些是适配型负号

### 正常的坐标变换负号

```c
steering_select_yaw_angle = DEG2RAD(-(*world_angle));
```

这是把世界系转到车体系时常见的逆变换，数学上正常。

### 适配型负号

```c
Vy = -speedy;
```

这不是通用运动学必然公式，而是当前工程里结合安装/历史约定留下来的适配层。

它是造成“直觉和代码不一致”的核心来源之一。

### 轮速反号

最短转角选择时，如果选了反向角，会做：

```c
V = -V;
```

这也不是“全车方向定义变了”，而是局部轮子为了少转角做的等效实现。

## 典型例子

下面都先按当前主线常见场景解释：

- `world_angle = 0`

### 例 1：只给 `speedx > 0`

输入：

```text
speedx = 100
speedy = 0
speedw = 0
```

则：

```text
Vx = 100
Vy = 0
```

四轮都相同：

- `mix = (100, 0)`
- `wheel_speed = 100`
- `wheel_angle = -PI/2`

解释：

- 四个轮都朝同一方向
- 这是纯 x 平动

### 例 2：只给 `speedy > 0`

输入：

```text
speedx = 0
speedy = 100
speedw = 0
```

内部会变成：

```text
Vx = 0
Vy = -100
```

四轮都相同：

- `mix = (0, -100)`
- `wheel_speed = 100`
- `wheel_angle = ±PI`

这说明：

```text
上层 speedy 的正方向
进入 steering_wheel 后
内部是反向的
```

### 例 3：纯旋转，`speedw > 0`

输入：

```text
speedx = 0
speedy = 0
speedw = 100
```

令：

```text
k = 100 / sqrt(2) = 70.71
```

则四轮：

- 1号轮：`mix = (+70.71, -70.71)`
- 2号轮：`mix = (+70.71, +70.71)`
- 3号轮：`mix = (-70.71, -70.71)`
- 4号轮：`mix = (-70.71, +70.71)`

注意：

这组正负号是当前代码内部坐标里的纯旋转模板，不应直接按物理俯视图的“右上左下”去逐项对应。

### 例 4：平动加自旋

输入：

```text
speedx = 100
speedy = 0
speedw = 100
```

则：

```text
Vx = 100
Vy = 0
k = 70.71
```

各轮合速度：

- 1号轮：`mix = (170.71, -70.71)`
- 2号轮：`mix = (170.71, +70.71)`
- 3号轮：`mix = (29.29, -70.71)`
- 4号轮：`mix = (29.29, +70.71)`

可以看出：

- 某些轮平动和自旋同向叠加，速度更大
- 某些轮被部分抵消，速度更小

这正是舵轮底盘“平动 + 自旋”时四个轮速度不同的原因。

## 为什么注释写“逆时针为正”，但实车看起来却像“顺时针为正”

这是最容易误读的地方。

### 注释说的“逆时针为正”主要是在说哪一层

主要是在说：

- `wheel_angle` 这套角度编码空间
- 以及理论上的旋转正方向约定

### 但整条链路里发生了什么

整条链路里还叠加了：

- `Vy = -speedy` 的 Y 轴镜像
- 以 `+Y` 为零点的角度编码
- 最短转角的局部反向
- 后续电机安装方向/零位定义

因此，从代码里的“数学角度正方向”到肉眼看到的“车体物理顺逆时针”，中间并不是一一直接对应的。

### 更准确的理解

当前代码里，最容易让人误会的不是“逆时针为正”这句话本身，而是：

```text
这句话没有明确区分：
- 上层速度坐标
- steering_wheel 内部分量坐标
- 轮角编码坐标
- 实车物理旋转方向
```

它们并不完全是同一套约定。

## 当前主线下哪些地方其实还没真正用起来

当前 `chassis` 主线里：

- 默认使用 `SELF`
- `self_yaw` 固定为 `0`

这意味着在大多数稳定场景下：

```text
world_angle = 0
steering_select_yaw_angle = 0
```

所以 `-world_angle` 这层虽然数学上存在，但在当前主线下大多相当于没有参与实际变换。

因此，当前最主要的阅读困难，并不是来自 `-world_angle`，而是来自：

- `Vy = -speedy`
- `wheel_angle` 的特殊零点定义
- 最短转角导致的轮速反号

## 最短结论

### 结论 1

`steering_wheel` 当前不是“纯标准直角坐标系下的四轮解算”。

### 结论 2

它内部至少做了两件会破坏直觉的事：

- `Y` 轴镜像：`Vy = -speedy`
- 角度零点放在 `+Y`

### 结论 3

因此读代码时会觉得它：

- 像是某根轴反了
- 又像角度坐标转了 90 度
- 但整条链又能正常跑

这不是错觉，而是当前实现确实如此。

### 结论 4

`1号轮纯旋转时为什么是 (+x, -y)` 的根本原因不是“物理上它就往右下”，而是：

```text
物理俯视图里的“右上”
在 steering_wheel 当前内部 mix 坐标里
会被记成“右 + 负Y”
```

## 后续如果要继续整理，建议怎么做

如果后续准备真正统一坐标约定，建议分三步处理，而不是直接改公式：

1. 先明确写文档区分三层坐标
   - 上层速度输入坐标
   - steering_wheel 内部分量坐标
   - wheel_angle 编码坐标

2. 再判断 `Vy = -speedy` 是否是必要的安装适配
   - 如果必要，就保留，但文档必须写清楚
   - 如果只是历史遗留，可以考虑后续整理掉

3. 最后再考虑把角度提取改成更直观的表达
   - 例如统一解释成某种明确的 `atan2` 角度定义
   - 避免继续依赖“看公式猜坐标”

## 阅读本模块时最容易犯的错

### 错误 1

把 `mix_x / mix_y` 直接当成物理俯视图里的标准 `x/y`

### 错误 2

把 `wheel_angle` 当成常见的“`+X = 0`”角度空间

### 错误 3

把中间几何角度 `wheel_angle` 和最终电机命令角 `set_wheel_angle` 当成一回事

### 错误 4

把“代码注释中的逆时针为正”和“实车物理观察到的顺逆时针”直接一一对应

## 总结

当前 `G4_2026/` 主线中，`steering_wheel` 能正常工作，不代表它的坐标约定是最直观的。

更准确地说：

- 它是一套内部自洽但阅读成本较高的实现
- 自洽来自前后一致
- 困惑来自不同层次的坐标约定没有在代码里显式拆开说明

因此，阅读时必须明确区分：

- 物理俯视图方向
- 上层速度输入方向
- `mix_x / mix_y` 的内部方向
- `wheel_angle` 的编码方向
- 最终轮子执行方向

只要这几层不混，当前代码的“奇怪感”就能被解释清楚。
