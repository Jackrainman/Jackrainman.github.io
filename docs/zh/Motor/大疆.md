# `dji_bldc_motor` 代码详解

本文按当前仓库代码解释 `Drivers/Bsp/DJI-Motor/dji_bldc_motor.c` 和 `Drivers/Bsp/DJI-Motor/dji_bldc_motor.h`，并结合当前 G4 单板底盘最小闭环中的真实调用链说明它的作用。

## 1. 这个模块在当前仓库里的位置

它位于底盘执行层的最末端，负责两件事：

1. 把 DJI 电机的 CAN 反馈帧解析进 `dji_motor_handle_t`
2. 把上层算好的电流指令打包成 8 字节 CAN 帧发出去

在当前仓库里，和它直接相关的上层调用链是：

```c
// User/Application/Src/chassis.c:462
for (uint32_t i = 0; i < WHEEL_NUM; i++) {
    dji_motor_init(&speed_motor[i], DJI_M3508, CAN_Motor1_ID + i,
                   can2_selected);
}

// User/Application/Src/chassis.c:397
dji_motor_set_current(
    can2_selected, DJI_MOTOR_GROUP1, speed_motor[0].set_value,
    speed_motor[1].set_value, speed_motor[2].set_value,
    speed_motor[3].set_value);
```

也就是说：

- `chassis_init_lower()` 里初始化 4 个 M3508 驱动电机
- `chassis_motor_ctrl_task()` 里做完 PID 后，把 `set_value` 交给 `dji_motor_set_current()`
- 本模块再通过 `can_send_message()` 把 4 路电流一起发到 `DJI_MOTOR_GROUP1`

## 2. 它和底层 CAN 框架怎么接上的

### 2.1 初始化时把“电机句柄 + 回调”注册进 CAN 哈希表

```c
// Drivers/Bsp/can_list/can_list.c:178
uint8_t can_list_add_new_node(can_selected_t can_select, void *node_data,
                              uint32_t id, uint32_t id_mask, uint32_t id_type,
                              can_callback_t callback) {
    ...
    new_node->can_data = node_data;
    new_node->id = id;
    new_node->id_mask = id_mask;
    new_node->callback = callback;
    ...
}
```

`dji_motor_init()` 调用上面这个接口时，把：

- `node_data` 设成 `dji_motor_handle_t *`
- `id` 设成电机 CAN ID，例如 `0x201`
- `callback` 设成 `can_callback`

这样 CAN 收到对应 ID 的反馈帧后，最终就会回调回本模块。

### 2.2 收到 CAN 帧后，由 `can_list` 找到节点并调用回调

```c
// Drivers/Bsp/can_list/can_list.c:613
can_node_t *node = table->table[id % table->len];

while ((node != NULL) && (node->id) != (id & node->id_mask)) {
    node = node->next;
}

if (node == NULL || node->callback == NULL) {
    return;
}

node->callback(node->can_data, &call_rx_header, rx_data);
```

这里说明一个关键点：**当前实现里，真正按 ID 分发回调的第一层过滤主要已经在 `can_list` 内完成了。**

### 2.3 本模块发 CAN 时，其实走的是 FDCAN 兼容层

```c
// Drivers/CSP/CAN_STM32G4xx.h:286
#define can_send_message fdcan_send_message
#define CAN_ID_STD       FDCAN_STANDARD_ID
```

所以 `dji_bldc_motor.c` 里虽然写的是 `can_send_message()`、`CAN_ID_STD`，但在 STM32G4 当前仓库里，底下实际走的是 FDCAN 兼容接口。

## 3. `dji_bldc_motor.c` 逐行详解

以下按 `Drivers/Bsp/DJI-Motor/dji_bldc_motor.c` 当前行号说明。

### 3.1 文件头与依赖：1-11

- `1-7`：文件头注释，声明文件名、作者、版本、日期；纯说明，不参与编译逻辑。
- `8`：空行，用于把文件头和正文分开。
- `9`：`#include "dji_bldc_motor.h"`，先引入自己的头文件，拿到结构体、枚举、函数声明。
- `10`：空行，分隔本模块头和其他依赖。
- `11`：`#include "can_list/can_list.h"`，因为本模块要把电机对象注册到 CAN 回调表里。

### 3.2 回调注释与函数声明：13-21

- `13-19`：注释说明 `can_callback()` 的用途：收到 CAN 帧后解析电机反馈。
- `20-21`：定义静态函数 `can_callback(...)`。
  - `static` 表示它只在本 `.c` 文件内部可见。
  - `node_obj` 是注册时传进来的用户对象，这里实际就是 `dji_motor_handle_t *`。
  - `can_rx_header` 是抽象后的 CAN 头。
  - `can_msg` 是收到的字节数组。

### 3.3 空指针检查：22-24

- `22`：判断 `node_obj == NULL`。
- `23`：为空就直接返回，避免后续解引用崩溃。
- `24`：结束这个保护分支。

### 3.4 恢复电机句柄：26

- `26`：把通用指针强转回 `dji_motor_handle_t *`，命名为 `motor_point`。

### 3.5 帧头二次过滤：28-31

```c
if (can_rx_header->id_type != CAN_ID_STD &&
    can_rx_header->id != motor_point->motor_id) {
    return;
}
```

- `28`：检查收到的是不是标准帧。
- `29`：再检查收到的 ID 是否和当前电机 `motor_id` 一致。
- `30`：两个条件同时满足时直接丢弃。
- `31`：结束分支。

这里要特别注意：

- 当前代码用了 `&&`，不是常见的 `||`。
- 再结合 `dji_motor_init()` 当前实现没有给 `motor->motor_id` 赋值，这里**并不是本模块真正可靠的主过滤点**。
- 但在当前仓库里，由于 `can_list` 已经先按 ID 把回调分发给对应节点，所以主链路通常仍能工作。

### 3.6 记录上一拍角度并解析新角度：33-34

- `33`：先把本次更新前的 `angle` 保存到 `last_angle`，给后面的过零判断使用。
- `34`：把 `can_msg[0:1]` 拼成 16 位角度值。
  - 高字节是 `can_msg[0]`
  - 低字节是 `can_msg[1]`
  - DJI 电机一圈编码为 `8192`

### 3.7 上电首帧零点获取：36-42

- `36`：如果还没拿到上电零点 `got_offset == false`，进入初始化分支。
- `37`：注释说明：获取上电初始角度。
- `38`：把当前角度记为 `offset_angle`，作为“上电零点”。
- `39`：同步把 `last_angle` 也设成当前值，避免第一拍就误判跨圈。
- `40`：把 `got_offset` 置为 `true`，表示只做一次。
- `41`：把圈数计数 `round_cnt` 清零。
- `42`：结束分支。

### 3.8 按电机型号解析反馈帧：44-71

- `44`：依据 `motor_model` 进入不同解析分支。
- `45`：如果编译时启用了 `DJI_MOTOR_USE_M3508_2006`，才编译 3508/2006 的分支。

#### `DJI_M3508`：46-51

- `46`：匹配到 M3508。
- `47`：把 `can_msg[2:3]` 解析到 `real_current`。
- `48`：把 `real_current` 再转成 `int16_t` 赋给 `speed_rpm`。
  - 按当前实现看，**这里把同一组字节既当电流又当转速使用**。
  - 这属于当前作者的具体解析写法，阅读时要以当前树为准，不要套别的驱动版本。
- `49-50`：把 `can_msg[4:5]` 解析到 `given_current`，并做 `/ -5.0f` 换算。
- `51`：结束 M3508 分支。

#### `DJI_M2006`：53-57

- `53`：匹配到 M2006。
- `54`：把 `can_msg[2:3]` 解析为 `speed_rpm`。
- `55-56`：把 `can_msg[4:5]` 换算成 `real_current`。
- `57`：结束 M2006 分支。

#### 宏结束与 `GM6020`：58-67

- `58`：结束 3508/2006 编译条件。
- `60`：如果启用了 GM6020，编译下面的分支。
- `61`：匹配到 `DJI_GM6020`。
- `62`：解析速度到 `speed_rpm`。
- `63-64`：解析转矩电流到 `torque_current`。
- `65`：第 7 字节作为温度 `temperature`。
- `66`：结束 GM6020 分支。
- `67`：结束 GM6020 编译条件。

#### 默认分支：69-71

- `69-70`：默认什么也不做，兼容未知型号。
- `71`：结束整个 `switch`。

### 3.9 记录霍尔值：73

- `73`：把 `can_msg[6]` 保存到 `hall`。
- 对 M3508/M2006，代码注释写的是“可能是霍尔传感器值”；当前实现统一把第 7 字节也存下来。

### 3.10 过零/跨圈判断：75-79

- `75`：如果 `angle - last_angle > 4096`，说明数值从小角度突然跳到大角度，更像是反向跨过零点。
- `76`：圈数减一。
- `77`：否则如果 `angle - last_angle < -4096`，说明从大角度跳回小角度，更像是正向跨过零点。
- `78`：圈数加一。
- `79`：结束跨圈判断。

这里的 `4096` 正好是 `8192 / 2`，表示“半圈阈值判断法”。

### 3.11 累积总角度：81-82

- `81`：`round_cnt * 4096 * 2` 其实就是 `round_cnt * 8192`，把整圈部分换算成编码数。
- `82`：再加上当前圈内角度，并减去上电零点 `offset_angle`，得到相对上电零点的 `total_angle`。

### 3.12 转子角度注释：84-90

- `84-90`：一大段说明 `rotor_degree` 的意义。
- 对 `M3508/M2006`：是**相对上电零点**、且已经除过减速比后的输出轴角度。
- 对 `GM6020`：是**绝对角度**，范围约 `0~360`，过零会跳变。

### 3.13 按型号换算 `rotor_degree`：91-114

- `91`：再次按型号分支，开始做角度单位换算。
- `92`：编译期条件，3508/2006 是否启用。

#### M3508：93-97

- `93`：M3508 分支。
- `94`：注释强调：3508 减速比可能改过，所以提成宏。
- `95-96`：公式为

```c
rotor_degree = total_angle / (GEAR_RATIO_M3508 * 8192.0f) * 360.0f;
```

含义是：

- `total_angle / 8192` 得到电机转子转过多少圈
- 再除减速比，得到输出轴多少圈
- 最后乘 `360` 变成角度

- `97`：结束分支。

#### M2006：99-103

- `99`：M2006 分支。
- `100`：注释标明减速比 `1:36`。
- `101-102`：和 M3508 同样公式，只是换成 `GEAR_RATIO_M2006`。
- `103`：结束分支。

#### GM6020：106-109

- `106`：启用 GM6020 分支。
- `107`：进入 GM6020。
- `108`：用 `angle / 22.75f` 直接得到角度，因为 `8192 / 360 = 22.755...`。
- `109`：结束分支。

#### 默认分支与函数结束：112-115

- `112-113`：默认分支仍然不做任何事。
- `114`：结束 `switch`。
- `115`：结束 `can_callback()`。

### 3.14 初始化函数注释：117-128

- `117-128`：说明 `dji_motor_init()` 的参数和返回值。
- 这里的“初始化”不是给电机发上电命令，而是**把本地电机对象接到 CAN 接收表里**。

### 3.15 `dji_motor_init()`：129-144

- `129-130`：函数签名，接收句柄、型号、CAN ID、CAN 口选择。
- `131`：先检查 `motor` 是否为空。
- `132`：空指针则返回错误码 `1`。
- `135`：保存电机型号到句柄。
- `136`：把 `got_offset` 清零，让首次反馈帧重新抓零点。
- `137`：保存用哪个 CAN 口通信。
- `138-139`：调用 `can_list_add_new_node()`，把当前电机对象注册到 CAN 表。
  - `can_select`：CAN1/CAN2/CAN3
  - `(void *)motor`：回调时再拿回来
  - `can_id`：期望接收的电机反馈 ID
  - `0x7FF`：标准帧 11 位掩码
  - `CAN_ID_STD`：标准帧
  - `can_callback`：收到后就调用本模块的解析函数
- `140`：注册失败返回错误码 `2`。
- `143`：注册成功返回 `0`。
- `144`：结束函数。

#### 当前实现注意点

当前这段代码**没有看到**：

```c
motor->motor_id = can_id;
```

这意味着：

- `dji_motor_deinit()` 里用到的 `motor->motor_id` 可能没有被这里初始化
- `can_callback()` 里的二次 ID 判断也不一定可靠地依赖这个字段

但就当前仓库底盘主链路看，接收匹配主要还是靠 `can_list` 注册的 `id` 在前面兜住。

### 3.16 `dji_motor_deinit()`：146-166

- `146-154`：反初始化函数注释。
- `155`：函数定义。
- `156-158`：空指针保护，为空返回 `1`。
- `160-161`：调用 `can_list_del_node_by_id()` 从 CAN 表中删掉这个电机节点。
- `162`：删除失败返回 `2`。
- `165`：成功返回 `0`。
- `166`：结束函数。

### 3.17 `dji_motor_set_current()`：168-201

- `168`：只有启用了 `DJI_MOTOR_USE_M3508_2006` 才编译这段。
- `170-181`：函数注释，说明它一次控制 4 路 M3508/2006 电流。
- `182-183`：函数定义，4 个 `iq` 对应同一组报文里的 4 台电机。
- `184`：检查分组 ID 是否是 `DJI_MOTOR_GROUP1` 或 `DJI_MOTOR_GROUP2`。
- `185`：注释说明：非法标识符。
- `186`：非法就直接返回，不发帧。
- `189`：创建 8 字节发送缓冲区。
- `190-197`：把 `iq1 ~ iq4` 依次按高字节、低字节拆开，塞入 `send_msg[0..7]`。
- `198`：调用 `can_send_message()` 发送标准帧。
- `199`：结束函数。
- `201`：结束编译条件。

这就是当前底盘速度环最终的“落地点”。上层在 `User/Application/Src/chassis.c:385-400` 里算出 4 个 `set_value` 后，最后就是从这里发出去。

### 3.18 `dji_gm6020_voltage_control()`：203-237

- `203`：启用 GM6020 代码块。
- `205-216`：函数注释，说明一次控制 4 路 GM6020 电压。
- `217-220`：函数定义。
- `221-225`：检查 `can_identify` 是否属于合法的 GM6020 电压控制分组；不合法直接返回。
- `227`：准备 8 字节发送缓存。
- `228-235`：与 `dji_motor_set_current()` 同理，把 4 路 `voltage` 依次拆成高低字节。
- `236`：调用 `can_send_message()` 发标准帧。
- `237`：结束函数。

### 3.19 `dji_gm6020_current_control()`：239-272

- `239-250`：函数注释，说明一次控制 4 路 GM6020 电流。
- `251-254`：函数定义。
- `255-259`：检查分组 ID 是否合法，不合法直接返回。
- `261`：准备 8 字节缓存。
- `262-269`：把 4 路 `current` 拆成 8 个字节。
- `271`：调用 `can_send_message()` 发出去。
- `272`：结束函数。

这一段在当前底盘最小闭环里不是主线，但模块一起提供了。

## 4. `dji_bldc_motor.h` 逐行详解

以下按 `Drivers/Bsp/DJI-Motor/dji_bldc_motor.h` 当前行号说明。

### 4.1 文件头与 include guard：1-28

- `1-17`：文件头、版本演进历史。
- `19-20`：头文件保护宏，防止重复包含。
- `22-24`：如果用 C++ 编译，导出 `extern "C"`，避免名字改编。
- `26`：引入 `CSP_Config.h`，里面会带上芯片支持包、CAN/FDCAN 等配置。
- `28`：引入 `stdbool.h`，因为结构体里要用 `bool`。

### 4.2 型号开关与分组宏：30-54

- `30-33`：两个总开关，控制是否编译 M3508/M2006 和 GM6020 相关代码。
- `35`：开始 M3508/M2006 宏区。
- `37-38`：定义 M3508/M2006 的两个控制分组 ID：
  - `0x200` 控制 1~4 号位
  - `0x1FF` 控制 5~8 号位
- `40`：定义 `GEAR_RATIO_M3508`，当前值是 `268/17`。
- `41`：定义 `GEAR_RATIO_M2006 = 36.0f`。
- `44`：结束 M3508/M2006 宏区。
- `46`：开始 GM6020 宏区。
- `48-49`：GM6020 电压控制分组 ID。
- `51-52`：GM6020 电流控制分组 ID。
- `54`：结束 GM6020 宏区。

### 4.3 电机型号枚举：56-63

- `56-58`：注释说明接下来是电机型号。
- `59-63`：定义 `dji_motor_model_t`。
  - `DJI_M3508 = 0x00`
  - `DJI_M2006 = 0x01`
  - `DJI_GM6020 = 0x02`

### 4.4 CAN ID 枚举：65-90

- `65-68`：注释说明下面是 CAN ID；并提醒 GM6020 与 M3508/2006 在 `0x205~0x207` 这些编号上会有复用关系。
- `69`：开始 `dji_can_id_t` 枚举。
- `70-79`：如果启用 M3508/2006，就依次定义 `CAN_Motor1_ID = 0x201` 到 `CAN_Motor8_ID = 0x208`。
- `81-89`：如果启用 GM6020，就定义 `CAN_GM6020_ID1 = 0x205` 到 `CAN_GM6020_ID7 = 0x20B`。
- `90`：结束枚举。

### 4.5 电机句柄结构体：92-136

- `92-94`：注释说明下面是电机参数结构体。
- `95`：开始定义 `dji_motor_handle_t`。

#### 3508/2006 专属字段：97-103

- `97`：如果启用 3508/2006，编译这些字段。
- `99`：注释标注“3508/2006 参数”。
- `100`：`real_current`，实际电流。
- `101`：`given_current`，期望电流；注释说明 M3508 才会赋值。
- `103`：结束这一条件块。

#### GM6020 专属字段：105-111

- `105`：如果启用 GM6020，编译这些字段。
- `107`：注释标注“GM6020 参数”。
- `108`：`torque_current`，实际转矩电流。
- `109`：`temperature`，温度。
- `111`：结束这一条件块。

#### 共用字段：113-135

- `113`：注释标注“共用参数”。
- `114`：`hall`，当前实现里统一把反馈第 7 字节存到这里。
- `116`：`got_offset`，是否已经抓到上电零点。
- `117`：`offset_angle`，上电时的初始角度。
- `119`：`last_angle`，上一拍角度。
- `120`：`angle`，当前绝对角度，一圈 `8192`。
- `121`：`total_angle`，从上电零点开始累计的总角度编码值。
- `122`：`round_cnt`，跨圈计数。
- `123-128`：`rotor_degree` 的详细注释。
  - 对 3508/2006：是上电后相对角度，已经折算过减速比
  - 对 6020：是绝对角度，会在 0/360 附近跳变
- `130`：`set_value`，上层准备下发的电压/电流值。
- `131`：`speed_rpm`，当前转速。
- `133`：`motor_id`，电机 ID。
- `134`：`motor_model`，电机型号。
- `135`：`can_select`，用哪个 CAN 口通信。
- `136`：结构体结束。

### 4.6 API 声明：138-163

- `138-140`：声明 `dji_motor_init()` 和 `dji_motor_deinit()`。
- `142-145`：如果启用 M3508/2006，声明 `dji_motor_set_current()`。
- `147-157`：如果启用 GM6020，声明电压控制和电流控制两个接口。
- `159-161`：结束 `extern "C"`。
- `163`：结束头文件保护宏。

## 5. 结合当前底盘主线再看一遍

在当前仓库最小闭环里，`dji_bldc_motor` 所处的位置可以压缩成下面这几步：

```text
remote_ctrl -> chassis_manual_ctrl_task -> steering_wheel_ctrl
-> speed_motor_target[i]
-> pid_calc(..., speed_motor[i].speed_rpm)
-> speed_motor[i].set_value
-> dji_motor_set_current()
-> can_send_message()
```

对应当前真实代码：

```c
// User/Application/Src/chassis.c:385
for (uint8_t i = 0; i < WHEEL_NUM; i++) {
    float pid_output = pid_calc(&speed_motor_pid[i], speed_motor_target[i],
                                speed_motor[i].speed_rpm);
    speed_motor[i].set_value = (int16_t)pid_output;
}

// User/Application/Src/chassis.c:397
dji_motor_set_current(
    can2_selected, DJI_MOTOR_GROUP1, speed_motor[0].set_value,
    speed_motor[1].set_value, speed_motor[2].set_value,
    speed_motor[3].set_value);
```

所以从“底盘最小闭环”的角度看，本模块最重要的两个字段就是：

- `speed_motor[i].speed_rpm`：反馈给速度环
- `speed_motor[i].set_value`：作为电流指令发出去

## 6. 当前实现里值得特别留意的点

### 6.1 `motor_id` 在 `dji_motor_init()` 里未见赋值

当前实现里 `dji_motor_handle_t` 有 `motor_id` 字段，但 `dji_motor_init()` 只设置了：

```c
motor->motor_model = motor_model;
motor->got_offset = false;
motor->can_select = can_select;
```

没有看到：

```c
motor->motor_id = can_id;
```

这会影响：

- `dji_motor_deinit()` 删除节点时按 `motor->motor_id` 删除
- `can_callback()` 里的二次 ID 判断

### 6.2 `can_callback()` 的过滤条件用了 `&&`

当前代码是：

```c
if (can_rx_header->id_type != CAN_ID_STD &&
    can_rx_header->id != motor_point->motor_id) {
    return;
}
```

这意味着只有“既不是标准帧、ID 也不匹配”时才返回。

按直觉，很多驱动会写成 `||`，表示只要任一条件不满足就拒绝。但**这里不能脱离当前仓库独立下判断**，因为前面 `can_list` 已经先按 ID 分发表了，所以这段代码在当前链路里更像附加保护，而不是主过滤层。

## 7. 一句话总结

`dji_bldc_motor` 在当前树里的本质就是：

- **接收侧**：把 DJI 电机反馈帧解析成 `speed_rpm / angle / total_angle / rotor_degree` 等状态
- **发送侧**：把 4 路电机控制量打成 8 字节标准 CAN 帧发出
- **在当前底盘最小闭环里**：它就是 `chassis_motor_ctrl_task()` 和实际驱动电机之间的最后一跳
