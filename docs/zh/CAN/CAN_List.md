# CAN 通信列表模块 (CAN List)

> **最近修改日期** ：2026-01-28
>
> **预计阅读时间**：15-20 分钟
>
> **参与者**：Deadline039（作者），Jackrainman（文档编写）
>
> **版本**：1.0
>
> **相关文档**：
> - [CAN List详解](./CAN_List.md) - 完整 API 参考与模块文档
> - [CAN总线基础](./CAN总线基础.md) - CAN 总线基础知识

## 1. 概述

### 1.1 模块简介

CAN-List 是一个专为 STM32 设计的 **CAN 消息分发器 (Dispatcher)**。在机器人开发中，总线上会有大量传感器和电机反馈数据。本模块的核心作用是：**根据收到的 CAN ID，自动找到对应的设备对象并调用其处理函数。**

它解决了传统开发中 `switch-case` 语句过于冗长、难以维护的问题，通过哈希表 (Hash Table) 实现了高效的查询。

### 1.2 核心功能

* **高效分发**：收到 CAN 消息后，根据 ID 快速查找并调用对应的回调函数，避免在中断中进行复杂的 `if-else` 判断。
* **多外设兼容**：同时支持 STM32 的 bxCAN (CAN 2.0) 和 FDCAN 外设，通过宏定义切换。
* **RTOS 支持**：可选集成 FreeRTOS，将消息处理从中断服务函数（ISR）转移到任务线程中，减少中断占用时间。
* **掩码匹配机制**：支持 ID 掩码，允许对 ID 的特定位进行匹配，适应复杂协议。
* **动态管理**：支持运行时添加、删除节点和更改回调函数。

### 1.3 适用场景

* 需要处理大量不同 CAN ID 设备的机器人底盘或云台控制。
* 需要统一管理 CAN 通信回调逻辑的工程项目。
* 对实时性要求较高，需要快速中断响应的系统。

## 2. 集成指南

### 2.1 文件部署

1. 将 `can_list` 文件夹整体复制到工程目录 `/Drivers/bsp/` 下。
2. 在板级支持包头文件（如 `bsp.h`）中包含本模块：
   ```c
   #include "can_list/can_list.h"
   ```

### 2.2 依赖说明

* **必需**：标准库 `<stdlib.h>` (用于内存管理)。
* **可选**：FreeRTOS (若开启 RTOS 模式)。
* **配置**：需依赖项目的 `CSP_Config.h` 或相应的 HAL 库头文件。

## 3. 配置宏说明

在 `can_list.h` 中根据硬件平台和需求修改以下宏定义：

| 宏定义名称 | 默认值 | 说明 |
| --- | --- | --- |
| `CAN_LIST_USE_FDCAN` | `0` | **硬件选择** 。<br>`1`: 启用 FDCAN 支持 (如 STM32G4/H7)。<br>`0`: 启用 bxCAN 支持 (如 STM32F1/F4)。 |
| `CAN_LIST_MAX_CAN_NUMBER` | `3` | **最大外设数量**。<br>限制系统支持的 CAN 控制器总数，防止数组越界。 |
| `CAN_LIST_USE_RTOS` | `1` | **系统集成**。<br>`1`: 创建 FreeRTOS 任务处理消息，需确保 CAN 中断优先级**低于** FreeRTOS 管理的最大优先级。<br>`0`: 在中断回调中直接处理消息。 |
| `CAN_LIST_MALLOC` | `malloc` | 内存分配函数，可替换为自定义实现。 |
| `CAN_LIST_CALLOC` | `calloc` | 内存分配函数（带清零），可替换为自定义实现。 |
| `CAN_LIST_FREE` | `free` | 内存释放函数，可替换为自定义实现。 |
| `CAN_LIST_TASK_NAME` | `"Can list"` | FreeRTOS 任务名称（仅 RTOS 模式）。 |
| `CAN_LIST_TASK_PRIORITY` | `2` | FreeRTOS 任务优先级（仅 RTOS 模式）。 |
| `CAN_LSIT_TASK_STK_SIZE` | `256` | FreeRTOS 任务栈大小（仅 RTOS 模式）。 |
| `CAN_LIST_QUEUE_LENGTH` | `5` | FreeRTOS 消息队列长度（仅 RTOS 模式）。 |

> **注意**：若启用 RTOS 模式，务必在调用 `vTaskStartScheduler()` 之后再使用 `can_list_add_can` 进行初始化。

## 4. API 参考手册

### 4.1 数据类型

#### `can_rx_header_t`
统一的消息头结构体，兼容 FDCAN 和 bxCAN。

```c
typedef struct {
    uint32_t id;         /*!< Message ID .                                     */
    uint32_t id_type;    /*!< ID type , `CAN_ID_STD` or `CAN_ID_EXT` .          */
    uint32_t frame_type; /*!< Frame type , `CAN_RTR_DATA` or `CAN_RTR_REMOTE` . */
    uint8_t data_length; /*!< Message Data length (DLC) . DLC编码范围0-8：如果DLC=1则数据位为1字节(8位)，如果DLC=8则数据位为8字节(64位)。 */
} can_rx_header_t;
```

#### `can_callback_t`
回调函数类型定义。

```c
typedef void (*can_callback_t)(void * /* node_obj */,
                               can_rx_header_t * /* can_rx_header */,
                               uint8_t * /* can_msg */);
```

### 4.2 函数说明

#### `can_list_add_can` – 初始化 CAN 列表

创建一个用于管理特定 CAN 外设的哈希表。

```c
uint8_t can_list_add_can(can_selected_t can_select, uint32_t std_len, uint32_t ext_len);
```

**参数**：
* `can_select`：目标 CAN 外设（如 `can1_selected`）。
* `std_len`：标准帧 ID 哈希表的桶（Bucket）长度。
* `ext_len`：扩展帧 ID 哈希表的桶（Bucket）长度。

**返回值**：
* `0`：成功。
* `1`：CAN 选择无效（超出 `CAN_LIST_MAX_CAN_NUMBER`）。
* `2`：该 CAN 已创建。
* `3`：内存分配失败。

**性能调优**：
* `len` 值越大，哈希冲突概率越低，查表速度越快，但内存占用增加。
* 若将 `len` 设置为 1，结构将退化为普通单向链表。

#### `can_list_add_new_node` – 注册接收节点

向 CAN 列表中添加一个新的设备节点及其回调函数。

```c
uint8_t can_list_add_new_node(can_selected_t can_select,
                              void *node_data,
                              uint32_t id,
                              uint32_t id_mask,
                              uint32_t id_type,
                              can_callback_t callback);
```

**参数**：
* `can_select`：使用哪个 CAN 外设接收。
* `node_data`：设备对象指针。触发回调时，该指针将作为参数传入，用于区分不同设备实例。
* `id`：期望匹配的 CAN ID（通常为设备反馈的 ID）。
* `id_mask`：ID 掩码。用于屏蔽 ID 中不相关的位（如包含状态码的高位）。匹配逻辑为：`id == (received_id & mask)`。
* `id_type`：ID 类型（`CAN_ID_STD` 或 `CAN_ID_EXT`）。
* `callback`：接收到匹配数据后的回调函数。**不能为空**。

**返回值**：
* `0`：成功。
* `1`：CAN 选择无效。
* `2`：该 CAN 表未创建。
* `3`：参数无效（如 `callback` 为空或 `id_type` 错误）。
* `4`：该 ID 已存在于表中。
* `5`：内存分配失败。

#### `can_list_del_node_by_id` – 删除节点

通过 ID 删除已注册的节点。

```c
uint8_t can_list_del_node_by_id(can_selected_t can_select, uint32_t id_type, uint32_t id);
```

**参数**：
* `can_select`：目标 CAN 外设。
* `id_type`：ID 类型（`CAN_ID_STD` 或 `CAN_ID_EXT`）。
* `id`：要删除的节点 ID。

**返回值**：
* `0`：成功。
* `1`：CAN 选择无效。
* `2`：该 CAN 表未创建。
* `3`：参数无效（`id_type` 错误）。
* `4`：节点不存在。

#### `can_list_change_callback` – 更改回调函数

动态更改已注册节点的回调函数。

```c
uint8_t can_list_change_callback(can_selected_t can_select, uint32_t id_type, uint32_t id, can_callback_t new_callback);
```

**参数**：
* `can_select`：目标 CAN 外设。
* `id_type`：ID 类型（`CAN_ID_STD` 或 `CAN_ID_EXT`）。
* `id`：目标节点 ID。
* `new_callback`：新的回调函数。

**返回值**：
* `0`：成功。
* `1`：CAN 选择无效。
* `2`：该 CAN 表未创建。
* `3`：参数无效（`id_type` 错误）。
* `4`：节点不存在。

## 5. 数据结构

每个 CAN 外设（如 CAN1, CAN2）都有一个独立的 `can_table_t` 指针，存储在全局数组中。

> **提示**：如需了解数据结构的深度解析（指针与多级结构、哈希表设计原理），请参考 [CAN_List.md](./CAN_List.md#4-数据结构设计) 第4章。

## 6. 架构详解

### 6.1 数据流架构图

为了便于理解数据从硬件中断到用户回调的全过程，请参考以下数据流向图：

```mermaid
graph LR
    HW[硬件 FIFO] -->|中断触发| ISR[中断服务函数]
    ISR -->|RTOS 路径: 写入队列| Q[FreeRTOS 消息队列]
    ISR -->|非 RTOS 路径: 直接调用| PROCESS[协议处理核心]
    Q -->|任务唤醒| PROCESS
    PROCESS -->|HAL_GetRxMessage| READ[读取寄存器数据]
    READ -->|id % len| HASH[定位哈希桶]
    HASH -->|id & mask| MATCH[链表遍历与匹配]
    MATCH -->|Callback| USER[用户业务逻辑]
```

#### bxCAN 中断系统架构

bxCAN 占用 4 个专用的中断向量，以保证通信的实时性。下图展示了中断标志位（Flag）与使能位（Enable）的 **与逻辑** 关系：

![CAN 中断系统](./Pictures/CAN4.png)

如图所示：

- **发送中断 (TX)**：当 3 个发送邮箱中至少有一个变为空（发送完成）时产生。

- **FIFO 0 中断** & **FIFO 1 中断**：

    - 收到新报文（FMP）。

    - FIFO 满（FULL）。

    - FIFO 溢出（OVR）。

- **状态改变错误中断 (SCE)**：处理出错（Error）、唤醒（Wakeup）或进入睡眠（Sleep）等事件。

### 6.2 详细数据流解析

我们将数据处理流程拆解为四个关键阶段：**中断接收**、**任务调度**、**路由匹配**、**用户回调**。

#### 阶段一：数据源头与中断接收 (Entry Point)

当 STM32 的 CAN 外设收到一帧完整报文并存入硬件 FIFO（先进先出缓存）后，会触发中断服务函数 (ISR)。

根据是否使用 RTOS，处理逻辑分为两条路径：

* **RTOS 路径（异步高效模式，推荐）**
  * **核心思想**：ISR 仅负责"通知"，不做繁重处理，确保系统高实时性。
  * **操作流程**：
    1. **打包元数据**：将 CAN 句柄 (`hcan`) 和 FIFO 编号 (`rx_fifo`) 封装入结构体。
    2. **推入队列**：调用 `xQueueSendFromISR` 将消息发送至后台任务队列。
    3. **屏蔽中断 (仅 bxCAN)**：暂时关闭当前中断，防止在后台处理完成前重复触发导致溢出。

* **非 RTOS 路径（同步模式）**
  * ISR 直接调用 `can_message_process` ，在中断上下文中完成所有解析工作。

**代码示例 (can_list.c):**

```c
/* bxCAN FIFO0 中断回调 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
#if CAN_LIST_USE_RTOS
    /* 1. 检查队列有效性 */
    if (can_list_queue_handle == NULL) return;

    /* 2. 打包消息来源 */
    send_msg_from_isr.hcan = hcan;
    send_msg_from_isr.rx_fifo = CAN_RX_FIFO0;

    /* 3. 发送至队列 (上下文切换) */
    xQueueSendFromISR(can_list_queue_handle, &send_msg_from_isr, NULL);

    /* 4. 暂时屏蔽中断 (bxCAN 特有) */
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
#else
    /* 非 RTOS 直接处理 */
    can_message_process(hcan, CAN_RX_FIFO0);
#endif
}
```

#### 阶段二：数据获取与格式统一 (Data Retrieval)

后台任务 `can_list_polling_task` 平时处于阻塞状态。一旦队列收到消息，任务立即被唤醒并执行以下操作：

1. **读取硬件寄存器**：根据 HAL 库差异，调用 `HAL_CAN_GetRxMessage` (bxCAN) 或 `HAL_FDCAN_GetRxMessage` (FDCAN)。
2. **数据搬运**：将数据从外设寄存器转移到内存变量 `rx_header` (帧头信息) 和 `rx_data` (数据载荷)。
3. **恢复中断**：对于 bxCAN，读取完成后立即调用 `HAL_CAN_ActivateNotification` 重新开启接收中断。

#### 阶段三：哈希路由与掩码匹配 (Core Logic)

这是本模块的核心逻辑，决定了数据属于哪个设备。

* **哈希定位 (Hash Mapping)**
  * 根据 ID 类型（标准帧或扩展帧）选择对应的哈希表。
  * 使用取模运算直接定位链表头：`index = id % table->len`。此算法避免了全局遍历，极大降低了 CPU 占用率。

* **链表遍历与掩码过滤**
  * 由于哈希冲突的存在，同一索引下可能挂载多个节点，需遍历链表。
  * **掩码 (Mask) 机制**：判断逻辑为 `node->id == (received_id & node->id_mask)` 。
  * **应用场景**：若设备 ID 包含动态数据（如最后 8 位为动态值），可将 Mask 设为 `0xFFFFFF00`，实现对一类 ID 的模糊匹配。

**核心逻辑代码:**

```c
/* 哈希定位 + 链表遍历 */
node = table->table[id % table->len]; // O(1) 定位

// 遍历链表 (O(n) 冲突处理)
while ((node != NULL) && (node->id) != (id & node->id_mask)) {
    node = node->next;
}
```

#### 阶段四：数据交付与回调执行 (Callback)

当匹配到注册节点 `node` 后，数据流到达终点。

1. **格式标准化**：将不同外设的头文件信息统一转换为 `can_rx_header_t` 结构体，确保用户层接口一致。
2. **透传设备指针**：`node->can_data` 是用户注册时绑定的设备对象指针（例如电机结构体 `&motor1`）。
3. **函数调用**：
   ```c
   node->callback(node->can_data, &call_rx_header, rx_data);
   ```

此时，控制权正式移交给用户的业务逻辑层。

## 7. 核心机制：ID 匹配与掩码逻辑

在复杂的协议中，一个 ID 可能包含多种信息（如：ID 的前几位是错误码，后几位才是设备号）。掩码机制允许我们只匹配关心的位。

### 7.1 掩码匹配公式

系统使用位运算来判断收到的消息是否属于某个节点：

```c
node->id == (received_id & node->id_mask)
```

简单来说，就是做与运算。

### 7.2 实际案例

假设电机反馈 ID 格式：低 8 位是设备 ID，高 24 位包含错误码和模式（动态变化）。

**配置示例：**
```c
uint32_t device_id = 0x01;   // 设备 ID
uint32_t mask = 0x000000FF; // 掩码：只匹配低 8 位

// 当收到 0x123401 时：0x123401 & 0x000000FF = 0x01
// 当收到 0x567801 时：0x567801 & 0x000000FF = 0x01
// 两种情况都匹配成功！
```

### 7.3 掩码设置指南

| 需求场景 | 掩码设置 | 说明 |
| --- | --- | --- |
| **全匹配** | `0x1FFFFFFF` (扩展帧) 或 `0x7FF` (标准帧) | 所有位都必须匹配 |
| **匹配低 8 位** | `0x000000FF` | 只比较 ID 的低 8 位 |
| **匹配低 16 位** | `0x0000FFFF` | 只比较 ID 的低 16 位 |

> **提示**：如需了解掩码机制深度剖析、位运算原理、复杂应用场景，请参考 [CAN_List.md](./CAN_List.md) 第4章"掩码机制深度剖析"。


## 8. 使用示例

### 8.1 场景描述

假设我们控制一个电机设备，通信协议定义如下：

* **主机 (Master)** ：控制端（我们）。
* **设备 (Device)** ：被控端（电机）。
* **反馈帧 (Device -> Master)** ：使用扩展帧。
  * ID 格式：`[29:22] 错误码 | [21:14] 模式 | [13:11] 数据标识 | [10:8] 保留 | [7:0] Master ID`。
  * 我们只关心 `[7:0]` 位是否匹配 Master ID。
* **控制帧 (Master -> Device)** ：
  * ID 格式：`[15:8] Master ID | [7:0] Dev ID`。

### 8.2 回调函数定义

回调函数必须符合 `can_callback_t` 类型：

```c
/**
 * @brief 电机数据反馈回调
 * @param node_obj      注册时传入的设备对象指针 (dev_demo)
 * @param can_rx_header CAN 接收头部信息
 * @param can_msg       CAN 数据负载 (Payload)
 */
static void motor_callback(void *node_obj, can_rx_header_t *can_rx_header, uint8_t *can_msg) {
    can_dev_t *dev = (can_dev_t *)node_obj;

    /* 提取 ID 中的数据标识位 [13:11] */
    uint32_t data_type = (can_rx_header->id >> 11) & 0x07;

    switch (data_type) {
        case 1:
            /* 处理类型 1 数据：int32, uint16, int16 */
            memcpy(&dev->data1, &can_msg[0], sizeof(int32_t));
            memcpy(&dev->data2, &can_msg[4], sizeof(uint16_t));
            memcpy(&dev->data3, &can_msg[6], sizeof(int16_t));
            break;
        case 2:
            /* 处理类型 2 数据：uint32, float */
            memcpy(&dev->data4, &can_msg[0], sizeof(uint32_t));
            memcpy(&dev->data5, &can_msg[4], sizeof(float));
            break;
        default:
            break;
    }
}
```

### 8.3 业务逻辑实现

```c
/* 定义设备结构体 */
typedef struct {
    uint32_t master_id;
    uint32_t dev_id;
    /* 接收数据缓存 */
    int32_t data1;
    uint16_t data2;
    int16_t data3;
    uint32_t data4;
    float data5;
} can_dev_t;

/* 全局设备实例 */
static can_dev_t dev_demo;

void app_demo_init(void) {
    /* 1. 初始化 CAN 底层 (BSP层) */
    // can1_init();

    /* 2. 配置设备 ID */
    dev_demo.master_id = 1;
    dev_demo.dev_id = 2;

    /* 3. 注册接收回调 */
    can_list_add_new_node(
        can1_selected,          /* 使用 CAN1 */
        &dev_demo,              /* 传入设备实例指针 */
        dev_demo.master_id,     /* 期望匹配的 ID (Master ID) */
        0xFF,                   /* 掩码：只匹配 ID 的低 8 位 (0xFF) */
        CAN_ID_EXT,             /* 使用扩展帧 */
        motor_callback          /* 注册回调函数 */
    );
}

void app_demo_loop(void) {
    uint8_t send_data[8] = {0};

    /* 4. 发送控制指令 */
    // can_send_message(can1_selected,
    //                  CAN_ID_EXT,
    //                  (dev_demo.master_id << 8) | dev_demo.dev_id,
    //                  8,
    //                  send_data);
}
```

## 9. 常见问题解答

### 9.1 为什么启用 RTOS 模式后，CAN 中断优先级必须低于 FreeRTOS 管理的最大优先级？

FreeRTOS 使用 PendSV 和 SysTick 异常进行任务调度，这些异常的优先级通常设置为最低可编程优先级。如果 CAN 中断优先级高于 FreeRTOS 管理的优先级，当中断发生时，FreeRTOS 无法进行任务切换，可能导致队列操作失败或系统挂起。

### 9.2 哈希表长度设置为多少合适？

哈希表长度应根据实际 ID 数量和分布情况选择。一般原则：
* 长度应为质数，以减少哈希冲突。
* 长度越大，冲突越少，但内存占用越多。
* 建议长度略大于预期节点数量（例如，预期有 10 个节点，长度可设为 13）。

### 9.3 掩码机制是否支持标准帧 ID？

支持。掩码机制对标准帧和扩展帧同样有效。但标准帧 ID 只有 11 位，掩码应相应调整（如 `0x7FF` 用于全匹配）。

### 9.4 如果多个节点使用相同的 ID 和掩码会怎样？

`can_list_add_new_node` 会检查 ID 是否已存在，如果存在则返回错误代码 4。因此，不允许重复注册相同 ID 的节点。

### 9.5 回调函数中可以进行耗时操作吗？

**强烈不建议** 。回调函数在中断上下文（非 RTOS 模式）或任务上下文（RTOS 模式）中执行，耗时操作会阻塞其他消息处理，影响系统实时性。应将耗时操作移至其他任务或使用队列异步处理。

## 10. 术语表

| 术语 | 全称 / 含义 | 代码中的作用 |
| --- | --- | --- |
| **FDCAN** | **Flexible Data-rate CAN** | 一种进阶的 CAN 协议，支持更高的数据传输速率（最高 8 Mbps）和更长的数据帧（最高 64 字节）。 |
| **bxCAN** | **Basic Extended CAN** | STM32 传统的 CAN 外设模块，遵循标准的 CAN 2.0B 协议（最高 1 Mbps，8 字节数据）。 |
| **HAL** | **Hardware Abstraction Layer** | 硬件抽象层库。代码中大量使用了以 `HAL_` 开头的函数来直接操作底层硬件。 |
| **FIFO** | **First In, First Out** | 先进先出队列。硬件接收到消息后会先存放在名为 FIFO0 或 FIFO1 的缓冲区中。 |
| **ISR** | **Interrupt Service Routine** | 中断服务程序。硬件产生特定事件（如收到消息）时，CPU 强行暂停当前任务去执行的紧急代码。 |
| **Callback** | **回调函数** | 预先写好的一段逻辑，当满足特定条件（如收到指定 ID）时由系统自动调用。 |
| **Hash Table** | **哈希表** | 一种数据结构，通过哈希函数将键映射到数组索引，实现快速查找。 |

## 11. 附录

### 11.1 CAN 总线基础知识

关于 CAN 总线物理层、差分信号、抗干扰原理等基础知识，请参阅 [CAN总线基础](./CAN总线基础.md)。


### 11.2 源码文件说明

- `can_list.h`：头文件，包含所有公开 API 和配置宏。
- `can_list.c`：源文件，实现所有功能逻辑。

### 11.3 版本历史

- **v1.0** (2024-11-24) ：初始版本，支持 bxCAN 和 FDCAN，可选 RTOS 集成。

---

**如有问题或建议，请联系模块作者或文档编写者。**
