# STM32F1xx USART IDLE 中断详解

## 1. USART1_IRQHandler 什么时候会被激发？

### 1.1 中断触发条件

`USART1_IRQHandler` 是 USART1 的中断服务程序，当以下任一事件发生时会被激发：

| 中断源 | 触发条件 |
|--------|----------|
| RXNE | 接收数据寄存器非空（接收到数据） |
| TXE | 发送数据寄存器空（可以发送新数据） |
| IDLE | 空闲线路检测（本期重点） |
| TC | 发送完成 |
| PE/FE/NE/ORE | 各种错误 |

### 1.2 IDLE 中断触发时机

**IDLE 中断在"空闲线路"被检测到时触发**。

#### 什么是"空闲线路"？

空闲线路意味着**整个 RX 线上的电平保持为"1"（高电平）持续了 1 个字节的时间**。

具体来说，当以下条件满足时：
- RX 引脚检测到连续 10 个（无奇偶校验）或 11 个（带奇偶校验）"1"位
- 这相当于 1 个字节的传输时间（起始位 + 8/9 数据位 + 停止位）

#### 实际场景

```
发送:  [0x01] [0x02] [0x03] [0x04]
       ↓    ↓    ↓    ↓
接收:  START START START START
       0000  0000  0000  0000  ← 接收过程中
              ↓
       [数据全部接收完毕]
              ↓
       1111 1111 1111 1111  ← 空闲检测（连续1）
              ↓
       IDLE中断触发!
```

当接收方收到最后一帧数据的停止位后，如果 RX 线保持空闲（高电平）达到 1 帧时间，IDLE 标志置位，触发 IDLE 中断。

### 1.3 本项目的 IDLE 中断用途

在本项目中，IDLE 中断用于**一帧数据接收完成的检测**：

```c
// UART_STM32F1xx.c:252
void USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&usart1_handle, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&usart1_handle);
        uart_dmarx_idle_callback(&usart1_handle);  // 处理接收完成
    }
    HAL_UART_IRQHandler(&usart1_handle);
}
```

---

## 2. __HAL_UART_GET_FLAG 宏深度解析

### 2.1 宏原型

```c
// stm32f1xx_hal_uart.h:472
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->SR & (__FLAG__)) == (__FLAG__))
```

### 2.2 翻译成易懂的代码

将宏展开：

```c
// 原型
((__HANDLE__)->Instance->SR & (__FLAG__)) == (__FLAG__)

// 代入参数后
((&usart1_handle)->Instance->SR & UART_FLAG_IDLE) == UART_FLAG_IDLE

// 进一步展开
((&usart1_handle)->Instance->SR & USART_SR_IDLE) == USART_SR_IDLE
```

### 2.3 底层含义

```c
// 读取 USART 状态寄存器 (SR) 的值
uint32_t sr = USART1->SR;

// 与 IDLE 标志位进行按位与运算
// USART_SR_IDLE = 0x00000010 (bit 4)
uint32_t result = (sr & 0x00000010) == 0x00000010;

// 等价于
bool flag_set = (sr & (1 << 4)) != 0;
```

### 2.4 位运算解释

```
SR 寄存器的值示例: 0x000000C0 (二进制: 1100 0000)
                        ↑
                   bit 7  bit 6
                   
与 IDLE (0x10) 进行 & 运算:
  1100 0000
& 0001 0000
= 0000 0000  → 结果为 0，IDLE 标志未置位

如果 SR = 0x00000010 (只有 IDLE 位为 1):
  0001 0000
& 0001 0000
= 0001 0000  → 结果非 0，IDLE 标志已置位
```

---

## 3. __HAL_UART_CLEAR_IDLEFLAG 宏解析

### 3.1 宏原型

```c
// stm32f1xx_hal_uart.h:542
#define __HAL_UART_CLEAR_IDLEFLAG(__HANDLE__) __HAL_UART_CLEAR_PEFLAG(__HANDLE__)

// stm32f1xx_hal_uart.h:504-509
#define __HAL_UART_CLEAR_PEFLAG(__HANDLE__)     \
  do{                                           \
    __IO uint32_t tmpreg = 0x00U;               \
    tmpreg = (__HANDLE__)->Instance->SR;        \
    tmpreg = (__HANDLE__)->Instance->DR;        \
    UNUSED(tmpreg);                             \
  } while(0U)
```

### 3.2 清除 IDLE 标志的步骤

```c
// 实际执行的操作
uint32_t tmpreg;
tmpreg = USART1->SR;    // 第1步: 读取 SR 寄存器
tmpreg = USART1->DR;    // 第2步: 读取 DR 寄存器
// IDLE 标志被清除
```

### 3.3 为什么要这样清除？

根据 STM32F1xx 参考手册：

> IDLE 标志的清除需要两步：先读取 USART_SR 寄存器，再读取 USART_DR 寄存器。

这是一个**硬件设计要求**，通过软件按照特定顺序访问这两个寄存器来清除标志。

---

## 4. USART 状态寄存器 (USART_SR) 结构体

### 4.1 结构体定义

```c
// stm32f103xb.h:506-515
typedef struct {
    __IO uint32_t SR;    // 状态寄存器,   地址偏移: 0x00
    __IO uint32_t DR;    // 数据寄存器,   地址偏移: 0x04
    __IO uint32_t BRR;   // 波特率寄存器, 地址偏移: 0x08
    __IO uint32_t CR1;   // 控制寄存器1,  地址偏移: 0x0C
    __IO uint32_t CR2;   // 控制寄存器2,  地址偏移: 0x10
    __IO uint32_t CR3;   // 控制寄存器3,  地址偏移: 0x14
    __IO uint32_t GTPR;  // 预分频寄存器, 地址偏移: 0x18
} USART_TypeDef;
```

### 4.2 SR 寄存器各位定义

```c
// stm32f103xb.h:9425-9455
typedef struct {
    uint32_t SR;  // 地址偏移 0x00
} USART_SR_BitDef;

// 各位定义:
bit 0  - PE   (Parity Error)        奇偶校验错误
bit 1  - FE   (Framing Error)       帧错误
bit 2  - NE   (Noise Error)         噪声错误
bit 3  - ORE  (OverRun Error)       接收溢出错误
bit 4  - IDLE (IDLE Line Detected)  空闲线路检测 ← 本期重点
bit 5  - RXNE (Read Data Ready)    读数据寄存器非空
bit 6  - TC   (Transmission Complete) 传输完成
bit 7  - TXE  (Transmit Empty)     发送数据寄存器空
bit 8  - LBD  (LIN Break Detection) LIN 断开检测
bit 9  - CTS  (CTS Flag)           CTS 标志
```

### 4.3 各标志位详细说明

| 位 | 名称 | 含义 | 触发条件 |
|----|------|------|----------|
| bit 0 | PE | 奇偶校验错误 | 奇偶校验失败 |
| bit 1 | FE | 帧错误 | 停止位检测失败 |
| bit 2 | NE | 噪声错误 | 采样检测到噪声 |
| bit 3 | ORE | 溢出错误 | 新数据到来时 DR 仍未读取 |
| bit 4 | **IDLE** | **空闲线路** | **RX 线保持高电平 1 帧时间** |
| bit 5 | RXNE | 接收非空 | 接收到数据（可读 DR） |
| bit 6 | TC | 发送完成 | 所有数据发送完成 |
| bit 7 | TXE | 发送空 | 可以写入新数据到 DR |
| bit 8 | LBD | LIN 断开 | 检测到 LIN 总线断开 |
| bit 9 | CTS | CTS 变化 | CTS 引脚状态变化 |

---

## 5. UART_HandleTypeDef 结构体

### 5.1 结构体定义

```c
// stm32f1xx_hal_uart.h
typedef struct {
    USART_TypeDef        *Instance;        // USART 寄存器基地址
    UART_InitTypeDef     Init;              // 初始化配置
    uint8_t              *pTxBuffPtr;       // 发送缓冲区指针
    uint16_t             TxXferSize;        // 发送大小
    __IO uint16_t        TxXferCount;      // 剩余发送计数
    uint8_t              *pRxBuffPtr;       // 接收缓冲区指针
    uint16_t             RxXferSize;        // 接收大小
    __IO uint16_t        RxXferCount;       // 剩余接收计数
    __IO HAL_UART_StateTypeDef   gState;    // 全局状态
    __IO HAL_UART_StateTypeDef   RxState;   // 接收状态
    DMA_HandleTypeDef   *hdmatx;           // DMA 发送句柄
    DMA_HandleTypeDef   *hdmarx;           // DMA 接收句柄
    // ... 其他成员
} UART_HandleTypeDef;
```

### 5.2 关键成员说明

| 成员 | 类型 | 说明 |
|------|------|------|
| Instance | USART_TypeDef* | 指向 USART1/USART2 等寄存器结构体 |
| Init | UART_InitTypeDef | 包含波特率、数据位、校验位等配置 |
| pRxBuffPtr | uint8_t* | 接收数据缓冲区地址 |
| RxXferSize | uint16_t | 期望接收的数据长度 |
| RxXferCount | uint16_t | 剩余待接收数据计数 |
| hdmarx | DMA_HandleTypeDef* | DMA 接收句柄（用于 DMA 模式） |

---

## 6. 完整数据流分析

### 6.1 本项目的 DMA + IDLE 中断接收流程

```
PC发送: 0x01 0x02 0x03 0x04 0x05 (共5字节)
                    ↓
            USART1 RX 引脚
                    ↓
            DMA 自动搬运到缓冲区
                    ↓
         [DMA CNDTR: 5 → 4 → 3 → 2 → 1 → 0]
                    ↓
            最后一字节接收完成
                    ↓
         RX 线变为空闲 (高电平)
                    ↓
         等待 1 帧时间后
                    ↓
         USART_SR.IDLE 置位
                    ↓
         USART1_IRQHandler 被调用
                    ↓
    检查: if (__HAL_UART_GET_FLAG(..., UART_FLAG_IDLE))
                    ↓
         清除 IDLE 标志
         (读取 SR + 读取 DR)
                    ↓
         uart_dmarx_idle_callback() 处理数据
                    ↓
         重启 DMA 准备下一帧
```

### 6.2 关键代码位置

```c
// Drivers/CSP/UART_STM32F1xx.c:252-259
void USART1_IRQHandler(void) {
    // 第1步: 检查 IDLE 标志是否置位
    if (__HAL_UART_GET_FLAG(&usart1_handle, UART_FLAG_IDLE)) {
        // 第2步: 清除 IDLE 标志（必须！）
        __HAL_UART_CLEAR_IDLEFLAG(&usart1_handle);
        // 第3步: 处理接收完成
        uart_dmarx_idle_callback(&usart1_handle);
    }
    // 第4步: 调用 HAL 库处理其他中断
    HAL_UART_IRQHandler(&usart1_handle);
}
```

---

## 7. 常见问题与注意事项

### 7.1 IDLE 中断不触发？

1. **检查是否使能了 IDLE 中断**
   ```c
   __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
   ```

2. **检查是否使能了 USART**
   ```c
   __HAL_UART_ENABLE(&huart1);
   ```

3. **检查 DMA 是否启动**
   ```c
   HAL_UART_Receive_DMA(&huart1, rx_buffer, BUFFER_SIZE);
   ```

### 7.2 IDLE 标志清除失败？

必须**同时读取 SR 和 DR**：
```c
// 错误方式 - 标志不会清除
__HAL_UART_CLEAR_IDLEFLAG(&huart1);  // 只读取 SR

// 正确方式
uint32_t tmp = huart1.Instance->SR;  // 读取 SR
tmp = huart1.Instance->DR;             // 读取 DR，标志清除
```

### 7.3 IDLE 与 DMA 的配合

- DMA 用于**批量接收数据**
- IDLE 用于**检测一帧数据接收完成**
- 两者配合可以实现**不定长数据接收**

---

## 8. 总结

| 概念 | 关键点 |
|------|--------|
| IDLE 中断 | RX 线空闲 1 帧时间后触发 |
| __HAL_UART_GET_FLAG | 读取 SR 寄存器并与标志位做 & 运算 |
| __HAL_UART_CLEAR_IDLEFLAG | 依次读取 SR 和 DR 寄存器 |
| USART_SR.IDLE | bit 4，值为 0x10 |
| 应用场景 | 不定长数据帧接收完成检测 |

---

*文档创建日期: 2026-02-26*
*基于 STM32F1xx HAL 库分析*
