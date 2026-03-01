Serial Peripheral interface 缩写，串行外围设备接口，是一种高速的全双工同步的通信总线，已经广泛应用在众多 MCU、存储芯片、AD 转换器和 LCD 之间。

SPI的引脚信息：

MISO（Master In / Slave Out）主设备数据输入，从设备数据输出。
MOSI（Master Out / Slave In）主设备数据输出，从设备数据输入。
SCLK（Serial Clock）时钟信号，由主设备产生。
CS（Chip Select）从设备片选信号，由主设备产生。

![](Pictures/SPI%20FLASH/file-20260214220149187.png)

正点原子给出的代码：
spi.h

```c
/* SPI1 引脚 定义 */
#define SPI1_SCK_GPIO_PORT GPIOA
#define SPI1_SCK_GPIO_PIN GPIO_PIN_5
#define SPI1_SCK_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define SPI1_MISO_GPIO_PORT GPIOA
#define SPI1_MISO_GPIO_PIN GPIO_PIN_6
#define SPI1_MISO_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define SPI1_MOSI_GPIO_PORT GPIOA
#define SPI1_MOSI_GPIO_PIN GPIO_PIN_7
#define SPI1_MOSI_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
/* SPI1 相关定义 */
#define SPI1_SPI SPI1
#define SPI1_SPI_CLK_ENABLE() do{ __HAL_RCC_SPI1_CLK_ENABLE(); }while(0)

```

通过宏定义标识符的方式去定义 SPI 通信用到的三个管脚 SCK、MISO 和 MOSI，同时还宏定义了 SPI1 的相关信息

spi.c
```c
/**
* @brief SPI 初始化代码
* @note 主机模式,8 位数据,禁止硬件片选
* @param 无
* @retval 无
*/
SPI_HandleTypeDef g_spi1_handler; /* SPI1 句柄 */
void spi1_init(void){
    SPI1_SPI_CLK_ENABLE(); /* 开启 SPI1 外设的门控时钟，不开启则无法操作寄存器 */
    g_spi1_handler.Instance = SPI1_SPI; /* 指向 SPI1 的基地址 */
    g_spi1_handler.Init.Mode = SPI_MODE_MASTER; /* 主机模式，STM32 掌握话语权，产生时钟 */
    g_spi1_handler.Init.Direction = SPI_DIRECTION_2LINES;  /* 双线模式 */
    g_spi1_handler.Init.DataSize = SPI_DATASIZE_8BIT; /* SPI 发送接收 8 位帧结构 */

    g_spi1_handler.Init.CLKPolarity = SPI_POLARITY_HIGH; /* CPOL=1，空闲时时钟线为高电平 */
    g_spi1_handler.Init.CLKPhase = SPI_PHASE_2EDGE;      /* CPHA=1，在时钟的第二个跳变沿采样数据 */

    g_spi1_handler.Init.NSS = SPI_NSS_SOFT; /* 不使用硬件自动控制 CS 引脚，改由代码手动拉低/拉高 */

    g_spi1_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; /* 初始频率设为最低（256分频） */
    g_spi1_handler.Init.FirstBit = SPI_FIRSTBIT_MSB; /* 高位先行，符合大多数传感器的通信协议 */

    g_spi1_handler.Init.TIMode = SPI_TIMODE_DISABLE; /* 禁用 TI 专有模式 */
    g_spi1_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; /* 禁用硬件循环冗余校验 */

    HAL_SPI_Init(&g_spi1_handler);      /* 将以上参数写入硬件寄存器 */
    __HAL_SPI_ENABLE(&g_spi1_handler);  /* 正式开启 SPI 模块 */

    spi1_read_write_byte(0Xff); /* 发送 dummy byte，用于清理状态寄存器，确保初次通信正常 */
}
```
以上两个函数实现初始化SPI1，并使能 SPI1 ，接下来发送和接收信息
```c
/**
* @brief SPI1 读写一个字节数据
* @param txdata : 要发送的数据(1 字节)
* @retval 接收到的数据(1 字节)
*/
uint8_t spi1_read_write_byte(uint8_t txdata){
    uint8_t rxdata;
    HAL_SPI_TransmitReceive(&g_spi1_handler, &txdata, &rxdata, 1, 100);
    return rxdata;
}
```
spi_read_write_byte 函数直接调用了 HAL 库内置的函数进行接收发送操作
由于不同的外设需要的通信速度不一样，所以这里我们还需要定义一个速度设置函数

```c
void spi1_set_speed(uint8_t speed){
    assert_param(IS_SPI_BAUDRATE_PRESCALER(speed));  /* 判断有效性 */
    __HAL_SPI_DISABLE(&g_spi1_handler);
    g_spi1_handler.Instance -> CR1 &= 0XFFC7;        /* 位3-5清零，设置波特率 */
    g_spi1_handler.Instance -> CR1 |= speed << 3;
    __HAL_SPI_ENABLE(&g_spi1_handler);
}
```


为什么norflash_read_id函数需要写三个0？

0x90 是 JEDEC ID 读取命令，其协议格式为：
| 字节 | 内容 | 说明 |
|------|------|------|
| 1 | 0x90 | 命令 |
| 2 | 0x00 | dummy address |
| 3 | 0x00 | dummy address |
| 4 | 0x00 | dummy address |
| 5 | - | 返回厂商ID (Winbond=0xEF) |
| 6 | - | 返回设备ID (W25Q64=0x16) |
SPI 是全双工通信，发送和接收同时进行。发送命令后必须发送 dummy bytes 来产生时钟信号，Flash 才能返回数据。
所以那三个 0 是地址占位符，用来产生时钟读取厂商ID和设备ID。

