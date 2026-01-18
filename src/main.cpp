/**
 * @file main.cpp
 * @brief ESP32 I2C 桥接程序 - 连接上位机与 TS20 触摸芯片
 *
 * @description
 * 本程序实现 ESP32 作为 I2C 桥接器的功能：
 *   - 作为 I2C 从机：接收上位机的命令和数据请求
 *   - 作为 I2C 主机：与下游 TS20 触摸芯片通信
 *
 * 通信架构：
 *   上位机 <--I2C从机--> ESP32 <--I2C主机--> TS20触摸芯片
 *
 * 主要功能：
 *   1. 接收上位机指令，初始化 TS20 芯片的寄存器配置
 *   2. 周期性读取 TS20 的触摸状态寄存器 (0x20, 0x21, 0x22)
 *   3. 响应上位机的读取请求，返回触摸状态数据
 *
 * @author julen
 * @date 2026-01-18
 * @version 1.0
 */

#include <Arduino.h>
#include <Wire.h>

// ==================== 硬件配置参数 ====================

/**
 * I2C 从机配置 (ESP32 作为从机，连接上位机)
 * 使用 ESP32 默认的 I2C 引脚
 */
#define I2C_SLAVE_SDA       21      // GPIO21 - ESP32 默认 I2C SDA 引脚
#define I2C_SLAVE_SCL       22      // GPIO22 - ESP32 默认 I2C SCL 引脚
#define I2C_SLAVE_ADDR      0x7A    // ESP32 作为从机的 I2C 地址

/**
 * I2C 主机配置 (ESP32 作为主机，连接 TS20 触摸芯片)
 * 使用独立的 GPIO 引脚，避免与从机冲突
 */
#define I2C_MASTER_SDA      16      // GPIO16 - 主机 SDA 引脚
#define I2C_MASTER_SCL      17      // GPIO17 - 主机 SCL 引脚
#define I2C_TARGET_ADDR     0x7A    // TS20 芯片 I2C 地址 (ADD引脚: GND=0x6A, VDD=0x7A)

/**
 * I2C 通信速率配置
 */
#define I2C_MASTER_FREQ     100000   // 主机 I2C 频率: 100kHz (适配 TS20 芯片)

#define READ_BUFFER_SIZE    3       // 要从ts20读取数据的大小

// ==================== 全局变量 ====================

/**
 * I2C 总线实例
 * ESP32 支持两个独立的 I2C 硬件控制器 (I2C0 和 I2C1)
 */
auto I2C_Master = TwoWire(0); // I2C0: 主机模式，用于与 TS20 通信
auto I2C_Slave = TwoWire(1); // I2C1: 从机模式，用于响应上位机

/**
 * 数据缓冲区
 * 使用 volatile 关键字，因为数据会在中断回调中被修改
 */
volatile uint8_t readBuffer[READ_BUFFER_SIZE]; // 从 TS20 读取数据的临时缓冲区

/**
 * TS20 触摸状态寄存器缓存
 * 这三个寄存器存储了 TS20 的触摸检测状态
 * - 0x20: 触摸状态寄存器1 (通道 0-7 的触摸状态)
 * - 0x21: 触摸状态寄存器2 (通道 8-15 的触摸状态)
 * - 0x22: 触摸状态寄存器3 (通道 16-20 的触摸状态)
 */
volatile uint8_t ts20_reg20 = 0; // 寄存器 0x20 缓存值
volatile uint8_t ts20_reg21 = 0; // 寄存器 0x21 缓存值
volatile uint8_t ts20_reg22 = 0; // 寄存器 0x22 缓存值
volatile uint8_t ts20_now_read_reg = 0; // 上位机当前请求读取的寄存器地址

/**
 * 状态控制标志
 */
volatile bool start_read_status = false; // 是否开始读取 TS20 状态 (上位机发起读请求后置 true)
bool maybe_need_init_ts20 = false; // 是否需要初始化 TS20 (上位机发送写命令后置 true)
bool is_called_init = false; // TS20 是否已完成初始化

/**
 * TS20 触摸芯片初始化配置数据
 * 格式: {寄存器地址, 寄存器值}
 *
 * 寄存器说明 (参考 TS20 规格书):
 * - 0x00-0x0A: Sensitivity/PWM1-11 灵敏度/PWM控制寄存器 (每通道4位)
 * - 0x0B: GTRL1 通用控制寄存器1 (响应时间、首次触摸控制、工作模式等)
 * - 0x0C: GTRL2 通用控制寄存器2 (软件复位、时钟关闭、阻抗选择等)
 * - 0x0D: Cal_CTRL 校准速度控制寄存器
 * - 0x0E: Port_CTRL1 端口控制寄存器1 (CH1-CH4)
 * - 0x0F: Port_CTRL2 端口控制寄存器2 (CH5-CH7)
 * - 0x10: Port_CTRL3 端口控制寄存器3 (CH8-CH11)
 * - 0x11: Port_CTRL4 端口控制寄存器4 (CH12-CH15)
 * - 0x12: Port_CTRL5 端口控制寄存器5 (CH16-CH19)
 * - 0x13: Port_CTRL6 端口控制寄存器6 (CH20)
 */
const uint8_t ts20_init_data[][2] = {
    {0x0C, 0x9A}, // GTRL2: 软件复位 (SRST=1, RB_SEL=10 正常频率)
    {0x0C, 0x92}, // GTRL2: 正常工作模式 (SRST=0, IMP_SEL=1 高阻抗, RB_SEL=10)
    {0x0E, 0x00}, // Port_CTRL1: CH1-CH4 设为触摸感应模式 (00=Sense)
    {0x0F, 0x00}, // Port_CTRL2: CH5-CH7 设为触摸感应模式 (00=Sense)
    {0x10, 0x50}, // Port_CTRL3: CH8-CH11 端口控制 (CH10=01保持, 其他=00感应)
    {0x11, 0x55}, // Port_CTRL4: CH12-CH15 设为通道保持模式 (01=Channel Hold)
    {0x12, 0x55}, // Port_CTRL5: CH16-CH19 设为通道保持模式 (01=Channel Hold)
    {0x13, 0x01}, // Port_CTRL6: CH20 设为通道保持模式 (01=Channel Hold)
    {0x00, 0x55}, // Sensitivity/PWM1: CH1/CH2 灵敏度=5 (中等灵敏度)
    {0x01, 0x55}, // Sensitivity/PWM2: CH3/CH4 灵敏度=5
    {0x02, 0x55}, // Sensitivity/PWM3: CH5/CH6 灵敏度=5
    {0x03, 0x55}, // Sensitivity/PWM4: CH7 灵敏度=5 (高4位保留建议设为0xF)
    {0x04, 0x55}, // Sensitivity/PWM5: CH8/CH9 灵敏度=5
    {0x05, 0x55}, // Sensitivity/PWM6: CH10/CH11 灵敏度=5
    {0x06, 0x55}, // Sensitivity/PWM7: CH12/CH13 灵敏度=5
    {0x07, 0x55}, // Sensitivity/PWM8: CH14/CH15 灵敏度=5
    {0x08, 0x55}, // Sensitivity/PWM9: CH16/CH17 灵敏度=5
    {0x09, 0x55}, // Sensitivity/PWM10: CH18/CH19 灵敏度=5
    {0x0A, 0x55} // Sensitivity/PWM11: CH20 灵敏度=5 (高4位无效)
};

// ==================== I2C 从机回调函数 ====================

/**
 * @brief I2C 从机接收回调函数
 * @param numBytes 接收到的字节数
 *
 * 当上位机向 ESP32 发送数据时触发此回调
 *
 * 通信协议:
 * - 接收 1 字节: 上位机请求读取某个寄存器 (该字节为寄存器地址)
 * - 接收 >1 字节: 上位机发送写入命令 (通常是初始化 TS20 的信号)
 */
void onReceiveCallback(const int numBytes) {
    if (numBytes == 1) {
        // 单字节: 上位机设置要读取的寄存器地址
        start_read_status = true;
        if (I2C_Slave.available()) {
            ts20_now_read_reg = I2C_Slave.read();
        }
    } else if (numBytes > 1) {
        // 多字节: 上位机发送写命令，标记需要初始化 TS20
        start_read_status = false;
        maybe_need_init_ts20 = true;
        I2C_Slave.flush(); // 清空接收缓冲区
    }
}

/**
 * @brief I2C 从机请求回调函数
 *
 * 当上位机请求读取数据时触发此回调
 * 根据 ts20_now_read_reg 返回对应的 TS20 寄存器缓存值
 */
void onRequestCallback() {
    uint8_t val = 0;

    // 根据请求的寄存器地址返回对应的缓存值
    switch (ts20_now_read_reg) {
        case 0x20:
            val = ts20_reg20; // 触摸状态寄存器1
            break;
        case 0x21:
            val = ts20_reg21; // 触摸状态寄存器2
            break;
        case 0x22:
            val = ts20_reg22; // 触摸状态寄存器3
            break;
        default:
            val = 0; // 未知寄存器返回 0
            break;
    }

    I2C_Slave.write(val);
}

// ==================== 辅助函数 ====================

/**
 * @brief 初始化 TS20 触摸芯片寄存器
 *
 * 按照 ts20_init_data 数组中定义的配置，
 * 依次向 TS20 写入寄存器初始化值
 */
void init_ts20_registers() {
    Serial.println("初始化 TS20 寄存器...");

    for (const auto &i: ts20_init_data) {
        I2C_Master.beginTransmission(I2C_TARGET_ADDR);
        I2C_Master.write(i, sizeof(i)); // 写入 [寄存器地址, 寄存器值]
        I2C_Master.endTransmission(true); // 发送停止位
        Serial.printf("  写入寄存器 0x%02X: 0x%02X\n", i[0], i[1]);
        delay(10); // 等待 TS20 处理完成
    }

    Serial.println("TS20 寄存器初始化完成。\n");
}

/**
* @brief 读取并更新 TS20 触摸芯片状态
*
* 从 TS20 的寄存器 0x20-0x22 连续读取触摸状态数据，
* 进行有效性过滤后更新全局缓存变量。
*/
bool get_and_update_ts20_status() {
    // 向 TS20 发送寄存器地址 0x20，准备连续读取 READ_BUFFER_SIZE 个寄存器
    I2C_Master.beginTransmission(I2C_TARGET_ADDR);
    I2C_Master.write(0x20); // 起始寄存器地址: 0x20
    auto ret = I2C_Master.endTransmission(false); // 发送重复起始条件 (不释放总线)
    if (ret != 0) {
        return false;
    }

    // 请求读取 READ_BUFFER_SIZE 字节数据 (0x20, 0x21, 0x22)
    ret = I2C_Master.requestFrom(static_cast<uint8_t>(I2C_TARGET_ADDR), static_cast<uint8_t>(READ_BUFFER_SIZE));
    if (ret != READ_BUFFER_SIZE) {
        return false;
    }

    // 读取返回的数据
    uint8_t idx = 0;
    while (idx < READ_BUFFER_SIZE && I2C_Master.available()) {
        readBuffer[idx++] = I2C_Master.read();
    }

    // ===== 步骤3: 更新寄存器缓存 =====
    if (idx != READ_BUFFER_SIZE) {
        return false;
    }
    // readBuffer[0] readBuffer[1] readBuffer[2]的值
    // 04 00 00 表示“开关”触摸按键
    // 20 00 00 表示“经济”触摸按键
    // 00 01 00 表示“风速+”触摸按键
    // 10 00 00 表示“风速-”触摸按键

    // 只有上面的值才是有效值，过滤出有效数据
    // readBuffer[0] -> 04 | 20 | 00 | 10 = 34
    // readBuffer[1] -> 00 | 00 | 01 | 00 = 01
    // readBuffer[2] -> 00 | 00 | 00 | 00 = 00
    if (0 != (readBuffer[0] & ~0x34) || 0 != (readBuffer[1] & ~0x01) || 0 != readBuffer[2]) {
        // 非法数据，直接丢弃
        return false;
    }

    if (readBuffer[0] != 0 || readBuffer[1] != 0) {
        Serial.print("TS20 触摸状态: ");
        Serial.printf("read_buff:%02X %02X\n", readBuffer[0], readBuffer[1]);
    }

    // 注意: 这里如果按照正常的顺序赋值，反而在上位机中看到的数据顺序是错位的
    // 问题肯定不是ts20芯片，因为上位机和ts20直接通信时是正常的
    // 这里认为的调整一下顺序: [0x22的值, 0x20的值, 0x21的值]
    // 上位机 0x20 -> readBuffer[1]
    // 上位机 0x21 -> readBuffer[2]
    // 上位机 0x22 -> readBuffer[0]
    // TODO: 后续需要进一步排查原因
    ts20_reg22 = readBuffer[0];
    ts20_reg20 = readBuffer[1];
    ts20_reg21 = readBuffer[2];

    return true;
}

// ==================== 主程序 ====================

/**
 * @brief Arduino 初始化函数
 *
 * 执行以下初始化操作:
 * 1. 初始化串口调试输出
 * 2. 打印硬件配置信息
 * 3. 初始化 I2C 从机 (响应上位机)
 * 4. 初始化 I2C 主机 (控制 TS20)
 */
void setup() {
    // 初始化串口，用于调试输出
    Serial.begin(115200);

    // 打印配置信息
    Serial.println("\n【配置信息】");
    Serial.print("  ESP32 从机地址:  0x");
    Serial.print(I2C_SLAVE_ADDR, HEX);
    Serial.print("  (SDA=GPIO");
    Serial.print(I2C_SLAVE_SDA);
    Serial.print(", SCL=GPIO");
    Serial.print(I2C_SLAVE_SCL);
    Serial.println(")");

    Serial.print("  目标芯片地址:    0x");
    Serial.print(I2C_TARGET_ADDR, HEX);
    Serial.print("  (SDA=GPIO");
    Serial.print(I2C_MASTER_SDA);
    Serial.print(", SCL=GPIO");
    Serial.print(I2C_MASTER_SCL);
    Serial.println(")");

    // 初始化 I2C 从机：注册回调函数并启动
    I2C_Slave.onReceive(onReceiveCallback); // 注册接收回调
    I2C_Slave.onRequest(onRequestCallback); // 注册请求回调
    auto ret = I2C_Slave.begin((uint8_t) I2C_SLAVE_ADDR, I2C_SLAVE_SDA, I2C_SLAVE_SCL, 0);
    Serial.printf("I2C 从机初始化完成:%s\n", ret ? "成功" : " [失败！]");

    // 初始化 I2C 主机：配置引脚和通信频率
    ret = I2C_Master.begin(I2C_MASTER_SDA, I2C_MASTER_SCL, I2C_MASTER_FREQ);
    Serial.printf("I2C 主机初始化完成:%s\n", ret ? "成功" : " [失败！]");


    Serial.println("\n────────── 等待通信 ──────────\n");
}

/**
 * @brief Arduino 主循环函数
 *
 * 主循环执行以下任务:
 * 1. 检查是否需要初始化 TS20 (收到上位机写命令后触发)
 * 2. 如果上位机请求读取状态，则从 TS20 读取触摸寄存器
 * 3. 更新寄存器缓存，供上位机读取
 *
 * 循环周期: 约 100ms
 */
void loop() {
    // ===== 步骤1: 检查并执行 TS20 初始化 =====
    if (maybe_need_init_ts20 && !is_called_init) {
        init_ts20_registers();
        is_called_init = true;
        maybe_need_init_ts20 = false;
    }

    // ===== 步骤2: 读取 TS20 触摸状态 =====
    if (start_read_status) {
        is_called_init = false; // 重置初始化标志，允许下次重新初始化
        get_and_update_ts20_status();
    } else {
        // 未开始读取时，清零缓存
        ts20_reg20 = 0;
        ts20_reg21 = 0;
        ts20_reg22 = 0;
    }

    delay(100); // 主循环延时，降低 CPU 占用
}
