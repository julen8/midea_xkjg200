#include <Arduino.h>
#include <Wire.h>

// ==================== 硬件配置参数 ====================

/**
 * I2C 从机配置 (ESP32 作为从机，连接上位机)
 * 使用 ESP32 默认的 I2C 引脚
 */
#define I2C_SLAVE_SDA  21    // GPIO21 - ESP32 默认 I2C SDA 引脚
#define I2C_SLAVE_SCL  22    // GPIO22 - ESP32 默认 I2C SCL 引脚
#define I2C_SLAVE_ADDR 0x7A  // ESP32 作为从机的 I2C 地址

/**
 * I2C 主机配置 (ESP32 作为主机，连接 TS20 触摸芯片)
 * 使用独立的 GPIO 引脚，避免与从机冲突
 */
#define I2C_MASTER_SDA  16    // GPIO16 - 主机 SDA 引脚
#define I2C_MASTER_SCL  17    // GPIO17 - 主机 SCL 引脚
#define I2C_TARGET_ADDR 0x7A  // TS20 芯片 I2C 地址 (ADD引脚: GND=0x6A, VDD=0x7A)

/**
 * 风速状态输入引脚配置
 * 外部输入低电平有效
 */
#define PIN_FAN_SPEED_LOW  25  // GPIO25 - 低风速状态输入引脚
#define PIN_FAN_SPEED_MID  26  // GPIO26 - 中风速状态输入引脚
#define PIN_FAN_SPEED_HIGH 27  // GPIO27 - 高风速状态输入引脚

/**
 * I2C 通信速率配置
 */
#define I2C_MASTER_FREQ 100000  // 主机 I2C 频率: 100kHz (适配 TS20 芯片)

#define READ_BUFFER_SIZE 3  // 要从ts20读取数据的大小

// ==================== 全局变量 ====================

/**
 * I2C 总线实例
 * ESP32 支持两个独立的 I2C 硬件控制器 (I2C0 和 I2C1)
 */
auto I2C_Master = TwoWire(0);  // I2C0: 主机模式，用于与 TS20 通信
auto I2C_Slave  = TwoWire(1);  // I2C1: 从机模式，用于响应上位机

/**
 * 数据缓冲区
 * 使用 volatile 关键字，因为数据会在中断回调中被修改
 */
volatile uint8_t readBuffer[READ_BUFFER_SIZE];  // 从 TS20 读取数据的临时缓冲区

/**
 * TS20 触摸状态寄存器缓存
 * 这三个寄存器存储了 TS20 的触摸检测状态
 * - 0x20: 触摸状态寄存器1 (通道 0-7 的触摸状态)
 * - 0x21: 触摸状态寄存器2 (通道 8-15 的触摸状态)
 * - 0x22: 触摸状态寄存器3 (通道 16-20 的触摸状态)
 */
volatile uint8_t ts20_reg20          = 0;  // 寄存器 0x20 缓存值
volatile uint8_t ts20_reg21          = 0;  // 寄存器 0x21 缓存值
volatile uint8_t ts20_reg22          = 0;  // 寄存器 0x22 缓存值
volatile uint8_t ts20_now_read_reg   = 0;  // 上位机当前请求读取的寄存器地址
volatile uint8_t slave_write_counter = 0;  // 从机接收计数器

/**
 * 状态控制标志
 */
volatile bool start_read_status    = false;  // 是否开始读取 TS20 状态 (上位机发起读请求后置 true)
volatile bool maybe_need_init_ts20 = false;  // 是否需要初始化 TS20 (上位机发送写命令后置 true)

/**
 * 风扇按键代码定义
 */
enum class FanKey : uint8_t {
    POWER          = 0,  // 开关按键
    ECONOMY        = 1,  // 经济按键
    FAN_SPEED_UP   = 2,  // 风速+
    FAN_SPEED_DOWN = 3,  // 风速-
};

/**
 * 风速状态
 * 没有FAN_SPEED_ON,当风速有任意档位时表示风扇已开启
 */
enum class FanSpeedState : uint8_t {
    FAN_SPEED_OFF = 0,
    FAN_SPEED_LOW,
    FAN_SPEED_MID,
    FAN_SPEED_HIGH,
};

volatile auto fan_speed_state = FanSpeedState::FAN_SPEED_OFF;  // 当前风速状态

/**
 * 风扇控制状态
 * 多了一个FAN_SPEED_ON,表示开启风扇但不改变当前风速档位
 */
enum class FanControlState : uint8_t {
    FAN_SPEED_OFF = 0,
    FAN_SPEED_LOW,
    FAN_SPEED_MID,
    FAN_SPEED_HIGH,
    FAN_SPEED_ON,
};

volatile bool ts20_is_called_init = false;  // TS20 是否已完成初始化

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
constexpr uint8_t ts20_init_data[][2] = {
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
    {0x0A, 0x55}, // Sensitivity/PWM11: CH20 灵敏度=5 (高4位无效)
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
        start_read_status    = false;
        maybe_need_init_ts20 = true;
        I2C_Slave.flush();  // 清空接收缓冲区
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
            val = ts20_reg20;  // 触摸状态寄存器1
            break;
        case 0x21:
            val = ts20_reg21;  // 触摸状态寄存器2
            break;
        case 0x22:
            val = ts20_reg22;  // 触摸状态寄存器3
            break;
        default:
            val = 0;  // 未知寄存器返回 0
            break;
    }

    I2C_Slave.write(val);
    ++slave_write_counter;
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

    for (const auto &i : ts20_init_data) {
        I2C_Master.beginTransmission(I2C_TARGET_ADDR);
        I2C_Master.write(i, sizeof(i));    // 写入 [寄存器地址, 寄存器值]
        I2C_Master.endTransmission(true);  // 发送停止位
        Serial.printf("  写入寄存器 0x%02X: 0x%02X\n", i[0], i[1]);
        delay(10);  // 等待 TS20 处理完成
    }

    Serial.println("TS20 寄存器初始化完成。\n");
}

void update_ts20_status_cache() {
    if (readBuffer[0] != 0 || readBuffer[1] != 0) {
        Serial.print("TS20 触摸状态: ");
        Serial.printf("read_buff:%02X %02X\n", readBuffer[0], readBuffer[1]);
    }

    // 注意: 这里如果按照正常的顺序赋值，反而在上位机中看到的数据顺序是错位的
    // 问题肯定不是ts20芯片，因为上位机和ts20直接通信时是正常的
    // 这里人为的调整一下顺序: [0x22的值, 0x20的值, 0x21的值]
    // 上位机 0x20 -> readBuffer[1]
    // 上位机 0x21 -> readBuffer[2]
    // 上位机 0x22 -> readBuffer[0]
    // TODO: 后续需要进一步排查原因
    ts20_reg22 = readBuffer[0];
    ts20_reg20 = readBuffer[1];
    ts20_reg21 = readBuffer[2];
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
    I2C_Master.write(0x20);                        // 起始寄存器地址: 0x20
    auto ret = I2C_Master.endTransmission(false);  // 发送重复起始条件 (不释放总线)
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

    update_ts20_status_cache();
    return true;
}

/**
 * @brief 模拟按下风扇按键
 * @param key_code 要模拟按下的按键代码
 *
 * 通过修改 readBuffer 模拟按键按下和释放的过程，
 * 并更新 TS20 状态缓存。
 */
void simulate_key_press(const FanKey key_code) {
    // readBuffer[0] readBuffer[1] readBuffer[2]的值
    // 04 00 00 表示“开关”触摸按键
    // 20 00 00 表示“经济”触摸按键
    // 00 01 00 表示“风速+”触摸按键
    // 10 00 00 表示“风速-”触摸按键
    switch (key_code) {
        case FanKey::POWER:
            readBuffer[0] = 0x04;
            readBuffer[1] = 0x00;
            readBuffer[2] = 0x00;
            break;
        case FanKey::ECONOMY:
            readBuffer[0] = 0x20;
            readBuffer[1] = 0x00;
            readBuffer[2] = 0x00;
            break;
        case FanKey::FAN_SPEED_UP:
            readBuffer[0] = 0x00;
            readBuffer[1] = 0x01;
            readBuffer[2] = 0x00;
            break;
        case FanKey::FAN_SPEED_DOWN:
            readBuffer[0] = 0x10;
            readBuffer[1] = 0x00;
            readBuffer[2] = 0x00;
            break;
    }

    update_ts20_status_cache();
    // 保持按键状态一段时间
    slave_write_counter = 0;
    // 这里等待从机写回调（3个寄存器值，最少能够完整的读取3次），确保上位机读取到按键信息
    // 最多等待400ms
    for (auto i = 0; i < 40 && slave_write_counter < 3 * 3; ++i) {
        delay(10);
    }

    // 释放按键，清空状态
    readBuffer[0] = 0x00;
    readBuffer[1] = 0x00;
    readBuffer[2] = 0x00;
    update_ts20_status_cache();
    slave_write_counter = 0;
    // 这里等待从机写回调（3个寄存器值，最少能够完整的读取3次），确保上位机读取到按键信息
    // 最多等待400ms
    for (auto i = 0; i < 40 && slave_write_counter < 3 * 3; ++i) {
        delay(10);
    }
}

/**
 * @brief 读取并更新风速状态
 *
 * 读取三个风速状态输入引脚 (低电平有效)，
 * 更新全局风速状态变量，并在状态变化时输出调试信息。
 */
void read_fan_speed_status() {
    // 保存上一次的状态，用于检测变化
    static auto last_fan_speed = FanSpeedState::FAN_SPEED_OFF;

    // 读取引脚状态 (低电平有效，所以取反)
    if (digitalRead(PIN_FAN_SPEED_LOW) == LOW) {
        fan_speed_state = FanSpeedState::FAN_SPEED_LOW;
    } else if (digitalRead(PIN_FAN_SPEED_MID) == LOW) {
        fan_speed_state = FanSpeedState::FAN_SPEED_MID;
    } else if (digitalRead(PIN_FAN_SPEED_HIGH) == LOW) {
        fan_speed_state = FanSpeedState::FAN_SPEED_HIGH;
    } else {
        fan_speed_state = FanSpeedState::FAN_SPEED_OFF;
    }

    // 检测状态是否发生变化
    if (last_fan_speed != fan_speed_state) {
        // 状态发生变化，输出调试信息
        Serial.print("风速状态变化: ");
        switch (fan_speed_state) {
            case FanSpeedState::FAN_SPEED_OFF:
                Serial.print("关闭");
                break;
            case FanSpeedState::FAN_SPEED_LOW:
                Serial.print("低风速");
                break;
            case FanSpeedState::FAN_SPEED_MID:
                Serial.print("中风速");
                break;
            case FanSpeedState::FAN_SPEED_HIGH:
                Serial.print("高风速");
                break;
        }
        Serial.println();

        // 更新上一次状态
        last_fan_speed = fan_speed_state;
    }
}

/**
 * @brief 设置风扇风速
 * @param speed 目标风速状态
 * @param retryCount 重试次数
 *
 * 根据目标风速状态，模拟按键操作调整风扇风速。
 * 如果设置失败，则根据重试次数进行重试。
 */
void set_fan_speed(const FanControlState speed, const uint8_t retryCount = 3) {
    if (speed != FanControlState::FAN_SPEED_OFF && fan_speed_state == FanSpeedState::FAN_SPEED_OFF) {
        Serial.println("开启风扇");
        simulate_key_press(FanKey::POWER);
    }

    read_fan_speed_status();
    switch (speed) {
        case FanControlState::FAN_SPEED_OFF:
            // 控制代码: 关闭风扇
            if (fan_speed_state != FanSpeedState::FAN_SPEED_OFF) {
                Serial.println("关闭风扇");
                simulate_key_press(FanKey::POWER);
            }
            break;
        case FanControlState::FAN_SPEED_LOW:
            // 控制代码: 设置低风速
            if (fan_speed_state != FanSpeedState::FAN_SPEED_LOW) {
                Serial.println("设置低风速");
                // 经济模式会设置为低风速
                simulate_key_press(FanKey::ECONOMY);
            }
            break;
        case FanControlState::FAN_SPEED_MID:
            // 控制代码: 设置中风速
            if (fan_speed_state == FanSpeedState::FAN_SPEED_LOW) {
                Serial.println("设置中风速");
                simulate_key_press(FanKey::FAN_SPEED_UP);
            } else if (fan_speed_state == FanSpeedState::FAN_SPEED_HIGH) {
                Serial.println("设置中风速");
                simulate_key_press(FanKey::FAN_SPEED_DOWN);
            }
            break;
        case FanControlState::FAN_SPEED_HIGH:
            // 控制代码: 设置高风速
            if (fan_speed_state == FanSpeedState::FAN_SPEED_LOW) {
                Serial.println("设置高风速");
                simulate_key_press(FanKey::FAN_SPEED_UP);
                simulate_key_press(FanKey::FAN_SPEED_UP);
            }
            if (fan_speed_state == FanSpeedState::FAN_SPEED_MID) {
                Serial.println("设置高风速");
                simulate_key_press(FanKey::FAN_SPEED_UP);
            }
            break;
        case FanControlState::FAN_SPEED_ON:
            // 上面已经处理过开启风扇的逻辑，这里不需要额外操作
            break;
    }

    // 验证设置结果
    read_fan_speed_status();
    if (speed == FanControlState::FAN_SPEED_ON) {
        if (fan_speed_state == FanSpeedState::FAN_SPEED_OFF) {
            Serial.printf("风速设置失败，重试中... (剩余重试次数: %d)\n", retryCount - 1);
            if (retryCount > 0) {
                set_fan_speed(speed, retryCount - 1);
            }
        }
    } else if (static_cast<uint8_t>(fan_speed_state) != static_cast<uint8_t>(speed)) {
        Serial.printf("风速设置失败，重试中... (剩余重试次数: %d)\n", retryCount - 1);
        if (retryCount > 0) {
            set_fan_speed(speed, retryCount - 1);
        }
    }
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
    // 最优先级别：初始化 I2C 从机
    // 初始化 I2C 从机：注册回调函数并启动
    I2C_Slave.onReceive(onReceiveCallback);  // 注册接收回调
    I2C_Slave.onRequest(onRequestCallback);  // 注册请求回调
    auto ret = I2C_Slave.begin(I2C_SLAVE_ADDR, I2C_SLAVE_SDA, I2C_SLAVE_SCL, 0);

    // 初始化风速状态输入引脚 (低电平有效，使用内部上拉)
    pinMode(PIN_FAN_SPEED_LOW, INPUT_PULLUP);
    pinMode(PIN_FAN_SPEED_MID, INPUT_PULLUP);
    pinMode(PIN_FAN_SPEED_HIGH, INPUT_PULLUP);

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

    Serial.printf("I2C 从机初始化完成:%s\n", ret ? "成功" : " [失败！]");

    // 初始化 I2C 主机：配置引脚和通信频率
    ret = I2C_Master.begin(I2C_MASTER_SDA, I2C_MASTER_SCL, I2C_MASTER_FREQ);
    Serial.printf("I2C 主机初始化完成:%s\n", ret ? "成功" : " [失败！]");

    Serial.println("\n【风速状态引脚】");
    Serial.printf("  低风速: GPIO%d\n", PIN_FAN_SPEED_LOW);
    Serial.printf("  中风速: GPIO%d\n", PIN_FAN_SPEED_MID);
    Serial.printf("  高风速: GPIO%d\n", PIN_FAN_SPEED_HIGH);

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
    if (maybe_need_init_ts20 && !ts20_is_called_init) {
        init_ts20_registers();
        ts20_is_called_init  = true;
        maybe_need_init_ts20 = false;
    }

    // ===== 步骤2: 读取 TS20 触摸状态 =====
    if (start_read_status) {
        ts20_is_called_init = false;  // 重置初始化标志，允许下次重新初始化
        get_and_update_ts20_status();
    } else {
        // 未开始读取时，清零缓存
        ts20_reg20 = 0;
        ts20_reg21 = 0;
        ts20_reg22 = 0;
    }

    // ===== 步骤3: 读取风速状态 =====
    read_fan_speed_status();

    // ===== 步骤4: 处理串口输入控制风速 =====
    if (Serial.available() > 0) {
        const char input = Serial.read();
        switch (input) {
            case '0':
                Serial.println("收到指令: 关闭风扇");
                set_fan_speed(FanControlState::FAN_SPEED_OFF);
                break;
            case '1':
                Serial.println("收到指令: 设置低风速");
                set_fan_speed(FanControlState::FAN_SPEED_LOW);
                break;
            case '2':
                Serial.println("收到指令: 设置中风速");
                set_fan_speed(FanControlState::FAN_SPEED_MID);
                break;
            case '3':
                Serial.println("收到指令: 设置高风速");
                set_fan_speed(FanControlState::FAN_SPEED_HIGH);
                break;
            case '4':
                Serial.println("收到指令: 开启风扇 (保持之前风速)");
                set_fan_speed(FanControlState::FAN_SPEED_ON);
                break;
            default:
                // 忽略其他输入（如换行符等）
                break;
        }
    }

    delay(100);  // 主循环延时，降低 CPU 占用
}
