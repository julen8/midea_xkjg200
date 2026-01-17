#include <Arduino.h>
#include <Wire.h>

// ==================== 配置参数 ====================
// I2C 从机配置 (连接上位机)
#define I2C_SLAVE_SDA       21      // GPIO21 - 默认I2C SDA，安全可用
#define I2C_SLAVE_SCL       22      // GPIO22 - 默认I2C SCL，安全可用
#define I2C_SLAVE_ADDR      0x7A    // ESP32 作为从机的地址

// I2C 主机配置 (连接下游芯片 TS20)
#define I2C_MASTER_SDA      16      // GPIO16 - 安全可用
#define I2C_MASTER_SCL      17      // GPIO17 - 安全可用
#define I2C_TARGET_ADDR     0x7A    // TS20 I2C 地址 (ADD引脚接GND时为0x6A, 接VDD时为0x7A)

// I2C 速率
#define I2C_MASTER_FREQ     50000  // 50kHz

// ==================== 全局变量 ====================
auto I2C_Master = TwoWire(0); // I2C 主机实例
auto I2C_Slave = TwoWire(1); // I2C 从机实例

// 读取缓冲区（准备返回给上位机的数据）
volatile uint8_t readBuffer[4];

volatile uint8_t ts20_reg20 = 0; // TS20寄存器0x20缓存
volatile uint8_t ts20_reg21 = 0; // TS20寄存器0x21缓存
volatile uint8_t ts20_reg22 = 0; // TS20寄存器0x22缓存
volatile uint8_t ts20_now_read_reg = 0; // 上位机当前想要读取的寄存器

volatile bool start_read_status = false; // 开始读取状态标志
bool maybe_need_init_ts20 = false; // 可能需要初始化TS20寄存器
bool is_called_init = false; // 是否已调用初始化函数

// 初始化ts20的寄存器值
const uint8_t ts20_init_data[][2] = {
    {0x0C, 0x9A},
    {0x0C, 0x92},
    {0x0E, 0x00},
    {0x0F, 0x00},
    {0x10, 0x50},
    {0x11, 0x55},
    {0x12, 0x55},
    {0x13, 0x01},
    {0x00, 0x55},
    {0x01, 0x55},
    {0x02, 0x55},
    {0x03, 0x55},
    {0x04, 0x55},
    {0x05, 0x55},
    {0x06, 0x55},
    {0x07, 0x55},
    {0x08, 0x55},
    {0x09, 0x55},
    {0x0A, 0x55}
};

// ==================== I2C 从机回调函数 ====================

void onReceiveCallback(int numBytes) {
    // 一般情况下，写入长度大于1表示写入寄存器和值 ， 等于1表示请求读取寄存器
    if (numBytes == 1) {
        start_read_status = true;
        if (I2C_Slave.available()) {
            ts20_now_read_reg = I2C_Slave.read();
        }
    } else if (numBytes > 1) {
        maybe_need_init_ts20 = true;
        I2C_Slave.flush();
    }
}

// 上位机请求读取数据时调用
void onRequestCallback() {
    uint8_t val = 0;

    switch (ts20_now_read_reg) {
        case 0x20:
            val = ts20_reg20;
            break;
        case 0x21:
            val = ts20_reg21;
            break;
        case 0x22:
            val = ts20_reg22;
            break;
        default:
            break;
    }

    I2C_Slave.write(val);
}

// ==================== 辅助函数 ====================

void init_ts20_registers() {
    Serial.println("初始化 TS20 寄存器...");

    for (const auto &i: ts20_init_data) {
        I2C_Master.beginTransmission(I2C_TARGET_ADDR);
        I2C_Master.write(i, sizeof(i));
        I2C_Master.endTransmission(true);
        Serial.printf("  写入寄存器 0x%02X: 0x%02X\n", i[0], i[1]);
        delay(10); // 确保写入稳定
    }

    Serial.println("TS20 寄存器初始化完成。\n");
}

void printHex(const char *prefix, volatile uint8_t *data, uint8_t len) {
    Serial.print(prefix);
    for (int i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        if (i < len - 1) Serial.print(" ");
    }
    Serial.println();
}

// ==================== 主程序 ====================

void setup() {
    Serial.begin(115200);

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

    // 初始化 I2C 从机
    I2C_Slave.onReceive(onReceiveCallback);
    I2C_Slave.onRequest(onRequestCallback);
    auto ret = I2C_Slave.begin((uint8_t) I2C_SLAVE_ADDR, I2C_SLAVE_SDA, I2C_SLAVE_SCL, 0);
    Serial.printf("[OK] I2C 从机初始化完成:%s", ret ? "成功" : " [失败！]");

    // 初始化 I2C 主机
    ret = I2C_Master.begin(I2C_MASTER_SDA, I2C_MASTER_SCL, I2C_MASTER_FREQ);
    Serial.printf("\n[OK] I2C 主机初始化完成:%s\n", ret ? "成功" : " [失败！]");


    Serial.println("\n────────── 等待通信 ──────────\n");
}

void loop() {
    if (maybe_need_init_ts20 && !is_called_init) {
        init_ts20_registers();
        is_called_init = true;
        maybe_need_init_ts20 = false;
    }

    if (start_read_status) {
        is_called_init = false;

        I2C_Master.beginTransmission(I2C_TARGET_ADDR);
        I2C_Master.write(0x20); // 0x20 0x21 0x22 是三个触摸状态寄存器
        I2C_Master.endTransmission(false);
        I2C_Master.requestFrom(static_cast<uint8_t>(I2C_TARGET_ADDR), static_cast<uint8_t>(3));

        uint8_t idx = 0;
        while (idx < 3 && I2C_Master.available()) {
            readBuffer[idx++] = I2C_Master.read();
        }

        if (idx == 3) {
            Serial.printf("read_buff:%02X %02X %02X\n", readBuffer[0], readBuffer[1], readBuffer[2]);
            // 更新寄存器缓存
            // 不清楚为什么顺序是22,20,21
            ts20_reg22 = readBuffer[0];
            ts20_reg20 = readBuffer[1];
            ts20_reg21 = readBuffer[2];
        }
    } else {
        ts20_reg20 = 0;
        ts20_reg21 = 0;
        ts20_reg22 = 0;
    }

    delay(100);
}
