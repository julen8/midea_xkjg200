/*
 * ESP32 I2C 透明透传桥接器
 *
 * 功能：
 * - 完全透明的 I2C 透传，上位机像直接操作目标芯片一样
 * - ESP32 作为 I2C 从机连接上位机
 * - ESP32 作为 I2C 主机连接下游芯片
 * - 串口打印所有交互信息用于调试
 *
 * 工作原理：
 * - 上位机写入数据 -> 立即透传到目标芯片
 * - 上位机读取数据 -> 从目标芯片读取并返回
 * - 支持标准 I2C 协议：写、读、写后读（重复起始）
 *
 * 硬件连接 (ESP32 NodeMCU-32S 安全引脚):
 * I2C 从机接口 (连接上位机):
 *   - SDA: GPIO 21
 *   - SCL: GPIO 22
 *
 * I2C 主机接口 (连接下游芯片):
 *   - SDA: GPIO 16
 *   - SCL: GPIO 17
 */

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
#define I2C_MASTER_FREQ     100000  // 100kHz

// 缓冲区大小
#define BUFFER_SIZE         128


// ==================== 全局变量 ====================
TwoWire I2C_Master = TwoWire(0); // I2C 主机实例
TwoWire I2C_Slave = TwoWire(1); // I2C 从机实例

// 写入缓冲区（上位机写入的数据）
volatile uint8_t writeBuffer[BUFFER_SIZE];
volatile uint8_t writeLen = 0;

// 读取缓冲区（准备返回给上位机的数据）
volatile uint8_t readBuffer[BUFFER_SIZE];
volatile uint8_t readLen = 0;
volatile uint8_t readIndex = 0;

volatile uint8_t ts20_reg20 = 0; // TS20寄存器0x20缓存
volatile uint8_t ts20_reg21 = 0; // TS20寄存器0x21缓存
volatile uint8_t ts20_reg22 = 0; // TS20寄存器0x22缓存
volatile uint8_t ts20_now_reg_val = 0; // 当前读取的寄存器值

volatile bool start_read_status = false; // 初始化完成

// 调试打印标志
volatile bool printResponseFlag = false;

uint8_t debugWriteBuf[BUFFER_SIZE];
uint8_t debugWriteLen = 0;
uint8_t debugReadBuf[BUFFER_SIZE];
uint8_t debugReadLen = 0;
uint8_t debugResponseBuf[BUFFER_SIZE];
uint8_t debugResponseLen = 0;

void printHex(const char *prefix, volatile uint8_t *data, uint8_t len);

// ==================== I2C 从机回调函数 ====================

// 上位机写入数据时调用
// 注意：回调函数在中断上下文中执行，不能执行耗时操作（如I2C主机通信）
// 否则会导致 rx_ring_buf_full 错误
void onReceiveCallback(int numBytes) {
    writeLen = 0;

    // 读取上位机发来的所有数据
    while (writeLen < numBytes && I2C_Slave.available()) {
        writeBuffer[writeLen++] = I2C_Slave.read();
    }

    // 透传到目标芯片
    if (writeLen > 1) {
        start_read_status = false;
        I2C_Master.beginTransmission(I2C_TARGET_ADDR);
        I2C_Master.write((uint8_t *) writeBuffer, writeLen);
        I2C_Master.endTransmission(true);
        printHex("->", writeBuffer, writeLen);
    } else if (writeLen == 1) {
        start_read_status = true;
        switch (writeBuffer[0]) {
            case 0x20:
                ts20_now_reg_val = ts20_reg20;
                break;
            case 0x21:
                ts20_now_reg_val = ts20_reg21;
                break;
            case 0x22:
                ts20_now_reg_val = ts20_reg22;
                break;
            default:
                Serial.print("Unknown register read request: 0x");
                Serial.println(writeBuffer[0], HEX);
                ts20_now_reg_val = 0;
                break;
        }
    }
}

// 上位机请求读取数据时调用
void onRequestCallback() {
    I2C_Slave.write((uint8_t *) (&ts20_now_reg_val), 1);
}

// ==================== 辅助函数 ====================

void printHex(const char *prefix, volatile uint8_t *data, uint8_t len) {
    Serial.print(prefix);
    for (int i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        if (i < len - 1) Serial.print(" ");
    }
    Serial.println();
}

void scanI2CBus() {
    Serial.println("\n扫描 I2C 主机总线...");
    uint8_t count = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        I2C_Master.beginTransmission(addr);
        if (I2C_Master.endTransmission() == 0) {
            Serial.print("  发现设备:  0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            count++;
        }
    }

    if (count == 0) {
        Serial.println("  未发现设备！请检查连接。");
    } else {
        Serial.print("  共 ");
        Serial.print(count);
        Serial.println(" 个设备");
    }
}

// ==================== 主程序 ====================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║   ESP32 I2C 透明透传桥接器             ║");
    Serial.println("╚════════════════════════════════════════╝");

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

    // 初始化 I2C 主机
    auto ret = I2C_Master.begin(I2C_MASTER_SDA, I2C_MASTER_SCL, I2C_MASTER_FREQ);
    Serial.printf("\n[OK] I2C 主机初始化完成:%s\n", ret ? "成功" : " [失败！]");

    // 扫描总线
    //scanI2CBus();

    // 初始化 I2C 从机
    // ESP32 Wire 库从机模式：先设置引脚，再以从机地址初始化
    I2C_Slave.onReceive(onReceiveCallback);
    I2C_Slave.onRequest(onRequestCallback);
    ret = I2C_Slave.begin((uint8_t) I2C_SLAVE_ADDR, I2C_SLAVE_SDA, I2C_SLAVE_SCL, 0);
    Serial.printf("[OK] I2C 从机初始化完成:%s", ret ? "成功" : " [失败！]");

    Serial.println("\n【使用说明】");
    Serial.println("  上位机直接以 0x" + String(I2C_SLAVE_ADDR, HEX) + " 地址操作，");
    Serial.println("  就像直接连接 0x" + String(I2C_TARGET_ADDR, HEX) + " 芯片一样。");
    Serial.println("\n  串口命令:  s=扫描总线, t=测试读取, h=帮助");

    Serial.println("\n────────── 等待通信 ──────────\n");
}

void loop() {
    if (start_read_status) {
        I2C_Master.beginTransmission(I2C_TARGET_ADDR);
        I2C_Master.write(0x20);
        I2C_Master.endTransmission(false);
        readLen = I2C_Master.requestFrom((uint8_t) I2C_TARGET_ADDR, (uint8_t) 3);
        uint8_t idx = 0;
        while (idx < 3 && I2C_Master.available()) {
            readBuffer[idx++] = I2C_Master.read();
        }
        readLen = idx;
        printHex("read_buff:", readBuffer, readLen);
        ts20_reg22 =readBuffer[0];
        ts20_reg20 =readBuffer[1];
        ts20_reg21 =readBuffer[2];
    } else {
        ts20_reg20 = 0;
        ts20_reg21 = 0;
        ts20_reg22 = 0;
    }

    delay(200);
}
