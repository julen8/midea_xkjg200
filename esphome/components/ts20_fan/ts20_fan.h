#pragma once

#include <Wire.h>

#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ts20_fan {

// ==================== 硬件配置参数 ====================

// I2C 从机配置 (ESP32 作为从机，连接上位机)
static constexpr uint8_t I2C_SLAVE_SDA  = 21;
static constexpr uint8_t I2C_SLAVE_SCL  = 22;
static constexpr uint8_t I2C_SLAVE_ADDR = 0x7A;

// I2C 主机配置 (ESP32 作为主机，连接 TS20 触摸芯片)
static constexpr uint8_t I2C_MASTER_SDA   = 16;
static constexpr uint8_t I2C_MASTER_SCL   = 17;
static constexpr uint8_t I2C_TARGET_ADDR  = 0x7A;
static constexpr uint32_t I2C_MASTER_FREQ = 100000;

// 风速状态输入引脚配置 (低电平有效)
static constexpr uint8_t PIN_FAN_SPEED_LOW  = 25;
static constexpr uint8_t PIN_FAN_SPEED_MID  = 26;
static constexpr uint8_t PIN_FAN_SPEED_HIGH = 27;

// 上位机复位控制引脚 (低电平复位)
static constexpr uint8_t PIN_HOST_RESET = 4;

static constexpr uint8_t READ_BUFFER_SIZE = 3;

// ==================== 枚举定义 ====================

enum class FanKey : uint8_t {
    POWER          = 0,
    ECONOMY        = 1,
    FAN_SPEED_UP   = 2,
    FAN_SPEED_DOWN = 3,
};

enum class FanSpeedState : uint8_t {
    FAN_SPEED_OFF  = 0,
    FAN_SPEED_LOW  = 1,
    FAN_SPEED_MID  = 2,
    FAN_SPEED_HIGH = 3,
};

// ==================== 全局变量 (用于 I2C 回调) ====================

// I2C 总线实例
static TwoWire I2C_Master_Bus(0);
static TwoWire I2C_Slave_Bus(1);

// 数据缓冲区
static volatile uint8_t ts20_read_buffer[READ_BUFFER_SIZE];

// TS20 寄存器缓存
static volatile uint8_t ts20_reg20          = 0;
static volatile uint8_t ts20_reg21          = 0;
static volatile uint8_t ts20_reg22          = 0;
static volatile uint8_t ts20_now_read_reg   = 0;
static volatile uint8_t slave_write_counter = 0;

// 状态标志
static volatile bool start_read_status    = false;
static volatile bool maybe_need_init_ts20 = false;
static volatile bool ts20_is_called_init  = false;

// 当前风速状态
static volatile FanSpeedState current_fan_speed_state = FanSpeedState::FAN_SPEED_OFF;

// ==================== TS20 初始化数据 ====================

static const uint8_t ts20_init_data[][2] = {
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
    {0x0A, 0x55},
};

// ==================== I2C 回调函数 ====================

static void ts20_on_receive_callback(int numBytes) {
    static uint8_t need_init_counter = 0;

    if (numBytes == 1) {
        need_init_counter = 0;
        start_read_status = true;
        if (I2C_Slave_Bus.available()) {
            ts20_now_read_reg = I2C_Slave_Bus.read();
        }
    } else if (numBytes > 1) {
        start_read_status = false;
        if (need_init_counter++ > 5) {
            need_init_counter    = 0;
            maybe_need_init_ts20 = true;
        }
        I2C_Slave_Bus.flush();
    }
}

static void ts20_on_request_callback() {
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
            val = 0;
            break;
    }

    I2C_Slave_Bus.write(val);
    ++slave_write_counter;
}

// ==================== TS20FanController 类 ====================

class TS20FanController : public Component {
public:
    TS20FanController() = default;

    bool slave_init_status  = false;
    bool master_init_status = false;

    void setup() override {
        // 首先将上位机复位引脚拉低，使上位机进入复位状态
        pinMode(PIN_HOST_RESET, OUTPUT);
        digitalWrite(PIN_HOST_RESET, LOW);
        ESP_LOGI("ts20_fan", "Host reset pin (GPIO%d) pulled LOW, host in reset state", PIN_HOST_RESET);

        // 等待上位机进入复位状态
        delay(50);

        // 初始化 I2C 从机
        I2C_Slave_Bus.onReceive(ts20_on_receive_callback);
        I2C_Slave_Bus.onRequest(ts20_on_request_callback);
        slave_init_status = I2C_Slave_Bus.begin(I2C_SLAVE_ADDR, I2C_SLAVE_SDA, I2C_SLAVE_SCL, 0);
        ESP_LOGI("ts20_fan", "I2C Slave init: %s", slave_init_status ? "OK" : "FAILED");

        // 初始化风速状态输入引脚
        pinMode(PIN_FAN_SPEED_LOW, INPUT_PULLUP);
        pinMode(PIN_FAN_SPEED_MID, INPUT_PULLUP);
        pinMode(PIN_FAN_SPEED_HIGH, INPUT_PULLUP);

        // 初始化 I2C 主机
        master_init_status = I2C_Master_Bus.begin(I2C_MASTER_SDA, I2C_MASTER_SCL, I2C_MASTER_FREQ);
        ESP_LOGI("ts20_fan", "I2C Master init: %s", master_init_status ? "OK" : "FAILED");

        // 读取初始风速状态
        update_speed_pins();

        // ESP32 初始化完成，将复位引脚改为高电平，释放上位机复位
        digitalWrite(PIN_HOST_RESET, HIGH);
        ESP_LOGI("ts20_fan", "Host reset pin (GPIO%d) pulled HIGH, host released from reset state", PIN_HOST_RESET);

        ESP_LOGI("ts20_fan", "TS20 Fan Controller initialized");
    }

    void loop() override {
        if (!slave_init_status || !master_init_status) {
            ESP_LOGI("ts20_fan", "slave_init_status: %s, master_init_status: %s", slave_init_status ? "OK" : "FAILED",
                     master_init_status ? "OK" : "FAILED");
        }

        // 检查并执行 TS20 初始化
        if (maybe_need_init_ts20 && !ts20_is_called_init) {
            init_ts20_registers();
            ts20_is_called_init  = true;
            maybe_need_init_ts20 = false;
        }

        // 读取 TS20 触摸状态
        if (start_read_status) {
            ts20_is_called_init = false;
            get_and_update_ts20_status();
        } else {
            ts20_reg20 = 0;
            ts20_reg21 = 0;
            ts20_reg22 = 0;
        }

        // 更新风速状态
        update_speed_pins();
        delay(100);
    }

    float get_setup_priority() const override {
        return setup_priority::HARDWARE;
    }

    void init_ts20_registers() {
        ESP_LOGI("ts20_fan", "Initializing TS20 registers...");

        for (size_t i = 0; i < sizeof(ts20_init_data) / sizeof(ts20_init_data[0]); i++) {
            I2C_Master_Bus.beginTransmission(I2C_TARGET_ADDR);
            I2C_Master_Bus.write(ts20_init_data[i], 2);
            I2C_Master_Bus.endTransmission(true);
            ESP_LOGD("ts20_fan", "Write reg 0x%02X: 0x%02X", ts20_init_data[i][0], ts20_init_data[i][1]);
            delay(10);
        }

        ESP_LOGI("ts20_fan", "TS20 registers initialized");
    }

    void update_speed_pins() {
        FanSpeedState new_state;

        if (digitalRead(PIN_FAN_SPEED_LOW) == LOW) {
            new_state = FanSpeedState::FAN_SPEED_LOW;
        } else if (digitalRead(PIN_FAN_SPEED_MID) == LOW) {
            new_state = FanSpeedState::FAN_SPEED_MID;
        } else if (digitalRead(PIN_FAN_SPEED_HIGH) == LOW) {
            new_state = FanSpeedState::FAN_SPEED_HIGH;
        } else {
            new_state = FanSpeedState::FAN_SPEED_OFF;
        }

        if (new_state != current_fan_speed_state) {
            current_fan_speed_state   = new_state;
            const char *state_names[] = {"OFF", "LOW", "MID", "HIGH"};
            ESP_LOGI("ts20_fan", "Fan speed changed: %s", state_names[static_cast<int>(new_state)]);
        }
    }

    int get_current_speed() {
        return static_cast<int>(current_fan_speed_state);
    }

    void set_fan_speed(int speed, int retry_count = 3) {
        ESP_LOGI("ts20_fan", "Setting fan speed: %d", speed);

        // 如果目标不是关闭且当前已关闭，先开启风扇
        if (speed != 0 && current_fan_speed_state == FanSpeedState::FAN_SPEED_OFF) {
            ESP_LOGI("ts20_fan", "Turning on fan");
            simulate_key_press(FanKey::POWER);
            update_speed_pins();
        }

        switch (speed) {
            case 0:
                if (current_fan_speed_state != FanSpeedState::FAN_SPEED_OFF) {
                    ESP_LOGI("ts20_fan", "Turning off fan");
                    simulate_key_press(FanKey::POWER);
                }
                break;

            case 1:
                if (current_fan_speed_state != FanSpeedState::FAN_SPEED_LOW) {
                    ESP_LOGI("ts20_fan", "Setting LOW speed (economy)");
                    simulate_key_press(FanKey::ECONOMY);
                }
                break;

            case 2:
                if (current_fan_speed_state == FanSpeedState::FAN_SPEED_LOW) {
                    ESP_LOGI("ts20_fan", "LOW -> MID");
                    simulate_key_press(FanKey::FAN_SPEED_UP);
                } else if (current_fan_speed_state == FanSpeedState::FAN_SPEED_HIGH) {
                    ESP_LOGI("ts20_fan", "HIGH -> MID");
                    simulate_key_press(FanKey::FAN_SPEED_DOWN);
                }
                break;

            case 3:
                if (current_fan_speed_state == FanSpeedState::FAN_SPEED_LOW) {
                    ESP_LOGI("ts20_fan", "LOW -> HIGH");
                    simulate_key_press(FanKey::FAN_SPEED_UP);
                    simulate_key_press(FanKey::FAN_SPEED_UP);
                } else if (current_fan_speed_state == FanSpeedState::FAN_SPEED_MID) {
                    ESP_LOGI("ts20_fan", "MID -> HIGH");
                    simulate_key_press(FanKey::FAN_SPEED_UP);
                }
                break;
        }

        // 验证设置结果
        update_speed_pins();
        int current = get_current_speed();
        if (current != speed && retry_count > 0) {
            ESP_LOGW("ts20_fan", "Speed set failed, retrying... (%d left)", retry_count - 1);
            set_fan_speed(speed, retry_count - 1);
        }
    }

private:
    void update_ts20_status_cache() {
        if (ts20_read_buffer[0] != 0 || ts20_read_buffer[1] != 0) {
            ESP_LOGI("ts20_fan", "TS20 touch: %02X %02X %02X", ts20_read_buffer[0], ts20_read_buffer[1],
                     ts20_read_buffer[2]);
        }

        ts20_reg22 = ts20_read_buffer[0];
        ts20_reg20 = ts20_read_buffer[1];
        ts20_reg21 = ts20_read_buffer[2];
    }

    bool get_and_update_ts20_status() {
        I2C_Master_Bus.beginTransmission(I2C_TARGET_ADDR);
        I2C_Master_Bus.write(0x20);
        auto ret = I2C_Master_Bus.endTransmission(false);
        if (ret != 0)
            return false;

        ret = I2C_Master_Bus.requestFrom((uint8_t)I2C_TARGET_ADDR, (uint8_t)READ_BUFFER_SIZE);
        if (ret != READ_BUFFER_SIZE)
            return false;

        uint8_t idx = 0;
        while (idx < READ_BUFFER_SIZE && I2C_Master_Bus.available()) {
            ts20_read_buffer[idx++] = I2C_Master_Bus.read();
        }

        if (idx != READ_BUFFER_SIZE)
            return false;

        if (0 != (ts20_read_buffer[0] & ~0x34) || 0 != (ts20_read_buffer[1] & ~0x01) || 0 != ts20_read_buffer[2]) {
            return false;
        }

        update_ts20_status_cache();
        return true;
    }

    void simulate_key_press(FanKey key_code) {
        switch (key_code) {
            case FanKey::POWER:
                ts20_read_buffer[0] = 0x04;
                ts20_read_buffer[1] = 0x00;
                ts20_read_buffer[2] = 0x00;
                break;
            case FanKey::ECONOMY:
                ts20_read_buffer[0] = 0x20;
                ts20_read_buffer[1] = 0x00;
                ts20_read_buffer[2] = 0x00;
                break;
            case FanKey::FAN_SPEED_UP:
                ts20_read_buffer[0] = 0x00;
                ts20_read_buffer[1] = 0x01;
                ts20_read_buffer[2] = 0x00;
                break;
            case FanKey::FAN_SPEED_DOWN:
                ts20_read_buffer[0] = 0x10;
                ts20_read_buffer[1] = 0x00;
                ts20_read_buffer[2] = 0x00;
                break;
        }

        update_ts20_status_cache();

        slave_write_counter = 0;
        for (int i = 0; i < 40 && slave_write_counter < 9; ++i) {
            delay(10);
        }

        ts20_read_buffer[0] = 0x00;
        ts20_read_buffer[1] = 0x00;
        ts20_read_buffer[2] = 0x00;
        update_ts20_status_cache();

        slave_write_counter = 0;
        for (int i = 0; i < 40 && slave_write_counter < 9; ++i) {
            delay(10);
        }
    }
};

}  // namespace ts20_fan
}  // namespace esphome
