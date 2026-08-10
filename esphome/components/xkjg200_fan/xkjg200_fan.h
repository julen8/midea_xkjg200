#pragma once

#include <cstdint>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/i2c_slave.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/fan/fan.h"

namespace esphome {
namespace xkjg200_fan {

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

// 上位机电源控制引脚 (低电平上位机断电)
static constexpr uint8_t PIN_HOST_POWER = 4;

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
static i2c_master_bus_handle_t i2c_master_bus = nullptr;
static i2c_master_dev_handle_t i2c_target     = nullptr;
static i2c_slave_dev_handle_t i2c_slave      = nullptr;
static QueueHandle_t i2c_response_queue      = nullptr;

// 数据缓冲区
static volatile uint8_t ts20_read_buffer[READ_BUFFER_SIZE];

// TS20 寄存器缓存
static volatile uint8_t ts20_reg20          = 0;
static volatile uint8_t ts20_reg21          = 0;
static volatile uint8_t ts20_reg22          = 0;
static volatile uint8_t ts20_now_read_reg   = 0;
static volatile uint8_t slave_write_counter = 0;
static portMUX_TYPE ts20_cache_lock         = portMUX_INITIALIZER_UNLOCKED;

// 状态标志
static volatile bool start_read_status    = false;
static volatile bool maybe_need_init_ts20 = false;
static volatile bool ts20_is_called_init  = false;

static FanKey pending_keys[8];
static uint8_t pending_key_count = 0;
static bool active_key = false;
static int64_t key_release_at = 0;
static constexpr uint8_t KEY_MIN_READS = 9;
static constexpr int64_t KEY_MAX_DURATION_US = 250000;
static constexpr int64_t KEY_RELEASE_GAP_US = 250000;
static int64_t next_key_at = 0;
static constexpr int64_t STATUS_POLL_INTERVAL_US = 100000;
static int64_t next_status_poll_at = 0;

// 当前风速状态
static volatile auto current_fan_speed_state = FanSpeedState::FAN_SPEED_OFF;

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

static bool ts20_on_receive_callback(i2c_slave_dev_handle_t, const i2c_slave_rx_done_event_data_t *event,
                                     void *) {
    static uint8_t need_init_counter = 0;
    const size_t num_bytes = event->length;

    if (num_bytes == 1) {
        need_init_counter = 0;
        start_read_status = true;
        ts20_now_read_reg = event->buffer[0];
    } else if (num_bytes > 1) {

        start_read_status = false;
        if (need_init_counter++ > 5) {
            need_init_counter    = 0;
            maybe_need_init_ts20 = true;
        }
    }
    return false;
}

static bool ts20_on_request_callback(i2c_slave_dev_handle_t, const i2c_slave_request_event_data_t *, void *) {
    uint8_t val = 0;
    portENTER_CRITICAL_ISR(&ts20_cache_lock);
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
    portEXIT_CRITICAL_ISR(&ts20_cache_lock);

    BaseType_t task_woken = pdFALSE;
    if (i2c_response_queue != nullptr) {
        xQueueSendFromISR(i2c_response_queue, &val, &task_woken);
    }
    ++slave_write_counter;
    return task_woken == pdTRUE;
}

static void i2c_response_task(void *) {
    uint8_t val = 0;
    while (true) {
        if (xQueueReceive(i2c_response_queue, &val, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint32_t written = 0;
        const esp_err_t result = i2c_slave_write(i2c_slave, &val, 1, &written, 10);
        if (result != ESP_OK || written != 1) {
            ESP_LOGW("xkjg200_fan", "Failed to prepare I2C slave response: %s", esp_err_to_name(result));
        }
    }
}

// ==================== XKJG200FanController 类 ====================

class XKJG200FanController : public Component, public fan::Fan {
public:
    XKJG200FanController() = default;
    virtual ~XKJG200FanController() = default;

    bool slave_init_status  = false;
    bool master_init_status = false;

    void set_speed_count(const int speed_count) { speed_count_ = speed_count; }

    fan::FanTraits get_traits() override { return fan::FanTraits(false, true, false, speed_count_); }

    void setup() override {
        // 首先将上位机电源断开
        ESP_LOGI("xkjg200_fan", "Host power pin (GPIO%d) pulled LOW", PIN_HOST_POWER);
        esp_err_t result = gpio_set_direction(static_cast<gpio_num_t>(PIN_HOST_POWER), GPIO_MODE_OUTPUT);
        if (result == ESP_OK) {
            result = gpio_set_level(static_cast<gpio_num_t>(PIN_HOST_POWER), 0);
        }
        if (result != ESP_OK) {
            ESP_LOGE("xkjg200_fan", "Failed to disable host power: %s", esp_err_to_name(result));
            mark_failed();
            return;
        }
        // 等待上位机掉电稳定
        delay(500);

        // 初始化风速状态输入引脚
        gpio_config_t speed_input_config = {};
        speed_input_config.mode = GPIO_MODE_INPUT;
        speed_input_config.pull_up_en = GPIO_PULLUP_ENABLE;
        speed_input_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        speed_input_config.intr_type = GPIO_INTR_DISABLE;
        speed_input_config.pin_bit_mask = (1ULL << PIN_FAN_SPEED_LOW) | (1ULL << PIN_FAN_SPEED_MID) |
                           (1ULL << PIN_FAN_SPEED_HIGH);
        result = gpio_config(&speed_input_config);
        if (result != ESP_OK) {
            ESP_LOGE("xkjg200_fan", "Failed to configure fan speed inputs: %s", esp_err_to_name(result));
            mark_failed();
            return;
        }

        // 初始化 I2C 主机
        const i2c_master_bus_config_t master_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = static_cast<gpio_num_t>(I2C_MASTER_SDA),
            .scl_io_num = static_cast<gpio_num_t>(I2C_MASTER_SCL),
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = true, .allow_pd = false},
        };
        master_init_status = i2c_new_master_bus(&master_config, &i2c_master_bus) == ESP_OK;
        if (master_init_status) {
            const i2c_device_config_t target_config = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = I2C_TARGET_ADDR,
                .scl_speed_hz = I2C_MASTER_FREQ,
                .scl_wait_us = 0,
                .flags = {.disable_ack_check = false},
            };
            master_init_status = i2c_master_bus_add_device(i2c_master_bus, &target_config, &i2c_target) == ESP_OK;
        }
        ESP_LOGI("xkjg200_fan", "I2C Master init: %s", master_init_status ? "OK" : "FAILED");

        // 初始化 I2C 从机
        const i2c_slave_config_t slave_config = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = static_cast<gpio_num_t>(I2C_SLAVE_SDA),
            .scl_io_num = static_cast<gpio_num_t>(I2C_SLAVE_SCL),
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .send_buf_depth = 32,
            .receive_buf_depth = 32,
            .slave_addr = I2C_SLAVE_ADDR,
            .addr_bit_len = I2C_ADDR_BIT_LEN_7,
            .intr_priority = 0,
            .flags = {.allow_pd = false, .enable_internal_pullup = true},
        };
        const i2c_slave_event_callbacks_t slave_callbacks = {
            .on_request = ts20_on_request_callback,
            .on_receive = ts20_on_receive_callback,
        };
        slave_init_status = i2c_new_slave_device(&slave_config, &i2c_slave) == ESP_OK;
        if (slave_init_status) {
            i2c_response_queue = xQueueCreate(8, sizeof(uint8_t));
            slave_init_status = i2c_response_queue != nullptr;
        }
        if (slave_init_status) {
            slave_init_status = xTaskCreate(i2c_response_task, "xkjg200_i2c", 2048, nullptr,
                                            configMAX_PRIORITIES - 2, nullptr) == pdPASS;
        }
        if (slave_init_status) {
            slave_init_status = i2c_slave_register_event_callbacks(i2c_slave, &slave_callbacks, nullptr) == ESP_OK;
        }
        ESP_LOGI("xkjg200_fan", "I2C Slave init: %s", slave_init_status ? "OK" : "FAILED");

        if (!master_init_status || !slave_init_status) {
            ESP_LOGE("xkjg200_fan", "I2C initialization failed; keeping host power off");
            mark_failed();
            return;
        }

        // ESP32 初始化完成，上位机上电
        ESP_LOGI("xkjg200_fan", "Host power pin (GPIO%d) pulled HIGH", PIN_HOST_POWER);
        result = gpio_set_level(static_cast<gpio_num_t>(PIN_HOST_POWER), 1);
        if (result != ESP_OK) {
            ESP_LOGE("xkjg200_fan", "Failed to enable host power: %s", esp_err_to_name(result));
            mark_failed();
            return;
        }

        ESP_LOGI("xkjg200_fan", "XKJG200 Fan Controller initialized");
    }

    void loop() override {
        // 检查并执行 TS20 初始化
        if (maybe_need_init_ts20 && !ts20_is_called_init) {
            init_ts20_registers();
            ts20_is_called_init  = true;
            maybe_need_init_ts20 = false;
        }

        // 读取 TS20 触摸状态
        process_key_press();
        const int64_t now = esp_timer_get_time();
        if (now < next_status_poll_at) {
            return;
        }
        next_status_poll_at = now + STATUS_POLL_INTERVAL_US;

        if (!active_key && start_read_status) {
            ts20_is_called_init = false;
            get_and_update_ts20_status();
        } else if (!active_key) {
            portENTER_CRITICAL(&ts20_cache_lock);
            ts20_reg20 = 0;
            ts20_reg21 = 0;
            ts20_reg22 = 0;
            portEXIT_CRITICAL(&ts20_cache_lock);
        }

        // 更新风速状态
        update_speed_pins();
        const int published_speed = static_cast<int>(current_fan_speed_state);
        if (published_speed != last_published_speed_) {
            state = published_speed != 0;
            speed = published_speed;
            publish_state();
            last_published_speed_ = published_speed;
        }
    }

    float get_setup_priority() const override {
        return setup_priority::HARDWARE;
    }

    static void init_ts20_registers() {
        if (i2c_target == nullptr) {
            ESP_LOGE("xkjg200_fan", "Cannot initialize TS20 registers: I2C master is unavailable");
            return;
        }

        ESP_LOGI("xkjg200_fan", "Initializing TS20 registers...");

        for (size_t i = 0; i < sizeof(ts20_init_data) / sizeof(ts20_init_data[0]); i++) {
            const esp_err_t result = i2c_master_transmit(i2c_target, ts20_init_data[i], 2, 100);
            if (result != ESP_OK) {
                ESP_LOGW("xkjg200_fan", "Failed to write TS20 reg 0x%02X: %s", ts20_init_data[i][0],
                         esp_err_to_name(result));
                continue;
            }
            ESP_LOGD("xkjg200_fan", "Write reg 0x%02X: 0x%02X", ts20_init_data[i][0], ts20_init_data[i][1]);
            delay(10);
        }

        ESP_LOGI("xkjg200_fan", "TS20 registers initialized");
    }

    static void update_speed_pins() {
        FanSpeedState new_state;

        if (gpio_get_level(static_cast<gpio_num_t>(PIN_FAN_SPEED_LOW)) == 0) {
            new_state = FanSpeedState::FAN_SPEED_LOW;
        } else if (gpio_get_level(static_cast<gpio_num_t>(PIN_FAN_SPEED_MID)) == 0) {
            new_state = FanSpeedState::FAN_SPEED_MID;
        } else if (gpio_get_level(static_cast<gpio_num_t>(PIN_FAN_SPEED_HIGH)) == 0) {
            new_state = FanSpeedState::FAN_SPEED_HIGH;
        } else {
            new_state = FanSpeedState::FAN_SPEED_OFF;
        }

        if (new_state != current_fan_speed_state) {
            current_fan_speed_state   = new_state;
            const char *state_names[] = {"OFF", "LOW", "MID", "HIGH"};
            ESP_LOGI("xkjg200_fan", "Fan speed changed: %s", state_names[static_cast<int>(new_state)]);
        }
    }

    static int get_current_speed() {
        return static_cast<int>(current_fan_speed_state);
    }

    static void set_fan_speed(const int speed) {
        ESP_LOGI("xkjg200_fan", "Setting fan speed: %d", speed);

        // 如果目标不是关闭且当前已关闭，先开启风扇
        if (speed != 0 && current_fan_speed_state == FanSpeedState::FAN_SPEED_OFF) {
            ESP_LOGI("xkjg200_fan", "Turning on fan");
            simulate_key_press(FanKey::POWER);
            simulate_key_press(FanKey::ECONOMY);
            if (speed >= 2) {
                simulate_key_press(FanKey::FAN_SPEED_UP);
            }
            if (speed >= 3) {
                simulate_key_press(FanKey::FAN_SPEED_UP);
            }
            return;
        }

        switch (speed) {
            case 0:
                if (current_fan_speed_state != FanSpeedState::FAN_SPEED_OFF) {
                    ESP_LOGI("xkjg200_fan", "Turning off fan");
                    simulate_key_press(FanKey::POWER);
                }
                break;

            case 1:
                if (current_fan_speed_state != FanSpeedState::FAN_SPEED_LOW) {
                    ESP_LOGI("xkjg200_fan", "Setting LOW speed (economy)");
                    simulate_key_press(FanKey::ECONOMY);
                }
                break;

            case 2:
                if (current_fan_speed_state == FanSpeedState::FAN_SPEED_LOW) {
                    ESP_LOGI("xkjg200_fan", "LOW -> MID");
                    simulate_key_press(FanKey::FAN_SPEED_UP);
                } else if (current_fan_speed_state == FanSpeedState::FAN_SPEED_HIGH) {
                    ESP_LOGI("xkjg200_fan", "HIGH -> MID");
                    simulate_key_press(FanKey::FAN_SPEED_DOWN);
                }
                break;

            case 3:
                if (current_fan_speed_state == FanSpeedState::FAN_SPEED_LOW) {
                    ESP_LOGI("xkjg200_fan", "LOW -> HIGH");
                    simulate_key_press(FanKey::FAN_SPEED_UP);
                    simulate_key_press(FanKey::FAN_SPEED_UP);
                } else if (current_fan_speed_state == FanSpeedState::FAN_SPEED_MID) {
                    ESP_LOGI("xkjg200_fan", "MID -> HIGH");
                    simulate_key_press(FanKey::FAN_SPEED_UP);
                }
                break;
            default:
                ESP_LOGW("xkjg200_fan", "Invalid speed %d, must be 0-3", speed);
                return;
        }

    }

private:
    int speed_count_ = 3;
    int last_published_speed_ = -1;

    void control(const fan::FanCall &call) override {
        if (call.get_state().has_value() && !*call.get_state()) {
            set_fan_speed(0);
            return;
        }
        if (call.get_speed().has_value()) {
            set_fan_speed(*call.get_speed());
        } else if (call.get_state().has_value() && *call.get_state()) {
            set_fan_speed(speed > 0 ? speed : 2);
        }
    }

    static void update_ts20_status_cache() {
        if (ts20_read_buffer[0] != 0 || ts20_read_buffer[1] != 0) {
            ESP_LOGD("xkjg200_fan", "TS20 touch: %02X %02X %02X", ts20_read_buffer[0], ts20_read_buffer[1],
                     ts20_read_buffer[2]);
        }

        // ESP32 cannot clock-stretch here. Its on_request callback runs after the current read completes,
        // so each callback prepares the byte consumed by the next 0x20 -> 0x21 -> 0x22 polling transaction.
        portENTER_CRITICAL(&ts20_cache_lock);
        ts20_reg22 = ts20_read_buffer[0];
        ts20_reg20 = ts20_read_buffer[1];
        ts20_reg21 = ts20_read_buffer[2];
        portEXIT_CRITICAL(&ts20_cache_lock);
    }

    static bool get_and_update_ts20_status() {
        if (i2c_target == nullptr) {
            return false;
        }

        const uint8_t register_address = 0x20;
        uint8_t read_buffer[READ_BUFFER_SIZE] = {};
        if (i2c_master_transmit_receive(i2c_target, &register_address, 1, read_buffer, READ_BUFFER_SIZE, 100) !=
            ESP_OK) {
            clear_ts20_status_cache();
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
        if (0 != (read_buffer[0] & ~0x34) || 0 != (read_buffer[1] & ~0x01) || 0 != read_buffer[2]) {
            clear_ts20_status_cache();
            return false;
        }

        for (size_t i = 0; i < READ_BUFFER_SIZE; ++i) {
            ts20_read_buffer[i] = read_buffer[i];
        }
        update_ts20_status_cache();
        return true;
    }

    static void clear_ts20_status_cache() {
        portENTER_CRITICAL(&ts20_cache_lock);
        ts20_reg20 = 0;
        ts20_reg21 = 0;
        ts20_reg22 = 0;
        portEXIT_CRITICAL(&ts20_cache_lock);
    }

    static void simulate_key_press(const FanKey key_code) {
        if (pending_key_count < sizeof(pending_keys) / sizeof(pending_keys[0])) {
            pending_keys[pending_key_count++] = key_code;
        }
    }

    static void process_key_press() {
        const int64_t now = esp_timer_get_time();
        if (active_key) {
            if (slave_write_counter < KEY_MIN_READS && now < key_release_at) {
                return;
            }
            ts20_read_buffer[0] = 0x00;
            ts20_read_buffer[1] = 0x00;
            ts20_read_buffer[2] = 0x00;
            update_ts20_status_cache();
            active_key = false;
            next_key_at = now + KEY_RELEASE_GAP_US;
        }

        if (pending_key_count == 0 || now < next_key_at) {
            return;
        }

        const FanKey key_code = pending_keys[0];
        for (uint8_t i = 1; i < pending_key_count; ++i) {
            pending_keys[i - 1] = pending_keys[i];
        }
        --pending_key_count;

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
        active_key = true;
        key_release_at = now + KEY_MAX_DURATION_US;
    }
};

}  // namespace xkjg200_fan
}  // namespace esphome
