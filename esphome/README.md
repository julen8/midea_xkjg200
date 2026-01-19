# ESPHome 智能风扇控制器

本项目将ESP32风扇控制器代码改造为ESPHome组件，可以通过Home Assistant控制风扇。

## 功能特性

- ✅ 风扇开关控制
- ✅ 3档风速调节（低/中/高）
- ✅ 实时风速状态反馈
- ✅ 通过Home Assistant控制
- ✅ 支持OTA无线升级
- ✅ 兼容原有上位机通信协议

## 硬件连接

### I2C 从机总线（响应上位机）
| 引脚 | GPIO | 说明 |
|------|------|------|
| SDA | GPIO21 | I2C数据线 |
| SCL | GPIO22 | I2C时钟线 |
| 地址 | 0x7A | 从机地址 |

### I2C 主机总线（控制TS20触摸芯片）
| 引脚 | GPIO | 说明 |
|------|------|------|
| SDA | GPIO16 | I2C数据线 |
| SCL | GPIO17 | I2C时钟线 |
| 地址 | 0x7A | TS20地址 |

### 风速状态输入引脚（低电平有效）
| 引脚 | GPIO | 说明 |
|------|------|------|
| 低风速 | GPIO25 | 低风速状态指示 |
| 中风速 | GPIO26 | 中风速状态指示 |
| 高风速 | GPIO27 | 高风速状态指示 |

## 文件结构

```
esphome/
├── fan-controller.yaml          # ESPHome主配置文件
├── secrets.yaml                 # 密钥配置（WiFi密码等）
├── README.md                    # 本文档
└── components/
    └── ts20_fan/
        └── ts20_fan.h           # TS20风扇控制器自定义组件
```

## 安装使用

### 1. 配置secrets.yaml

编辑 `secrets.yaml` 文件，填入你的WiFi和API密钥：

```yaml
wifi_ssid: "你的WiFi名称"
wifi_password: "你的WiFi密码"
api_encryption_key: "你的API密钥"  # 可用 openssl rand -base64 32 生成
ota_password: "你的OTA密码"
```

### 2. 安装ESPHome

```bash
pip install esphome
```

### 3. 编译并上传

```bash
cd esphome
esphome run fan-controller.yaml
```

首次上传需要USB连接，之后可以通过OTA无线更新。

### 4. 添加到Home Assistant

设备启动后，Home Assistant会自动发现该设备。在Home Assistant中：

1. 进入 **设置** → **设备与服务**
2. 找到 **ESPHome** 集成
3. 点击 **配置** 添加设备

## Home Assistant 控制

添加设备后，你将获得以下实体：

| 实体 | 类型 | 说明 |
|------|------|------|
| `fan.风扇` | 风扇 | 主风扇控制，支持开关和3档调速 |
| `sensor.当前风速档位` | 传感器 | 显示当前风速档位(0-3) |
| `text_sensor.风速状态` | 文本传感器 | 显示中文风速状态 |
| `switch.重新初始化_ts20` | 开关 | 手动重新初始化TS20芯片 |

### 自动化示例

```yaml
# 定时开启风扇
automation:
  - alias: "晚上开启风扇"
    trigger:
      - platform: time
        at: "22:00:00"
    action:
      - service: fan.turn_on
        target:
          entity_id: fan.风扇
        data:
          percentage: 33  # 低风速
```

## 风速对应关系

| ESPHome Speed | 档位 | 百分比 |
|---------------|------|--------|
| 0 | 关闭 | 0% |
| 1 | 低风速 | 33% |
| 2 | 中风速 | 66% |
| 3 | 高风速 | 100% |

## 调试

查看设备日志：

```bash
esphome logs fan-controller.yaml
```

## 注意事项

1. 本组件同时作为I2C从机和主机，因此使用两个独立的I2C硬件控制器
2. 如果需要修改引脚配置，请同时修改 `fan-controller.yaml` 和 `components/ts20_fan/ts20_fan.h`
3. 首次使用时，上位机需要发送初始化命令来初始化TS20芯片

## 故障排除

### 无法连接WiFi
- 检查 `secrets.yaml` 中的WiFi配置
- 设备会创建名为 "Fan-Controller-AP" 的热点，可通过该热点重新配置

### 风速控制无响应
- 检查I2C接线是否正确
- 使用"重新初始化TS20"开关尝试重新初始化
- 查看日志排查问题

### Home Assistant无法发现设备
- 确保设备和HA在同一网络
- 检查API加密密钥配置
