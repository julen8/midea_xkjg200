# 美的 XKJG200 新风控制器 - ESP32 项目

基于 ESP32 的美的 XKJG200 新风系统控制器，通过模拟 TS20 触摸芯片按键来实现远程风扇控制。

## 项目简介

本项目是一个 PlatformIO 项目，使用 ESP32 (NodeMCU-32S) 开发板来控制美的 XKJG200 新风系统。ESP32 同时作为 I2C 主机和从机，实现与上位机（XKJG200 控制面板）和 TS20 触摸芯片的双向通信。

## 项目结构

```
fan-esp32/
├── platformio.ini          # PlatformIO 项目配置
├── src/
│   └── main.cpp            # Arduino 框架主程序
├── esphome/                # ESPHome 实现版本
│   ├── fan-controller.yaml # ESPHome 配置文件
│   ├── components/         # 自定义 ESPHome 组件
│   │   └── xkjg200_fan/    # XKJG200 风扇控制器组件
│   └── README.md           # ESPHome 版本详细说明
├── include/
├── lib/
└── test/
```

## 工作原理

```
┌─────────────────┐      I2C (从机)      ┌─────────────────┐
│   XKJG200       │ ◄──────────────────► │                 │
│   控制面板       │                       │     ESP32       │
│   (上位机)       │      风速LED状态      │                 │
│                 │ ────────────────────► │                 │
└─────────────────┘                       └────────┬────────┘
                                                   │
                                                   │ I2C (主机)
                                                   ▼
                                          ┌─────────────────┐
                                          │   TS20 触摸芯片  │
                                          └─────────────────┘
```

ESP32 同时作为：
1. **I2C 从机**：响应上位机（XKJG200 控制面板）读取触摸芯片状态
2. **I2C 主机**：控制 TS20 触摸芯片
3. **状态监测**：读取风速状态引脚获取当前风速

## 功能特性

- ✅ 支持 3 档风速调节（低/中/高）
- ✅ 实时监测当前风速状态
- ✅ 模拟 TS20 触摸芯片按键输入
- ✅ I2C 双向通信（主机+从机模式）

## 硬件配置

### 开发板
- ESP32 NodeMCU-32S

### 引脚配置

| 功能 | GPIO 引脚 | 说明 |
|-----|----------|------|
| I2C 从机 SDA | GPIO 21 | 连接上位机 |
| I2C 从机 SCL | GPIO 22 | 连接上位机 |
| I2C 主机 SDA | GPIO 16 | 连接 TS20 |
| I2C 主机 SCL | GPIO 17 | 连接 TS20 |
| 低风速状态 | GPIO 25 | 风速 LED 状态输入 |
| 中风速状态 | GPIO 26 | 风速 LED 状态输入 |
| 高风速状态 | GPIO 27 | 风速 LED 状态输入 |

### I2C 地址
- 从机地址（响应上位机）：`0x7A`
- TS20 芯片地址：`0x7A`（ADD 引脚接 VDD）

## 编译与烧录

### 使用 PlatformIO

```bash
# 编译项目
pio run

# 烧录到 ESP32
pio run --target upload

# 打开串口监视器
pio device monitor
```

### 使用 PlatformIO IDE

1. 使用 VS Code 打开项目文件夹
2. 安装 PlatformIO 扩展
3. 点击 PlatformIO 工具栏的 Build/Upload 按钮

## ESPHome 版本

`esphome/` 目录包含本项目的 ESPHome 实现版本，支持通过 Home Assistant 进行远程控制。

### ESPHome 功能特性

- ✅ 通过 Home Assistant 远程控制新风系统
- ✅ 支持 3 档风速调节（低/中/高）
- ✅ 实时显示当前风速状态
- ✅ Web 界面控制
- ✅ OTA 固件更新

详细信息请参阅 [esphome/README.md](esphome/README.md)

## 参考资料

- [TS20 触摸芯片规格书](触摸芯片_TS20_规格书_WJ292953.PDF)
- [PlatformIO 文档](https://docs.platformio.org/)
- [ESPHome 文档](https://esphome.io/)
