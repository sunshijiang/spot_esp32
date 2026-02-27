# spot-esp32

spot-esp32 是基于 ESP32-S3 与 ESP-IDF 的项目骨架。

## 项目结构

- `main/app_main.c`：程序入口，初始化全部模块并创建 UI 任务。
- `main/ui`：UI 模块骨架。
- `main/display`：显示模块骨架。
- `main/encoder`：编码器模块骨架。
- `main/weld`：焊接控制模块骨架。
- `main/charge`：充电管理模块骨架。
- `main/adc_monitor`：ADC 监测模块骨架。
- `main/buzzer`：蜂鸣器模块骨架。
- `main/drivers/st7735s.*`：ST7735S 驱动骨架。
- `main/drivers/ina226.*`：INA226 驱动骨架。

## 阶段说明

当前为第一阶段骨架版本，仅提供可编译的工程结构与模块初始化接口，不包含具体业务逻辑与外设通信实现。
