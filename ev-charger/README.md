# 基于 I.MX6ULL 的新能源汽车智能充电桩

## 项目简介
基于 ARM Cortex-A7（I.MX6ULL）+ Linux 的智能充电桩终端，实现电能计量、
多传感器安全监测、LVGL 人机交互与 OneNET 云平台远程上报。

## 硬件平台
- 开发板：I.MX6ULL（Cortex-A7）
- 显示屏：LCD 触摸屏（FrameBuffer + LVGL）
- 传感器：DHT11 枪头温度、LM75 柜内温度、MQ-2 烟雾、ADXL345 震动/倾倒
- 电表：SPT660 三相多功能表（RS485 / Modbus-RTU）

## 技术栈
Linux 字符设备驱动（miscdevice）、设备树、platform 总线、I2C、SPI、
ADC、1-Wire、UART、Modbus-RTU、MQTT、LVGL、FrameBuffer

## 目录结构
- app/：应用层（传感器调度、充电状态机、故障判定、Modbus 电表解析、MQTT 上报）
- drivers/：Linux 内核字符设备驱动（DHT11 / LM75 / ADXL345 / MQ-2）

## 功能模块
- 电能计量：Modbus-RTU 轮询电表，PT/CT 变比换算电压/电流/功率/电量
- 安全监测：过温、烟雾、震动、倾倒越限自动断开继电器并蜂鸣报警
- 人机交互：LVGL 充电状态机界面 + 实时数据曲线
- 云平台：MQTT 每 10 秒上报 OneNET
