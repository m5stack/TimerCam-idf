# Timer Cam Example

[English](README.md) | 中文

## 描述

该案例为基于esp-idf平台开发, 用于**TimerCAM**与上位机软件进行交互，实现图片即时拍摄，定时拍摄功能等功能，支持串口与WiFi两种设备连接模式。

**TimerCAM**是一款基于ESP32的摄像头模块，集成ESP32芯片，板载8M PSRAM，采用300万像素的摄像头（OV3660）可视角66.5°，最高可实现拍摄1600 x 1200分辨率的照片，带有状态指示灯，主打超低功耗设计，通过RTC(BM8563)可实现定时休眠与唤醒，休眠电流可降低至2μA，板上预留电池接口，用户可自行接入电池供电。模块支持WiFi图像传输和USB端口调试，底部HY2.0-4P端口输出，可连接其他外设。

## IDF版本支持

- ESP-IDF v3.x    不支持
- [ESP-IDF v4.0.1](https://github.com/espressif/esp-idf/releases/tag/v4.0.1)   测试正常
- ESP-IDF v4.1.x  不支持

## Cam Lib Base

[reference](https://github.com/espressif/esp32-camera)

## 项目结构

```
.
├── components
│   ├── battery -> Battery ouput control and voltage monitoring 
│   ├── bm8563 -> RTC time control and irq wakeup setting
│   ├── esp32-camera -> Camera control
│   ├── led -> Led brightness control
│   ├── network -> AP or STA connect 
│   └── uart_frame -> Used to connect PC software
├── main
│   ├── protocal.c - > Used to connect PC software
│   ├── cam_cmd.c -> Used to connect PC software
│   ├── app_httpd.c -> web jpeg view
│   ├── factory_test.c -> Base Features test
│   ├── timer_cam_config.c -> Pins of the board And  Used to connect PC software
│   └── main.c
```
## API

### Get img data

```c
camera_fb_t * fb = NULL;
// will get a img frame
fb = esp_camera_fb_get();
// img buf
uint8_t *buf = fb->buf;
// img buf len
unit32_t buf_len = fb->len;

/* --- do some something --- */

// need return img buf
esp_camera_fb_return(fb);
```

### Set ov3660 config

```c
sensor_t *s = esp_camera_sensor_get();
s->set_framesize(s, FRAMESIZE_VGA);
s->set_quality(s, 10);
...
```

查看详情 [sensor.h](components/esp32-camera/driver/include/sensor.h)

# PC上位机软件

[软件下载 & 使用教程](https://docs.m5stack.com/#/en/quick_start/timer_cam/quick_start_cameratool)

## 管脚映射

**摄像头驱动芯片 OV3660 接口**

| *接口*             | *Camera Pin*| *TimerCamera*  |
| :-------------------  | :--------:| :------:  |
| SCCB Clock            | SIOC     |IO23        |
| SCCB Data             | SIOD     |IO25       |
| System Clock          | XCLK     |IO27       |
| Vertical Sync         | VSYNC    |IO22       |
| Horizontal Reference  | HREF     |IO26       |
| Pixel Clock           | PCLK     |IO21       |
| Pixel Data Bit 0      | D0       |IO32       |
| Pixel Data Bit 1      | D1       |IO35       |
| Pixel Data Bit 2      | D2       |IO34       |
| Pixel Data Bit 3      | D3       |IO5        |
| Pixel Data Bit 4      | D4       |IO39       |
| Pixel Data Bit 5      | D5       |IO18       |
| Pixel Data Bit 6      | D6       |IO36       |
| Pixel Data Bit 7      | D7       |IO19       |
| Camera Reset          | RESET    |IO15       |
| Camera Power Down     | PWDN     |-1         |
| Power Supply 3.3V     | 3V3      | 3V3       |
| Ground                | GND      | GND       |

**GROVE 接口**

| *Grove*         | *TimerCamera*  | 
| :-----------: | :------:  | 
| SCL           | IO13      | 
| SDA           | IO4       |
| 5V            | 5V        |
| GND           | GND       | 

**LED 接口**

| *LED*         | *TimerCamera*  |
| :-----------:| :------:  | 
| LED_Pin      | IO2     | 

**BAT 接口**

| *BAT*         | *TimerCamera*  |
| :-----------:| :------:  | 
| BAT_ADC_Pin     | IO33     | 

# 相关链接

[TimerCAM Docs](https://docs.m5stack.com/#/zh_CN/unit/timercam)

[官方商店&购买](https://m5stack-store.myshopify.com/products/esp32-psram-timer-camera-x-ov3660?_pos=2&_sid=461e48736&_ss=r)

