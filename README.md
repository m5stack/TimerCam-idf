# Timer Cam Example

## Description
... 

used to connect pc software... 

timer wake ...

bat ...


## IDF-Version support
- ESP-IDF v3.x    Not test
- ESP-IDF v4.0.x  Test Ok
- ESP-IDF v4.1.x  Not Test

## Cam Lib Base
https://github.com/espressif/esp32-camera

## File Description
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

Detailed view [sensor.h](components/esp32-camera/driver/include/sensor.h)

# PC Software
todo: add image

download link: ...

how to use: ...

# Buy link
...

