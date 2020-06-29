#ifndef __CONFIG_H__
#define __CONFIG_H__

/* Define ------------------------------------------------------------------- */
//#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   15 //software reset will be performed
#define CAM_PIN_XCLK    27
#define CAM_PIN_VSYNC   22
#define CAM_PIN_PCLK    21
#define CAM_PIN_HREF    26

#define CAM_PIN_SIOC    23
#define CAM_PIN_SIOD    25

#define CAM_PIN_D7      19
#define CAM_PIN_D6      36
#define CAM_PIN_D5      18
#define CAM_PIN_D4      39
#define CAM_PIN_D3      5
#define CAM_PIN_D2      34
#define CAM_PIN_D1      35
#define CAM_PIN_D0      32

#define CAM_XCLK_FREQ   10000000

#define CAM_PIN_TX 4
#define CAM_PIN_RX 13

#define CAMERA_LED_GPIO 16

#endif
