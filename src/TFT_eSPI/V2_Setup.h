// V2-only setup. No shared Arduino library settings are used.
#pragma once
#define USER_SETUP_LOADED
#define USER_SETUP_INFO "V2 mixed-bank ILI9486"
#define TFT_PARALLEL_8_BIT
#define ESP32_PARALLEL
#define ILI9486_DRIVER
#define TFT_CS 42
#define TFT_DC 47
#define TFT_WR 41
#define TFT_RST 48
// RD is physically tied to RST; never perform display readback.
#define TFT_RD -1
#define TFT_D0 1
#define TFT_D1 3
#define TFT_D2 19
#define TFT_D3 20
#define TFT_D4 21
#define TFT_D5 38
#define TFT_D6 39
#define TFT_D7 40
#define LOAD_GLCD
#define LOAD_FONT2
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error This local display driver requires ESP32-S3
#endif
#if ARDUINO_USB_CDC_ON_BOOT
#error V2 uses GPIO19/20 for LCD data; select USB CDC On Boot Disabled and UART upload
#endif
