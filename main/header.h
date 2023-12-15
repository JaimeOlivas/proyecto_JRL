/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include <driver/adc.h>
#include <driver/uart.h>
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_vendor.h"
#include "gpio_JRL/gpio.h"
#include "gpio_JRL/gpio.c"

static const char *TAG = "example";

#define I2C_HOST  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           21
#define EXAMPLE_PIN_NUM_SCL           22
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical

#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define S_IN    PIN18 //SW1
#define S_OUT   PIN19 //SW2
#define MODEP    PIN26
#define COOLP    PIN27
#define BTN_ENC PIN4  
#define LED_ENC PIN5
#define DOOR    PIN16
#define FAN     PIN17

#define SP      15
#define RED     PIN14
#define GREEN   PIN13
#define BLUE    PIN12
#define ITERACIONES 15

#define TXD_PIN (UART_PIN_NO_CHANGE)
#define RXD_PIN (UART_PIN_NO_CHANGE)
#define BAUD_RATE 9600

#define MAX_PEOPLE 4 //maximo 8 personas dentro
#define MAX_TEMP  10
#define MIN_TEMP  0

char *status = "";
char dato[41] = "";

bool FLAG_ENC =  false;
bool MODO = false; //false: AUTO true: ON
bool COOL = false; //false: cool true: heat

float tv, tr, y, TEMPAMB, TEMPCOR, tempcor[5] = {0};
int adc_read1 = 0, adc_read2 = 0;


uint8_t people_in = 0;
uint16_t iteraciones = 0;