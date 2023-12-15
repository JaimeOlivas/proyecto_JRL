/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "header.h"

void init_io(void){
    gpio_pinMode(RED, OUTPUT);
    gpio_pinMode(GREEN, OUTPUT);
    gpio_pinMode(BLUE, OUTPUT);

    gpio_pinMode(LED_ENC, OUTPUT);
    gpio_pinMode(DOOR, OUTPUT);
    gpio_pinMode(FAN, OUTPUT);

    gpio_pinMode(S_IN, INPUT_PULLUP);
    gpio_pinMode(S_OUT, INPUT_PULLUP);
    gpio_pinMode(BTN_ENC, INPUT_PULLUP);
    gpio_pinMode(MODEP, INPUT);
    gpio_pinMode(COOLP, INPUT);
    gpio_write(RED, HIGH);
    gpio_write(GREEN, HIGH);
    gpio_write(BLUE, HIGH);
    gpio_write(LED_ENC, LOW);
    gpio_write(DOOR, LOW);
    gpio_write(FAN, LOW);
   
}


void init_uart(void){
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0); 
}
/****************************************EJEMPLO OLED*******************************************************/
extern void example_lvgl_demo_ui(lv_disp_t *disp, uint8_t selec);

/* The LVGL port component calls esp_lcd_panel_draw_bitmap API for send data to the screen. There must be called
lvgl_port_flush_ready(disp) after each transaction to display. The best way is to use on_color_trans_done
callback from esp_lcd IO config structure. In IDF 5.1 and higher, it is solved inside LVGL port component. */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t * disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}
/*****************************************************************************************************************/
void init_adc(void){

    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); 
    adc1_config_width(ADC_WIDTH_BIT_12);
    

}    


void app_main(void)
{   
     init_io();
     init_adc();
     init_uart();

/**********************************************EJEMPLO OLED****************************************************************/
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet

    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));


    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t * disp = lvgl_port_add_disp(&disp_cfg);
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
/*****************************************************************************************************************/
    lv_label_set_text(label, "Sistema: OFF\0");
    while(1){

        vTaskDelay(100/ (( TickType_t ) 1000 / 100));
        
        if(gpio_read(BTN_ENC) == 0X00){  //encender y apagar sistema
            if(!FLAG_ENC){              //ENCENDER

                gpio_write(LED_ENC, HIGH);
                lv_label_set_text(label, "SISTEMA: ON\nDOOR: closed\0"); 
                FLAG_ENC = true;    
            }else {                     //APAGAR

                //gpio_write(LED_ENC, HIGH); 

                lv_label_set_text(label, "SISTEMA: OFF\0"); 
                FLAG_ENC = false;
                gpio_write(LED_ENC, LOW);
                gpio_write(FAN, LOW);
            }
            while(gpio_read(BTN_ENC) == 0X00);   

        }
        if(FLAG_ENC){

            for(int l = 0; l <= 4; ++l){ //PROMEDIADO DE LAS MEDIOCIONES DEL ADC PARA LM35
               tempcor[l] = adc1_get_raw( ADC1_CHANNEL_5); //PIN33
            }

            TEMPCOR = 0;
            for (int l = 0; l <= 4; ++l)
            {
                TEMPCOR += tempcor[l];
            }

            TEMPCOR /= 5;
            TEMPCOR = TEMPCOR*330/4095.0;
/****************FORMULA DE EJEMPLO PYTHON*********************************/
            adc_read1 = adc1_get_raw( ADC1_CHANNEL_0);
            tv = 3.3 * adc_read1 / 4095.0;
            tr = tv * 10000.0 / (3.3 - tv);
            y = log(tr/10000.0);
            y = (1.0/298.15) + (y *(1.0/4050.0));
            TEMPAMB = 1.0/y;
            TEMPAMB = TEMPAMB -273.15;  //debe ser temperatura del lm
/**************************************************************************/
            if(gpio_read(S_IN) == 0X00){

                if(people_in < MAX_PEOPLE && TEMPCOR < MAX_TEMP && TEMPCOR > MIN_TEMP){
                    people_in += 1;
                    gpio_write(DOOR, HIGH); 
                    lv_label_set_text(label, "SISTEMA: ON\nDOOR: open\0");
                    vTaskDelay(5000/ (( TickType_t ) 1000 / 100));
                    status = "DOOR: CLOSED\n\r";
                    lv_label_set_text(label, "SISTEMA: ON\nDOOR: closed\0");
                    gpio_write(DOOR, LOW);
                }else if (people_in == MAX_PEOPLE && TEMPCOR < MAX_TEMP && TEMPCOR > MIN_TEMP){
 
                    lv_label_set_text(label, "SISTEMA: ON\nWe are full,\nwait\0"); 
                    vTaskDelay(500/ (( TickType_t ) 1000 / 100));
                } else if ( (TEMPCOR > MAX_TEMP || TEMPCOR < MIN_TEMP)){

                    sprintf(dato, "TEMP_OUT\nOF_RANGE\n TEMPCOR: %0.2f", TEMPCOR);
                    status = dato; 
                    lv_label_set_text(label, status);

                    for(int r = 0; r < 5;++r){
                        gpio_write(RED, HIGH);
                        gpio_write(BLUE, LOW);
                        vTaskDelay(500/ (( TickType_t ) 1000 / 100));
                        gpio_write(BLUE, HIGH);
                        gpio_write(RED, LOW);
                        vTaskDelay(500/ (( TickType_t ) 1000 / 100));
                    }
                    gpio_write(RED, HIGH);
                    lv_label_set_text(label, "SISTEMA: ON\nDOOR: closed\0");

                }
                while(gpio_read(S_IN) == 0X00);
            } else if(gpio_read(S_OUT) == 0X00){
                
                if(people_in>0) people_in -= 1;
                while(gpio_read(S_OUT) == 0X00);

            } else if(gpio_read(MODEP) == 0X00){
                if(MODO)
                    MODO = false;
                else 
                    MODO = true;

                status = MODO? "MODO: ON\n\r": "MODO: AUTO\n\r";
                lv_label_set_text(label, status);
                vTaskDelay(2000/ (( TickType_t ) 1000 / 100));
                while(gpio_read(MODEP) == 0X00);

            }else if(gpio_read(COOLP) == 0X00){
                if(COOL)
                    COOL = false;
                else 
                    COOL = true;
               status = COOL? " HEAT\n\r": " COOL\n\r";
                lv_label_set_text(label, status);
                vTaskDelay(2000/ (( TickType_t ) 1000 / 100));
                while(gpio_read(COOLP) == 0X00);
            }else{

                if (iteraciones == ITERACIONES)
                {
                    status = "SISTEMA: ON | DOOR: CLOSED";
                    uart_write_bytes(UART_NUM_0, status, strlen(status)); 
                    status = MODO?  " MODO:   ON |": " MODO: AUTO |";
                    uart_write_bytes(UART_NUM_0, status, strlen(status));
                    status = COOL? " HEAT |": " COOL |";
                    uart_write_bytes(UART_NUM_0, status, strlen(status));
                    sprintf(dato, " PERSONAS DENTRO: %d |", people_in);
                    status = dato;
                    uart_write_bytes(UART_NUM_0, status, strlen(status));
                    sprintf(dato, " TEMPAMB: %0.2fC    TEMPCORP: %0.2fC\n\r", TEMPAMB, TEMPCOR);
                    status = dato;
                    uart_write_bytes(UART_NUM_0, status, strlen(status));
                    iteraciones = 0;
                }else ++iteraciones;
                
                sprintf(dato, "SISTEMA: ON\nDOOR: closed\n%0.2fC   %0.2fC", TEMPAMB, TEMPCOR);
                status = dato;
                lv_label_set_text(label, status);

                if((!MODO && !COOL && TEMPAMB > SP)|| (!MODO && COOL && TEMPAMB < SP) || MODO) 
                    gpio_write(FAN, HIGH);
                else 
                    gpio_write(FAN, LOW);
                           
                
            }   
        } else {
            gpio_write(FAN, LOW);
        }
    }
}