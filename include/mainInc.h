#ifndef MAIN_H
#define MAIN_H
#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

typedef struct{
    uint16_t                gridV;
    double                  gridI;
    uint16_t                homeV;
    double                  homeI;  
    uint16_t                rePvV;
    double                  rePvI;   
    uint16_t                battV;
    double                  battI;
    float                   gridP;
    float                   homeP;
    float                   rePvP;
    float                   battP;
}cnsmpData;


#endif