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

// typedef struct{
//     uint16_t                gridV;
//     uint16_t                gridI;
//     uint16_t                homeV;
//     uint16_t                homeI;  
//     uint16_t                rePvV;
//     uint16_t                rePvI;   
//     uint16_t                battV;
//     uint16_t                battI;
//     uint32_t                gridP;
//     uint32_t                homeP;
//     uint32_t                rePvP;
//     uint32_t                battP;
// }cnsmpData;


#endif