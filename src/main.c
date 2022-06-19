#include "mainInc.h"
#include "ads1115.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#define LOGLVL              ESP_LOG_DEBUG
#define BUF_SIZE__          1024
#define I2C_PORT            I2C_NUM_0
#define SLV_ADDR_GND        0b1001000
#define SLV_ADDR_VDD        0b1001001
#define SLV_ADDR_SDA        0b1001010
#define SLV_ADDR_SCL        0b1001011
#define INA_DEFAULT         0x40
#define INA_BRIDGE_A0       0x41
#define INA_BRIDGE_A1       0x44
#define INA_BRIDGE_A0_A1    0x45
#define I2C_MASTER_FREQ_HZ  100000
#define TXD_PIN             GPIO_NUM_17
#define RXD_PIN             GPIO_NUM_16
static const char *TAGMAIN = "main_app : ";

static xTaskHandle      readDataTask;
static QueueHandle_t    uart0_queue;
static xTimerHandle     sendDataTimer;
ads1115_t configGV;
ads1115_t configHV;
ads1115_t configPV;
ads1115_t configBV;


void init_i2c(){
    // configGV = ads1115_config(I2C_PORT,SLV_ADDR_GND);
    // ads1115_set_rdy_pin(&configGV, GPIO_NUM_15);
    // ads1115_set_mux(&configGV,ADS1115_MUX_0_GND);
    // ads1115_set_pga(&configGV,ADS1115_FSR_6_144);
    // ads1115_set_mode(&configGV,ADS1115_MODE_CONTINUOUS);

    // configHV = ads1115_config(I2C_PORT,SLV_ADDR_GND);
    // ads1115_set_rdy_pin(&configHV, GPIO_NUM_15);
    // ads1115_set_mux(&configHV,ADS1115_MUX_0_GND);
    // ads1115_set_pga(&configHV,ADS1115_FSR_6_144);
    // ads1115_set_mode(&configHV,ADS1115_MODE_CONTINUOUS);

    // configPV = ads1115_config(I2C_PORT,SLV_ADDR_GND);
    // ads1115_set_rdy_pin(&configPV, GPIO_NUM_15);
    // ads1115_set_mux(&configPV,ADS1115_MUX_0_GND);
    // ads1115_set_pga(&configPV,ADS1115_FSR_6_144);
    // ads1115_set_mode(&configPV,ADS1115_MODE_CONTINUOUS);

    // configBV = ads1115_config(I2C_PORT,SLV_ADDR_GND);
    // ads1115_set_rdy_pin(&configBV, GPIO_NUM_15);
    // ads1115_set_mux(&configBV,ADS1115_MUX_0_GND);
    // ads1115_set_pga(&configBV,ADS1115_FSR_6_144);
    // ads1115_set_mode(&configBV,ADS1115_MODE_CONTINUOUS);

    i2c_config_t config ={
        .mode               =   I2C_MODE_MASTER,
        .sda_io_num         =   21,
        .scl_io_num         =   22,
        .sda_pullup_en      =   GPIO_PULLUP_ENABLE,
        .scl_pullup_en      =   GPIO_PULLUP_ENABLE,
        .master.clk_speed   =   I2C_MASTER_FREQ_HZ,
        .clk_flags          =   0,
    };
    i2c_param_config(I2C_PORT,&config);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT,I2C_MODE_MASTER,BUF_SIZE__*2,BUF_SIZE__,0));

}

void data_capture_task(void* pvParameter){
    // cnsmpData data;
    init_i2c();
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAGMAIN,"int task loop");
    while(true){
        // ESP_LOGI(TAGMAIN,"pin 0 reading :%f",ads1115_get_voltage(&configGV));
        // ESP_LOGI(TAGMAIN,"pin 1 reading :%f",ads1115_get_voltage(&configHV));
        // ESP_LOGI(TAGMAIN,"pin 2 reading :%f",ads1115_get_voltage(&configPV));       
        // ESP_LOGI(TAGMAIN,"pin 3 reading :%f",ads1115_get_voltage(&configBV));
        u_int8_t    data[5];
        i2c_cmd_handle_t cmd;
        esp_err_t ret;
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd); // generate start command
        i2c_master_write_byte(cmd,(SLV_ADDR_GND<<1) | I2C_MASTER_READ,1); // specify address and read command
        i2c_master_read(cmd, data, sizeof(u_int16_t), 0); // read all wanted data
        i2c_master_stop(cmd); // generate stop command
        ret = i2c_master_cmd_begin(I2C_PORT, cmd,pdMS_TO_TICKS(1000)); // send the i2c command
        if(ret){
            ESP_LOGE(TAGMAIN," Error in reading data");
        }
        else
            ESP_LOG_BUFFER_HEX(TAGMAIN,data,5);
        i2c_cmd_link_delete(cmd);
        vTaskDelay(60000);
    }
}

void init_uart(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, BUF_SIZE__ * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void data_send_timer_fun(xTimerHandle timerX){
    const char* data= "HELLO World how is the josh wahhhh \n";
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    ESP_LOGI(TAGMAIN, "Wrote %d bytes", txBytes);
}

// static void rx_task(void *arg)
// {
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     uint8_t* data = (uint8_t*) malloc(BUF_SIZE__*2+1);
//     while (1) {
//         const int rxBytes = uart_read_bytes(UART_NUM_2, data, BUF_SIZE__*2, 1000 / portTICK_PERIOD_MS);
//         if (rxBytes > 0) {
//             data[rxBytes] = 0;
//             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//             ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
//         }
//     }
//     free(data);
// }



void app_main() {
    ESP_LOGI(TAGMAIN, "[APP] Startup..");
    ESP_LOGI(TAGMAIN, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAGMAIN, "[APP] IDF version: %s", esp_get_idf_version());


    init_uart();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // esp_log_level_set(TAGMAIN,LOGLVL);
    BaseType_t TaskStatus ;
    TaskStatus = xTaskCreate(data_capture_task, "uart_rx_task", BUF_SIZE__*4, NULL, 10, readDataTask);
    if(TaskStatus == pdPASS){
        ESP_LOGI(TAGMAIN,"Task created to read sensors Data :)");
    }
    else{
        ESP_LOGE(TAGMAIN,"Not able to create task to read sensor Data! :(");
    }

    sendDataTimer = xTimerCreate("data send timer",pdMS_TO_TICKS(45000),pdTRUE,1,data_send_timer_fun);
    if(sendDataTimer != NULL){
        if(xTimerStart(sendDataTimer,pdMS_TO_TICKS(20000)) != pdPASS){
            ESP_LOGE(TAGMAIN," Not able to start the send data timer :) ");
        }
        else{
            ESP_LOGI(TAGMAIN," Timer Started...! :)");
        }
    }
}