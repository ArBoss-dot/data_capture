#include "mainInc.h"
#include "ads1115.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "math.h"

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
static const char *TAGMAIN  = "main_app : ";
// static const double vref    = 3.53*220;
// static const double vRatio       = 0.2;
typedef enum{
    DEFAULT =0,
    SHORT_A0,
    SHORT_A1,
    SHORT_A0_A1,
}INA_ADDRESS;

static xTaskHandle      readDataTask;
static QueueHandle_t    uart0_queue;
static xTimerHandle     sendDataTimer;
ads1115_t configV;


void init_i2c(){
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

esp_err_t get_voltage(uint8_t *data, INA_ADDRESS address){
    i2c_cmd_handle_t cmd;
    esp_err_t ret;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // generate start command
    switch (address)
    {
    case DEFAULT:
        i2c_master_write_byte(cmd,(INA_DEFAULT<<1) | I2C_MASTER_READ,1); // specify address and read command
        break;
    case SHORT_A0:
        i2c_master_write_byte(cmd,(INA_BRIDGE_A0<<1) | I2C_MASTER_READ,1); // specify address and read command
        break;

    case SHORT_A1:ESP_LOGI(TAGMAIN,"Start to measure parameters");
        // static double vMax = 0;
        // vTaskDelay(pdMS_TO_TICKS(10));
        // ads1115_set_mux(&configV,ADS1115_MUX_0_GND);
        // for(int itter=0;itter<80;itter++){
        //     double rawVolt = ads1115_get_voltage(&configV);
        //     vMax = vMax<rawVolt?rawVolt:vMax;
        // }
        // if(vMax>2.7 && vMax < 3.9){
        //     ESP_LOGI(TAGMAIN,"AC Grid voltage measure is %f",vref/vMax);
        // }
        // vMax = 0;
        // vTaskDelay(pdMS_TO_TICKS(10));
        // ads1115_set_mux(&configV,ADS1115_MUX_1_GND);
        // for(int itter=0;itter<80;itter++){
        //     double rawVolt = ads1115_get_voltage(&configV);
        //     vMax = vMax<rawVolt?rawVolt:vMax;
        // }
        // if(vMax>2.7 && vMax < 3.9){
        //     ESP_LOGI(TAGMAIN,"AC Home voltage measure is %f",vref/vMax);
        // }
        // vMax = 0;
        // vTaskDelay(pdMS_TO_TICKS(10));
        // ads1115_set_mux(&configV,ADS1115_MUX_2_GND);
        // for(int itter=0;itter<80;itter++){
        //     double rawVolt = ads1115_get_voltage(&configV);
        //     vMax = vMax<rawVolt?rawVolt:vMax;
        // }
        // if(vMax>0.15 && vMax < 5){
        //     ESP_LOGI(TAGMAIN,"DC Solar PV voltage measured is = %f",vMax/vRatio);
        // }
        
        // vMax = 0;
        // vTaskDelay(pdMS_TO_TICKS(10));
        // ads1115_set_mux(&configV,ADS1115_MUX_3_GND);
        // for(int itter=0;itter<80;itter++){
        //     double rawVolt = ads1115_get_voltage(&configV);
        //     vMax = vMax<rawVolt?rawVolt:vMax;
        // }
        // if(vMax>0.15 && vMax < 5){
        //     ESP_LOGI(TAGMAIN,"DC Battery voltage measured is = %f",vMax/vRatio);
        // }
        
        i2c_master_write_byte(cmd,(INA_BRIDGE_A1<<1) | I2C_MASTER_READ,1); // specify address and read command
        break;

    case SHORT_A0_A1:
        i2c_master_write_byte(cmd,(INA_BRIDGE_A0_A1<<1) | I2C_MASTER_READ,1); // specify address and read command
        break;
    }
    i2c_master_read(cmd, data, sizeof(u_int16_t), 0); // read all wanted data
    i2c_master_stop(cmd); // generate stop command
    ret = i2c_master_cmd_begin(I2C_PORT, cmd,pdMS_TO_TICKS(1000)); // send the i2c command
    i2c_cmd_link_delete(cmd);
    return ret;

}


void data_capture_task(void* pvParameter){
    cnsmpData data = {
        .gridV = 0,
        .gridI = 0,
        .homeV = 0,
        .homeI = 0,
        .rePvV = 0,
        .rePvI = 0,
        .battV = 0,
        .battI = 0,
        .gridP = 0,
        .homeP = 0,
        .rePvP = 0,
        .battP = 0,
    };
    init_i2c();
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAGMAIN,"int task loop");
    configV = ads1115_config(I2C_PORT,SLV_ADDR_GND);
    ads1115_set_rdy_pin(&configV, GPIO_NUM_15);
    ads1115_set_pga(&configV,ADS1115_FSR_6_144);
    ads1115_set_mode(&configV,ADS1115_MODE_CONTINUOUS);
    ads1115_set_sps(&configV,ADS1115_SPS_475);
    while(true){
        // ESP_LOGI(TAGMAIN,"Start to measure parameters");
        // static double vMax = 0;
        // vTaskDelay(pdMS_TO_TICKS(10));
        // ads1115_set_mux(&configV,ADS1115_MUX_0_GND);
        // for(int itter=0;itter<80;itter++){
        //     double rawVolt = ads1115_get_voltage(&configV);
        //     vMax = vMax<rawVolt?rawVolt:vMax;
        // }
        // if(vMax>2.7 && vMax < 3.9){
        //     ESP_LOGI(TAGMAIN,"AC Grid voltage measure is %f",vref/vMax);
        // }
        // vMax = 0;
        // vTaskDelay(pdMS_TO_TICKS(10));
        // ads1115_set_mux(&configV,ADS1115_MUX_1_GND);
        // for(int itter=0;itter<80;itter++){
        //     double rawVolt = ads1115_get_voltage(&configV);
        //     vMax = vMax<rawVolt?rawVolt:vMax;
        // }
        // if(vMax>2.7 && vMax < 3.9){
        //     ESP_LOGI(TAGMAIN,"AC Home voltage measure is %f",vref/vMax);
        // }
        // vMax = 0;
        // vTaskDelay(pdMS_TO_TICKS(10));
        // ads1115_set_mux(&configV,ADS1115_MUX_2_GND);
        // for(int itter=0;itter<80;itter++){
        //     double rawVolt = ads1115_get_voltage(&configV);
        //     vMax = vMax<rawVolt?rawVolt:vMax;
        // }
        // if(vMax>0.15 && vMax < 5){
        //     ESP_LOGI(TAGMAIN,"DC Solar PV voltage measured is = %f",vMax/vRatio);
        // }
        
        // vMax = 0;
        // vTaskDelay(pdMS_TO_TICKS(10));
        // ads1115_set_mux(&configV,ADS1115_MUX_3_GND);
        // for(int itter=0;itter<80;itter++){
        //     double rawVolt = ads1115_get_voltage(&configV);
        //     vMax = vMax<rawVolt?rawVolt:vMax;
        // }
        // if(vMax>0.15 && vMax < 5){
        //     ESP_LOGI(TAGMAIN,"DC Battery voltage measured is = %f",vMax/vRatio);
        // }
        
        vTaskDelay(pdMS_TO_TICKS(300));
        u_int8_t    data[2];
        
        esp_err_t ret = get_voltage(data,DEFAULT);
        if(ret){
            ESP_LOGE(TAGMAIN," Error in reading data Current Data");
        }
        else{

            ESP_LOG_BUFFER_HEX(TAGMAIN,data,5);
            uint16_t value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
            ESP_LOGI(TAGMAIN,"%d",value);
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
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
    // static const char *RX_TASK_TAG = "RX_TASK";
    // esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    // uint8_t* data = (uint8_t*) malloc(BUF_SIZE__*2+1);
    // while (1) {
    //     const int rxBytes = uart_read_bytes(UART_NUM_2, data, BUF_SIZE__*2, 1000 / portTICK_PERIOD_MS);
    //     if (rxBytes > 0) {
    //         data[rxBytes] = 0;
    //         ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
    //         ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
    //     }
    // }
    // free(data);
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

    // sendDataTimer = xTimerCreate("data send timer",pdMS_TO_TICKS(45000),pdTRUE,1,data_send_timer_fun);
    // if(sendDataTimer != NULL){
    //     if(xTimerStart(sendDataTimer,pdMS_TO_TICKS(20000)) != pdPASS){
    //         ESP_LOGE(TAGMAIN," Not able to start the send data timer :) ");
    //     }
    //     else{
    //         ESP_LOGI(TAGMAIN," Timer Started...! :)");
    //     }
    // }
}