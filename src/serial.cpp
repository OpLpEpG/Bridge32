#include <Arduino.h>
#include "HardwareSerial.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include"serial.h"

#define UART_BAUD 125000
#define UART_BAUD1 500000
#define UART_BAUD2 1000000
#define SMD_TURBO 0xFD


uint8_t buf[UART_BUF_LEN];
volatile int read_count = 0; 
volatile int write_count = 0;
volatile bool UartBreack;

static const char *TAG = "uart_events";
static QueueHandle_t queue;
static uint32_t TurboTimer = 0;
#define TIMOUT_TurboTimer 16000UL
static bool FlagTurbo;
static uint8_t Turbo;

void Uart2Send(const uint8_t* buf, size_t len)
{
  // TURBO setup then write next data after TURBO command 
  if (FlagTurbo)
  {
    FlagTurbo = false;
    if (Turbo == 1)
    {
      uart_ll_set_baudrate(UART_LL_GET_HW(UART_NUM_2), UART_BAUD1);
      ESP_LOGE(TAG,"=== turbo: 500");
    }
    else if (Turbo == 2)
    {
      uart_ll_set_baudrate(UART_LL_GET_HW(UART_NUM_2), UART_BAUD2);
      ESP_LOGE(TAG,"=== turbo: 1000");
    }
    else
    {
      TurboTimer = 0;
      uart_ll_set_baudrate(UART_LL_GET_HW(UART_NUM_2), UART_BAUD);      
    }      
  }
  uart_write_bytes(UART_NUM_2, buf, len);
  read_count = 0;
  write_count = 0;
  // TURBO prepare 
  if ((buf[0] == SMD_TURBO) && (len == 4))
  {
    FlagTurbo = true;
    Turbo = buf[1];
    if (!Turbo) ESP_LOGE(TAG,"=== UN");
  }
  // reset TURBO timer
  if (Turbo) TurboTimer = millis();
}

void Uart2TurboHandler(void)
{
  if (TurboTimer && (TurboTimer < millis() - TIMOUT_TurboTimer))
    {
        TurboTimer = 0;
        uart_ll_set_baudrate(UART_LL_GET_HW(UART_NUM_2), UART_BAUD);
        ESP_LOGE(TAG,"UN turbo timeout");
    }
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                //  if (event.timeout_flag)  ESP_LOGE(TAG, "[UART DATA]: %d %d", event.size, event.timeout_flag);
                    uart_read_bytes(UART_NUM_2, &buf[read_count], event.size, portMAX_DELAY);
                    read_count += event.size;
                    UartBreack = event.timeout_flag;
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGE(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_2);
                    xQueueReset(queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGE(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_2);
                    xQueueReset(queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                  //  ESP_LOGE(TAG, "uart rx break");                    
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGE(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

void Uart2Start(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 125000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 0x2000, 256, 20, &queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM_2, 3));
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM_2,UART_MODE_RS485_HALF_DUPLEX));
    // //Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 16, 21, UART_PIN_NO_CHANGE));
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);    
}
