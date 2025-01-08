#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include "esp_timer.h"
#include "EZLink.h"
#include "EZLink_Proto.h"

// UART configuration
#define UART_NUM UART_NUM_1
#define TXD_PIN GPIO_NUM_4
#define RXD_PIN GPIO_NUM_5
#define BUF_SIZE 256

// LED configuration
#define LED_PIN GPIO_NUM_2

// Task configuration
#define STACK_SIZE 4096
#define TASK_PRIORITY 5
#define POLL_DELAY_MS 10

EZLink comm(
    // TX callback
    [](const uint8_t* data, size_t len) {
        return uart_write_bytes(UART_NUM, data, len);
    },
    // RX callback
    [](uint8_t* data, size_t maxLen) {
        return uart_read_bytes(UART_NUM, data, maxLen, 0);
    }
);

// Poll task
void pollTask(void* parameter) {
    while(1) {
        auto result = comm.poll();
        if(result == EZLink::SUCCESS) {
            printf("Message processed\n");
        }
        else if(result != EZLink::NOTHING_TO_DO) {
            printf("Poll error %d\n", result.status);
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
    }
}

extern "C" void app_main() {
    // Configure GPIO
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    
    // Register protos
    comm.registerRequest<SetLedMsg>();
    comm.registerRequest<SetPwmMsg>();
    comm.registerRequest<GetStatusMsg>();
    comm.registerResponse<StatusResponseMsg>();
    
    // Setup handlers
    comm.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
        gpio_set_level(LED_PIN, msg.state);
    });
    
    comm.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        // Use LEDC for PWM
        static bool pwm_initialized = false;
        if (!pwm_initialized) {
            ledc_timer_config_t timer_conf = {
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_10_BIT,
                .timer_num = LEDC_TIMER_0,
                .freq_hz = msg.freq,
                .clk_cfg = LEDC_AUTO_CLK
            };
            ledc_timer_config(&timer_conf);
            
            ledc_channel_config_t channel_conf = {
                .gpio_num = msg.pin,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = LEDC_CHANNEL_0,
                .timer_sel = LEDC_TIMER_0,
                .duty = 512,  // 50%
                .hpoint = 0
            };
            ledc_channel_config(&channel_conf);
            pwm_initialized = true;
        }
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, msg.freq);
    });
    
    comm.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        resp.state = gpio_get_level(LED_PIN);
        resp.uptime = esp_timer_get_time() / 1000;  // Convertir µs en ms
    });

    // Create poll task
    xTaskCreate(
        pollTask,
        "poll_task",
        STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        NULL
    );

    // Main task can now terminate
    printf("Slave initialized and running\n");
} 