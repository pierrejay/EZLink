#include <driver/uart.h>
#include <driver/gpio.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

// UART configuration
#define UART_NUM UART_NUM_1
#define TXD_PIN GPIO_NUM_4
#define RXD_PIN GPIO_NUM_5
#define BUF_SIZE 256

// Task configuration
#define STACK_SIZE 4096
#define TASK_PRIORITY 5
#define COMMAND_DELAY_MS 1000

SimpleComm comm(
    // TX callback
    [](const uint8_t* data, size_t len) {
        return uart_write_bytes(UART_NUM, data, len);
    },
    // RX callback
    [](uint8_t* data, size_t maxLen) {
        return uart_read_bytes(UART_NUM, data, maxLen, 0);
    }
);

// Main task that sends commands periodically
void commandTask(void* parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    while(1) {
        // 1. Fire and forget - LED control
        SetLedMsg ledMsg{.state = 1};
        auto result = comm.sendMsg(ledMsg);
        if(result == SimpleComm::SUCCESS) {
            printf("LED ON command sent\n");
        }
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));

        ledMsg.state = 0;
        result = comm.sendMsg(ledMsg);
        if(result == SimpleComm::SUCCESS) {
            printf("LED OFF command sent\n");
        }
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));

        // 2. With ACK - PWM control
        SetPwmMsg pwmMsg{
            .pin = GPIO_NUM_5,
            .freq = 1000
        };
        result = comm.sendMsgAck(pwmMsg);
        if(result == SimpleComm::SUCCESS) {
            printf("PWM command acknowledged\n");
        } else {
            printf("PWM command failed: %d\n", result.status);
        }
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));

        // 3. Request/Response - Get status
        GetStatusMsg req;
        StatusResponseMsg resp;
        result = comm.sendRequest(req, resp);
        if(result == SimpleComm::SUCCESS) {
            printf("Status: LED=%d, uptime=%lu ms\n", 
                resp.state, resp.uptime);
        } else {
            printf("Status request failed: %d\n", result.status);
        }
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    }
}

extern "C" void app_main() {
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

    // Create command task
    xTaskCreate(
        commandTask,
        "command_task",
        STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        NULL
    );

    // Main task can now terminate
    printf("Master initialized and running\n");
} 