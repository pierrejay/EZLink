/* @file master_main.cpp 
 *
 * EZLink v2 Arduino Master Example
 * Demonstrates the Quick Start Guide examples: SetLedMsg and GetSensorDataReq
 */

#include <Arduino.h>
#include "EZLink.hpp"
#include "hal/EZLinkHAL-UART_Arduino.hpp"
#include "Prototypes.h"

// Adjust pins to your board (XIAO ESP32S3 defaults)
#define UART1_RX D5
#define UART1_TX D6

// HAL and Link for the master side
ezlink::hal::UART uartHAL(&Serial1);
ezlink::Link link(uartHAL);

void setup() {
    Serial.begin(115200);  // For debug printing

    Serial1.begin(
        115200,
        SERIAL_8N1,
        UART1_RX,
        UART1_TX
    ); // For EZLink communication

    delay(10);

    // Initialize the HAL (if required)
    uartHAL.begin();

    // Register the message and request/response pair
    link.registerMessage<SetLedMsg>();
    link.registerRequest<GetSensorDataReq>();
    link.registerResponse<SensorDataResp>();

    Serial.println("EZLink Master ready");
}

void loop() {
    // Send a one-way LED command
    SetLedMsg msg = {.red = 255, .green = 0, .blue = 128};

    auto result = link.sendMsg(msg);

    if (result != ezlink::SUCCESS) {
        Serial.printf("sendMsg failed: %s\n", toString(result));
    } else {
        Serial.printf("Set LED to R=%d G=%d B=%d", 
            msg.red, msg.green, msg.blue);
    }

    delay(1000);

    // Create a request & response placeholder
    GetSensorDataReq req = {.sensor_id = 42};
    SensorDataResp resp;

    // Send a request and wait for the response
    result = link.sendRequest(req, resp);

    if (result != ezlink::SUCCESS) {
        Serial.printf("Cannot read sensor: %s\n", 
            toString(result));
    } else {
        Serial.printf("Sensor %d: Temperature: %.2f°C, Humidity: %.2f%%\n",
            req.sensor_id, resp.temperature, resp.humidity);
    }

    // Process any inbound frames if needed (non-blocking poll)
    auto pollResult = link.poll();
    if (pollResult != ezlink::NODATA && pollResult != ezlink::SUCCESS) {
        Serial.printf("Poll error: %s (id=%u)\n", 
            toString(pollResult), pollResult.id);
    }

    delay(5000);
}
