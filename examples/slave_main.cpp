/* @file slave_main.cpp 
 *
 * EZLink v2 Arduino Slave Example
 * Demonstrates the Quick Start Guide examples: SetLedMsg handler and GetSensorDataReq handler
 * 
 */

#include <Arduino.h>
#include "EZLink.hpp"
#include "hal/EZLinkHAL-UART_Arduino.hpp"
#include "Prototypes.h"

// Adjust pins to your board (XIAO ESP32S3 defaults)
#define UART1_RX D7
#define UART1_TX D8

// Define RGB LED pins - adjust to your board
#define LED_R_PIN D0
#define LED_G_PIN D1
#define LED_B_PIN D2

// HAL and Link for the slave side
ezlink::hal::UART uartHAL(&Serial1);
ezlink::Link link(uartHAL);

// MESSAGE handler - apply RGB values to the LED
static void handleSetLed(const SetLedMsg& msg, void* ctx) {
    analogWrite(LED_R_PIN, msg.red);
    analogWrite(LED_G_PIN, msg.green);
    analogWrite(LED_B_PIN, msg.blue);
    Serial.printf("Set LED to R=%d G=%d B=%d\n",
        msg.red, msg.green, msg.blue);
}

// Dummy sensor reading functions (replace with real sensor code)
static float readTempSensor(uint8_t sensor_id) {
    return 23.5 + (sensor_id % 10); // Fake data
}

static float readHumiditySensor(uint8_t sensor_id) {
    return 45.2 + (sensor_id % 20); // Fake data
}

// REQUEST handler - read sensor data and put it in the response object
static void handleGetSensorData(const GetSensorDataReq& req, SensorDataResp& resp, void* ctx) {
    resp.temperature = readTempSensor(req.sensor_id);
    resp.humidity = readHumiditySensor(req.sensor_id);
    Serial.printf("Data request for sensor %d: T=%.2f, RH=%.2f\n",
        req.sensor_id, resp.temperature, resp.humidity);
}

void setup() {
    Serial.begin(115200);  // For debug printing

    Serial1.begin(115200,
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

    // Register the handlers
    link.onMessage<SetLedMsg>(handleSetLed);
    link.onRequest<GetSensorDataReq>(handleGetSensorData);

    Serial.println("EZLink Slave ready");
}

void loop() {
    // Continuously process incoming messages and requests (non-blocking poll)
    link.poll();
}
