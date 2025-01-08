#include <Arduino.h>
#include "EZLink.h"
#include "EZLink_Proto.h"

EZLink comm(&Serial1);  // Directly use HardwareSerial

void setup() {
    Serial.begin(115200);  // Debug
    Serial1.begin(115200); // Communication
    
    // Register protos
    comm.registerRequest<SetLedMsg>();
    comm.registerRequest<SetPwmMsg>();
    comm.registerRequest<GetStatusMsg>();
    comm.registerResponse<StatusResponseMsg>();
}

void loop() {
    // 1. Fire and forget
    SetLedMsg ledOn{.state = 1};
    comm.sendMsg(ledOn);
    delay(1000);
    
    // 2. With ACK
    SetPwmMsg pwm{.pin = 1, .freq = 1000};
    auto result = comm.sendMsgAck(pwm);
    if(result == EZLink::SUCCESS) {
        Serial.println("PWM set successfully");
    }
    delay(1000);
    
    // 3. Request/Response
    GetStatusMsg req;
    StatusResponseMsg resp;
    result = comm.sendRequest(req, resp);
    if(result == EZLink::SUCCESS) {
        Serial.printf("LED state: %d, uptime: %d ms\n", 
            resp.state, resp.uptime);
    }
    delay(1000);
} 