#include <Arduino.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

SimpleComm comm(&Serial);

void setup() {
    Serial.begin(115200);
    
    // Register protos (even if we don't receive them)
    comm.registerProto<SetLedMsg>();
    comm.registerProto<SetPwmMsg>();
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
}

void loop() {
    // 1. Fire and forget
    SetLedMsg ledOn{.state = 1};
    comm.sendMsg(ledOn);  // No response wait
    delay(1000);
    
    // 2. With ACK
    SetPwmMsg pwm{.pin = 1, .freq = 1000};
    auto result = comm.sendMsgAck(pwm);  // Wait for echo
    if(result == SimpleComm::SUCCESS) {
        Serial.println("PWM set successfully");
    }
    delay(1000);
    
    // 3. Request/Response
    GetStatusMsg req{.dummy = 0};
    StatusResponseMsg resp;
    result = comm.sendRequest(req, resp);  // Wait for typed response
    
    if(result == SimpleComm::SUCCESS) {
        Serial.printf("LED state: %d, uptime: %d ms\n", 
            resp.state, resp.uptime);
    }
    
    delay(1000);
    
    // Process incoming messages (if necessary)
    comm.poll();
} 