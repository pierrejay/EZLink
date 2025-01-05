#include <Arduino.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

SimpleComm comm(&Serial1);  // Directly use HardwareSerial

void setup() {
    Serial.begin(115200);  // Debug
    Serial1.begin(115200); // Communication
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Register protos
    comm.registerRequest<SetLedMsg>();
    comm.registerRequest<SetPwmMsg>();
    comm.registerRequest<GetStatusMsg>();
    comm.registerResponse<StatusResponseMsg>();
    
    // Setup handlers
    comm.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
        digitalWrite(LED_BUILTIN, msg.state);
    });
    
    comm.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        analogWrite(msg.pin, msg.freq);
    });
    
    comm.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        resp.state = digitalRead(LED_BUILTIN);
        resp.uptime = millis();
    });
}

void loop() {
    // Poll regularly in main loop
    comm.poll();
} 