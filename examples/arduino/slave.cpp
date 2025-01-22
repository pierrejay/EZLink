#include <Arduino.h>
#include "EZLink.h"
#include "EZLink_Proto.h"

EZLink comm(&Serial1);  // Directly use HardwareSerial

// Handlers definition
void onSetLedMsg(const SetLedMsg& msg) {
    digitalWrite(LED_BUILTIN, msg.state);
}

void onSetPwmMsg(const SetPwmMsg& msg) {
    analogWrite(msg.pin, msg.freq);
}

void onGetStatusMsg(const GetStatusMsg& req, StatusResponseMsg& resp) {
    resp.state = digitalRead(LED_BUILTIN);
    resp.uptime = millis();
}

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
    comm.onReceive<SetLedMsg>(onSetLedMsg);
    comm.onReceive<SetPwmMsg>(onSetPwmMsg);
    comm.onRequest<GetStatusMsg>(onGetStatusMsg);
}

void loop() {
    // Poll regularly in main loop
    comm.poll();
} 