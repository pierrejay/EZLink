#include <Arduino.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

SimpleComm comm(&Serial1);

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Register all protos
    comm.registerProto<SetLedMsg>();
    comm.registerProto<SetPwmMsg>();
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
    
    // Handler for SetLedMsg (fire & forget)
    comm.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
        digitalWrite(LED_BUILTIN, msg.state);
    });
    
    // Handler for SetPwmMsg (with ACK)
    comm.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        analogWrite(msg.pin, msg.freq);
    });
    
    // Handler for GetStatusMsg (with automatic response)
    comm.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        resp.state = digitalRead(LED_BUILTIN);
        resp.uptime = millis();
    });
}

void loop() {
    comm.poll();  // Process incoming messages automatically
} 