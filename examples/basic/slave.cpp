#include <Arduino.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

SimpleComm comm(&Serial);

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Enregistrer tous les protos
    comm.registerProto<SetLedMsg>();
    comm.registerProto<SetPwmMsg>();
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
    
    // Handler pour SetLedMsg (fire & forget)
    comm.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
        digitalWrite(LED_BUILTIN, msg.state);
    });
    
    // Handler pour SetPwmMsg (avec ACK)
    comm.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        analogWrite(msg.pin, msg.freq);
    });
    
    // Handler pour GetStatusMsg (avec réponse automatique)
    comm.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        resp.state = digitalRead(LED_BUILTIN);
        resp.uptime = millis();
    });
}

void loop() {
    comm.poll();  // Traite les messages entrants
} 