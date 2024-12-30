#include <Arduino.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

SimpleComm comm(&Serial);

void setup() {
    Serial.begin(115200);
    
    // Enregistrer les protos (même si on ne reçoit pas)
    comm.registerProto<SetLedMsg>();
    comm.registerProto<SetPwmMsg>();
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
}

void loop() {
    // 1. Fire and forget
    SetLedMsg ledOn{.state = 1};
    comm.sendMsg(ledOn);  // Pas d'attente de réponse
    delay(1000);
    
    // 2. Avec ACK
    SetPwmMsg pwm{.pin = 1, .freq = 1000};
    auto result = comm.sendMsgAck(pwm);  // Attend l'écho
    if(result == SimpleComm::SUCCESS) {
        Serial.println("PWM set successfully");
    }
    delay(1000);
    
    // 3. Requête/Réponse
    GetStatusMsg req{.dummy = 0};
    StatusResponseMsg resp;
    result = comm.sendRequest(req, resp);  // Attend la réponse typée
    
    if(result == SimpleComm::SUCCESS) {
        Serial.printf("LED state: %d, uptime: %d ms\n", 
            resp.state, resp.uptime);
    }
    
    delay(1000);
    
    // Traiter les messages entrants (si nécessaire)
    comm.poll();
} 