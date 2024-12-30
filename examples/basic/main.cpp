#include <Arduino.h>
#include "SimpleComm.h"

// Message "fire and forget"
struct SetLedMsg {
    static constexpr SimpleComm::ProtoType type = SimpleComm::ProtoType::FIRE_AND_FORGET;
    static constexpr uint8_t fc = 1;
    uint8_t state;
};

// Message avec ACK
struct SetPwmMsg {
    static constexpr SimpleComm::ProtoType type = SimpleComm::ProtoType::ACK_REQUIRED;
    static constexpr uint8_t fc = 2;
    uint8_t pin;
    uint32_t freq;
};

// Réponse à GetStatus
struct StatusResponseMsg {
    static constexpr SimpleComm::ProtoType type = SimpleComm::ProtoType::RESPONSE;
    static constexpr uint8_t fc = 4;
    uint8_t state;
    uint32_t uptime;
};

// Requête de status avec lien vers sa réponse
struct GetStatusMsg {
    static constexpr SimpleComm::ProtoType type = SimpleComm::ProtoType::REQUEST;
    static constexpr uint8_t fc = 3;
    using ResponseType = StatusResponseMsg;  // Lien vers le type de réponse
    uint8_t dummy;
};

// Variables globales
SimpleComm comm(&Serial);

void setup() {
    Serial.begin(115200);
    
    // Enregistrer tous les protos
    comm.registerProto<SetLedMsg>();
    comm.registerProto<SetPwmMsg>();
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
    
    // Handler pour SetLedMsg
    comm.onMessage<SetLedMsg>([](const SetLedMsg& msg) {
        digitalWrite(LED_BUILTIN, msg.state);
    });
    
    // Handler pour GetStatusMsg
    comm.onMessage<GetStatusMsg>([](const GetStatusMsg&) {
        StatusResponseMsg resp {
            .state = digitalRead(LED_BUILTIN),
            .uptime = millis()
        };
        comm.sendMsg(resp);
    });
}

void loop() {
    // Traiter les messages reçus
    comm.processRx();
} 