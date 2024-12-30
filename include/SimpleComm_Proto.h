#pragma once
#include "SimpleComm.h"

// Utiliser les types de SimpleComm sans préfixe
using ProtoType = SimpleComm::ProtoType;

// Message "fire and forget"
struct SetLedMsg {
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "SET_LED";
    static constexpr uint8_t fc = 1;
    uint8_t state;
} __attribute__((packed));

// Message avec ACK
struct SetPwmMsg {
    static constexpr ProtoType type = SimpleComm::ProtoType::ACK_REQUIRED;
    static constexpr const char* name = "SET_PWM";
    static constexpr uint8_t fc = 2;
    uint8_t pin;
    uint32_t freq;
} __attribute__((packed));

// Réponse à GetStatus
struct StatusResponseMsg {
    static constexpr ProtoType type = SimpleComm::ProtoType::RESPONSE;
    static constexpr const char* name = "RSP_STATUS";
    static constexpr uint8_t fc = 4;
    uint8_t state;
    uint32_t uptime;
} __attribute__((packed));

// Requête de status
struct GetStatusMsg {
    static constexpr ProtoType type = SimpleComm::ProtoType::REQUEST;
    using ResponseType = StatusResponseMsg;
    static constexpr const char* name = "REQ_STATUS";
    static constexpr uint8_t fc = 3;
    uint8_t dummy;
} __attribute__((packed));