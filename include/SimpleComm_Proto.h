#pragma once
#include "SimpleComm.h"

// Allow to use the ProtoType enum without the SimpleComm prefix
using ProtoType = SimpleComm::ProtoType;

/**
 * Test messages for SimpleComm
 * 
 * Examples of using the different types of messages :
 * 
 * 1. FIRE_AND_FORGET : Simple message without response expected
 *    - Used for simple commands (LED, buzzer...)
 *    - No confirmation of reception
 *    - Ex: SetLedMsg
 * 
 * 2. ACK_REQUIRED : Message requiring confirmation
 *    - The receiver sends back the message identically
 *    - Allows to validate reception
 *    - Ex: SetPwmMsg
 * 
 * 3. REQUEST/RESPONSE : Request/response exchange
 *    - The REQUEST message defines its response type (ResponseType)
 *    - The receiver will respond with the correct type
 *    - Ex: GetStatusMsg/StatusResponseMsg
 */

// Message FIRE_AND_FORGET : simple LED command
struct SetLedMsg {
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "SET_LED";
    static constexpr uint8_t fc = 1;
    uint8_t state;  // 0=OFF, 1=ON
} __attribute__((packed));

// Message ACK_REQUIRED : PWM configuration
struct SetPwmMsg {
    static constexpr ProtoType type = ProtoType::ACK_REQUIRED;
    static constexpr const char* name = "SET_PWM";
    static constexpr uint8_t fc = 2;
    uint8_t pin;    // Pin number
    uint32_t freq;  // Frequency in Hz
} __attribute__((packed));

// Message RESPONSE : status response
struct StatusResponseMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr const char* name = "RSP_STA";
    static constexpr uint8_t fc = 4;
    uint8_t state;    // Global state
    uint32_t uptime;  // Uptime since startup
} __attribute__((packed));

// Message REQUEST : status request
struct GetStatusMsg {
    static constexpr ProtoType type = ProtoType::REQUEST;
    static constexpr const char* name = "REQ_STA";
    static constexpr uint8_t fc = 3;
    using ResponseType = StatusResponseMsg;  // Expected response type
} __attribute__((packed));
