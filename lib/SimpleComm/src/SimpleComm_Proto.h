#pragma once
#include "SimpleComm.h"

// Allow to use the ProtoType enum without the SimpleComm prefix
using ProtoType = SimpleComm::ProtoType;

/**
 * Test messages for SimpleComm
 * 
 * Examples of using the different types of messages :
 * 
 * 1. MESSAGE : Simple message without response expected
 *    - Used for simple commands (LED, buzzer...)
 *    - No confirmation of reception
 *    - Ex: SetLedMsg
 * 
 * 2. MESSAGE_ACK : Message requiring confirmation
 *    - The receiver sends back the message identically
 *    - Allows to validate reception
 *    - Ex: SetPwmMsg
 * 
 * 3. REQUEST/RESPONSE : Request/response exchange
 *    - The REQUEST message defines its response type (ResponseType)
 *    - The receiver will respond with the correct type
 *    - Responses must be declared before requests in the proto.h file (otherwise the compiler will not find the response type)
 *    - Ex: GetStatusMsg/StatusResponseMsg
 * 
 * FC (Function Code) rules:
 * - User assigns FC between 1-127 for requests/commands
 * - Library automatically assigns FC+128 for responses
 * - FC 0 is reserved (invalid)
 * - When defining a RESPONSE message type, use the same FC 
 *   as its request (the library will handle the FC+128)
 * 
 */

// Message MESSAGE : simple LED command
struct SetLedMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE;
    static constexpr const char* name = "SET_LED";
    static constexpr uint8_t fc = 1;
    uint8_t state;  // 0=OFF, 1=ON
} __attribute__((packed));

// Message MESSAGE_ACK : PWM configuration
struct SetPwmMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE_ACK;
    static constexpr const char* name = "SET_PWM";
    static constexpr uint8_t fc = 2;
    uint8_t pin;    // Pin number
    uint32_t freq;  // Frequency in Hz
} __attribute__((packed));

// Message RESPONSE : status response
struct StatusResponseMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr const char* name = "RSP_STA";
    static constexpr uint8_t fc = 3; // Must be the same as the request FC
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
