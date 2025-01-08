#pragma once
#include "EZLink.h"

// Allow to use the ProtoType enum without the EZLink prefix
using ProtoType = EZLink::ProtoType;

/**
 * Test messages for EZLink
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
 *    - Ex: GetStatusMsg/StatusResponseMsg
 *    - N.B.: Responses must be declared before requests in the proto.h 
 *      file (otherwise the compiler will not find the response type)
 *      or you should use a forward declaration (struct StatusResponseMsg;).
 * 
 * ID (Function Code) rules:
 * - User assigns ID between 1-127 for requests/commands
 * - Library automatically assigns ID+128 for responses
 * - ID 0 is reserved (invalid)
 * - When defining a RESPONSE message type, use the same ID 
 *   as its request (the library will handle the ID+128)
 * 
 */

// MESSAGE proto : simple LED command
struct SetLedMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE;
    static constexpr uint8_t id = 1;
    uint8_t state;  // 0=OFF, 1=ON
} __attribute__((packed));

// MESSAGE_ACK proto : PWM configuration
struct SetPwmMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE_ACK;
    static constexpr uint8_t id = 2;
    uint8_t pin;    // Pin number
    uint32_t freq;  // Frequency in Hz
} __attribute__((packed));

// RESPONSE proto : status response
struct StatusResponseMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr uint8_t id = 3; // Must be the same as the request ID
    uint8_t state;    // Global state
    uint32_t uptime;  // Uptime since startup
} __attribute__((packed));

// REQUEST proto : status request
struct GetStatusMsg {
    static constexpr ProtoType type = ProtoType::REQUEST;
    static constexpr uint8_t id = 3;
    using ResponseType = StatusResponseMsg;  // Expected response type
} __attribute__((packed));
