# SimpleComm Library

A lightweight, type-safe communication framework library for embedded systems.

## Concept

SimpleComm is designed to provide a robust and efficient communication layer between microcontrollers with minimal overhead and maximum type safety. Unlike more complex solutions like Protocol Buffers, it avoids the need for external compilation tools while maintaining strong type checking at compile time.

### Key Features

- User-friendly API & protocol definition
- Type-safe message definitions using C++ templates
- Zero-copy message handling
- Minimal memory footprint (<1KB Flash typically)
- Runtime protocol validation (CRC, message size)
- Support for different communication patterns:
  - Fire and forget messages
  - Messages with acknowledgment
  - Request/response patterns
- Compile-time checks:
  - Message type validation
  - Function code uniqueness
  - Message size limits

### Design Philosophy

The library follows these principles:
- Keep it simple: messages are just packed structs
- Make it safe: extensive compile-time and runtime checks
- Make it efficient: minimal overhead in both Flash and RAM
- Make it user-friendly: intuitive API with clear error handling

The template-based approach ensures type safety without the complexity of a message compiler. Message definitions are straightforward C++ structs with static attributes defining their behavior.

## Technical Details

### Wire Protocol

Messages are transmitted in frames with the following format:

[SOF 1byte][LEN 1byte][FC 1byte][DATA ...][CRC 1byte]

- SOF: Start of Frame marker (0xAA)
- LEN: Total frame length including all fields
- FC: Function Code identifying the message type
- DATA: Message payload (packed struct)
- CRC: CRC8 checksum of the entire frame

The protocol includes several safety features:
- Frame validation with SOF detection
- Length checking
- CRC verification
- Inter-byte timeout detection
- Response timeout for request/response patterns

### Message Types

Messages must be defined as packed structs with specific static attributes:

```cpp
// Fire and forget: one-way message
struct SetLedMsg {
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr uint8_t fc = 1;
    uint8_t state;
} __attribute__((packed));

// With acknowledgment: expects echo back
struct SetPwmMsg {
    static constexpr ProtoType type = ProtoType::ACK_REQUIRED;
    static constexpr uint8_t fc = 2;
    uint8_t pin;
    uint32_t freq;
} __attribute__((packed));

// Request/Response pair
struct StatusResponseMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr uint8_t fc = 4;
    uint8_t state;
    uint32_t uptime;
} __attribute__((packed));

struct GetStatusMsg {
    static constexpr ProtoType type = ProtoType::REQUEST;
    using ResponseType = StatusResponseMsg;  // Link to response type
    static constexpr uint8_t fc = 3;
    uint8_t dummy;
} __attribute__((packed));
```

### Message Registration and Handlers

#### Registration
All messages must be registered before use, on both master and slave sides:
```cpp
SimpleComm comm(&Serial);

void setup() {
    // Register all message types
    comm.registerProto<SetLedMsg>();
    comm.registerProto<SetPwmMsg>();
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
}
```

Compile-time checks ensure:
- Valid function codes (0 is reserved)
- Message size limits
- Standard layout types (POD)
Runtime checks prevent double registration of the same FC.

#### Message Handlers

Two types of handlers are available, with type-safety enforced at compile time:

1. Simple message handler (for FIRE_AND_FORGET and ACK_REQUIRED):
```cpp
// Handle incoming LED control message
comm.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
    digitalWrite(LED_BUILTIN, msg.state);
});
```

2. Request handler with automatic response (for REQUEST type):
```cpp
// Automatically sends response back
comm.onRequest<GetStatusMsg>([](const GetStatusMsg& req, 
                               StatusResponseMsg& resp) {
    resp.state = digitalRead(LED_BUILTIN);
    resp.uptime = millis();
});
```

### Best Practices

1. Message Definitions
- Keep all message definitions in a separate header file (e.g., `MyProto.h`)
- Group related messages together
- Use clear naming conventions for request/response pairs
- Keep message sizes small, respect MAX_FRAME_SIZE limit

2. Handler Organization
```cpp
// Group related handlers
void setupLedHandlers() {
    comm.onReceive<SetLedMsg>([](const auto& msg) { ... });
    comm.onRequest<GetLedStatusMsg>([](const auto& req, auto& resp) { ... });
}

// Use meaningful lambdas for complex handlers
auto handlePwm = [](const SetPwmMsg& msg) {
    // Validation
    if(msg.freq > MAX_FREQ) return;
    // Processing
    setPwm(msg.pin, msg.freq);
};
comm.onReceive<SetPwmMsg>(handlePwm);
```

3. Error Handling
```cpp
// Always check results when sending messages
auto result = comm.sendMsgAck(pwmMsg);
if(result != SimpleComm::SUCCESS) {
    handleError(result.status, result.fc);
}
```

### Communication Patterns & Blocking Behavior

The library uses blocking calls for ACK_REQUIRED messages and REQUEST/RESPONSE pairs. This design choice was made to ensure:

1. Message Semantics
- FIRE_AND_FORGET: Non-blocking, no guarantee of delivery
- ACK_REQUIRED: Blocks until acknowledgment received
- REQUEST/RESPONSE: Blocks until response received or timeout

2. Robust Error Handling
- Centralized timeout management
- Built-in response validation
- Consistent system states

Example:
```cpp
// Fire and forget - returns immediately
comm.sendMsg(ledMsg);

// With acknowledgment - blocks until ACK received or timeout
comm.sendMsgAck(pwmMsg);

// Request/Response - blocks until response received or timeout
comm.sendRequest(statusReq, statusResp);
```

If non-blocking behavior is needed, several options are available:

1. Use FIRE_AND_FORGET messages:
```cpp
// Master side
comm.sendMsg(commandMsg);  // Returns immediately

// Slave side
comm.onReceive<CommandMsg>([](const auto& msg) {
    // Process command
    // Send status back as separate message
    StatusMsg status = {...};
    comm.sendMsg(status);
});
```

2. Run SimpleComm in a dedicated thread:
```cpp
// Communication thread
void commThread() {
    while(1) {
        // Blocking calls don't affect main thread
        comm.sendRequest(req, resp);
        processResponse(resp);
        delay(1000);
    }
}

// Main thread continues normally
void loop() {
    // Other processing
}
```

This approach provides both safety and flexibility: the library handles communication robustly by default, while allowing users to implement non-blocking patterns when needed.

## Usage Examples

### Complete Master/Slave Implementation

#### Protocol Definition (MyProto.h)
```cpp
#pragma once
#include "SimpleComm.h"

using ProtoType = SimpleComm::ProtoType;

// LED Control
struct SetLedMsg {
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr uint8_t fc = 1;
    uint8_t state;
} __attribute__((packed));

// PWM Control with acknowledgment
struct SetPwmMsg {
    static constexpr ProtoType type = ProtoType::ACK_REQUIRED;
    static constexpr uint8_t fc = 2;
    uint8_t pin;
    uint32_t freq;
} __attribute__((packed));

// Status Request/Response
struct StatusResponseMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr uint8_t fc = 4;
    uint8_t state;
    uint32_t uptime;
} __attribute__((packed));

struct GetStatusMsg {
    static constexpr ProtoType type = ProtoType::REQUEST;
    using ResponseType = StatusResponseMsg;
    static constexpr uint8_t fc = 3;
    uint8_t dummy;
} __attribute__((packed));
```

#### Slave Implementation
```cpp
#include "MyProto.h"

SimpleComm comm(&Serial);

void setup() {
    Serial.begin(115200);
    
    // Register all message types
    comm.registerProto<SetLedMsg>();
    comm.registerProto<SetPwmMsg>();
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
    
    // Simple control message
    comm.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
        digitalWrite(LED_BUILTIN, msg.state);
    });
    
    // Message requiring acknowledgment
    comm.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        analogWrite(msg.pin, msg.freq);
    });
    
    // Request with automatic response
    comm.onRequest<GetStatusMsg>([](const GetStatusMsg& req, 
                                  StatusResponseMsg& resp) {
        resp.state = digitalRead(LED_BUILTIN);
        resp.uptime = millis();
    });
}

void loop() {
    // Process incoming messages
    auto result = comm.poll();
    if(result != SimpleComm::SUCCESS && 
       result != SimpleComm::NOTHING_TO_DO) {
        handleError(result);
    }
}
```

#### Master Implementation
```cpp
#include "MyProto.h"

SimpleComm comm(&Serial);

void setup() {
    Serial.begin(115200);
    
    // Still need to register messages even if only sending
    comm.registerProto<SetLedMsg>();
    comm.registerProto<SetPwmMsg>();
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
}

void loop() {
    // 1. Fire and forget message
    SetLedMsg ledMsg{.state = 1};
    comm.sendMsg(ledMsg);
    
    // 2. Message with acknowledgment
    SetPwmMsg pwmMsg{.pin = 3, .freq = 1000};
    auto result = comm.sendMsgAck(pwmMsg);
    if(result == SimpleComm::SUCCESS) {
        Serial.println("PWM set confirmed");
    }
    
    // 3. Request with response
    GetStatusMsg req{};
    StatusResponseMsg resp;
    result = comm.sendRequest(req, resp);
    if(result == SimpleComm::SUCCESS) {
        Serial.printf("LED: %d, Uptime: %lu\n", 
                     resp.state, resp.uptime);
    }
    
    delay(1000);
}
```

## Future Improvements & Considerations

### Protocol Enhancements
1. Message Queueing
  - Buffer for outgoing messages
  - Priority levels
  - Queue overflow handling

2. Flow Control
  - Sender/receiver window management
  - Bandwidth throttling
  - Buffer full notification

3. Transport Layer Options
  - Multiple UART support
  - SPI/I2C adapters
  - Stream interface abstraction

### Message Features
1. Message Routing
  - Node addressing
  - Network topology support
  - Message forwarding

2. Message Versioning
  - Protocol version field
  - Message structure versioning
  - Backward compatibility

3. Message Extensions
  - Optional fields
  - Message chaining for large payloads
  - Dynamic message sizes

### Reliability Features
1. Communication Patterns
  - Publish/Subscribe patterns
  - Broadcast messages
  - Multi-response requests

2. Error Recovery
  - Message retransmission
  - Connection state management
  - Communication recovery procedures

3. Diagnostics
  - Statistics collection
  - Error logging
  - Protocol analyzer mode

### Development Tools
1. Message Designer
  - Web-based message definition tool
  - Protocol documentation generator
  - Message validation tool

2. Testing Tools
  - Protocol simulator
  - Stress testing tools
  - Conformance test suite

### Additional Features
1. Security
  - Message encryption
  - Authentication
  - Secure pairing

2. Power Management
  - Sleep mode coordination
  - Wake-on-message
  - Low power protocols

3. Debug Support
  - Protocol trace logging
  - Message dump utilities
  - Interactive debugging