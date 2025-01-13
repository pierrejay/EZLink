# EZLink Documentation

> *Lightweight, robust & user-friendly messaging library for secure structured communication between microcontrollers*

# Table of Contents

- [Introduction](#introduction)
- [EZLink Minimal Example](#ezlink-minimal-example)
- [Design Goals & Motivations](#design-goals--motivations)
- [Key Features](#key-features)
- [Quick Start Guide](#quick-start-guide)
- [Architecture & Design Overview](#architecture--design-overview)
- [Practical Considerations & Best Practices](#practical-considerations--best-practices)
- [Testing & Validation](#testing--validation)
- [Examples](#examples)
- [License](#license)
- [Final Notes](#final-notes)

## Introduction

**EZLink** is a C++ library designed for **robust & low-overhead communication** between microcontrollers (typically via UART, but suitable to any transport layer carrying binary data). Its primary goal is to simplifythe process of exchanging structured binary frames, without requiring you to define your own protocol from scratch or adopt complex serialization frameworks.

The library is built around message structs that serve as message prototypes. You simply declare these structs in a declarative way using native C++ types, and the library handles all the framing, encoding, decoding and validation automatically. Think of it as a very lightweight alternative to Protobuf, combined with a robust transport layer that handles message framing and validation. All of this using simple C++ structs that double as both message definitions and instances, making the protocol self-documenting and easy to maintain.

Key design points:
- Minimal Flash/RAM footprint (as low as ~2KB code size, optimization WIP): suitable for the most constrained microcontrollers such as `STM32F03x` series.
- Simple API with a strong focus on reliability and explicit error reporting. 
- User-friendly, declarative approach to define message prototypes. 
- Built-in support for **messages** (one-way), **acknowledged messages**, and **request/response** flows.  
- Extendable with your own structured types (PODs).  
- **No** code generation toolchain required (unlike Protobuf/Cap’nProto).  
- Works seamlessly on **Arduino** platforms or via custom TX/RX callbacks on bare-metal/RTOS-based firmware as long as your target supports C++11.

Whether you are building a Master/Slave setup over UART or need robust bidirectional communications, **EZLink** aims to keep things **KISS** (Keep It Simple, Stupid) while maximizing runtime safety (CRC checks, well-defined message boundaries, error codes, etc.).

Note: the current implementation is fully tested and functional, but work is still in progress to improve the software. I am currently focused on further reducing code size. The template approach generates lots of duplicate code, taking up ~500B of flash memory for each additional registered message prototype. Serializing message structures earlier in the process should further reduce it down to less than 1KB + ~150B per message prototype.

## EZLink Minimal Example

### Shared Message Definition (Prototypes.h)
```cpp
#include "EZLink.h"
using MsgType = EZLink::MsgType;

// Simple control message with acknowledgment
struct ControlMsg {
    static constexpr MsgType type = MsgType::MESSAGE_ACK;
    static constexpr uint8_t id = 1;
    
    uint8_t channel;     // Which channel to control
    uint16_t value;      // Control value
    uint8_t flags;       // Control flags
} __attribute__((packed));
```

### Master Code
```cpp
#include "EZLink.h"
#include "Prototypes.h"

EZLink master(&UART);

void setup() {
    UART.begin(115200);
    master.begin();
    master.registerRequest<ControlMsg>();
}

void loop() {
    // Send control message
    ControlMsg msg{
        .channel = 1,
        .value = 1000,
        .flags = 0x01
    };
    auto result = master.sendMsgAck(msg);
    if (result != EZLink::SUCCESS) {
        // handle error
    }
    delay(1000);
}
```

### Slave Code
```cpp
#include "EZLink.h"
#include "Prototypes.h"

EZLink slave(&UART);

void setup() {
    UART.begin(115200);
    slave.begin();
    
    // Register message and handler
    slave.registerRequest<ControlMsg>();
    slave.onReceive<ControlMsg>([](const ControlMsg& msg) {
        // Process received message
        processControl(msg.channel, msg.value, msg.flags);
    }); 
    // Acknowledgment is automatically sent back to the master after processing
}

void loop() {
    // Process incoming messages
    slave.poll();
}
```

That's it! A complete bidirectional communication system in ~50 lines of code.

## Design Goals & Motivations

### Origin & Context
- **Primary Use Case**: Secure, structured, but resource-constrained communication between an ESP32 and an STM32 over UART, with the latter having only 16 KB of Flash.  
- **Requirement**: A robust framing and validation system that does not blow up code size or add heavy toolchain dependencies.  

### Why Not Protobuf, SerialCommands, or TinyFrame?
- **Protobuf**: Great for structured data but generally overkill for small microcontrollers due to runtime overhead, code size, and compiler plugin complexities.  
- **SerialCommands**: Simple, but often text-based or lacks robust binary framing (no built-in CRC or typed messages).  
- **TinyFrame**: A similar concept, but still can be less direct if you want strongly typed C++ messages and integrated request/response patterns.

**EZLink** stands out by allowing you to:
- Declare message structures in pure C++ with minimal boilerplate.  
- Rely on compile-time checks (via templates & `static_assert`s) for correctness.  
- Have built-in request/response, acknowledgment flows, and detailed error codes.  
- Keep code size minimal.

## Key Features

1. **Simplicity**:  
   - Minimal user API (just register your prototypes, send, receive, done).  
   - No manual parsing logic; a message is always read/written as a strongly typed C++ struct.

2. **Lightweight & fast**:  
   - Fits into tight STM32 flash constraints (on the order of 2KB compiled).
   - Ultra-low latency communications.

3. **Full Safety by Default**:  
   - CRC16 for integrity checking on all frames.  
   - Acknowledgment or request/response flows if you want guaranteed delivery.  
   - Automatic error detection and cleanup on malformed frames.

4. **Flexible Usage**:  
   - **Synchronous** approach by default (for acknowledgment or request/response).  
   - **Asynchronous** usage based on unidirectional messages for non-blocking communications (see the appropriate section for details).

5. **Transport Agnostic**:  
   - Built-in support for **Arduino `HardwareSerial`**.  
   - Alternative approach: provide your own TX/RX callbacks (interrupt, DMA, RTOS queues, etc.).  

## Quick Start Guide

### Directory Structure
A minimal project layout example is shown below:

```
├── lib/ 
│ └── EZLink/                 <- Library files
│     └── src/
│         ├── ScrollBuffer.h
│         └── EZLink.h 
├── src/ 
│   └── main.cpp              <- Your application code
└── include/ 
    └── Prototypes.h          <- Your message prototypes 
                                (shared between all targets)
```

### 1. Installation
- **Arduino/PlatformIO**: Copy or clone the `lib/EZLink/` folder into your project’s `lib/`.  
- **Bare-metal**: Include the `.h` files in your build system. Ensure you compile `EZLink.h` and `ScrollBuffer.h`.

### 2. Declaring Message Prototypes (Protos)

This can be done directly in your source code, but the best practice is to do this in a separate header file that is shared between all targets. This will ensure that your message definitions are consistent across emitters and receivers.

#### Message Types
- **`MESSAGE`** : A one-way message. No confirmation is expected.  
- **`MESSAGE_ACK`** : A one-way message where the receiver automatically **echoes** the same message back (flipping ID bit 7).  
- **`REQUEST`** : A message that requires a specific typed **`RESPONSE`**.  
- **`RESPONSE`** : A message that answers a specific `REQUEST`.

#### Naming & ID Rules
- Each proto struct declares:  
  - `static constexpr MsgType type;` (see above)
  - `static constexpr uint8_t id;` (must be in **1..127**).  

- Basic rules:
  - `id=0` is invalid.  
  - When you define a `RESPONSE`, it **must** have the same `id` as its request. The library handles linking them together.

- Under the hood:
  - The library automatically uses `id | 0x80` for responses (IDs 128..255). 
  - It automatically links requests and responses together.
  - It can recognize the type of message as well as the request/response relationship.
  - It reject frames with unrecognized IDs, inconsistent request/response types, as well as responses that don't match any request.

Basically, each message received is sure to be of the type expected by its listener (message/request when polling, ACK/response after sending a request).

#### Example Proto Declarations

```cpp
#include "EZLink.h"

using MsgType = EZLink::MsgType;

// Motor control message (one-way command)
struct SetMotorMsg {
    static constexpr MsgType type = MsgType::MESSAGE;
    static constexpr uint8_t id = 1;
    uint8_t motor_id;      // Which motor (1-4)
    int16_t speed;         // -1000 to +1000
    uint8_t acceleration;  // 0-255
    uint8_t mode;         // 0=normal, 1=smooth, 2=precise
} __attribute__((packed));

// PID configuration (requires acknowledgment)
struct ConfigPidMsg {
    static constexpr MsgType type = MsgType::MESSAGE_ACK;
    static constexpr uint8_t id = 2;
    uint8_t channel;    // Which control loop
    float kp;          // Proportional gain
    float ki;          // Integral gain
    float kd;          // Derivative gain
} __attribute__((packed));

// Sensor data response
struct SensorDataResponse {
    static constexpr MsgType type = MsgType::RESPONSE;
    static constexpr uint8_t id = 3;
    int16_t temperature;   // Celsius x100 (-4000 to +15000)
    uint16_t humidity;     // RH x100 (0 to 10000)
    uint32_t pressure;     // Pascal
    uint8_t status;       // Bit flags for sensor status
} __attribute__((packed));

// Request sensor data
struct GetSensorDataMsg {
    static constexpr MsgType type = MsgType::REQUEST;
    static constexpr uint8_t id = 3;
    uint8_t sensorId;      // Target sensor ID
    using ResponseType = SensorDataResponse;
} __attribute__((packed));
```

### 3. Initialization

#### Arduino Usage
Instantiate the `EZLink` object with a reference to the `HardwareSerial` port you want to use for communication.
```cpp
// Basic initialization
EZLink comm(&Serial1);

// With custom response timeout (default: 500ms)
EZLink comm(&Serial1);
comm.setResponseTimeout(1000);  // Set to 1 second

#ifdef EZLINK_DEBUG
// With debug output
EZLink comm(&Serial1,      // Communication serial port
            &Serial,       // Debug serial port
            "DBG_MASTER"); // Optional prefix for debug messages
#endif
```

#### Custom Transport Usage
For non-Arduino environments, provide your own TX/RX callbacks:
```cpp
// Define callbacks
EZLink::TxCallback txCb = [](const uint8_t* data, size_t len) {
    return yourCustomUartWrite(data, len);
};
EZLink::RxCallback rxCb = [](uint8_t* data, size_t maxLen) {
    return yourCustomUartRead(data, maxLen);
};

// Basic initialization
EZLink comm(txCb, rxCb);
// Custom response timeout can also be set
// Debug is also available but it outputs to a Stream which needs to be implemented
```

Make sure to call `begin()` after construction, for example in the `setup()` function if you're using the Arduino framework:
```cpp
void setup() {
    Serial1.begin(115200);  // Initialize hardware first
    comm.begin();           // Then call begin()
}
```
This method is here only to clear the RX buffer since garbage can be present right after UART initialization. Consider adding a small delay before calling `begin()` if you're still catching garbage. Anyway, it shouldn't cause issues:
- In emitter mode, the RX buffer will be cleaned up upon the next message sent.
- In receiver mode, the first call to `poll()` might throw an `ERR_RCV_INVALID_SOF` error, but it will be automatically recovered on the next legit bytes received.

### 4. Registering Message Prototypes
For the communication to work properly, you must:
1. Register all messages you want to send with `registerRequest<T>()`
2. Register all responses you expect to receive with `registerResponse<T>()`
This must be done after instantiating the `EZLink` object.

#### Order of Registration
The order of registration matters:
```cpp
// Correct order:
comm.registerRequest<GetStatusMsg>();       // Register the request first
comm.registerResponse<StatusResponseMsg>(); // Then its response

// Incorrect - will return ERR_REG_INVALID_ID:
comm.registerResponse<StatusResponseMsg>(); // Can't register response first
comm.registerRequest<GetStatusMsg>();       // Request must be registered before
```

#### Registration Requirements by Message Type
Each message type has specific registration requirements:

| Message Type  | Register With       | Notes                                   |
|--------------|---------------------|----------------------------------------|
| MESSAGE      | registerRequest     | One-way messages                       |
| MESSAGE_ACK  | registerRequest     | Messages expecting echo                |
| REQUEST      | registerRequest     | Messages expecting specific response   |
| RESPONSE     | registerResponse    | Must register after its request        |


### 5. Defining Callbacks & Handlers
After registering your messages and responses, you can attach callbacks that will trigger an action upon reception of a message or request (for example in the `setup()` function if you're using the Arduino framework).

- **For `MESSAGE` or `MESSAGE_ACK`**:  
  ```cpp
  comm.registerRequest<SetLedMsg>();
  comm.onReceive<SetLedMsg>([](const SetLedMsg& msg) { 
      /* handle message received */ 
  });
  ```

- **For `REQUEST/RESPONSE` pairs**:  
  ```cpp
  // Register both request and response
  comm.registerRequest<GetStatusMsg>();
  comm.registerResponse<StatusResponseMsg>();
  
  // Then setup the request handler
  comm.onRequest<GetStatusMsg>([](
      const GetStatusMsg& req, 
      StatusResponseMsg& resp
  ) { 
      /* handle request & fill response */ 
  });
  ```

Callbacks will be automatically called by the library when a matching message is received.

Example:
```cpp
slave.registerRequest<SetLedMsg>();
slave.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
    digitalWrite(LED_BUILTIN, msg.state); 
});
```

### 6. Sending & Receiving Messages

#### Sending One-Way Messages (`MESSAGE`)
```cpp
SetLedMsg msg{.state = 1};
auto result = comm.sendMsg(msg);
if (result != EZLink::SUCCESS) {
  // handle error
}
```
- No response is expected; `poll()` can still be used to receive inbound messages from the other side if needed.

#### Sending Acknowledged Messages (`MESSAGE_ACK`)
```cpp
SetPwmMsg pwmMsg{.pin = 5, .freq = 1000};
auto result = comm.sendMsgAck(pwmMsg); 
if (result != EZLink::SUCCESS) {
  // handle error, e.g., ERR_RCV_TIMEOUT
}
```
- Under the hood, the library clears the RX buffer, sends `SetPwmMsg`, and waits for an exact echo (flipped ID).
- If no echo arrives (or it mismatches the data), you get an error.
- The approach is deliberately synchronous (i.e. blocking) to guarantee message delivery and avoid issues with sending the same message multiple times.
- Echo is sent after executing the receiver's `onReceive` callback : when an echo is received, the sender is sure that the receiver is ready to process the next incoming message.

#### Request/Response Exchanges (`REQUEST` & `RESPONSE`)
```cpp
GetStatusMsg req;
StatusResponseMsg resp;
auto result = comm.sendRequest(req, resp);
if (result == EZLink::SUCCESS) {
  // use resp.state, resp.uptime, ...
}
else {
  // error handling
}
```
- Like `MESSAGE_ACK`, it blocks waiting for the correct `RESPONSE`.  
- On the receiver side:
  ```cpp
  comm.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp){
      resp.state = 1;
      resp.uptime = millis();
  });
  ```
- Reponse is sent after executing the receiver's `onRequest` callback : when a response is received, the sender is sure that the receiver is ready to process the next incoming request.


## Architecture & Design Overview

### Communication Model
EZLink implements a simple **frame-based** protocol over a raw byte stream:
- Each frame starts with a **Start of Frame (SOF) byte** `0xAA`.  
- Followed by **length**, **message identifier (ID)**, the **payload**, and **CRC16**.  
```
Frame Format (total size = PAYLOAD_SIZE + 5 bytes overhead)

+-------+-------+--------+----- - - - - ------+---------+
|  SOF  |  LEN  |   ID   |      PAYLOAD       |  CRC16  |
+-------+-------+--------+----- - - - - ------+---------+
  0xAA     N+5    1-127         N bytes         2 bytes

Notes:
- Default PAYLOAD: Maximum 27 bytes (MAX_FRAME_SIZE[32] - FRAME_OVERHEAD[5])
- LEN includes all fields (SOF + LEN + ID + PAYLOAD + CRC16)
- ID: 1-127 for requests, 128-255 for responses (request_id | 0x80)
- CRC16: Calculated over all preceding bytes (SOF to end of PAYLOAD)
```

Internally, the library:
- Buffers incoming bytes in a small ring buffer (`ScrollBuffer`).  
- Scans for the next valid SOF.  
- Verifies length, ID, and CRC to confirm a valid message frame.  
- Calls the corresponding handler if registered.

### Transport Layer
- By default ("Arduino mode"), you provide a `HardwareSerial*`. The library will `write()` frames out and `read()` bytes in automatically.  
- Otherwise, you can supply custom callbacks (`std::function` handlers):
  - `TxCallback` for TX
  - `RxCallback` for RX
- TX/RX callbacks will be called automatically by the library when needed to send and receive bytes to the hardware interface. This allows integration with any hardware driver, buffer, or OS primitives.
- You can control how bytes are sent/received. This is especially useful in RTOS contexts or when using DMA-based ring buffers.
- This callback-based approach allows EZLink to send and receive even during synchronous operations: even if the main thread is blocked, the library still has access to the hardware interface. It also reduces memory footprint by avoiding the need for dedicated buffers where the user would push/pull incoming and outgoing data: existing comm buffers are generally sufficient, especially with short message size like the default 32B - but you can still implement your own if needed.
- The transport layer supports response timeout and transmission errors handling. However, retries are not handled by the library, you need to implement the logic yourself if required.

### Synchronous vs. Asynchronous
- **Synchronous**: For `MESSAGE_ACK` or `REQUEST/RESPONSE`, EZLink blocks internally waiting for the correct acknowledgment or response. It also cleans the receive buffer to avoid stale data.  
- **Asynchronous**: For `MESSAGE` type, no response is expected, so the user can simply send and forget.

If you want purely asynchronous behavior:
- Use only `MESSAGE` types or treat each exchange as unidirectional.  
- You can call `comm.poll()` periodically or within a specific task/thread to process inbound frames.  
- For concurrency, note that EZLink is **not** inherently thread-safe. If multiple threads call `sendMsg()` concurrently, you must protect them externally.

### Buffer Management & Large Frames
- By default, `MAX_FRAME_SIZE` is set to `32`. This limits the maximum payload. You can adjust it in `EZLinkDfs` if needed.  
- The "scroll buffer" approach implemented in the library ensures partial frames or garbage are eventually discarded without losing any valid subsequent frames. 
- Several error cases have been thought of and handled, such as truncated frame, invalid length, invalid SOF, SOF present in data, etc. See unit tests for more details.
- TLDR: as long as you continuously call `poll()` on the receiver side, the message processing pipeline will jump from one frame to the next and end up synchronizing with the next valid frame even if there's garbage in-between.

### CRC Validation & Protocol Robustness
- Every frame includes a 2-byte CRC16.  
- The library discards frames with invalid CRC, tries to find the next valid SOF in the buffer, and continues.  
- If you have extremely noisy lines, consider adding re-transmissions or switching to `MESSAGE_ACK` or `REQUEST/RESPONSE` flows for guaranteed data integrity.

### Error Handling & Diagnostics

#### Overview
EZLink provides detailed error reporting through its `Status` enum and `Result` structure. Each operation returns a `Result` containing both a `status` code and the relevant message `id`.

```cpp
struct Result {
    Status status;  // Status or error code
    uint8_t id;     // Related message ID (0 if not applicable)
};
```

The `==` and `!=` operators are overloaded for `Result` so you can easily check the status without having to extract it from the struct.

#### Success Codes (0-9)
| Code | Name | Description | Common Causes | Solution |
|------|------|-------------|---------------|----------|
| 0 | `SUCCESS` | Operation completed successfully | N/A | N/A |
| 1 | `NOTHING_TO_DO` | No data to process | Empty RX buffer, no messages to handle | Normal condition during polling |

#### Registration Errors (10-19)
| Code | Name | Description | Common Causes | Solution |
|------|------|-------------|---------------|----------|
| 10 | `ERR_ID_ALREADY_REGISTERED` | Message ID already in use | Duplicate ID in message definitions | Ensure unique IDs across all messages |
| 11 | `ERR_TOO_MANY_PROTOS` | Maximum number of prototypes exceeded | Too many registered messages | Increase `MAX_PROTOS` or reduce message types |
| 12 | `ERR_REG_INVALID_ID` | Invalid message ID used | ID=0 or ID≥128 for requests | Use IDs between 1-127 for requests |
| 13 | `ERR_REG_PROTO_MISMATCH` | Protocol type mismatch | Wrong message type registration method | Use correct register method for message type |

#### Communication State Errors (20-29)
| Code | Name | Description | Common Causes | Solution |
|------|------|-------------|---------------|----------|
| 20 | `ERR_BUSY_RECEIVING` | Frame capture in progress | Attempting to send while receiving | Wait and retry, or call cleanupRxBuffer() |

#### Reception Errors (30-39)
| Code | Name | Description | Common Causes | Solution |
|------|------|-------------|---------------|----------|
| 31 | `ERR_RCV_INVALID_ID` | Received unknown message ID | Message not registered, corrupted frame | Register message, check connection |
| 32 | `ERR_RCV_INVALID_SOF` | Invalid start of frame | Noise, desynchronization | Will auto-recover on next valid frame |
| 33 | `ERR_RCV_INVALID_LEN` | Invalid frame length | Corrupted frame, noise | Will auto-recover on next valid frame |
| 34 | `ERR_RCV_PROTO_MISMATCH` | Message type mismatch | Wrong message type received | Check message definitions match |
| 35 | `ERR_RCV_ACK_MISMATCH` | Wrong acknowledgment | Corrupted response, wrong echo | Check connection, retry if needed |
| 36 | `ERR_RCV_RESP_MISMATCH` | Wrong response type | Response size mismatch | Check message definitions match |
| 37 | `ERR_RCV_CRC` | CRC check failed | Corrupted frame, noise | Will auto-recover on next valid frame |
| 38 | `ERR_RCV_UNEXPECTED_RESPONSE` | Unsolicited response | Late response, protocol error | Check timing, clean buffers |
| 39 | `ERR_RCV_TIMEOUT` | Response timeout | No response received | Check connection, increase timeout |

#### Transmission Errors (40-49)
| Code | Name | Description | Common Causes | Solution |
|------|------|-------------|---------------|----------|
| 41 | `ERR_SND_INVALID_ID` | Invalid message ID | Message not registered | Register message before sending |
| 42 | `ERR_SND_EMPTY_DATA` | No data to send | Null pointer, zero length | Check message content |
| 43 | `ERR_SND_PROTO_MISMATCH` | Protocol type mismatch | Wrong message type for operation | Use correct send method |

#### Hardware Errors (50-59)
| Code | Name | Description | Common Causes | Solution |
|------|------|-------------|---------------|----------|
| 50 | `ERR_HW_FLOOD` | Hardware buffer overflow | Too much incoming data | Check for emitter flooding the RX buffer |
| 51 | `ERR_HW_TX_FAILED` | TX hardware failure | Buffer full, hardware error | Check hardware, retry |

#### Basic Error Handling Example
```cpp
auto result = comm.sendMsg(msg);
if (result != EZLink::SUCCESS) {
    // Handle error
    handleError(result.status, result.id);
}
```

### Debug Support
Enable debug output to get detailed error information with the `EZLINK_DEBUG` flag

```cpp
#define EZLINK_DEBUG

EZLink comm(&Serial1,       // Serial port to use for communication
            &Serial,        // Stream for debug output (Serial=USB on ESP32)
            "DBG_MASTER");  // Debug tag (prefix to log messages)
```

The debug output to a `Stream` will provide:
- Hex dumps of frames with errors
- Error descriptions and message IDs
- Frame analysis (SOF, LEN, ID values)
- State transitions

This makes debugging very easy on the Arduino framework, where you can easily attach a custom logger or Serial port to the debug stream.

Here is an example of debug output with the parameters above:
```
// On MASTER side
[DBG_MASTER] RX buffer cleaned
[DBG_MASTER] TX frame (10B) SOF=0xAA LEN=10 ID=0x01 => AA 0A 01 02 00 00 00 00 D4 C6 
[DBG_MASTER] RX valid frame (10B) SOF=0xAA LEN=10 ID=0x81 => AA 0A 81 02 00 00 00 00 00 E6 

// On SLAVE side
[DBG_SLAVE] RX valid frame (10B) SOF=0xAA LEN=10 ID=0x01 => AA 0A 01 01 00 00 00 00 3A 14 
[DBG_SLAVE] TX frame (10B) SOF=0xAA LEN=10 ID=0x81 => AA 0A 81 01 00 00 00 00 EE 34 
```

## Practical Considerations & Best Practices

- **Always** register the same prototypes on both ends: matching types, IDs, and data structures. Make sure to use a common `Prototypes.h` (or similar) file.
- **Never** forget to use the `__attribute__((packed))` keyword on your message structs to avoid padding issues.
- In your `Prototypes.h` file, for `REQUEST/RESPONSE` pairs, ensure the Response struct is declared before the Request struct or use a forward declaration for the Response struct. Otherwise the compiler might throw an error, as when it's parsing the Request struct, he yet doesn't know the `ResponseType` declared inside.
- **Synchronous** patterns (like `sendMsgAck` or `sendRequest`) block until a response arrives or times out. In a busy system, call them from a context where blocking the current thread is acceptable.  
- On the receiver side, ensure the `poll()` method is called regularly during execution of your program, or better, run it in a dedicated thread/task (e.g. FreeRTOS task on ESP32) to keep your main loop clean.
- Keep processing loops short inside callbacks to avoid the sender waiting for a response. If you need to perform long operations, consider using an asynchronous pattern with a second message to indicate the outcome of the operation.
- If you need fully async interactions, do not rely on the built-in synchronous request/response calls. Instead, use your own logic with unidirectional messages.
- If you see errors like `ERR_BUSY_RECEIVING`, it means a partial frame capture is ongoing. Wait or poll more frequently to allow the library to finish capturing the current frame.
- For debugging, enabling `EZLINK_DEBUG` can help identify framing or registration issues quickly.

## Testing & Validation

### Native Unit Tests
Under `test/test_native`, there are `UNITY`-based tests that run on a desktop environment with a mock of `Arduino.h`. These tests cover:
- Registration edge cases (duplicate ID, etc.).  
- Sending/receiving frames, partial frames, CRC errors.  
- Request/response logic and timeouts.  

To run them locally (PlatformIO example):
```sh
pio test -e native_test
```

### Hardware Integration Tests
Under `test/test_hardware`, you’ll find tests that run on actual hardware, exchanging messages across real UART lines. This verifies timing, buffering, and physical transport behavior. The tests have been run on an ESP32S3 dev board in a "loopback" setup (2 UART peripherals chained together with RX/TX pins crossed). 

```sh
pio test -e hardware_test
```

As a side note, the raw performance was measured at a ~2ms round-trip time per `MESSAGE_ACK`/`REQUEST` (w/o processing in the callback) between two ESP32S3s on a 115200 baud UART line, and as low as ~150µs using USB CDC between an MBP M1 (running a Python script to build, send & parse frames) and an ESP32S3, which means that the frame processing overhead is negligible compared to the transmission time.

## Examples

### Arduino Examples
Inside `examples/arduino/`:
- **`loopback.cpp`**: A simple loopback test on a single device with logging.  
- **`master.cpp` & `slave.cpp`**: A typical Master/Slave scenario. The master sends `SetLedMsg` or `GetStatusMsg`, and the slave toggles a pin or responds with uptime data.

### ESP-IDF Examples
Under `examples/esp-idf/`:
- Illustrates using `EZLink` in a typical ESP-IDF project, with non-Arduino drivers.

## License
This library is released under the [MIT License](./LICENSE) (if applicable). Feel free to use and modify it to suit your needs.

## Final Notes
**EZLink**’s approach is intentionally **minimalistic**, but it offers enough structure to avoid “reinventing the wheel” each time you need robust UART-based message handling. With compile-time validation, CRC checks, and straightforward message definitions, you can focus on **business logic** rather than protocol plumbing.

If you encounter issues or have feature requests, please open an issue or PR on the repo! I'll be happy to get feedback and contributions to improve the library.

**Happy hacking!** 