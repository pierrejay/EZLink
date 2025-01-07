# SimpleComm Documentation

> *Lightweight, robust, simple messaging library for secure structured communication between microcontrollers*

This document provides a thorough overview of **SimpleComm**, from its motivations and design rationale to detailed usage examples, advanced features, and testing strategies. It is intended for professional developers who need a dependable solution without excessive overhead or complex toolchains.

## Introduction

**SimpleComm** is a C++ library designed for **lightweight, robust, and secure communication** between microcontrollers (typically via UART, but suitable to any transport layer carrying binary data). Its primary goal is to **simplify** the process of exchanging structured binary frames, without requiring you to define your own protocol from scratch or adopt complex serialization frameworks.

Key design points:
- Minimal Flash/RAM footprint (as low as ~2KB code size, optimization WIP): suitable for the most constrained microcontrollers such as STM32F03x series.
- Simple API with a strong focus on reliability and explicit error reporting. 
- User-friendly, declarative approach to define message prototypes. 
- Built-in support for **messages** (one-way), **acknowledged messages**, and **request/response** flows.  
- Extendable with your own structured types (PODs).  
- **No** code generation toolchain required (unlike Protobuf/Cap’nProto).  
- Works seamlessly on **Arduino** platforms or via custom TX/RX callbacks on bare-metal or RTOS-based firmware as long as your target supports C++11.
- Lighting fast.

Whether you are building a Master/Slave setup over UART or need robust bidirectional communications, **SimpleComm** aims to keep things **KISS** (Keep It Simple, Stupid) while maximizing runtime safety (CRC checks, well-defined message boundaries, error codes, etc.).

Note: the current implementation is fully tested and functional (see below for details), but work is still in progress to improve the software. I am currently focused on further reducing code size. The template approach generates lots of duplicate code, taking up ~500B of flash memory for each additional prototype. Serializing message structures earlier in the process should further reduce the code size down to less than 1KB + ~200B per message prototype.

## SimpleComm Minimal Example

### Shared Message Definition (messages.h)
```cpp
#include "SimpleComm.h"
using ProtoType = SimpleComm::ProtoType;

// Simple control message
struct ControlMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE;
    static constexpr uint8_t id = 1;
    
    uint8_t channel;     // Which channel to control
    uint16_t value;      // Control value
    uint8_t flags;       // Control flags
} __attribute__((packed));
```

### Master Code
```cpp
#include "SimpleComm.h"
#include "messages.h"

SimpleComm master(&Serial1);

void setup() {
    Serial1.begin(115200);
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
    master.sendMsg(msg);
    delay(1000);
}
```

### Slave Code
```cpp
#include "SimpleComm.h"
#include "messages.h"

SimpleComm slave(&Serial1);

void setup() {
    Serial1.begin(115200);
    slave.begin();
    
    // Register message and handler
    slave.registerRequest<ControlMsg>();
    slave.onReceive<ControlMsg>([](const ControlMsg& msg) {
        // Process received message
        processControl(msg.channel, msg.value, msg.flags);
    });
}

void loop() {
    slave.poll();  // Process incoming messages
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

**SimpleComm** stands out by allowing you to:
- Declare message structures in pure C++ with minimal boilerplate.  
- Rely on compile-time checks (via templates & `static_assert`s) for correctness.  
- Have built-in request/response, acknowledgment flows, and detailed error codes.  
- Keep code size minimal.

## Key Features

1. **Simplicity**:  
   - Minimal user API (just register your prototypes, send, receive, done).  
   - No manual parsing logic; a message is always read/written as a strongly typed C++ struct.

2. **Lightweight Footprint**:  
   - Fits into tight STM32 flash constraints (on the order of 2KB compiled).

3. **Full Safety by Default**:  
   - CRC16 for integrity checking on all frames.  
   - Acknowledgment or request/response flows if you want guaranteed delivery.  
   - Automatic error detection and cleanup on malformed frames.

4. **Flexible Usage**:  
   - **Synchronous** approach by default (for acknowledgment or request/response).  
   - **Asynchronous** usage if you only need unidirectional messages.  

5. **Transport Agnostic**:  
   - Built-in support for **Arduino `HardwareSerial`**.  
   - Alternative approach: provide your own TX/RX callbacks (interrupt, DMA, RTOS queues, etc.).  

## Getting Started

### Directory Structure
A minimal project layout example is shown below:

```
├── lib/ 
│ └── SimpleComm/             <- Library files
│     └── src/
│         ├── ScrollBuffer.h
│         └── SimpleComm.h 
├── src/ 
│   └── main.cpp              <- Your application code
└── include/ 
    └── Prototypes.h          <- Your message prototypes 
                                (shared between all targets)
```


### Installation
- **Arduino/PlatformIO**: Copy or clone the `lib/SimpleComm/` folder into your project’s `lib/`. Or use the library manager if it’s published.  
- **Bare-metal**: Include the `.h` files in your build system. Ensure you compile `SimpleComm.h` and `ScrollBuffer.h`.

### Quick Example (Arduino-Style)

**Minimal Master Sketch** (Pseudo-Example):

```cpp
#include <Arduino.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"  // your custom prototypes

SimpleComm master(&Serial1);

void setup() {
  Serial1.begin(115200);
  master.begin();

  // Register your message prototypes
  master.registerRequest<SetLedMsg>();
}

void loop() {
  // Send a simple message
  SetLedMsg msg{.state = 1};
  auto result = master.sendMsg(msg);
  // Check success
  if (result.status != SimpleComm::SUCCESS) {
    // handle error
  }

  // Possibly poll for incoming messages if you expect them:
  auto rxResult = master.poll();
  // ...
  delay(1000);
}
```
On the Slave side, you would do something similar, registering SetLedMsg and providing a handler. See full examples under examples/.

## Defining & Registering Prototypes (Protos)

### Message Types
- **`MESSAGE`** : A one-way message. No confirmation is expected.  
- **`MESSAGE_ACK`** : A one-way message where the receiver automatically **echoes** the same message back (flipping ID bit 7).  
- **`REQUEST`** : A message that requires a specific typed **`RESPONSE`**.  
- **`RESPONSE`** : A message that answers a specific `REQUEST`.

### Naming & ID Rules
- Each proto struct declares:  
  - `static constexpr ProtoType type;`  
  - `static constexpr uint8_t id;` (must be in **1..127** for requests/messages).  
- The library automatically uses `id | 0x80` for responses (IDs 128..255).  
- `id=0` is invalid.  
- When you define a `RESPONSE`, it **must** have the same `id` as its request. The library handles flipping the MSB.  

### Example Proto Declarations

```cpp
#include "SimpleComm.h"

using ProtoType = SimpleComm::ProtoType;

struct SetLedMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE;
    static constexpr uint8_t id = 1;
    uint8_t state;  // 0=OFF, 1=ON
} __attribute__((packed));

struct SetPwmMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE_ACK;
    static constexpr uint8_t id = 2;
    uint8_t pin;
    uint32_t freq;  
} __attribute__((packed));

struct StatusResponseMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr uint8_t id = 3; // same as the request's ID
    uint8_t state;
    uint32_t uptime;
} __attribute__((packed));

struct GetStatusMsg {
    static constexpr ProtoType type = ProtoType::REQUEST;
    static constexpr uint8_t id = 3;
    using ResponseType = StatusResponseMsg;
} __attribute__((packed));
```

### Callbacks & Handlers
After registering a proto, you can attach a callback:
- **For `MESSAGE` or `MESSAGE_ACK`**:  
  `comm.onReceive<T>([](const T& msg) { /* handle */ });`
- **For `REQUEST`**:  
  `comm.onRequest<REQ>([](const REQ& req, typename REQ::ResponseType& resp){ /* fill resp */ });`
Callbacks will be automatically called by the library when a matching message is received.

Example:
```cpp
slave.registerRequest<SetLedMsg>();
slave.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
    digitalWrite(LED_BUILTIN, msg.state); 
});
```

---

## Sending & Receiving Messages

### Sending One-Way Messages (`MESSAGE`)
```cpp
SetLedMsg msg{.state = 1};
auto result = comm.sendMsg(msg);
if (result != SimpleComm::SUCCESS) {
  // handle error
}
```
- No response is expected; `poll()` can still be used to receive inbound messages from the other side if needed.

### Sending Acknowledged Messages (`MESSAGE_ACK`)
```cpp
SetPwmMsg pwmMsg{.pin = 5, .freq = 1000};
auto result = comm.sendMsgAck(pwmMsg); 
if (result != SimpleComm::SUCCESS) {
  // handle error, e.g., ERR_RCV_TIMEOUT
}
```
- Under the hood, the library clears the RX buffer, sends `SetPwmMsg`, and waits for an exact echo (flipped ID).
- If no echo arrives (or it mismatches the data), you get an error.
- The approach is deliberately synchronous (i.e. blocking) to ensure message delivery and avoid issues with sending the same message multiple times.

### Request/Response Exchanges (`REQUEST` & `RESPONSE`)
```cpp
GetStatusMsg req;
StatusResponseMsg resp;
auto result = comm.sendRequest(req, resp);
if (result == SimpleComm::SUCCESS) {
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

## Architecture & Design Overview

### Communication Model
SimpleComm implements a simple **frame-based** protocol over a raw byte stream:
- Each frame starts with a **Start of Frame (SOF) byte** `0xAA`.  
- Followed by **length**, **message identifier (ID)**, the **payload**, and **CRC16**.  
- 
```
Frame Format (total size = PAYLOAD_SIZE + 5 bytes overhead)

+--------+--------+--------+----- - - - - ------+---------+
|  SOF   |  LEN   |   ID   |      PAYLOAD       |  CRC16  |
+--------+--------+--------+----- - - - - ------+---------+
  0xAA      N+5     1-127          N bytes        2 bytes

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
- Otherwise, you can supply custom callbacks: `std::function<size_t(const uint8_t*, size_t)>` for TX and `std::function<size_t(uint8_t*, size_t)>` for RX. They will be called automatically by the library when needed. This allows integration with any hardware driver, buffer, or OS primitives.

### Synchronous vs. Asynchronous
- **Synchronous**: For `MESSAGE_ACK` or `REQUEST/RESPONSE`, SimpleComm blocks internally waiting for the correct acknowledgment or response. It also cleans the receive buffer to avoid stale data.  
- **Asynchronous**: For `MESSAGE` type, no response is expected, so the user can simply send and forget.

### Error Handling & Diagnostics

#### Typical Error Codes
SimpleComm’s `Status` enum covers both registration errors and runtime failures. Common values include:
- `SUCCESS`  
- `NOTHING_TO_DO`  
- `ERR_ID_ALREADY_REGISTERED` (ID already in use)  
- `ERR_RCV_INVALID_SOF`, `ERR_RCV_INVALID_LEN`, `ERR_RCV_CRC` (Malformed frames)  
- `ERR_RCV_TIMEOUT` (No response in expected time)  
- `ERR_BUSY_RECEIVING` (Attempted to send a message while a frame is partially captured)  
- `ERR_HW_TX_FAILED` (Transmission hardware buffer full, or TX error)
-...

You receive them as a `Result` struct:  
```cpp
struct Result {
  Status status; // Status or error code (enum above)
  uint8_t id;    // Message ID (0 if irrelevant)
};
```
The "==" and "!=" operators are overloaded for `Result` so you can easily check the status without having to extract it from the struct.

#### Debugging
If you define `SIMPLECOMM_DEBUG` (e.g., in `platformio.ini` or as a compiler flag) and supply a debug `Stream*`, the library will print logs (hex dumps of frames, error messages, etc.).

```cpp
#define SIMPLECOMM_DEBUG
#define SIMPLECOMM_DEBUG_TAG "DBG"
...
SimpleComm comm(&Serial1, 500, &Serial, "MASTER_INSTANCE");
```
This makes debugging especially easier on the Arduino framework, where you can easily attach a custom logger or Serial port to the debug stream.

## Advanced Usage

### Using Custom TX/RX Callbacks (Non-Arduino)
Instead of passing a `HardwareSerial*`, construct with:
```cpp
SimpleComm::TxCallback txCb = [](const uint8_t* data, size_t len) {
    // Write to your custom UART driver
    return yourCustomUartWrite(data, len);
};
SimpleComm::RxCallback rxCb = [](uint8_t* data, size_t maxLen) {
    // Read from your custom UART driver
    return yourCustomUartRead(data, maxLen);
};

SimpleComm comm(txCb, rxCb);
comm.begin();
```
This way, you control how bytes are sent/received. This is especially useful in RTOS contexts or when using DMA-based ring buffers.

### Asynchronous Handling
If you want purely asynchronous behavior:
- Use only `MESSAGE` types or treat each exchange as unidirectional.  
- You can call `comm.poll()` periodically or within a specific task/thread to process inbound frames.  
- For concurrency, note that SimpleComm is **not** inherently thread-safe. If multiple threads call `sendMsg()` concurrently, you must protect them externally.

### Buffer Management & Large Frames
- By default, `MAX_FRAME_SIZE` is set to `32`. This limits the maximum payload. You can adjust it in `SimpleCommDfs` if needed.  
- The "scroll buffer" approach implemented in the library ensures partial frames or garbage are eventually discarded without losing any valid subsequent frames. 
- Several error cases have been thought of and handled, such as truncated frame, invalid length, invalid SOF, SOF present in data, etc. See unit tests for more details.
- TLDR: as long as you continuously call `poll()` on the receiver side, the message processing pipeline will jump from one frame to the next and end up synchronizing with the next valid frame even if there's garbage in-between.

### CRC Validation & Protocol Robustness
- Every frame includes a 2-byte CRC16.  
- The library discards frames with invalid CRC, tries to find the next valid SOF in the buffer, and continues.  
- If you have extremely noisy lines, consider adding re-transmissions or switching to `MESSAGE_ACK` or `REQUEST/RESPONSE` flows for guaranteed data integrity.

## Practical Considerations & Best Practices

- **Always** register the same prototypes on both ends (matching IDs, data sizes).  
- For reliable request/response, ensure the **response** struct is **registered before** the request struct or use a forward declaration.  
- **Synchronous** patterns (like `sendMsgAck` or `sendRequest`) block until a response arrives or times out. In a busy system, call them from a context where blocking is acceptable.  
- If you need fully async interactions, do not rely on the built-in synchronous request/response calls. Instead, use your own logic with unidirectional messages.  
- If you see errors like `ERR_BUSY_RECEIVING`, it means a partial frame capture is ongoing. Wait or poll more frequently to allow the library to finish capturing the frame.  
- For debugging, enabling `SIMPLECOMM_DEBUG` can help identify framing or registration issues quickly.

---

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

As a side note, the performance was measured at a gross ~0.2 ms round-trip time per `MESSAGE_ACK` (no processing in the callback) using an USB CDC line (the host being a RPi running a Python script sending the frames).

## Examples

### Arduino Examples
Inside `examples/arduino/`:
- **`loopback.cpp`**: A simple loopback test on a single device with logging.  
- **`master.cpp` & `slave.cpp`**: A typical Master/Slave scenario. The master sends `SetLedMsg` or `GetStatusMsg`, and the slave toggles a pin or responds with uptime data.

### ESP-IDF Examples
Under `examples/esp-idf/`:
- Illustrates using `SimpleComm` in a typical ESP-IDF project, with non-Arduino drivers.

### Python Examples
Under `examples/py-master/`:
- Illustrates using `SimpleComm` in a Python script, with a master sending messages to a slave.

## License
This library is released under the [MIT License](./LICENSE) (if applicable). Feel free to use and modify it to suit your needs.

## Final Notes
**SimpleComm**’s approach is intentionally **minimalistic**, but it offers enough structure to avoid “reinventing the wheel” each time you need robust UART-based message handling. With compile-time validation, CRC checks, and straightforward message definitions, you can focus on **business logic** rather than protocol plumbing.

If you encounter issues or have feature requests, please open an issue or PR on the repo! I'll be happy to get feedback and contributions to improve the library.

**Happy hacking!** 