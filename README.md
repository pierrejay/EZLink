# SimpleComm Documentation

> **A lightweight, robust, and straightforward library for secure structured communication over UART between microcontrollers (e.g., ESP32 & STM32).**

This document provides a thorough overview of **SimpleComm**, from its motivations and design rationale to detailed usage examples, advanced features, and testing strategies. It is intended for professional developers who need a dependable solution without excessive overhead or complex toolchains.

---

## Table of Contents
1. [Introduction](#introduction)  
2. [Design Goals & Motivations](#design-goals--motivations)  
   - [Why Not Protobuf, SerialCommands, or TinyFrame?](#why-not-protobuf-serialcommands-or-tinyframe)  
3. [Key Features (USPs)](#key-features-usps)  
4. [Architecture & Design Overview](#architecture--design-overview)  
   - [Communication Model](#communication-model)  
   - [Transport Layer](#transport-layer)  
   - [Synchronous vs. Asynchronous](#synchronous-vs-asynchronous)  
5. [Getting Started](#getting-started)  
   - [Directory Structure](#directory-structure)  
   - [Installation](#installation)  
   - [Quick Example (Arduino-Style)](#quick-example-arduino-style)  
6. [Defining & Registering Prototypes (Protos)](#defining--registering-prototypes-protos)  
   - [Message Types](#message-types)  
   - [Naming & ID Rules](#naming--id-rules)  
   - [Example Proto Declarations](#example-proto-declarations)  
   - [Callbacks & Handlers](#callbacks--handlers)  
7. [Sending & Receiving Messages](#sending--receiving-messages)  
   - [Sending One-Way Messages (`MESSAGE`)](#sending-one-way-messages-message)  
   - [Sending Acknowledged Messages (`MESSAGE_ACK`)](#sending-acknowledged-messages-message_ack)  
   - [Request/Response Exchanges (`REQUEST` & `RESPONSE`)](#requestresponse-exchanges-request--response)  
8. [Error Handling & Diagnostics](#error-handling--diagnostics)  
   - [Typical Error Codes](#typical-error-codes)  
   - [Debugging](#debugging)  
9. [Advanced Usage](#advanced-usage)  
   - [Using Custom TX/RX Callbacks (Non-Arduino)](#using-custom-txrx-callbacks-non-arduino)  
   - [Asynchronous Handling](#asynchronous-handling)  
   - [Buffer Management & Large Frames](#buffer-management--large-frames)  
   - [CRC Validation & Protocol Robustness](#crc-validation--protocol-robustness)  
10. [Practical Considerations & Best Practices](#practical-considerations--best-practices)  
11. [Testing & Validation](#testing--validation)  
    - [Native Unit Tests](#native-unit-tests)  
    - [Hardware Integration Tests](#hardware-integration-tests)  
12. [Examples](#examples)  
    - [Arduino Examples](#arduino-examples)  
    - [ESP-IDF Examples](#esp-idf-examples)  
13. [FAQ](#faq)  
14. [License](#license)  

---

## 1. Introduction

**SimpleComm** is a C++ library designed for **lightweight, robust, and secure communication** between microcontrollers (typically via UART). Its primary goal is to **simplify** the process of exchanging structured binary data, without requiring you to define your own protocol from scratch or adopt complex serialization frameworks.

Key design points:
- Minimal Flash/RAM footprint (as low as ~1KB code size).  
- Straightforward API with a strong focus on reliability and explicit error reporting.  
- Built-in support for **messages** (one-way), **acknowledged messages**, and **request/response** flows.  
- Extendable with your own structured types (PODs).  
- **No** code generation toolchain required (unlike Protobuf/Cap’nProto).  
- Works seamlessly on **Arduino** platforms or via custom TX/RX callbacks on bare-metal STM32 or RTOS-based firmware.

Whether you are building a Master/Slave setup over UART or need robust bidirectional communications, **SimpleComm** aims to keep things **KISS** (Keep It Simple, Stupid) while maximizing runtime safety (CRC checks, well-defined message boundaries, error codes, etc.).

---

## 2. Design Goals & Motivations

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
- Keep code size minimal (~1KB).

---

## 3. Key Features (USPs)

1. **Simplicity**:  
   - Minimal user API (just register your prototypes, send, receive, done).  
   - No manual parsing logic; a message is always read/written as a strongly typed C++ struct.

2. **Lightweight Footprint**:  
   - Fits into tight STM32 flash constraints (on the order of 1KB compiled).

3. **Full Safety by Default**:  
   - CRC16 for integrity checking on all frames.  
   - Acknowledgment or request/response flows if you want guaranteed delivery.  
   - Automatic error detection and cleanup on malformed frames.

4. **Flexible Usage**:  
   - **Synchronous** approach by default (for acknowledgment or request/response).  
   - **Asynchronous** usage if you only need unidirectional messages.  
   - Arduino-friendly constructors or custom callbacks for “bare-metal” usage.

5. **Transport Agnostic**:  
   - Built-in support for **Arduino `HardwareSerial`**.  
   - Alternative approach: provide your own TX/RX callbacks (interrupt, DMA, RTOS queues, etc.).  

---

## 4. Architecture & Design Overview

### Communication Model
SimpleComm implements a **frame-based** protocol over a raw byte stream:
- Each frame starts with a **Start of Frame (SOF) byte** `0xAA`.  
- Followed by **length**, **function code (ID)**, the **payload**, and **CRC16**.  

Internally, the library:
- Buffers incoming bytes in a small ring buffer (`ScrollBuffer`).  
- Scans for the next valid SOF.  
- Verifies length, ID, and CRC to confirm a valid message frame.  
- Calls the corresponding handler if registered.

### Transport Layer
- By default (Arduino mode), you provide a `HardwareSerial*`. The library will `write()` frames out and `read()` bytes in.  
- In “expert” mode, you can supply custom `std::function<size_t(const uint8_t*, size_t)>` for TX and `std::function<size_t(uint8_t*, size_t)>` for RX. This allows integration with any hardware driver, buffer, or OS primitives.

### Synchronous vs. Asynchronous
- **Synchronous**: For `MESSAGE_ACK` or `REQUEST/RESPONSE`, SimpleComm blocks internally waiting for the correct acknowledgment or response. It also cleans the receive buffer to avoid stale data.  
- **Asynchronous**: For `MESSAGE` type, no response is expected, so the user can simply send and forget.

---

## 5. Getting Started

### Directory Structure
A typical PlatformIO / Arduino library layout is shown below (taken from this repository’s example):

```
├── examples/ 
│ ├── arduino/ 
│ │ ├── loopback.cpp 
│ │ ├── master.cpp 
│ │ └── slave.cpp 
│ └── esp-idf/ 
│ ├── master.cpp 
│ └── slave.cpp 
├── include/ 
│ └── README 
├── lib/ 
│ ├── SimpleComm/ 
│ │   └── src/ 
│ │       ├── ScrollBuffer.h 
│ │       ├── SimpleComm_Proto.h 
│ │       └── SimpleComm.h 
│ └── README 
├── src/ 
│   └── main.cpp 
├── test/ 
│   ├── test_hardware/ 
│   │   └── test_main.cpp 
│   ├── test_native/ 
│   │   ├── mock/ 
│   │   │   └── Arduino.h 
│   │   └── test_main.cpp 
│   └── README 
└── platformio.ini
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

## 6. Defining & Registering Prototypes (Protos)

### Message Types
1. **`MESSAGE`**  
   A one-way message. No confirmation is expected.  
2. **`MESSAGE_ACK`**  
   A one-way message where the receiver automatically **echoes** the same message back (flipping ID bit 7).  
3. **`REQUEST`**  
   A message that requires a specific typed **`RESPONSE`**.  
4. **`RESPONSE`**  
   A message that answers a specific `REQUEST`.

### Naming & ID Rules
- Each proto struct declares:  
  - `static constexpr ProtoType type;`  
  - `static constexpr const char* name;`  
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
    static constexpr const char* name = "SET_LED";
    static constexpr uint8_t id = 1;
    uint8_t state;  // 0=OFF, 1=ON
} __attribute__((packed));

struct SetPwmMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE_ACK;
    static constexpr const char* name = "SET_PWM";
    static constexpr uint8_t id = 2;
    uint8_t pin;
    uint32_t freq;  
} __attribute__((packed));

struct StatusResponseMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr const char* name = "RSP_STA";
    static constexpr uint8_t id = 3; // same as the request's ID
    uint8_t state;
    uint32_t uptime;
} __attribute__((packed));

struct GetStatusMsg {
    static constexpr ProtoType type = ProtoType::REQUEST;
    static constexpr const char* name = "REQ_STA";
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

Example:
```cpp
slave.registerRequest<SetLedMsg>();
slave.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
    digitalWrite(LED_BUILTIN, msg.state); 
});
```

---

## 7. Sending & Receiving Messages

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

---

## 8. Error Handling & Diagnostics

### Typical Error Codes
SimpleComm’s `Status` enum covers both registration errors and runtime failures. Common values include:
- `SUCCESS`  
- `NOTHING_TO_DO`  
- `ERR_ID_ALREADY_REGISTERED` (ID already in use)  
- `ERR_RCV_INVALID_SOF`, `ERR_RCV_INVALID_LEN`, `ERR_RCV_CRC` (Malformed frames)  
- `ERR_RCV_TIMEOUT` (No response in expected time)  
- `ERR_BUSY_RECEIVING` (Attempted to send a message while a frame is partially captured)  
- `ERR_HW_TX_FAILED` (Transmission hardware buffer full, or TX error)

You receive them as a `Result` struct:  
```cpp
struct Result {
  Status status;
  uint8_t id;  
};
```

### Debugging
If you define `SIMPLECOMM_DEBUG` (e.g., in `platformio.ini` or as a compiler flag) and supply a debug `Stream*`, the library will print logs (hex dumps of frames, error messages, etc.).

```cpp
#define SIMPLECOMM_DEBUG
#define SIMPLECOMM_DEBUG_TAG "DBG"
...
SimpleComm comm(&Serial1, 500, &Serial, "MASTER_INSTANCE");
```

---

## 9. Advanced Usage

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
- You can periodically call `comm.poll()` to process inbound frames.  
- For concurrency, note that SimpleComm is **not** inherently thread-safe. If multiple threads call `sendMsg()` concurrently, you must protect them externally.

### Buffer Management & Large Frames
- By default, `MAX_FRAME_SIZE` is set to `32`. This limits the maximum payload. You can adjust it in `SimpleCommDfs` if needed.  
- A ring buffer approach ensures partial frames or garbage are eventually discarded without losing valid subsequent frames.

### CRC Validation & Protocol Robustness
- Every frame includes a 2-byte CRC16.  
- The library discards frames with invalid CRC, tries to find the next valid SOF in the buffer, and continues.  
- If you have extremely noisy lines, consider adding re-transmissions or switching to `MESSAGE_ACK` or `REQUEST/RESPONSE` flows for guaranteed data integrity.

---

## 10. Practical Considerations & Best Practices

- **Always** register the same prototypes on both ends (matching IDs, data sizes).  
- For reliable request/response, ensure the **response** struct is **registered before** the request struct or use a forward declaration.  
- **Synchronous** patterns (like `sendMsgAck` or `sendRequest`) block until a response arrives or times out. In a busy system, call them from a context where blocking is acceptable.  
- If you need fully async interactions, do not rely on the built-in synchronous request/response calls. Instead, use your own logic with unidirectional messages.  
- If you see errors like `ERR_BUSY_RECEIVING`, it means a partial frame capture is ongoing. Wait or poll more frequently to allow the library to finish capturing the frame.  
- For debugging, enabling `SIMPLECOMM_DEBUG` can help identify framing or registration issues quickly.

---

## 11. Testing & Validation

### Native Unit Tests
Under `test/test_native`, there are `UNITY`-based tests that run on a desktop environment with a mock of `Arduino.h`. These tests cover:
- Registration edge cases (duplicate ID, name collision, etc.).  
- Sending/receiving frames, partial frames, CRC errors.  
- Request/response logic and timeouts.  

To run them locally (PlatformIO example):
```sh
pio test -e native
```

### Hardware Integration Tests
Under `test/test_hardware`, you’ll find tests that run on actual boards (ESP32 or Arduino) exchanging messages across real UART pins. This verifies timing, buffering, and physical transport behavior.  

```sh
pio test -e esp32dev
```

---

## 12. Examples

### Arduino Examples
Inside `examples/arduino/`:
- **`loopback.cpp`**: A simple loopback test on a single device.  
- **`master.cpp` & `slave.cpp`**: A typical Master/Slave scenario. The master sends `SetLedMsg` or `GetStatusMsg`, and the slave toggles a pin or responds with uptime data.

### ESP-IDF Examples
Under `examples/esp-idf/`:
- Illustrates using `SimpleComm` in a typical ESP-IDF project, with non-Arduino drivers.

---

## 13. FAQ

1. **Q**: *I keep getting `ERR_SND_INVALID_ID`: what does this mean?*  
   **A**: The message ID you are trying to send was never registered (`registerRequest<T>()` or `registerResponse<T>()`). Make sure you register **all** your message types first.

2. **Q**: *What if I need bigger than 32-byte frames?*  
   **A**: Increase `MAX_FRAME_SIZE` in `SimpleCommDfs`. This will also increase the ring buffer size. Ensure it still fits your device’s memory constraints.

3. **Q**: *Can I have multiple requests in flight at once?*  
   **A**: The library’s synchronous approach expects you to finish one request/response before starting another. If you need concurrency, consider asynchronous patterns or writing concurrency wrappers around the library.

4. **Q**: *Does it handle half-duplex (RS-485) automatically?*  
   **A**: No built-in half-duplex control. You can integrate your RS-485 driver toggling in the custom TX callback though.

---

## 14. License
This library is released under the [MIT License](./LICENSE) (if applicable). Feel free to use and modify it to suit your needs.

---

## Final Notes
**SimpleComm**’s approach is intentionally **minimalistic**, but it offers enough structure to avoid “reinventing the wheel” each time you need robust UART-based message handling. With compile-time validation, CRC checks, and straightforward message definitions, you can focus on **business logic** rather than protocol plumbing.

If you encounter issues or have feature requests, please open an issue or PR on the GitHub repository (if public), or adapt the code in your private repository if needed.

**Happy hacking!** 