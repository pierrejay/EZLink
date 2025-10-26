# EZLink Documentation

A lightweight, robust, and user-friendly messaging library for secure, structured communication between microcontrollers.

- [Introduction](#introduction)
- [Quick Start Guide](#quick-start-guide)
- [Advanced usage & configuration](#advanced-usage--configuration)
  - [Usage with RTOS (single/multi-threading)](#usage-with-rtos-singlemulti-threading)
  - [Thread safety](#thread-safety)
  - [Poll timeout parameter](#poll-timeout-parameter)
  - [Empty messages](#empty-messages)
  - [Runtime configuration](#runtime-configuration)
  - [Build-time configuration](#build-time-configuration)
  - [Error recovery](#error-recovery)
  - [Debug logging](#debug-logging)
  - [Handler context (`void* ctx` parameter in message & request handlers)](#handler-context-void-ctx-parameter-in-message--request-handlers)
  - [Common pitfalls](#common-pitfalls)
- [The HAL Architecture](#the-hal-architecture)
- [Error Handling](#error-handling)
- [Design Rationale for EZLink](#design-rationale-for-ezlink)

## Introduction

**EZLink** is a header-only C++17 library designed to simplify binary communication between microcontrollers (e.g., via UART, CAN, SPI...). Its core principle is simple: you define your messages as plain C++ `struct`s, and EZLink handles the rest: framing, serialization, validation, and routing to the correct callback to trigger actions in your code when messages/requests are received.

The latest version is built on a **Hardware Abstraction Layer (HAL)**, making the core logic completely transport-agnostic. You provide a HAL implementation that tells EZLink how to send and receive raw data frames, and the library takes care of the high-level protocol. The library ships with two reference HALs for UART (Arduino `Serial`) and CANbus (ESP32).

### Key Features

*   **Simple by design**: Define message protocols using clean, self-documenting C++ `struct`s.
*   **Transport-agnostic**: Works with any transport layer (UART, CAN, I2C, SPI, RTOS queues) by implementing a simple HAL interface.
*   **Extremely lightweight**: Minimal Flash and RAM footprint (usually <2KB), suitable for highly constrained MCUs.
*   **Robust by default**: Relies on templates to provide (almost) fully compile-time safety. The provided UART HAL includes framing and CRC checks for data integrity on every frame.
*   **Flexible communication patterns**: Natively supports one-way messages (`MESSAGE`) and blocking request/response flows (`REQUEST`/`RESPONSE`).
*   **RTOS-Ready**: Features an explicit threading model for safe & CPU-efficient use in multi-threaded environments like FreeRTOS.
*   **Zero dependencies**: No external tools or code generation required like Protobuf-based messaging frameworks. Only requires C++17 or up.

## Quick Start Guide

Let's build a complete bidirectional communication system between two devices in five simple steps.

### Step 1: Define Your Messages (`Prototypes.h`)

Create a shared header file for all your message prototypes. This ensures both devices speak the same language. In order to avoid memory-related issues for serialization & parsing, always discard padding by using `__attribute__((packed))`.

```cpp
/* @file Prototypes.h */
#pragma once
#include <EZLink.hpp>

using MsgType = ezlink::MsgType;

// A simple one-way command to set brightness/color of an RGB LED (no response expected)

struct SetLedMsg {
    // Mandatory EZLink metadata
    static constexpr MsgType type = ezlink::MESSAGE;
    static constexpr uint8_t id = 1;
    // Your message schema below
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    /* ... more fields ... */
} __attribute__((packed));


// A request/response pair for fetching sensor data

// 1. First, define the RESPONSE struct - same layout as `MESSAGE`
struct SensorDataResp {
    static constexpr MsgType type = ezlink::RESPONSE;
    static constexpr uint8_t id = 2;      // <- Must match the request ID (see below)
    float temperature;
    float humidity;
} __attribute__((packed));

// 2. Then, define the REQUEST that expects the response - both `id` must match
struct GetSensorDataReq {
    static constexpr MsgType type = ezlink::REQUEST;
    static constexpr uint8_t id = 2;     // <- Unique ID for a request/response pair
    uint8_t sensor_id;
    using ResponseType = SensorDataResp; // <- Link the response type
} __attribute__((packed));
```

The `RESPONSE` struct must be declared before the `REQUEST` struct (or use a forward declaration). Otherwise, the compiler won't know `SensorDataResp` when parsing the request's `ResponseType`.

The `id` field is up to you, but :
- It must be unique for each simple message type or request/response pair.
- The value can be **anything between `0x01 (1)` and `0x7F (127)`** (the maximum number of prototypes itself is capped by the `MAX_PROTOS` setting - default is 16).
- A response's `id` must match its request's `id`, think of it as an op code rather than a unique identifier.

### Step 2: Setup HAL & Link instances

- The `ezlink::Link` class is the core component that manages sending/receiving messages, parsing/serializing/validating data & calling your handlers, regardless of the underlying transport layer or hardware peripherals. It must be instantiated with a HAL instance.

- The HAL is the bridge between EZLink and your hardware. Two reference HAL are included with EZLink:
    - `EZLinkHAL-UART_Arduino.hpp` provides `ezlink::hal::UART` for UART communication using the Arduino framework.
    - `EZLinkHAL-CAN_ESP32.hpp` provides `ezlink::hal::CAN_ESP32` for CANbus communication with an ESP32, relying on the `ESP32-TWAI-CAN` library (wrapper around native ESP-IDF `TWAI` driver).

In your `main.cpp` file (or other source file where you intend to use EZLink), create the objects and initialize the HAL if required:

```cpp
#include "hal/EZLinkHAL-UART_Arduino.hpp"
#include "EZLink.hpp"
#include "Prototypes.h"

// Create an HAL instance
ezlink::hal::UART uartHAL(&Serial1); // Here we use Serial1 for communication

// Create a Link instance with the HAL object
ezlink::Link link(uartHAL);

void setup() {
    Serial.begin(115200);  // For debug printing
    Serial1.begin(115200); // For EZLink communication

    // Initialize the HAL (if required)
    uartHAL.begin();

    // Link doesn't need to be initialized, all checks are done during compilation.

    /* ...registration in the next step... */
}
```

### Step 3: Define your message handlers (slave/receiver side)

Handlers are the functions that will be called automatically by EZLink when a valid message or request is received. They directly manipulate the messages you defined in `Prototypes.h` as plain structs: no cast is needed here - when a handler is called, you can be 100% sure that the message is of the expected type.

There are two different signatures for `MESSAGE` and `REQUEST` handlers, since for `REQUEST`, we not only perform an action but also return a response to the peer.

```cpp
// MESSAGE handler - apply RGB values to the LED
static void handleSetLed(const SetLedMsg& msg, void* ctx) {
    analogWrite(LED_R_PIN, msg.red);
    analogWrite(LED_G_PIN, msg.green);
    analogWrite(LED_B_PIN, msg.blue);
    Serial.printf("Set LED to R=%d G=%d B=%d\n", 
        msg.red, msg.green, msg.blue);
}

// REQUEST handler - read sensor data and put it in the response object
static void handleGetSensorData(const GetSensorDataReq& req, SensorDataResp& resp, void* ctx) {
    // Populate the response object with data
    resp.temperature = readTempSensor(req.sensor_id);
    resp.humidity = readHumiditySensor(req.sensor_id);
    Serial.printf("Data request for sensor %d: T=%.2f, RH=%.2f\n", 
        req.sensor_id, resp.temperature, resp.humidity);
    // EZLink will automatically send the `resp` object to the peer
    // when this handler returns
}
```

**Notes:** 
- All handlers must have a `void* ctx` parameter even if not used. It is necessary for advanced usage where a resource (variable, function...) accessed inside the handler is not available from the handler function itself e.g. when it's not declared globally. It's explained more in detail later in this documentation.
- Your handlers must stay as quick as possible and not block to avoid delaying EZLink operations. Design them like an ISR and if required, offload the work to a separate routine. Consequently, **NEVER** call `sendRequest()` or `sendMsg()` from your message handlers!

### Step 4: Register Prototypes and Handlers

Then, in your `setup()` function or init section of your `main()`, register the prototypes and handlers with the link:

```cpp
void setup() {
    /* ...other init code... */

    // Register the message and request/response pair - on both devices
    link.registerMessage<SetLedMsg>();
    link.registerRequest<GetSensorDataReq>();
    link.registerResponse<SensorDataResp>();

    // Register the handlers - only on the slave/receiver side
    link.onMessage<SetLedMsg>(handleSetLed);
    link.onRequest<GetSensorDataReq>(handleGetSensorData);

    /* ...other init code... */
}
```

**⚠️ Registration order matters:** For `REQUEST/RESPONSE` pairs, you **must** call `registerRequest<>()` before `registerResponse<>()`. Registering in the wrong order will return `ERR_REG_PROTO_MISMATCH`.

### Step 5: Send Messages/Requests and Poll for Incoming Data

On the **sending device**, you can now send messages and requests. On the **receiving device**, you must call `link.poll()` continuously in your main loop to process incoming data. If your node is both a sender and a receiver, you will need to do both in the same loop.

#### Example: Sender Code

```cpp
void loop() {
    // Create a request & response placeholder
    // Here, we request a reading on sensor #42
    GetSensorDataReq req = {.sensor_id = 42};
    SensorDataResp resp;

    // Send a request and wait for the response
    auto result = link.sendRequest(req, resp);

    if (result != ezlink::SUCCESS) {
        Serial.println("Cannot read sensor!");
    } else {
        /* Handle success... */
        Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n",
            resp.temperature, resp.humidity);
    }

    delay(5000); // Loop every 5 seconds
}
```

For one-way messages (no response expected), use `sendMsg()`:

```cpp
// Send a one-way LED command
SetLedMsg msg = {.red = 255, .green = 0, .blue = 128};

auto result = link.sendMsg(msg);

if (result != ezlink::SUCCESS) {
    /* Handle error... */
} else {
    /* Handle success... */
}
```

#### Example: Receiver Code

```cpp
void loop() {
    // Continuously process incoming messages and requests
    // When a request is received, the handler will be called
    // and the response will be sent automatically
    link.poll();

    /* Do other stuff... */
}
```

### That's it!

The following sections provide more details on the library, but with the simple examples above, you should have everything you need to start building your application using EZLink.

## Advanced usage & configuration

### Usage with RTOS (single/multi-threading)

If you're using an RTOS like FreeRTOS, you might want to run `poll()` in a **dedicated background task** that does nothing but listen for incoming messages, while sending requests from other application tasks.

In this case, you need to tell EZLink by defining the build flag `EZLINK_RTOS_POLL_TASK` in your `platformio.ini`, `CMakeLists.txt`, or compiler flags:

```ini
# platformio.ini
build_flags = -DEZLINK_RTOS_POLL_TASK
```

**When to use this flag:**
- You have a dedicated task that calls `poll()` continuously in a loop.
- You call `sendRequest()` from **a different context** than where `poll()` runs.
- Your HAL supports RTOS multi-threading (implements `takeSemaphore()` and `giveSemaphore()`).

**Example: Dedicated Poll Task**

```cpp
ezlink::Link link(canHAL);  // With EZLINK_RTOS_POLL_TASK build flag

// Task 1: Dedicated receiver (high priority)
void pollTask(void* param) {
    while(1) {
        // This task ONLY processes incoming messages
        link.poll(portMAX_DELAY);
    }
}

// Task 2: Application logic (normal priority)
void appTask(void* param) {
    while(1) {
        /* Do application work... */

        // Send a request - this will block until response arrives
        GetSensorDataReq req = {.sensor_id = 42};
        SensorDataResp resp;
        auto result = link.sendRequest(req, resp);

        /* Do other stuff... */
    }
}

void setup() {
    uartHAL.begin();

    /* Register protos and handlers... */

    // Create poll & application tasks
    xTaskCreate(pollTask, "Poll", 2048, NULL, 5, NULL);
    xTaskCreate(appTask, "App", 2048, NULL, 3, NULL);
}
```

**How it works:** When `sendRequest()` is called from `appTask`, if the flag is set, it sends the request and then waits on a semaphore. The `pollTask` receives the response and signals the semaphore to wake up the waiting task.

**⚠️ Important:** 
- If you define `EZLINK_RTOS_POLL_TASK` but your HAL doesn't support it, you'll get a clear compilation error with instructions on how to fix it.
- If you're using a **single context** (loop/task/thread...) for **all EZLink communication** (standard Arduino or a single FreeRTOS task), do **NOT** set this flag!
- For single receivers OR if you are only sending asynchronous messages (no request/response), this doesn't matter, the flag isn't necessary since there is no concurrency in this case.

### Thread safety

**Important:** EZLink is **not fully thread-safe**. Follow these rules to avoid deadlocks or race conditions:

- **`poll()`**: Only **one thread/task** should explicitly call `poll()` in your entire program.
- **`sendRequest()`**: **Not thread-safe in all cases** even if your HAL supports concurrent send operations - protect with mutexes if called from multiple threads.
- **`sendMsg()`**: Depends if your HAL supports concurrent send operations. In doubt, avoid concurrent calls at all.
- **NEVER** call `sendRequest()` or `sendMsg()` from your message handlers! Beyond potential concurrency issues, this is bad practice: your callbacks must be as quick as possible and not block like explained in the previous section.
- **Registration methods**: Call only during initialization from a single thread.

```cpp
// WRONG: Multiple threads calling sendRequest()
void taskA() { link.sendRequest(reqA, respA); }  // ❌ Race condition
void taskB() { link.sendRequest(reqB, respB); }  // ❌ Race condition

// RIGHT: Serialize access with mutex
SemaphoreHandle_t linkMutex = xSemaphoreCreateMutex();
void taskA() {
    xSemaphoreTake(linkMutex, portMAX_DELAY);
    link.sendRequest(reqA, respA);
    xSemaphoreGive(linkMutex);
}
```

**Note to HAL developers:** if your HAL supports RTOS multi-threading, it MUST handle concurrent calls to `sendFrame()` somehow (queue, mutex...), since `Link::poll()` executing in the background task may call `HAL::sendFrame()` to reply to an incoming request, while another thread calls `Link::sendRequest()` in the meantime, which itself will also call `HAL::sendFrame()`.

### Poll timeout parameter

The `poll()` methods allows to pass an `uint32_t timeoutMs` parameter as argument. This is especially useful for efficient CPU usage in RTOS environments. If the HAL supports it (e.g. the included CAN ESP32 HAL), you can choose your own behavior:

- **Busy-wait**: Use `timeout = 0` (or no argument) to return immediately if no data
- **Blocking with timeout**: Use a specific timeout value (e.g., `200`) to wait up to that many milliseconds
- **Blocking indefinitely**: Use `portMAX_DELAY` to block until data arrives (FreeRTOS)

```cpp
link.poll(0);             // Non-blocking: return immediately if no data
link.poll(200);           // Block up to 200ms waiting for messages
link.poll(portMAX_DELAY); // Block indefinitely until message arrives (FreeRTOS)
```

**Implementation note**: The provided **UART Arduino HAL** uses busy-wait polling as it sticks to the basic Arduino Serial API, while the **CAN ESP32 HAL** uses native ESP32 blocking calls through the underlying IDF `TWAI` driver's message queues.

### Empty messages

EZLink supports empty messages (no data fields), which avoids using dummy values when the receiver doesn't expect any data from the sender, such as:
- Command messages
- Simple ACKs

Basic example:

```cpp
struct PingAck {
    static constexpr MsgType type = RESPONSE;
    static constexpr uint8_t id = 5;
    // No data fields!
} __attribute__((packed));

struct PingReq {
    static constexpr MsgType type = REQUEST;
    static constexpr uint8_t id = 5;
    // No data fields!
    using ResponseType = PingAck;
} __attribute__((packed));
```

### Runtime configuration

You can configure timeouts when creating the `Link`:

```cpp
ezlink::Link link(
    hal,   // HAL instance (mandatory)
    2000,  // responseTimeoutMs: wait 2s for responses
    500    // txTimeoutMs: wait 500ms for HAL to send data
);
```

- **`responseTimeoutMs`**: How long `sendRequest()` waits for a response (defaults to 1000 ms)
- **`txTimeoutMs`**: How long to wait for HAL TX operations to complete (defaults to 100 ms)

### Build-time configuration

Set those build flags before including EZLink headers or in your `platformio.ini`, `CMakeLists.txt`, etc.:

```cpp
// EZLink: configure memory usage, debug & threading
#define EZLINK_MAX_PROTOS 32      // Support up to 32 message types (default: 16)
#define EZLINK_DEBUG              // Enable debug logging (default: undefined)
#define EZLINK_RTOS_POLL_TASK     // Enable RTOS dedicated poll task mode (see above)

// UART Arduino HAL: configure message size limits
#define EZLINK_UART_MAX_FRAME_SIZE 128  // Max frame size (default: 32)
#define EZLINK_UART_MAX_CHUNK_SIZE 64   // TX chunk size (default: 64)

#include <EZLink.hpp>
```

### Error recovery

EZLink reports errors but **does not retry automatically**. Error recovery is your responsibility:

```cpp
auto result = link.sendRequest(req, resp);
if (result != ezlink::SUCCESS) {
    if (result == ezlink::ERR_SND_TX_FAILED) {
        // Decide: retry, fail, or fallback behavior
        Serial.println("Timeout - retrying...");
        result = link.sendRequest(req, resp);  // Manual retry
    }
    /* Handle other errors... */
}
```

### Debug logging

EZLink includes optional debug logging that goes through your HAL's `printLog()` method. To enable it, define `EZLINK_DEBUG` before including the library (or as a build flag in `platformio.ini`, `CMakeLists.txt`, etc.).

When enabled, EZLink outputs diagnostic messages prefixed with `[EZLink]`:

```
[EZLink] Registered proto: id=1, type=0, size=3
[EZLink] Received frame: id=1, payloadLen=3
[EZLink] Message processed: id=1
[EZLink] Error: Send TX failed, id=2
```

The HAL is responsible for implementing `printLog()` (usually forwarding to `printf()` or similar).

### Handler context (`void* ctx` parameter in message & request handlers)

EZLink handlers use C-style function pointers for performance and minimal overhead. The `void* ctx` parameter allows you to pass context to handlers, similar to lambda captures. This is essential when your handler needs to access resources that aren't globally available (e.g., class members, local state).

```cpp
// Example: Access class members from a handler
class LedController {
private:
    int ledPin;
    int brightness;

    // Handler must be static (no 'this' pointer)
    static void handleSetLed(const SetLedMsg& msg, void* ctx) {
        LedController* self = static_cast<LedController*>(ctx);
        self->brightness = msg.value;
        analogWrite(self->ledPin, self->brightness);
    }

public:
    LedController(int pin) : ledPin(pin), brightness(0) {}

    void registerHandlers(ezlink::Link& link) {
        // Pass 'this' as context to access class members
        link.onMessage<SetLedMsg>(handleSetLed, this);
    }
};
```

### **⚠️ Critical lifetime requirements**

**Both handler functions & context pointers** must remain valid for the entire `Link` lifetime (a.k.a. forever) once they are registered! Use `static` or global functions if they are registered within their own scope, otherwise you will get a hard fault as soon as they are called.

### Common pitfalls

```cpp
// WRONG: Non-static member function (can't take address)
class Foo {
    void handler(const Msg& msg, void* ctx) { }  // ❌ Won't compile
};

// WRONG: Lambda with capture (can't convert to function pointer)
auto handler = [someValue](const Msg& msg, void* ctx) { /* Do stuff... */ };
link.onMessage<Msg>(handler); // ❌ Won't compile

// WRONG: Context goes out of scope
void setup() {
    LedController ctrl(13);  // ❌ Destroyed when setup() exits
    ctrl.registerHandlers(link);
}

// RIGHT: Static handler + persistent context
class LedController {
    static void handler(const Msg& msg, void* ctx) { }  // ✅ Static function
};

LedController ctrl(13);  // ✅ Global: valid for program lifetime
void main() {
    ctrl.registerHandlers(link);
}
```

## The HAL Architecture

EZLink clearly separates protocol logic from physical transport.

*   **`ezlink::Link` (the Core)**: Manages message registration, handlers, and the state of request/response transactions. It works only with *payloads* (a `message ID` byte plus your `struct` data).
*   **The HAL (the Transport)**: Is responsible for everything else. It takes a payload from the core, wraps it in a physical frame (e.g., adding a Start-of-Frame byte, length, and CRC for UART), and transmits it. On reception, it does the reverse, validating the frame and passing the clean payload up to the core.

This design means you can easily adapt EZLink to any communication bus by creating your own HAL class that implements the required methods. A well-documented HAL template is available at `src/hal/EZLinkHAL-Template.hpp` to guide you through implementing a custom HAL:

```cpp
class MyCustomHAL {
public:
    // Max EZLink message payload size
    static constexpr size_t MAX_PAYLOAD_SIZE = 64;

    // Core transport functions
    int sendFrame(const uint8_t* payload, size_t len, uint32_t timeoutMs);
    int recvFrame(uint8_t* payload, size_t maxLen, uint32_t timeoutMs);

    // System utility functions
    uint32_t getTimestampMs() const;
    void yield();

    // RTOS synchronization (optional - only if supporting EZLINK_RTOS_POLL_TASK)
    bool takeSemaphore(uint32_t timeoutMs);
    void giveSemaphore();

    // Debug logging hook
    void printLog(const char* format, ...);
};
```

## Error Handling

All public methods return an `ezlink::Result` struct, which contains a `Status` code and the relevant message `id` if applicable (`NULL_ID = 0` otherwise). You can check for success easily:

```cpp
MyMsg myMsg{/* Some data... */};
auto result = link.sendMsg(myMsg);
if (result != ezlink::SUCCESS) {
    // Handle error. For debugging, you can print the status in plain text:
    printf("Operation failed on message ID %d: %s\n", result.id, ezlink::toString(result));
}
```

The `poll()` method also returns a `Result` that you can check for your own error handling or logging purposes:

```cpp
auto result = link.poll();
if (result == ezlink::NODATA) {
    // No message to process - this is normal, not an error
} else if (result == ezlink::SUCCESS) {
    // Message was received and processed successfully
    // Access the received message ID with result.id
} else {
    // RX error occurred (invalid ID, CRC error, etc.)
}
```

The `ezlink::Status` enum provides a comprehensive list of potential success and error codes for robust diagnostics.

## Design Rationale for EZLink

This library was built to address a common gap in embedded development. When you need to pass structured data, you're often faced with two choices: manually packing bytes with `Serial.write()`, or pulling in a full serialization framework like Protobuf. EZLink is the alternative.

### 1. Low resource consumption

A primary requirement was to keep the footprint minimal for tiny MCUs such as STM32G0.

*   **Flash (code size):** The core logic is header-only. The main cost comes from the HAL implementation. The provided UART HAL, which handles framing and CRC16, adds about **2KB** to the final binary. This is a fixed, predictable cost for getting a robust transport layer.

*   **RAM (memory usage):** All memory is allocated statically. The total usage is determined by:
    1.  The `MAX_PROTOS` setting (the size of your message registry, defaults to 16 slots).
    2.  The maximum size of a message frame, fixed in HAL: `MAX_PAYLOAD_SIZE` (64B for UART, 8B for CAN by default)
    3.  The HAL's internal buffers (e.g., the UART HAL uses an RX buffer of `MAX_FRAME_SIZE * 2`).

For a typical project, the RAM footprint stays **well under 1KB**.

### 2. Built-in protocol reliability

Manually implementing framing, length checks, and checksums is error-prone. EZLink's HALs are meant to solve the burden of implementing these features in user code.

*   **Integrity is handled:** The default UART HAL enforces a simple `SOF | Length | Payload | CRC16` frame structure. The library's parser is designed to find the next valid frame even if there's noise or corrupted data on the line. It discards garbage for you.

*   **Compile-time type safety:** The API relies on C++ templates. This means you work with actual `struct`s, not `void*` pointers and `memcpy`. If you pass the wrong `struct` to a handler or a `send` function, the compiler will fail the build. This catches a whole class of bugs before the code ever runs.

### 3. KISS workflow

The goal is to define and use a communication protocol without leaving your C++ environment. No external scripts, no code generation steps.

This table summarizes the trade-offs:

| Feature | EZLink | "Raw Serial" (`read`/`write`) | Protobuf / nanopb |
| :--- | :--- | :--- | :--- |
| **Message Definition**| C++ `struct`s | Manual byte packing/unpacking | `.proto` schema files |
| **Tooling** | **None required** | None | Requires code generation toolchain |
| **Framing & Integrity**| **Built-in (HAL)**| Do-It-Yourself | Do-It-Yourself |
| **Safety**| **High** (comptime safety with templates) | Low (manual, runtime errors) | High (generated code) |
| **Code Size** | **Tiny** (<2KB) | Minimal (ad-hoc) | Small (2~4KB) |

In short, EZLink is the choice when you need the guarantees of a real messaging framework (framing, validation, typed messages) but want to implement it with minimal dependencies and without adding a code generation step to your build process. With message prototypes defined in a separate header file, it's pretty straightforward to keep track of changes and manage versioning separately from your application code.

## Examples

Complete working examples based on the Quick Start Guide are available in the `examples/` directory:
- `Prototypes.h`: Shared message definitions for the master/slave examples
- `master_main.cpp`: Master device implementation (sends requests, receives responses)
- `slave_main.cpp`: Slave device implementation (registers handlers, responds to requests)

These examples demonstrate a complete bidirectional communication system and can serve as a starting point for your own projects. Out of simplicity, they are based on the Arduino framework, but you can easily adapt them to other platforms/SDKs since EZLink is platform-agnostic.
