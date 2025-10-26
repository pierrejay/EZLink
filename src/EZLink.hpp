/* @file EZLink.hpp */

#pragma once

#include <cstdio>
#include <type_traits>

#ifndef EZLINK_MAX_PROTOS
    #define EZLINK_MAX_PROTOS 16
#endif

namespace ezlink {

// ===================================================================================
// CONSTANTS
// ===================================================================================

static constexpr uint8_t NULL_ID = 0;
static constexpr uint8_t MAX_ID = 0x7F;
static constexpr uint8_t RESP_BIT = 0x80;
static constexpr uint8_t MAX_PROTOS = (uint8_t)EZLINK_MAX_PROTOS;
static constexpr uint32_t DEFAULT_RESPONSE_TIMEOUT_MS = 1000;
static constexpr uint32_t DEFAULT_TX_TIMEOUT_MS = 100;


// ===================================================================================
// TYPEDEFS
// ===================================================================================

/* @brief Message type for proto declaration
 */
enum class MsgType {
    MESSAGE,    // No response expected
    REQUEST,    // Expect a specific response
    RESPONSE    // Is a response to a request
};
// Aliases for user convenience
constexpr MsgType MESSAGE = MsgType::MESSAGE;
constexpr MsgType REQUEST = MsgType::REQUEST;
constexpr MsgType RESPONSE = MsgType::RESPONSE;


// ===================================================================================
// RESULT TYPE
// ===================================================================================

/* @brief Result type
 */
enum Status {
    SUCCESS = 0,
    NODATA,
    // Registration errors
    ERR_REG_ID_ALREADY_REGISTERED,
    ERR_REG_PROTO_STORE_FULL,
    ERR_REG_INVALID_ID,
    ERR_REG_PROTO_MISMATCH,
    // RX errors
    ERR_RCV_INVALID_ID,
    ERR_RCV_PROTO_MISMATCH,
    ERR_RCV_RESP_MISMATCH,
    ERR_RCV_UNEXPECTED_RESPONSE,
    ERR_RCV_RX_FAILED,
    ERR_RCV_RESP_TX_FAILED,
    // TX errors
    ERR_SND_INVALID_ID,
    ERR_SND_PROTO_MISMATCH,
    ERR_SND_BUSY,
    ERR_SND_TX_FAILED,
    ERR_SND_RESPONSE_TIMEOUT
};

/* @brief Helper to onvert a Result to string
 * @param status The Status enum to convert
 * @return The string representation of the Status enum
 */
constexpr const char* toString(Status status) {
    switch (status) {
        case Status::SUCCESS: return "Success";
        case Status::NODATA: return "No data";
        case Status::ERR_REG_ID_ALREADY_REGISTERED: return "ID already registered";
        case Status::ERR_REG_PROTO_STORE_FULL: return "Proto store full";
        case Status::ERR_REG_INVALID_ID: return "Reg invalid ID";
        case Status::ERR_REG_PROTO_MISMATCH: return "Reg proto mismatch";
        case Status::ERR_RCV_INVALID_ID: return "Recv invalid ID";
        case Status::ERR_RCV_PROTO_MISMATCH: return "Recv proto mismatch";
        case Status::ERR_RCV_RESP_MISMATCH: return "Recv response mismatch";
        case Status::ERR_RCV_UNEXPECTED_RESPONSE: return "Recv unexpected response";
        case Status::ERR_RCV_RX_FAILED: return "Recv RX failed";
        case Status::ERR_RCV_RESP_TX_FAILED: return "Recv response TX failed";
        case Status::ERR_SND_INVALID_ID: return "Send invalid ID";
        case Status::ERR_SND_PROTO_MISMATCH: return "Send proto mismatch";
        case Status::ERR_SND_BUSY: return "Request already in progress";
        case Status::ERR_SND_TX_FAILED: return "Send TX failed";
        case Status::ERR_SND_RESPONSE_TIMEOUT: return "Send response timeout";
        default: return "Unknown";
    }
}

/* @brief Complete result of an operation 
 */
struct Result {
    Status status;
    uint8_t id;  // ID of the processed message (if status == SUCCESS)

    // Overload operators for direct comparison with Status
    bool operator==(Status s) const { return status == s; }
    bool operator!=(Status s) const { return status != s; }
};

/* @brief Helper to convert a Result to string
 * @param result The Result to convert
 * @return The string representation of the Result's status code
 */
constexpr const char* toString(Result result) {
    return toString(result.status);
}


// ===================================================================================
// MAIN `LINK` CLASS
// ===================================================================================

/* @brief Main EZLink class
 * @tparam HAL The HAL implementation
 */
template<typename HAL>
class Link {
private:

    // ===================================================================================
    // COMPILE-TIME VALIDATION
    // ===================================================================================

    #ifdef EZLINK_RTOS_POLL_TASK
        // Detect if HAL has semaphore support
        template<typename H, typename = void>
        struct has_semaphore_support : std::false_type {};

        template<typename H>
        struct has_semaphore_support<H, std::void_t<
            decltype(std::declval<H&>().takeSemaphore(std::declval<uint32_t>())),
            decltype(std::declval<H&>().giveSemaphore())
        >> : std::true_type {};

        // Compile-time check with clear error message
        static_assert(has_semaphore_support<HAL>::value,
            "EZLink configuration error:\n"
            "EZLINK_RTOS_POLL_TASK is defined, but your HAL does not support RTOS multi-threading.\n"
            "- Option 1: Use a single thread for all EZLink operations & compile again without EZLINK_RTOS_POLL_TASK build flag\n"
            "- Option 2: Implement semaphore support in your HAL and make sure HAL::sendFrame() is thread-safe (or protect it)"
        );
    #endif

    // ===================================================================================
    // HELPERS
    // ===================================================================================

    /* @brief Logging helper (needed here for no-op define) */
    #ifdef EZLINK_DEBUG
        #define EZL_LOGF(...) hal.printLog(__VA_ARGS__)
    #else
        #define EZL_LOGF(...) do{}while(0)
    #endif

    /* @brief Helper to compute the REAL data size of a struct (will detect empty structs
    *        unlike sizeof() that would return 1, in order to allow empty messages)
    * @tparam T The struct type to compute the data size of
    * @return The REAL data size of the struct
    */
    template<typename T>
    static constexpr size_t DATA_SIZE() {
        if constexpr (std::is_empty_v<T>) {
            return 0;
        } else {
            return sizeof(T);
        }
    }

    /* @brief Helper to cast an error
     * @param status Status of the operation
     * @param id ID of the corresponding message (if applicable)
     * @return The error Result
     */
    inline Result Error(Status status, uint8_t id = NULL_ID) {
        #ifdef EZLINK_DEBUG
            if (id != NULL_ID) {
                EZL_LOGF("[EZLink] Error: %s, id=%d", toString(status), id);
            } else {
                EZL_LOGF("[EZLink] Error: %s", toString(status));
            }
        #endif
        return Result{status, id};
    }

    /* @brief Helper to cast a success
     * @param id ID of the corresponding message (if applicable)
     * @return The success Result
     */
    inline Result Success(uint8_t id = NULL_ID) {
        return Result{Status::SUCCESS, id};
    }

    /* @brief Helper to cast a no data result
     * @return The no data Result
     */
    inline Result NoData() {
        return Result{Status::NODATA, NULL_ID};
    }


public:

    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    // Max message size is determined by the limit fixed in HAL
    static constexpr size_t MAX_MSG_SIZE = HAL::MAX_PAYLOAD_SIZE - 1;


    // ===================================================================================
    // CONSTRUCTOR/DESTRUCTOR
    // ===================================================================================

    /* @brief Constructor
     * @param hal The HAL implementation
     * @param responseTimeoutMs Timeout to wait for a response to a request in milliseconds
     * @param txTimeoutMs Timeout to wait for a message to be sent by HAL in milliseconds
     * @note EZLink is intented to live during the entire runtime and is not expected to be destroyed.
     * @note If you have a dedicated task running poll() in a loop (separate from sendRequest()),
     *       define -DEZLINK_RTOS_POLL_TASK in your build flags.
     */
    Link(HAL& hal,
        uint32_t responseTimeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS,
        uint32_t txTimeoutMs = DEFAULT_TX_TIMEOUT_MS)
        : hal(hal)
        , responseTimeoutMs(responseTimeoutMs)
        , txTimeoutMs(txTimeoutMs)
    {
        static_assert(MAX_MSG_SIZE > 0, "HAL max payload size must be greater than 1");
        static_assert(EZLINK_MAX_PROTOS > 0, "EZLINK_MAX_PROTOS must be greater than 0");
        static_assert(EZLINK_MAX_PROTOS <= 127, "EZLINK_MAX_PROTOS must be less than or equal to 127");
        responseBuffer.clear();
    }

    // ===================================================================================
    // PUBLIC METHODS - PROTO REGISTRATION
    // ===================================================================================

    /* @brief Register a MESSAGE prototype
     * @tparam T The message type to register
     * @return The result of the registration
     * @note IMPORTANT: This method is not thread-safe! Only call it once at init from a single thread.
     */
    template<typename T>
    Result registerMessage() {
        static_assert(T::type == MsgType::MESSAGE,
            "registerMessage() only works with MESSAGE types");
        static_assert(T::id != NULL_ID,
            "ID=0 is reserved/invalid");
        static_assert(T::id <= MAX_ID,
            "ID exceeds MAX_ID");
        static_assert(std::is_standard_layout<T>::value,
            "Message type must be POD/standard-layout");
        static_assert(DATA_SIZE<T>() <= MAX_MSG_SIZE,
            "Message size exceeds HAL limits");

        return registerProtoInternal(T::id, T::type, DATA_SIZE<T>());
    }

    /* @brief Register a REQUEST prototype
     * @tparam T The request type to register
     * @return The result of the registration
     * @note IMPORTANT: This method is not thread-safe! Only call it once at init from a single thread.
     */
    template<typename T>
    Result registerRequest() {
        static_assert(T::type == MsgType::REQUEST,
            "registerRequest() only works with REQUEST types");
        static_assert(T::id != NULL_ID,
            "ID=0 is reserved/invalid");
        static_assert(T::id <= MAX_ID,
            "ID exceeds MAX_ID");
        static_assert(std::is_standard_layout<T>::value,
            "Message type must be POD/standard-layout");
        static_assert(DATA_SIZE<T>() <= MAX_MSG_SIZE,
            "Message size exceeds HAL limits");

        return registerProtoInternal(T::id, T::type, DATA_SIZE<T>());
    }

    /* @brief Register a RESPONSE prototype
     * @tparam T The response type to register
     * @return The result of the registration
     * @note IMPORTANT: This method is not thread-safe! Only call it once at init from a single thread.
     */
    template<typename T>
    Result registerResponse() {
        static_assert(T::type == MsgType::RESPONSE,
            "registerResponse() only works with RESPONSE types");
        static_assert(T::id != NULL_ID,
            "ID=0 is reserved/invalid");
        static_assert(T::id <= MAX_ID,
            "ID exceeds MAX_ID");
        static_assert(std::is_standard_layout<T>::value,
            "Message type must be POD/standard-layout");
        static_assert(DATA_SIZE<T>() <= MAX_MSG_SIZE,
            "Message size exceeds HAL limits");

        ProtoStore* requestProto = findProto(T::id);
        // Check if corresponding request exists
        if (requestProto == nullptr) return Error(ERR_REG_INVALID_ID, T::id);
        // Verify it's actually a REQUEST (should never happen)
        if (requestProto->type != MsgType::REQUEST) return Error(ERR_REG_PROTO_MISMATCH, T::id);

        return registerProtoInternal(T::id | RESP_BIT, T::type, DATA_SIZE<T>());
    }


    // ===================================================================================
    // PUBLIC METHODS - MESSAGE/REQUEST HANDLERS
    // ===================================================================================

    /* @brief Register MESSAGE handler
     * @tparam T The message type to register
     * @param handler The handler function
     * @param userCtx The user context
     * @return The result of the registration
     * @note IMPORTANT: This method is not thread-safe! Only call it once at init from a single thread.
     */
    template<typename T>
    Result onMessage(void (*handler)(const T&, void*), void* userCtx = nullptr) {
        static_assert(T::type == MsgType::MESSAGE,
            "onMessage() only works with MESSAGE types");

        ProtoStore* proto = findProto(T::id);
        if (proto == nullptr) {
            return Error(ERR_REG_INVALID_ID, T::id);
        }
        if (proto->type != MsgType::MESSAGE) {
            return Error(ERR_REG_PROTO_MISMATCH, T::id);
        }

        using H = MsgHolder<T>;
        static_assert(sizeof(proto->holderStorage) >= sizeof(H), "holderStorage too small");

        // Use placement-new to construct holder in storage
        new (proto->holderStorage) H{handler, userCtx};

        proto->onMsg = &H::call;
        proto->hasHandler = true;

        return Success(T::id);
    }

    /* @brief Register REQUEST handler with automatic response
     * @tparam REQ The request type to register
     * @param handler The handler function
     * @param userCtx The user context
     * @return The result of the registration
     * @note IMPORTANT: This method is not thread-safe! Only call it once at init from a single thread.
     */
    template<typename REQ>
    Result onRequest(void (*handler)(const REQ&, typename REQ::ResponseType&, void*), void* userCtx = nullptr) {
        static_assert(REQ::type == MsgType::REQUEST,
            "onRequest() only works with REQUEST types");
        using RESP = typename REQ::ResponseType;
        static_assert(RESP::type == MsgType::RESPONSE,
            "ResponseType must be a RESPONSE type");

        ProtoStore* proto = findProto(REQ::id);
        if (proto == nullptr) {
            return Error(ERR_REG_INVALID_ID, REQ::id);
        }
        if (proto->type != MsgType::REQUEST) {
            return Error(ERR_REG_PROTO_MISMATCH, REQ::id);
        }

        using H = ReqHolder<REQ>;
        static_assert(sizeof(proto->holderStorage) >= sizeof(H), "holderStorage too small");

        // Use placement-new to construct holder in storage
        new (proto->holderStorage) H{handler, userCtx};

        proto->onReq = &H::call;
        proto->hasHandler = true;

        return Success(REQ::id);
    }


    // ===================================================================================
    // PUBLIC METHODS - MESSAGE/REQUEST SENDING
    // ===================================================================================

    /* @brief Send MESSAGE
     * @tparam T The message type to send
     * @param msg The message to send
     * @return The result of the transmission
     * @note IMPORTANT: This method is not thread-safe! Make sure it cannot be called from multiple
     *       contexts at the same time. Only one pending message at a time is allowed.
     */
    template<typename T>
    Result sendMsg(const T& msg) {
        static_assert(T::type == MsgType::MESSAGE,
            "sendMsg() only works with MESSAGE types");
        static_assert(std::is_standard_layout<T>::value,
            "Message type must be POD/standard-layout");

        ProtoStore* proto = findProto(T::id);
        // Check if proto is registered
        if (proto == nullptr) return Error(ERR_SND_INVALID_ID, T::id);
        // Check if the data size matches the registered proto
        if (proto->size != DATA_SIZE<T>()) return Error(ERR_SND_PROTO_MISMATCH, T::id);
        
        // Build the payload & send it through HAL
        uint8_t payload[HAL::MAX_PAYLOAD_SIZE];
        payload[0] = T::id;
        memcpy(&payload[1], reinterpret_cast<const uint8_t*>(&msg), DATA_SIZE<T>());
        constexpr size_t payloadLen = DATA_SIZE<T>()+1;

        int result = hal.sendFrame(payload, payloadLen, txTimeoutMs);

        if (result != (int)payloadLen) {
            return Error(ERR_SND_TX_FAILED, T::id);  // Transport error
        }

        return Success(T::id);
    }

    /* @brief Send REQUEST and wait for RESPONSE
     * @tparam REQ The request type to send
     * @tparam RESP The response type to wait for
     * @param req The request to send
     * @param resp The response to wait for
     * @return The result of the transmission
     * @note Synchronous call: the function will return when a response is received or a timeout/error
     *       occurs. The behavior during wait depends on your HAL & implementation (busy-wait loop or
     *       RTOS semaphore if implemented & poll() called from its own thread).
     * @note IMPORTANT: This method is not thread-safe! Make sure it cannot be called from multiple
     *       contexts at the same time. Only one pending request at a time is allowed.
     */
    template<typename REQ, typename RESP>
    Result sendRequest(const REQ& req, RESP& resp) {
        static_assert(REQ::type == MsgType::REQUEST,
            "sendRequest() only works with REQUEST types");
        static_assert(RESP::type == MsgType::RESPONSE,
            "Response must be a RESPONSE type");
        static_assert(std::is_same<RESP, typename REQ::ResponseType>::value,
            "Response type doesn't match request's ResponseType");
        static_assert(REQ::id == RESP::id,
            "Response ID must match Request ID");

        if (inRequest) {
            // Another request is already in progress. This is a logic error in the user's code.
            return Error(ERR_SND_BUSY);
        }

        constexpr uint8_t expectedResponseId = RESP::id | RESP_BIT;

        ProtoStore* reqProto = findProto(REQ::id);
        // Check if request proto is registered
        if (reqProto == nullptr) return Error(ERR_SND_INVALID_ID, REQ::id);
        // Check if the data size matches the registered proto
        if (reqProto->size != DATA_SIZE<REQ>()) return Error(ERR_SND_PROTO_MISMATCH, REQ::id);

        ProtoStore* respProto = findProto(expectedResponseId);
        // Check if response proto is registered
        if (respProto == nullptr) return Error(ERR_SND_INVALID_ID, RESP::id);
        // Check if the data size matches the registered proto
        if (respProto->size != DATA_SIZE<RESP>()) return Error(ERR_SND_PROTO_MISMATCH, RESP::id);

        // Setup for response capture
        responseBuffer.clear();  // Clear buffer and flag
        responseBuffer.expectedId = expectedResponseId;
        responseBuffer.expectedSize = respProto->size;  // Store expected size for validation in poll()

        inRequest = true;
        // One-liner RAII guard to be sure we set inRequest to false in all return paths
        struct Defer { volatile bool* p; ~Defer(){ *p = false; } } close{&inRequest};

        // Build the payload & send it through HAL
        uint8_t payload[HAL::MAX_PAYLOAD_SIZE];
        payload[0] = REQ::id;
        memcpy(&payload[1], reinterpret_cast<const uint8_t*>(&req), DATA_SIZE<REQ>());
        constexpr size_t payloadLen = DATA_SIZE<REQ>() + 1;

        int result = hal.sendFrame(payload, payloadLen, txTimeoutMs);

        if (result != (int)payloadLen) {
            return Error(ERR_SND_TX_FAILED, REQ::id);  // Transport error
        }

        // Wait for response based on configuration
        #ifdef EZLINK_RTOS_POLL_TASK
            // Multi-thread mode: wait for semaphore from poll() running in dedicated task
            if (!hal.takeSemaphore(responseTimeoutMs)) {
                return Error(ERR_SND_RESPONSE_TIMEOUT, REQ::id);
            }
        #else
            // Single-thread mode: busy wait with polling
            uint32_t startTime = hal.getTimestampMs();
            while (hal.getTimestampMs() - startTime < responseTimeoutMs) {
                poll(0); // Non-blocking poll - ignore errors if any

                // Check if we got our response
                if (responseBuffer.ready) break;

                hal.yield();  // Give time to scheduler if needed
            }
        #endif

        // Defensive check: should always be true if we reach here
        if (!responseBuffer.ready) {
            return Error(ERR_SND_RESPONSE_TIMEOUT, REQ::id);
        }

        // Defensive: should never happen
        if (responseBuffer.expectedId != expectedResponseId) {
            return Error(ERR_RCV_RESP_MISMATCH, responseBuffer.expectedId);
        }

        if (responseBuffer.len != DATA_SIZE<RESP>()) {
            return Error(ERR_RCV_PROTO_MISMATCH, responseBuffer.expectedId);
        }

        memcpy(&resp, responseBuffer.data, DATA_SIZE<RESP>());
        return Success(RESP::id);
    }


    // ===================================================================================
    // PUBLIC METHODS - INCOMING MESSAGE/REQUEST PROCESSING ROUTINE
    // ===================================================================================

    /* @brief Process incoming messages
     * @param timeoutMs The timeout in milliseconds
     * @return The result of the processing
     * @note This method should be called periodically to handle incoming messages. In an RTOS
     *       environment, it can live in a loop inside a dedicated thread.
     * @note If you are calling this method from a dedicated task, your HAL must implement the
     *       Semaphore methods to allow synchronization with sendRequest() (see example boilerplate)
     */
    Result poll(uint32_t timeoutMs = 0) {
        uint8_t payloadBuf[HAL::MAX_PAYLOAD_SIZE];
        uint8_t* dataBuf = payloadBuf + 1; // Safe because HAL::MAX_PAYLOAD_SIZE > 1 (asserted)

        const int result = hal.recvFrame(payloadBuf, HAL::MAX_PAYLOAD_SIZE, timeoutMs);

        if (result < 0 || result > HAL::MAX_PAYLOAD_SIZE) return Error(ERR_RCV_RX_FAILED);
        if (result == 0) return NoData();

        const uint8_t recvMsgId = payloadBuf[0];
        const size_t dataLen = (size_t)(result - 1);

        const bool isResponse = (recvMsgId & RESP_BIT) != 0;

        EZL_LOGF("[EZLink] Received frame: id=%d, payloadLen=%d", recvMsgId, result);

        // ----- CAPTURE EXPECTED RESPONSE -----

        // If we're waiting for a response AND this is the one we expect
        if (inRequest && isResponse && responseBuffer.expectedId == recvMsgId) {
            // Abort if buffer is already populated (should never happen)
            if (responseBuffer.ready) return NoData();

            // Validate response size matches expected size (set in sendRequest)
            if (dataLen != responseBuffer.expectedSize) {
                EZL_LOGF("[EZLink] Response size mismatch: id=%d, expected=%d, got=%d",
                         recvMsgId, responseBuffer.expectedSize, dataLen);
                return Error(ERR_RCV_PROTO_MISMATCH, recvMsgId);
            }

            // Capture response for sendRequest() to process it
            memcpy(responseBuffer.data, dataBuf, dataLen);
            responseBuffer.len = dataLen;
            responseBuffer.ready = true;  // Written LAST for thread safety
            #ifdef EZLINK_RTOS_POLL_TASK
                hal.giveSemaphore(); // Wake up sendRequest() waiter
            #endif
            EZL_LOGF("[EZLink] Response captured: id=%d, dataLen=%d", recvMsgId, dataLen);
            return Success(recvMsgId);
        }

        // ----- IGNORE UNEXPECTED RESPONSES -----

        // Silently ignore unexpected responses (wrong ID or not waiting for any)
        if (isResponse) return Error(ERR_RCV_UNEXPECTED_RESPONSE, recvMsgId);

        // ----- PROCESS INCOMING MESSAGE/REQUEST -----

        return handleIncomingFrame(recvMsgId, dataBuf, dataLen);
    }


private:

    // ===================================================================================
    // PRIVATE ALIASES
    // ===================================================================================

    // Type definitions for holder-based wrappers
    using WrapperMsg = void (*)(void* holder, const void* data);
    using WrapperReq = void (*)(void* holder, const void* reqData, void* respData);


    // ===================================================================================
    // PRIVATE TYPES
    // ===================================================================================

    // Thread-safe response buffer template
    template<size_t MAX_SIZE>
    struct ResponseBuffer {
        uint8_t expectedId = NULL_ID;
        size_t expectedSize = 0;      // Expected response size (validated in sendRequest)
        uint8_t data[MAX_SIZE] = {0};
        size_t len = 0;
        volatile bool ready = false;  // Written LAST by poll() for thread safety

        void clear() {
            expectedId = NULL_ID;
            expectedSize = 0;
            len = 0;
            ready = false;
        }
    };

    struct ProtoStore {
        MsgType type = MsgType::MESSAGE;
        uint8_t id = NULL_ID;
        size_t size = 0;

        // Storage for holder objects (enough for 2 pointers: fn + ctx)
        alignas(alignof(void*)) unsigned char holderStorage[sizeof(void*) * 2];

        // Wrapper function pointers
        union {
            WrapperMsg onMsg;
            WrapperReq onReq;
        };

        bool hasHandler = false;
    };

    // Holder for MESSAGE handlers (stores typed fn pointer + context)
    template<typename T>
    struct MsgHolder {
        using Fn = void (*)(const T&, void*);
        Fn fn;
        void* userCtx;

        static void call(void* holder, const void* dataBytes) {
            auto* self = static_cast<MsgHolder*>(holder);
            T msg;
            memcpy(&msg, dataBytes, DATA_SIZE<T>());
            self->fn(msg, self->userCtx);
        }
    };

    // Holder for REQUEST handlers (stores typed fn pointer + context)
    template<typename REQ>
    struct ReqHolder {
        using RESP = typename REQ::ResponseType;
        using Fn = void (*)(const REQ&, RESP&, void*);
        Fn fn;
        void* userCtx;

        static void call(void* holder, const void* reqBytes, void* respBytes) {
            auto* self = static_cast<ReqHolder*>(holder);
            REQ req;
            RESP resp;
            memcpy(&req, reqBytes, DATA_SIZE<REQ>());
            self->fn(req, resp, self->userCtx);
            memcpy(respBytes, &resp, DATA_SIZE<RESP>());
        }
    };


    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    HAL& hal;
    uint32_t responseTimeoutMs;  // Response timeout configured at init
    uint32_t txTimeoutMs;  // TX timeout configured at init
    ProtoStore protos[MAX_PROTOS];

    // Request/Response synchronization
    volatile bool inRequest = false;        // Currently waiting for response
    ResponseBuffer<MAX_MSG_SIZE> responseBuffer;  // Thread-safe response buffer


    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    /* @brief Handle incoming MESSAGE or REQUEST (not RESPONSE)
     * @param recvMsgId The received message ID
     * @param dataBuf Pointer to data buffer
     * @param dataLen Length of data
     * @return Result of processing
     */
    Result handleIncomingFrame(uint8_t recvMsgId, const uint8_t* dataBuf, size_t dataLen) {
        EZL_LOGF("[EZLink] Processing message/request: id=%d, dataLen=%d", recvMsgId, dataLen);

        // Check if the proto exists & is valid
        ProtoStore* proto = findProto(recvMsgId);
        if (proto == nullptr) return Error(ERR_RCV_INVALID_ID, recvMsgId);
        if (dataLen != proto->size) return Error(ERR_RCV_PROTO_MISMATCH, recvMsgId);
        if (!proto->hasHandler) return Success(recvMsgId);  // Valid frame received, no handler attached

        // Call user handler with the received frame & for requests, generate and send the response
        switch (proto->type) {
            case MsgType::REQUEST: {
                // Build response buffer
                uint8_t respPayloadBuf[HAL::MAX_PAYLOAD_SIZE];
                uint8_t* respDataBuf = respPayloadBuf + 1; // Safe because HAL::MAX_PAYLOAD_SIZE > 1 (asserted)
                // Call user handler
                proto->onReq(proto->holderStorage, dataBuf, respDataBuf);
                EZL_LOGF("[EZLink] Request processed: id=%d", recvMsgId);
                // Validate response proto
                uint8_t respId = recvMsgId | RESP_BIT;
                ProtoStore* respProto = findProto(respId);
                if (!respProto) return Error(ERR_RCV_INVALID_ID, respId);
                respPayloadBuf[0] = respId;
                const size_t respPayloadLen = 1 + respProto->size;
                // Send response
                EZL_LOGF("[EZLink] Sending response: id=%d, payloadLen=%d", respId, respPayloadLen);
                int sent = hal.sendFrame(respPayloadBuf, respPayloadLen, txTimeoutMs);
                if (sent != respPayloadLen) {
                    return Error(ERR_RCV_RESP_TX_FAILED, respId);
                }
                EZL_LOGF("[EZLink] Response TX success: id=%d", respId);
                return Success(respId);
            }
            case MsgType::MESSAGE: {
                proto->onMsg(proto->holderStorage, dataBuf);
                EZL_LOGF("[EZLink] Message processed: id=%d", recvMsgId);
                break;
            }
            default:
                return NoData();
        }

        return Success(recvMsgId);
    }

    Result registerProtoInternal(uint8_t id, MsgType type, size_t size) {
        // Check if already registered
        for (size_t i = 0; i < MAX_PROTOS; i++) {
            if (protos[i].id != NULL_ID && protos[i].id == id) {
                return Error(ERR_REG_ID_ALREADY_REGISTERED, id);
            }
        }

        // Find free slot
        ProtoStore* slot = nullptr;
        for (size_t i = 0; i < MAX_PROTOS; i++) {
            if (protos[i].id == NULL_ID) {
                slot = &protos[i];
                break;
            }
        }

        if (slot == nullptr) {
            return Error(ERR_REG_PROTO_STORE_FULL, id);
        }

        slot->id = id;
        slot->type = type;
        slot->size = size;
        slot->hasHandler = false;

        EZL_LOGF("[EZLink] Registered proto: id=%d, type=%d, size=%d", id, type, size);
        return Success(id);
    }


    ProtoStore* findProto(uint8_t id) {
        for (size_t i = 0; i < MAX_PROTOS; i++) {
            if (protos[i].id == id) {
                return &protos[i];
            }
        }
        return nullptr;
    }
}; // class Link


// ===================================================================================
// C++17 CTAD guides
// ===================================================================================

// Syntactic sugars to write `ezlink::Link link(hal)` instead of `ezlink::Link<HAL> link(hal)`

template<typename HAL>
Link(HAL&, uint32_t, uint32_t) -> Link<HAL>;
template<typename HAL>
Link(HAL&, uint32_t) -> Link<HAL>;
template<typename HAL>
Link(HAL&) -> Link<HAL>;

} // namespace ezlink