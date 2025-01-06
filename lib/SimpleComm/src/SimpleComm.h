#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#endif

#include <functional>
#include <type_traits>
#include "ScrollBuffer.h"

// Uncomment to enable debug (or define in platformio.ini)
// #define SIMPLECOMM_DEBUG
// #define SIMPLECOMM_DEBUG_TAG "DBG"

/**
 * SimpleComm - Simple communication protocol
 * 
 * Message types:
 * - MESSAGE: Simple message without response (ID 1-127)
 * - MESSAGE_ACK: Message requiring echo confirmation (ID 1-127, response ID = request ID | 0x80)
 * - REQUEST: Message requiring specific response (ID 1-127)
 * - RESPONSE: Response to a REQUEST (ID = request ID | 0x80)
 * 
 * Communication patterns:
 * 1. Synchronous exchanges (MESSAGE_ACK and REQUEST/RESPONSE)
 *    - Designed for immediate response expectation
 *    - RX buffer is automatically cleaned before sending to ensure reliable response capture
 *    - Timeout-based waiting for response
 *    - Best suited for command/control scenarios requiring confirmation
 * 
 * 2. Asynchronous exchanges (MESSAGE)
 *    - No response expected
 *    - No automatic buffer cleaning
 *    - User must implement their own synchronization if needed (e.g. session ID, message ID...)
 *    - Best suited for periodic data transmission or non-critical commands
 * 
 * ID (Function Code) rules:
 * - Request/Command ID must be between 1-127
 * - Response ID are automatically set to request ID | 0x80 (128-255)
 * - ID 0 is reserved (invalid)
 * 
 * Usage:
 * - Register requests with registerRequest<T>()
 * - Register responses with registerResponse<T>()
 * - Responses are automatically rejected by poll() to prevent late response handling
* 
 * Safety & Robustness features:
 * 1. Buffer management
 *    - Buffer-driven approach (no interbyte timeout)
 *    - Automatic buffer cleanup before synchronous exchanges
 *    - Automatic buffer cleanup on invalid frames
 *    - Frame capture state tracking
 * 
 * 2. Protocol safety
 *    - Request/Response distinction using ID MSB
 *    - Automatic rejection of late responses
 *    - Protection against malformed frames
 *    - CRC validation
 * 
 * 3. Bidirectional safety
 *    - Detection of pending frame capture
 *    - Prevention of message collisions
 *    - Protection against response conflicts
 * 
 * 4. Compile-time validations
 *    - Message type checking
 *    - ID range validation (1-127 requests, 128-255 responses)
 *    - Request/Response type matching
 * 
 * Error handling ensures proper cleanup and state reset in all cases,
 * making the protocol suitable for both master/slave and bidirectional
 * communication patterns.
 * 
 * Thread safety:
 * The library is not thread-safe by design. In a multithreaded environment:
 * 1. Synchronous operations (MESSAGE_ACK and REQUEST/RESPONSE)
 *    - Must be called from a single thread
 *    - Will automatically clean RX buffer before sending
 *    - Will block until response/timeout
 *    - Consider using a dedicated communication thread
 * 
 * 2. Asynchronous operations (MESSAGE)
 *    - Can be safely called from multiple threads
 *    - No automatic buffer cleaning
 *    - No blocking
 *    - User must implement synchronization if needed
 * 
 * Note: The frameCapturePending flag protects against concurrent frame captures,
 * but does not guarantee thread-safety for message sending operations.
 */

// Constants that must stay out of the class as they cannot
// be inlined in C++11 (use SimpleCommDfs:: to access them)
namespace SimpleCommDfs {

// Frame format
static constexpr size_t MAX_FRAME_SIZE = 32;      // Defines max RX buffer size
static constexpr uint8_t START_OF_FRAME = 0xAA;   // Start Of Frame marker
static constexpr size_t FRAME_OVERHEAD = 5;       // SOF + LEN + ID + CRC*2
// ID constants
static constexpr uint8_t NULL_ID = 0;             // ID=0 reserved/invalid
static constexpr uint8_t MAX_PROTOS = 10;         // Maximum number of protos registered overall
static constexpr uint8_t ID_RESPONSE_BIT = 0x80;  // Bit 7 set for responses
static constexpr uint8_t ID_MAX_USER = 0x7F;      // Max 127 user-defined ID (1-127)
// Timeouts & protections
static constexpr uint32_t DEFAULT_RESPONSE_TIMEOUT_MS = 500;   // Wait max 500ms for a response
static constexpr size_t RX_CLEANUP_BYTES_LIMIT = 256;          // Limite pour détecter un flood au cleanup

} // namespace SimpleCommDfs

class SimpleComm {
    
public:

    using TxCallback = std::function<size_t(const uint8_t*, size_t)>; // Write to external interface
    using RxCallback = std::function<size_t(uint8_t*, size_t)>;        // Read from external interface

    /* @brief Proto types */
    enum class ProtoType {
        MESSAGE,  // No response expected
        MESSAGE_ACK,     // Echo expected
        REQUEST,          // Expect a specific response
        RESPONSE          // Is a response to a request
    };

    /* @brief Result of an operation */
    enum Status {
        SUCCESS = 0,
        // Poll result
        NOTHING_TO_DO = 1,
        // RegisterProto errors
        ERR_ID_ALREADY_REGISTERED = 10,     // ID already registered
        ERR_NAME_ALREADY_REGISTERED = 11,   // Name already registered
        ERR_TOO_MANY_PROTOS = 12,           // Too many protos registered
        ERR_INVALID_NAME = 13,              // Name is null or empty
        ERR_REG_INVALID_ID = 14,            // ID not registered
        ERR_REG_PROTO_MISMATCH = 15,        // Inconsistency between expected and registered proto
        // Poll & sendMsg errors
        ERR_BUSY_RECEIVING = 20,            // Frame capture pending, we must wait for the end of the frame to send a new message
        // RX errors
        ERR_RCV_MIN = 30,                   // Dummy error code
        ERR_RCV_INVALID_ID = 31,            // ID not registered
        ERR_RCV_INVALID_SOF = 32,           // Invalid SOF
        ERR_RCV_INVALID_LEN = 33,           // Invalid length
        ERR_RCV_PROTO_MISMATCH = 34,        // Message received but inconsistent with expected proto
        ERR_RCV_ACK_MISMATCH = 35,          // ACK received but inconsistent with sent message
        ERR_RCV_RESP_MISMATCH = 36,         // Response received but inconsistent with expected response
        ERR_RCV_CRC = 37,                   // Invalid CRC
        ERR_RCV_UNEXPECTED_RESPONSE = 38,   // Received a response but not expected
        ERR_RCV_TIMEOUT = 39,               // No response
        ERR_RCV_MAX = 40,                   // Dummy error code
        // TX errors
        ERR_SND_MIN = 40,                   // Dummy error code
        ERR_SND_INVALID_ID = 41,            // ID not registered
        ERR_SND_EMPTY_DATA = 42,            // Empty data to send
        ERR_SND_PROTO_MISMATCH = 43,        // Inconsistency between expected and sent proto
        ERR_SND_MAX = 44,                   // Dummy error code
        // Hardware interface errors
        ERR_HW_FLOOD = 50,                   // Hardware interface probably flooded
        ERR_HW_TX_FAILED = 51               // Hardware TX failure
    };

    /* @brief Complete result of a reception operation */
    struct Result {
        Status status;
        uint8_t id;  // ID of the processed message (if status == SUCCESS)

        // Overload operators for direct comparison with Status
        bool operator==(Status s) const { return status == s; }
        bool operator!=(Status s) const { return status != s; }
    };
    // Helpers to create results
    static constexpr Result Error(Status status, uint8_t id = SimpleCommDfs::NULL_ID) { return Result{status, id}; }
    static constexpr Result Success(uint8_t id = SimpleCommDfs::NULL_ID) { return Result{SUCCESS, id}; }
    static constexpr Result NoData() { return Result{NOTHING_TO_DO, SimpleCommDfs::NULL_ID}; }

    /* @brief Constructor with callbacks (always available) */
    explicit SimpleComm(TxCallback tx, 
                       RxCallback rx,
                       uint32_t responseTimeoutMs = SimpleCommDfs::DEFAULT_RESPONSE_TIMEOUT_MS
                       #ifdef SIMPLECOMM_DEBUG
                       , Stream* debugStream = nullptr
                       , const char* instanceName = ""
                       #endif
                       ): 
        txCallback(tx),
        rxCallback(rx),
        responseTimeoutMs(responseTimeoutMs)
        #ifdef SIMPLECOMM_DEBUG
        , debugStream(debugStream)
        , instanceName(instanceName)
        #endif
        {
        }

    #if defined(ARDUINO)
    /* @brief Arduino constructor (wrapper) */
    explicit SimpleComm(HardwareSerial* serial,
                       uint32_t responseTimeoutMs = SimpleCommDfs::DEFAULT_RESPONSE_TIMEOUT_MS
                       #ifdef SIMPLECOMM_DEBUG
                       , Stream* debugStream = nullptr
                       , const char* instanceName = ""
                       #endif
                       ) :
        responseTimeoutMs(responseTimeoutMs)
        #ifdef SIMPLECOMM_DEBUG
        , debugStream(debugStream)
        , instanceName(instanceName)
        #endif
        {
            txCallback = [serial](const uint8_t* data, size_t len) {
                // Check available space
                if(len > serial->availableForWrite()) {
                    return (size_t)0;  // Failure -> ERR_HW_TX_FAILED
                }
                size_t written = serial->write(data, len);
                serial->flush();
                return written;
            };
            rxCallback = [serial](uint8_t* data, size_t maxLen) {
                size_t n = 0;
                while (n < maxLen && serial->available()) {
                    data[n++] = serial->read();
                }
                return n;
            };
        }
    #endif

    /* @brief Cleanup the RX buffer
     * @return Result of the operation */
    Result cleanupRxBuffer() {
        uint8_t byte;
        size_t totalRead = 0;
        bool remainingData = true;

        // Clear the hardware buffer first
        while(totalRead < SimpleCommDfs::RX_CLEANUP_BYTES_LIMIT && remainingData) {
            if(rxCallback(&byte, 1) == 0) {
                remainingData = false;  // No more data to read
            } else {
                totalRead++;
            }
        }
        // Clear our internal RX buffer
        rxBuffer.clear();
        frameCapturePending = false;

        #ifdef SIMPLECOMM_DEBUG
        if(remainingData) {  // If we exited because of the limit
            debugPrint("Warning: RX cleanup stopped at limit - probable flood");
        } else {
            debugPrint("RX buffer cleaned");
        }
        #endif
        
        return remainingData ? Error(ERR_HW_FLOOD) : Success();
    }

    /* @brief Initialize the communication */
    void begin() {
        // Clean any residual data that might be in the hardware buffer
        auto result = cleanupRxBuffer();
        
        #ifdef SIMPLECOMM_DEBUG
        if (result == SUCCESS) {
            debugPrint("Instance initialized");
        }
        #endif
    }

    /* @brief Extract data from a message
     * @param msg: The message to extract data from
     * @param outData: The buffer to store the extracted data
     * @param outDataLen: The length of the extracted data */
    template<typename T>
    void extractData(const T& msg, uint8_t* outData, size_t& outDataLen) {
        if (outData) {
            outDataLen = sizeof(T);
            memcpy(outData, &msg, outDataLen);
        }
    }

    /* @brief Register a REQUEST prototype
     * @return Result of the operation */
    template<typename T>
    Result registerRequest() {
        static_assert(T::type == ProtoType::REQUEST || 
                     T::type == ProtoType::MESSAGE || 
                     T::type == ProtoType::MESSAGE_ACK,
                     "You tried to register a RESPONSE with registerRequest() - use registerResponse() for RESPONSE types");
        static_assert(T::id != SimpleCommDfs::NULL_ID, "ID=0 is reserved/invalid");
        static_assert((T::id & SimpleCommDfs::ID_RESPONSE_BIT) == 0, "Request ID must be <= 127");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(sizeof(T) + SimpleCommDfs::FRAME_OVERHEAD <= SimpleCommDfs::MAX_FRAME_SIZE, "Message too large");
        return registerProtoInternal<T>(T::id);
    }

    /* @brief Register a RESPONSE prototype
     * @return Result of the operation */
    template<typename T>
    Result registerResponse() {
        static_assert(T::type == ProtoType::RESPONSE, 
                     "Wrong message type - registerResponse() only works with RESPONSE types");
        static_assert(T::id != SimpleCommDfs::NULL_ID, "ID=0 is reserved/invalid");
        static_assert((T::id & SimpleCommDfs::ID_RESPONSE_BIT) == 0, "Response ID must be <= 127");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(sizeof(T) + SimpleCommDfs::FRAME_OVERHEAD <= SimpleCommDfs::MAX_FRAME_SIZE, "Message too large");

        // Check if a request with this ID is registered
        ProtoStore* requestProto = findProto(T::id);  // Search with original ID
        if (requestProto == nullptr) {
            return Error(ERR_REG_INVALID_ID, T::id);  // No corresponding request
        }

        // Replace the ID by its complement (ID | 0x80)
        return registerProtoInternal<T>(T::id | SimpleCommDfs::ID_RESPONSE_BIT);
    }

    /* @brief Handler for MESSAGE and MESSAGE_ACK messages
     * @param handler: The handler to call when a message is received
     * @return Result of the operation */
    template<typename T>
    Result onReceive(std::function<void(const T&)> handler) {
        static_assert(T::type == ProtoType::MESSAGE || 
                     T::type == ProtoType::MESSAGE_ACK,
                     "onReceive only works with MESSAGE or MESSAGE_ACK messages");
        return onMessage<T>(handler);
    }

    /* @brief Handler for REQUEST messages with automatic response
     * @param handler: The handler to call when a message is received
     * @return Result of the operation */
    template<typename REQ>
    Result onRequest(std::function<void(const REQ&, typename REQ::ResponseType&)> handler) {
        static_assert(REQ::type == ProtoType::REQUEST, 
                     "onRequest only works with REQUEST messages");
        static_assert(REQ::ResponseType::type == ProtoType::RESPONSE, 
                     "Response type must be RESPONSE");
        
        return onMessage<REQ>([this, handler](const REQ& req) {
            typename REQ::ResponseType resp;
            // Call the handler
            handler(req, resp);

            // Send the response automatically
            Result result = sendMsgInternal(resp, true);
            if (result != SUCCESS) {
                #ifdef SIMPLECOMM_DEBUG
                debugPrintf("Erreur envoi response ID=0x%02X, code=%d", REQ::id, result.status);
                #endif
            } else {
                #ifdef SIMPLECOMM_DEBUG
                debugPrintf("Response ID=0x%02X envoyee avec succes", REQ::id);
                #endif
            }
        });
    }

    /* @brief Type-safe send for MESSAGE messages
     * @param msg: The message to send
     * @return Result of the operation */
    template<typename T>
    Result sendMsg(const T& msg) {
        static_assert(T::type == ProtoType::MESSAGE, "Wrong message type, sendMsg() only works with MESSAGE messages");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert((T::id & SimpleCommDfs::ID_RESPONSE_BIT) == 0, "ID must be <= 127");
        static_assert(T::id != SimpleCommDfs::NULL_ID, "ID must not be NULL (0)");  // Check at compilation
        return sendMsgInternal(msg, false);
    }

    /* @brief Send message and wait for acknowledgement (same message echoed back)
     * @param msg: The message to send
     * @return Result of the operation */
    template<typename T>
    Result sendMsgAck(const T& msg) {
        static_assert(T::type == ProtoType::MESSAGE_ACK, "Wrong message type, sendMsgAck() only works with MESSAGE_ACK messages");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert((T::id & SimpleCommDfs::ID_RESPONSE_BIT) == 0, "ID must be <= 127");
        static_assert(T::id != SimpleCommDfs::NULL_ID, "ID must not be NULL (0)");  // Check at compilation
        
        // Clean the RX buffer before sending
        auto result = cleanupRxBuffer();
        if (result != SUCCESS) {
            return result;
        }
        
        // Send message first
        result = sendMsgInternal(msg, false);
        if (result != SUCCESS) {
            return Error(result.status, T::id);
        }
        
        // Wait for echo with timeout
        uint8_t frame[SimpleCommDfs::MAX_FRAME_SIZE];
        size_t frameLen;
        
        unsigned long startTime = millis();
        while(millis() - startTime < responseTimeoutMs) {
            // We expect a response with a complement of the message ID (ID | 0x80)
            result = captureFrame(T::id | SimpleCommDfs::ID_RESPONSE_BIT, frame, &frameLen);
            if(result == NOTHING_TO_DO) {
                continue;
            }
            if(result != SUCCESS) {
                return result;
            }
            
            // Check that the response is the same size as the request
            if(frameLen - SimpleCommDfs::FRAME_OVERHEAD != sizeof(T)) {
                return Error(ERR_RCV_ACK_MISMATCH);
            }
            
            // Check that the content is identical
            if(memcmp(&frame[3], &msg, sizeof(T)) != 0) {
                return Error(ERR_RCV_ACK_MISMATCH);
            }
            
            return Success(T::id);
        }
        
        frameCapturePending = false; // Reset frame capture - protects against late responses
        return Error(ERR_RCV_TIMEOUT);
    }

    /* @brief Send request and wait for specific response type
     * @param req: The request to send
     * @param resp: The response to receive
     * @return Result of the operation */
    template<typename REQ, typename RESP>
    Result sendRequest(const REQ& req, RESP& resp) {
        static_assert(REQ::type == ProtoType::REQUEST, "Wrong message type, sendRequest() only works with REQUEST messages");
        static_assert(RESP::type == ProtoType::RESPONSE, "Response must be a RESPONSE type");
        static_assert(std::is_same<RESP, typename REQ::ResponseType>::value, "Response type doesn't match request's ResponseType");
        static_assert(std::is_standard_layout<REQ>::value, "Request type must be POD/standard-layout");
        static_assert(std::is_standard_layout<RESP>::value, "Response type must be POD/standard-layout");
        static_assert(REQ::id != SimpleCommDfs::NULL_ID, "ID must not be NULL (0)");
        static_assert((REQ::id & SimpleCommDfs::ID_RESPONSE_BIT) == 0, "Request ID must be <= 127");
        static_assert(REQ::id == RESP::id, "Response ID must match Request ID"); 
        
        // Clean the RX buffer before sending
        auto result = cleanupRxBuffer();
        if (result != SUCCESS) {
            return result;
        }
        
        // Send request
        result = sendMsgInternal(req);
        if (result != SUCCESS) {
            return Error(result.status, REQ::id);
        }
        
        // Wait for response with timeout
        uint8_t frame[SimpleCommDfs::MAX_FRAME_SIZE];
        size_t frameLen;
        
        unsigned long startTime = millis();
        while(millis() - startTime < responseTimeoutMs) {
            // We expect a response with a complement of the request ID (ID | 0x80)
            result = captureFrame(RESP::id | SimpleCommDfs::ID_RESPONSE_BIT, frame, &frameLen);  // Wait for the ID with response bit
            if(result == NOTHING_TO_DO) {
                continue;
            }
            if(result != SUCCESS) {
                return result;
            }
            
            // Check that the response is the right size
            if(frameLen - SimpleCommDfs::FRAME_OVERHEAD != sizeof(RESP)) {
                return Error(ERR_RCV_RESP_MISMATCH);
            }
            
            // Captured frame, copy it into the response
            memcpy(&resp, &frame[3], sizeof(RESP));
            return Success(RESP::id);
        }
        
        frameCapturePending = false; // Reset frame capture - protects against late responses
        return Error(ERR_RCV_TIMEOUT);
    }

    /* @brief Process received messages with optional frame capture
     * @return Result of the operation */
    Result poll() {
        uint8_t rxBuffer[SimpleCommDfs::MAX_FRAME_SIZE];  // Local buffer
        size_t frameLen;

        // Try to capture a frame in the local buffer
        Result result = captureFrame(0, rxBuffer, &frameLen);
        
        // If capture failed, return the error
        if(result != SUCCESS && result != NOTHING_TO_DO) {
            return result;
        }
        
        // If capture succeeded, check that it's not a response
        if(result == SUCCESS) {
            
            // Reject responses (bit 7 set)
            if((result.id & SimpleCommDfs::ID_RESPONSE_BIT) != 0) {
                return Error(ERR_RCV_UNEXPECTED_RESPONSE, result.id);
            }

            // Find the proto and check the size
            ProtoStore* proto = findProto(result.id);
            if(proto) {
                // Check that the size matches the proto
                if(frameLen - SimpleCommDfs::FRAME_OVERHEAD != proto->size) {
                    return Error(ERR_RCV_PROTO_MISMATCH, result.id);
                }
                
                // Call the handler if registered
                if(proto->callback) {
                    proto->callback(&rxBuffer[3]);
                }

                // If it's a MESSAGE_ACK message, send the ACK automatically,
                // flipping the ID
                uint8_t ackId = result.id | SimpleCommDfs::ID_RESPONSE_BIT;
                if(proto->type == ProtoType::MESSAGE_ACK) {
                    sendFrame(ackId, &rxBuffer[3], proto->size);
                }
            }
            return result;
        }
        
        // If nothing captured, return NOTHING_TO_DO
        return NoData();
    }

    // Setter for timeouts
    bool setResponseTimeout(uint32_t ms) { 
        if(ms == 0) return false;
        responseTimeoutMs = ms; 
        return true;
    }
    uint32_t getResponseTimeout() { 
        return responseTimeoutMs;
    }

    /* @brief Utility function to calculate CRC8
     * @param data: The data to calculate the CRC8 from
     * @param size: The size of the data
     * @return The CRC8 value */
    static uint8_t calculateCRC8(const uint8_t* data, size_t size) {
        uint8_t crc = 0;
        for (size_t i = 0; i < size; i++) {
            uint8_t inbyte = data[i];
            for (uint8_t j = 0; j < 8; j++) {
                uint8_t mix = (crc ^ inbyte) & 0x01;
                crc >>= 1;
                if (mix) {
                    crc ^= 0x8C;
                }
                inbyte >>= 1;
            }
        }
        return crc;
    }

    /* @brief Utility function to calculate CRC16
     * @param data: The data to calculate the CRC16 from
     * @param size: The size of the data
     * @return The CRC16 value */
    static uint16_t calculateCRC16(const uint8_t* data, size_t size) {
        uint16_t crc = 0xFFFF;  // Initial value
        for (size_t i = 0; i < size; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }

private:
    /* @brief Structure to store message prototypes */
    struct ProtoStore {
        ProtoType type = ProtoType::MESSAGE;
        const char* name = nullptr;
        uint8_t id = SimpleCommDfs::NULL_ID;
        size_t size = 0;
        std::function<void(const void*)> callback = nullptr;
    };

    // Attributes
    TxCallback txCallback;                          // Callback for sending frames
    RxCallback rxCallback;                          // Callback for receiving frames
    uint32_t responseTimeoutMs;                     // Timeout for responses
    ProtoStore protos[SimpleCommDfs::MAX_PROTOS];   // Array of prototypes
    bool frameCapturePending = false;               // Flag to indicate if a frame is being captured

    #ifdef SIMPLECOMM_DEBUG
    Stream* debugStream;            // Debug port (optional)
    const char* instanceName;       // Instance name for logs

    /* @brief Helper for debug logs
     * @param msg: The message to log */
    void debugPrint(const char* msg) {
        if (debugStream) {
            debugStream->print("[");
            debugStream->print(SIMPLECOMM_DEBUG_TAG);
            debugStream->print("_");
            debugStream->print(instanceName);
            debugStream->print("]: ");
            debugStream->print(msg);
            debugStream->print("\n");  // Single \n explicit
        }
    }
    
    /* @brief Helper for debug logs
     * @param format: The format string
     * @param ...: The arguments to format */
    void debugPrintf(const char* format, ...) {
        if (debugStream) {
            char buffer[128];
            va_list args;
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            debugStream->print("[");
            debugStream->print(SIMPLECOMM_DEBUG_TAG);
            debugStream->print("_");
            debugStream->print(instanceName);
            debugStream->print("]: ");
            debugStream->println(buffer);
        }
    }

    /* @brief Helper for debug logs
     * @param prefix: The prefix to log
     * @param data: The data to log
     * @param len: The length of the data */
    void debugHexDump(const char* prefix, const uint8_t* data, size_t len) {
        if (!debugStream) return;
        debugStream->print("[");
        debugStream->print(SIMPLECOMM_DEBUG_TAG);
        debugStream->print("_");
        debugStream->print(instanceName);
        debugStream->print("]: ");
        debugStream->print(prefix);
        debugStream->print(" [");
        debugStream->print(len);
        debugStream->print(" bytes]");
        
        // Display quick analysis if it's a frame
        if (len >= SimpleCommDfs::FRAME_OVERHEAD) {
            debugStream->print(" SOF=0x");
            if (data[0] < 0x10) debugStream->print("0");
            debugStream->print(data[0], HEX);
            debugStream->print(" LEN=");
            debugStream->print(data[1]);
            debugStream->print(" ID=0x");
            if (data[2] < 0x10) debugStream->print("0");
            debugStream->print(data[2], HEX);
        }
        
        debugStream->print(" => ");
        
        // Display hexdump
        for(size_t i = 0; i < len; i++) {
            if(data[i] < 0x10) debugStream->print("0");
            debugStream->print(data[i], HEX);
            debugStream->print(" ");
        }
        debugStream->println();
    }
    #endif

    /* @brief Register message prototype
     * @param id: The ID of the prototype
     * @return Result of the operation */
    template<typename T>
    Result registerProtoInternal(uint8_t id) {
        // Proto validation is already done in registerRequest() and registerResponse()
        
        // Check if already registered (by ID or by name)
        for(size_t i = 0; i < SimpleCommDfs::MAX_PROTOS; i++) {
            if(protos[i].id != SimpleCommDfs::NULL_ID) {
                if(protos[i].id == id) {  // On compare avec le ID modifié !
                    return Error(ERR_ID_ALREADY_REGISTERED, T::id);
                }
            }
        }
        
        // Find a free slot
        ProtoStore* slot = findFreeSlot();
        if(slot == nullptr) {
            return Error(ERR_TOO_MANY_PROTOS, T::id);
        }
        
        // Register the proto
        slot->id = id;  // Use the modified ID passed as parameter
        slot->type = T::type;
        slot->size = sizeof(T);
        
        return Success(T::id);
    }

    /* @brief Check if proto is registered and matches
     * @param id: The ID of the prototype
     * @param outProto: The pointer to the proto found (optional)
     * @return Result of the operation */
    template<typename T>
    Result checkProto(uint8_t id, ProtoStore** outProto = nullptr) {
        ProtoStore* proto = findProto(id);
        if (proto == nullptr) {
            return Error(ERR_SND_INVALID_ID, id);
        }
        if(outProto) {
            *outProto = proto;
        }
        return Success(id);
    }

    /* @brief Send message
     * @param msg: The message to send
     * @param flipId: Whether to flip the ID (for responses)
     * @return Result of the operation */
    template<typename T>
    Result sendMsgInternal(const T& msg, bool flipId = false) {

        // Define ID, flip if needed (for responses)
        uint8_t id = msg.id;
        if (flipId) id |= SimpleCommDfs::ID_RESPONSE_BIT;

        // Check if proto is registered and matches
        Result protoCheck = checkProto<T>(id);
        if(protoCheck != SUCCESS) {
            #ifdef SIMPLECOMM_DEBUG
            debugPrintf("Erreur validation proto response ID=0x%02X, code=%d", id, protoCheck.status);
            #endif
            return protoCheck;
        }

        // Send frame
        return sendFrame(id, (uint8_t*)&msg, sizeof(T));
    }

    /* @brief Send message
     * @param id: The ID of the message
     * @param data: The data to send
     * @param dataLen: The length of the data
     * @return Result of the operation */
    Result sendFrame(uint8_t id, uint8_t* data, size_t dataLen) {
        if (!data || dataLen == 0) {
            return Error(ERR_SND_EMPTY_DATA);
        }

        // Check if we are already capturing a frame
        if(frameCapturePending) {
            return Error(ERR_BUSY_RECEIVING);
        }
        
        // Calculate frame size
        size_t frameSize = dataLen + SimpleCommDfs::FRAME_OVERHEAD; // Only includes non-static fields (data) + overhead

        // Prepare frame
        uint8_t frame[SimpleCommDfs::MAX_FRAME_SIZE];
        frame[0] = SimpleCommDfs::START_OF_FRAME;
        frame[1] = frameSize;
        frame[2] = id;
        memcpy(&frame[3], data, dataLen);
        uint16_t crc = calculateCRC16(frame, frameSize-2);
        frame[frameSize-2] = (uint8_t)(crc >> 8);    // MSB
        frame[frameSize-1] = (uint8_t)(crc & 0xFF);  // LSB

        
        #ifdef SIMPLECOMM_DEBUG
        // Log the sent frame
            debugHexDump("TX frame", frame, frameSize);
        #endif

        // Send frame
        if(txCallback(frame, frameSize) != frameSize) {
            return Error(ERR_HW_TX_FAILED);
        }

        return Success(id);
    }

    /* @brief "Scrolling" ring buffer for reception */
    ScrollBuffer<uint8_t, SimpleCommDfs::MAX_FRAME_SIZE> rxBuffer;

    /* @brief Capture a frame
     * @param expectedId: The expected ID of the frame (0 for any)
     * @param outFrame: The pointer to the frame found (optional)
     * @param outLen: The pointer to the length of the frame found (optional)
     * @return Result of the operation */
    Result captureFrame(uint8_t expectedId = 0, uint8_t* outFrame = nullptr, size_t* outLen = nullptr) {

        const uint8_t sof = SimpleCommDfs::START_OF_FRAME;

        // First read all available on the serial port
        processRx();

        // Not enough data
        if (rxBuffer.size() == 0) {
            return NoData();
        }

        // If no capture in progress, search for a valid SOF
        if (!frameCapturePending) {
            // No data
            if (rxBuffer.size() == 0) {
                return NoData();
            }
            
            // Current byte must be a SOF
            if (rxBuffer.peek(0) != sof) {
                // Log invalid SOF
                uint8_t invalidSof = rxBuffer.peek(0);
                #ifdef SIMPLECOMM_DEBUG
                debugHexDump("RX invalid SOF", &invalidSof, 1);
                #endif
                
                // Otherwise slide to the next SOF
                rxBuffer.scrollTo(sof); 
                return Error(ERR_RCV_INVALID_SOF);
            }
            
            // Now we have a valid SOF, start capture
            frameCapturePending = true;
        }

        // Capture in progress, we must have at least 2 bytes to read the LEN
        if (rxBuffer.size() < 2) {
            return NoData();
        }

        // Check LEN
        uint8_t frameSize = rxBuffer.peek(1);
        if (frameSize < SimpleCommDfs::FRAME_OVERHEAD || frameSize > SimpleCommDfs::MAX_FRAME_SIZE) {
            // Log invalid SOF + LEN
            #ifdef SIMPLECOMM_DEBUG
            uint8_t header[2];
            header[0] = rxBuffer.peek(0);
            header[1] = rxBuffer.peek(1);
            debugHexDump("RX invalid LEN", header, 2);
            #endif
            
            // If invalid LEN, discard SOF
            rxBuffer.dump(1);
            // Try to slide to the next SOF
            rxBuffer.scrollTo(sof);
            return Error(ERR_RCV_INVALID_LEN);
        }

        // Wait for the full frame
        if (rxBuffer.size() < frameSize) {
            return NoData();
        }

        // We have a full frame, copy it for CRC
        uint8_t tempFrame[SimpleCommDfs::MAX_FRAME_SIZE];
        for (uint8_t i = 0; i < frameSize; i++) {
            tempFrame[i] = rxBuffer.peek(i);
        }

        // Validate CRC first before checking the content
        uint16_t receivedCRC = ((uint16_t)tempFrame[frameSize-2] << 8) | tempFrame[frameSize-1];
        uint16_t calculatedCRC = calculateCRC16(tempFrame, frameSize-2);
        if (receivedCRC != calculatedCRC) {
            frameCapturePending = false;
            // Invalid CRC, may be a truncated frame followed by a valid frame,
            // discard SOF and slide to the next.
            // - If it was a truncated frame, the next one will be processed at the next poll().
            // - If it was not a truncated frame, we may find a SOF present in the invalid frame data, it will be rejected.
            // In all cases, no valid frame is discarded, as long as we regularly call
            // poll() we will eventually find the SOF of a valid frame.
            #ifdef SIMPLECOMM_DEBUG
            debugHexDump("RX invalid CRC", tempFrame, frameSize);
            #endif
            rxBuffer.dump(1);
            rxBuffer.scrollTo(sof);
            return Error(ERR_RCV_CRC);
        }

        // CRC is valid, we can now check the ID if needed
        uint8_t id = rxBuffer.peek(2);
        if (expectedId > 0 && id != expectedId) {
            frameCapturePending = false;
            // Invalid ID, discard the entire frame and slide to the next SOF
            #ifdef SIMPLECOMM_DEBUG
            debugHexDump("RX invalid ID", tempFrame, frameSize);
            #endif
            rxBuffer.dump(frameSize);
            rxBuffer.scrollTo(sof);
            return Error(ERR_RCV_INVALID_ID, id);
        }

        #ifdef SIMPLECOMM_DEBUG
        debugHexDump("RX valid frame", tempFrame, frameSize);
        #endif

        // Valid frame !
        if (outFrame && outLen) {
            memcpy(outFrame, tempFrame, frameSize);
            *outLen = frameSize;
        }

        // Consume the frame
        rxBuffer.dump(frameSize);
        frameCapturePending = false;

        return Success(id);
    }

    /* @brief Generic handler (internal usage only)
     * @param handler: The handler to call when a message is received
     * @return Result of the operation */
    template<typename T>
    Result onMessage(std::function<void(const T&)> handler) {
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(T::id < 256, "ID must be < 256");
        
        ProtoStore* proto = findProto(T::id);
        if(proto == nullptr) {
            return Error(ERR_REG_INVALID_ID, T::id);
        }
        
        proto->callback = [handler](const void* data) {
            handler(*static_cast<const T*>(data));
        };
        
        return Success(T::id);
    }

    /* @brief Process incoming data */
    void processRx() {
        // Read only if there is space in the buffer
        size_t available = SimpleCommDfs::MAX_FRAME_SIZE - rxBuffer.size();
        if(available == 0) return;

        // Direct read into the buffer
        uint8_t byte;
        while(available > 0 && rxCallback(&byte, 1) > 0) {
            rxBuffer.push(byte);
            available--;
        }
    }

    /* @brief Find a proto by its ID
     * @param id: The ID of the proto to find
     * @return The pointer to the proto found or nullptr if not found */
    ProtoStore* findProto(uint8_t id) {
        for(size_t i = 0; i < SimpleCommDfs::MAX_PROTOS; i++) {
            if(protos[i].id == id) {
                return &protos[i];
            }
        }
        return nullptr;
    }

    /* @brief Find a free slot
     * @return The pointer to the free slot found or nullptr if no free slot is found */
    ProtoStore* findFreeSlot() {
        for(size_t i = 0; i < SimpleCommDfs::MAX_PROTOS; i++) {
            if(protos[i].id == SimpleCommDfs::NULL_ID) {
                return &protos[i];
            }
        }
        return nullptr;
    }

    /* @brief Compare 2 strings
     * @param a: The first string
     * @param b: The second string
     * @return True if the strings are equal, false otherwise */
    static bool strEqual(const char* a, const char* b) {
        return strcmp(a, b) == 0;
    }

#ifdef UNIT_TESTING
public:
    /* @brief Method to get the proto store (for tests only)
     * @param id: The ID of the proto to get
     * @return The pointer to the proto found or nullptr if not found */
    const ProtoStore* getProtoStore(uint8_t id) const {
        for(size_t i = 0; i < SimpleCommDfs::MAX_PROTOS; i++) {
            if(protos[i].id == id) {
                return &protos[i];
            }
        }
        return nullptr;
    }
#endif

};