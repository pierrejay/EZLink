#pragma once
#include <Arduino.h>
#include <functional>
#include <type_traits>

class SimpleComm {
public:
    // Protocol constants
    static constexpr uint8_t START_OF_FRAME = 0xAA;
    static constexpr uint8_t FRAME_HEADER_SIZE = 2;  // SOF + LEN
    static constexpr size_t FRAME_OVERHEAD = 4;      // SOF + LEN + FC + CRC
    static constexpr size_t MAX_FRAME_SIZE = 32;
    static constexpr uint8_t NULL_FC = 0;  // FC=0 reserved/invalid
    static constexpr uint8_t MAX_PROTOS = 10;  // Maximum number of protos
    
    // Default timeouts
    static constexpr uint32_t DEFAULT_INTERBYTE_TIMEOUT_MS = 10;   // 10ms between bytes
    static constexpr uint32_t DEFAULT_RESPONSE_TIMEOUT_MS = 500;   // 500ms for a response
    
    // Proto types
    enum class ProtoType {
        FIRE_AND_FORGET,  // No response expected
        ACK_REQUIRED,     // Echo expected
        REQUEST,          // Expect a specific response
        RESPONSE          // Is a response to a request
    };

    // Result of an operation
    enum Status {
        SUCCESS = 0,
        // Poll result
        NOTHING_TO_DO,
        // RegisterProto errors
        ERR_FC_ALREADY_REGISTERED,  // FC already registered
        ERR_NAME_ALREADY_REGISTERED, // Name already registered
        ERR_TOO_MANY_PROTOS,       // Too many protos registered
        ERR_INVALID_NAME,          // Name is null or empty
        // Poll & sendMsg errors
        ERR_INVALID_FC,     // FC not registered
        ERR_INVALID_SOF,    // Invalid SOF
        ERR_INVALID_LEN,    // Invalid length
        ERR_PROTO_MISMATCH, // Inconsistency between expected and received proto
        ERR_TIMEOUT,        // No response
        ERR_CRC,           // Invalid CRC
        ERR_OVERFLOW,       // Buffer too small
    };

    // Complete result of a reception operation
    struct Result {
        Status status;
        uint8_t fc;  // FC of the processed message (if status == SUCCESS)

        // Overload operators for direct comparison with Status
        bool operator==(Status s) const { return status == s; }
        bool operator!=(Status s) const { return status != s; }
    };

    // Constructor
    explicit SimpleComm(HardwareSerial* serial, 
                        uint32_t interbyteTimeoutMs = DEFAULT_INTERBYTE_TIMEOUT_MS,
                        uint32_t responseTimeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS) 
        : serial(serial)
        , interbyteTimeoutMs(interbyteTimeoutMs)
        , responseTimeoutMs(responseTimeoutMs)
    {}


    // Register message prototype
    template<typename T>
    Result registerProto() {
        static_assert(T::fc != NULL_FC, "FC=0 is reserved/invalid");
        static_assert(T::name != nullptr, "Message must have a name");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(T::fc < 256, "FC must be < 256");
        static_assert(sizeof(T) + FRAME_OVERHEAD <= MAX_FRAME_SIZE, "Message too large");
        
        // Invalid names (nullptr or "") are blocked at compilation
        if(T::name == nullptr || T::name[0] == '\0') {
            return Error(ERR_INVALID_NAME, T::fc);
        }
        
        // Check if already registered (by FC or by name)
        for(size_t i = 0; i < MAX_PROTOS; i++) {
            if(protos[i].fc != NULL_FC) {
                if(protos[i].fc == T::fc) {
                    return Error(ERR_FC_ALREADY_REGISTERED, T::fc);
                }
                if(strEqual(protos[i].name, T::name)) {
                    return Error(ERR_NAME_ALREADY_REGISTERED, T::fc);
                }
            }
        }
        
        // Find a free slot
        ProtoStore* slot = findFreeSlot();
        if(slot == nullptr) {
            return Error(ERR_TOO_MANY_PROTOS, T::fc);
        }
        
        // Register the proto
        slot->fc = T::fc;
        slot->type = T::type;
        slot->size = sizeof(T);
        slot->name = T::name;  // Store the name
        
        return Success(T::fc);
    }

    // Handler for FIRE_AND_FORGET and ACK_REQUIRED messages
    template<typename T>
    Result onReceive(std::function<void(const T&)> handler) {
        static_assert(T::type == ProtoType::FIRE_AND_FORGET || 
                     T::type == ProtoType::ACK_REQUIRED,
                     "onReceive only works with FIRE_AND_FORGET or ACK_REQUIRED messages");
        return onMessage<T>(handler);
    }

    // Handler for REQUEST messages with automatic response
    template<typename REQ>
    Result onRequest(std::function<void(const REQ&, typename REQ::ResponseType&)> handler) {
        static_assert(REQ::type == ProtoType::REQUEST, 
                     "onRequest only works with REQUEST messages");
        static_assert(REQ::ResponseType::type == ProtoType::RESPONSE, 
                     "Response type must be RESPONSE");
        
        return onMessage<REQ>([this, handler](const REQ& req) {
            typename REQ::ResponseType resp;
            handler(req, resp);
            sendMsgInternal(resp);  // Automatic response
        });
    }

    // Type-safe send for FIRE_AND_FORGET messages
    template<typename T>
    Result sendMsg(const T& msg) {
        static_assert(T::type == ProtoType::FIRE_AND_FORGET, "Wrong message type");
        return sendMsgInternal(msg);
    }

    // Send message and wait for acknowledgement (same message echoed back)
    template<typename T>
    Result sendMsgAck(const T& msg) {
        static_assert(T::type == ProtoType::ACK_REQUIRED, "Wrong message type");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(T::fc < 256, "FC must be < 256");
        
        // Send message first
        Result result = sendMsgInternal(msg);
        if (result != SUCCESS) {
            return Error(result.status, T::fc);
        }
        
        // Wait for echo
        T echo;
        Result rcvResult = receiveMsg(echo, T::fc, true);
        if (rcvResult != SUCCESS) {
            return Error(rcvResult.status, T::fc);
        }
        
        // Verify echo matches
        if (memcmp(&msg, &echo, sizeof(T)) != 0) {
            return Error(ERR_PROTO_MISMATCH, T::fc);
        }
        
        return Success(T::fc);
    }

    // Send request and wait for specific response type
    template<typename REQ, typename RESP>
    Result sendRequest(const REQ& req, RESP& resp) {
        static_assert(REQ::type == ProtoType::REQUEST, "Wrong message type");
        static_assert(RESP::type == ProtoType::RESPONSE, "Response must be a RESPONSE type");
        static_assert(std::is_standard_layout<REQ>::value, "Request type must be POD/standard-layout");
        static_assert(std::is_standard_layout<RESP>::value, "Response type must be POD/standard-layout");
        static_assert(std::is_same<RESP, typename REQ::ResponseType>::value, "Response type doesn't match request's ResponseType");
        static_assert(REQ::fc < 256 && RESP::fc < 256, "FC must be < 256");
        
        // Send request
        Result result = sendMsgInternal(req);
        if (result != SUCCESS) {
            return Error(result.status, REQ::fc);
        }
        
        // Wait for response
        Result rcvResult = receiveMsg(resp, RESP::fc, true);
        if (rcvResult != SUCCESS) {
            return Error(rcvResult.status, RESP::fc);
        }
        
        return Success(RESP::fc);
    }

    // Process received messages
    Result poll() {
        uint8_t byte;
        
        // Check if there's data available
        if(!serial->available()) {
            return NoData();
        }
        
        // Wait for START_OF_FRAME
        if(!waitByte(byte)) {
            return Error(ERR_TIMEOUT);
        }
        if(byte != START_OF_FRAME) {
            flushRxBufferUntilSOF();
            return Error(ERR_INVALID_SOF);
        }
        
        // Get length
        if(!waitByte(byte)) {
            // If waitByte fails, the buffer is already empty
            return Error(ERR_TIMEOUT);
        }
        uint8_t frameLen = byte;
        
        // Validate frame length
        if(frameLen < FRAME_OVERHEAD || frameLen > MAX_FRAME_SIZE) {
            flushRxBufferUntilSOF();
            return Error(ERR_INVALID_LEN);
        }
        
        // Get FC
        if(!waitByte(byte)) {
            // If waitByte fails, the buffer is already empty
            return Error(ERR_TIMEOUT);
        }
        uint8_t fc = byte;

        // Check if FC is registered
        ProtoStore* proto = findProto(fc);
        if(proto == nullptr) {
            return Error(ERR_INVALID_FC, fc);
        }
        
        // Validate FC and expected message size
        // Note: We can't verify the message name here as we don't have the type T
        // TODO: Add stronger proto verification with hash?
        if(proto->size == 0) {
            // We have already consumed the whole buffer
            return Error(ERR_INVALID_FC, fc);
        }
        if (proto->size != frameLen - FRAME_OVERHEAD) {
            // We have already consumed the whole buffer
            return Error(ERR_PROTO_MISMATCH, fc);
        }
        
        // Get data + CRC
        size_t remainingBytes = frameLen - FRAME_HEADER_SIZE - 1;
        if(!waitBytes(rxBuffer + FRAME_HEADER_SIZE + 1, remainingBytes)) {
            return Error(ERR_TIMEOUT, fc);
        }
        
        // Reconstruct full frame for CRC check
        rxBuffer[0] = START_OF_FRAME;
        rxBuffer[1] = frameLen;
        rxBuffer[2] = fc;
        
        // Validate CRC
        uint8_t receivedCRC = rxBuffer[frameLen-1];
        uint8_t calculatedCRC = calculateCRC(rxBuffer, frameLen-1);
        if(receivedCRC != calculatedCRC) {
            return Error(ERR_CRC, fc);
        }
        
        // Call handler if registered
        if(proto->callback) {
            proto->callback(&rxBuffer[3]);
            return Success(fc);
        }
        
        return Success(fc);  // Valid message even without handler
    }

    // Setters for timeouts
    bool setInterbyteTimeout(uint32_t ms) { 
        if(ms == 0) return false;
        interbyteTimeoutMs = ms; 
        return true;
    }
    bool setResponseTimeout(uint32_t ms) { 
        if(ms == 0) return false;
        responseTimeoutMs = ms; 
        return true;
    }

    // Getters for timeouts
    uint32_t getInterbyteTimeout() { 
        return interbyteTimeoutMs;
    }
    uint32_t getResponseTimeout() { 
        return responseTimeoutMs;
    }

    // Utility function to calculate CRC
    static uint8_t calculateCRC(const uint8_t* data, size_t size) {
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

private:

    // Structure to store message prototypes
    struct ProtoStore {
        ProtoType type = ProtoType::FIRE_AND_FORGET;
        const char* name = nullptr;  // Add name
        uint8_t fc = NULL_FC;  // FC of the message
        size_t size = 0;
        std::function<void(const void*)> callback = nullptr;
    };

    // Attributes
    HardwareSerial* serial;
    uint32_t interbyteTimeoutMs;  // Timeout between bytes of the same frame
    uint32_t responseTimeoutMs;   // Timeout for waiting for a response
    ProtoStore protos[MAX_PROTOS];  // Indexed by FC
    uint8_t rxBuffer[MAX_FRAME_SIZE];


    // Helpers to create results
    static constexpr Result Error(Status status, uint8_t fc = NULL_FC) { return Result{status, fc}; }
    static constexpr Result Success(uint8_t fc) { return Result{SUCCESS, fc}; }
    static constexpr Result NoData() { return Result{NOTHING_TO_DO, NULL_FC}; }

    // Send message
    template<typename T>
    Result sendMsgInternal(const T& msg) {
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(T::fc < 256, "FC must be < 256");
        static_assert(T::fc != NULL_FC, "FC must not be NULL (0)");  // Check at compilation
        
        // Check if proto is registered and matches
        ProtoStore* proto = findProto(T::fc);
        if (proto == nullptr) {
            return Error(ERR_INVALID_FC, T::fc);
        }
        if (!strEqual(proto->name, T::name)) {
            return Error(ERR_PROTO_MISMATCH, T::fc);
        }
        
        // Check if we have enough space
        size_t frameSize = sizeof(T) + FRAME_OVERHEAD; // Only includes non-static fields (data) + overhead
        if(frameSize > serial->availableForWrite()) {
            return Error(ERR_OVERFLOW, T::fc);
        }

        // Prepare frame
        uint8_t frame[MAX_FRAME_SIZE];
        frame[0] = START_OF_FRAME;
        frame[1] = frameSize;
        frame[2] = T::fc;
        memcpy(&frame[3], &msg, sizeof(T));
        frame[frameSize-1] = calculateCRC(frame, frameSize-1);

        // Send frame (now we are sure we have enough space)
        serial->write(frame, frameSize);
        serial->flush();

        return Success(T::fc);
    }

   // Reception methods
   bool waitByte(uint8_t& byte) {
       uint32_t startTime = millis();
       
       while(!serial->available()) {
           if(millis() - startTime > interbyteTimeoutMs) {
               return false;
           }
       }
       
       byte = serial->read();
       return true;
   }

   bool waitBytes(uint8_t* buffer, size_t size) {
       uint32_t startTime = millis();
       size_t bytesRead = 0;
       
       while(bytesRead < size) {
           if(millis() - startTime > interbyteTimeoutMs) {
               return false;
           }
           
           if(serial->available()) {
               buffer[bytesRead] = serial->read();
               bytesRead++;
               startTime = millis(); // Reset timeout on each byte received
           }
       }
       
       return true;
   }

    // Flush the reception buffer
    void flushRxBuffer() {
        uint32_t startTime = millis();
        
        // Continue reading until no more data or timeout
        while(millis() - startTime < interbyteTimeoutMs) {
            if(serial->available()) {
                serial->read();
                startTime = millis(); // Reset timeout on each byte read
            }
        }
    }

    // Find the next START_OF_FRAME in the buffer without consuming it
    bool flushRxBufferUntilSOF() {
        uint32_t startTime = millis();
        
        while(millis() - startTime < interbyteTimeoutMs) {
            if(serial->available()) {
                if(serial->peek() == START_OF_FRAME) {
                    return true; // SOF found but not consumed
                }
                serial->read(); // Consume only non-SOF bytes
                startTime = millis(); // Reset timeout on each byte read
            }
        }
        
        return false; // No SOF found but buffer flushed
    }

    // Helper for receiving specific message type
    template<typename T>
    Result receiveMsg(T& msg, uint8_t expectedFc, bool checkFc = false) {
        uint32_t startTime = millis();
        
        while (millis() - startTime < responseTimeoutMs) {  // Long timeout
            Result result = poll();  // Use short timeout internally
            
            if (result == NOTHING_TO_DO) {
                continue;
            }
            
            if (result != SUCCESS) {
                return Error(result.status, result.fc);
            }
            
            // If we expect specific FC, check it
            if (checkFc && result.fc != expectedFc) {
                continue;
            }
            
            memcpy(&msg, &rxBuffer[3], sizeof(T));
            return Success(result.fc);
        }
        
        return Error(ERR_TIMEOUT);
    }

    // Generic handler (internal usage only)
    template<typename T>
    Result onMessage(std::function<void(const T&)> handler) {
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(T::fc < 256, "FC must be < 256");
        
        ProtoStore* proto = findProto(T::fc);
        if(proto == nullptr) {
            return Error(ERR_INVALID_FC, T::fc);
        }
        if (!strEqual(proto->name, T::name)) {
            return Error(ERR_PROTO_MISMATCH, T::fc);
        }
        
        proto->callback = [handler](const void* data) {
            handler(*static_cast<const T*>(data));
        };
        
        return Success(T::fc);
    }

    // Find a proto by its FC
    ProtoStore* findProto(uint8_t fc) {
        for(size_t i = 0; i < MAX_PROTOS; i++) {
            if(protos[i].fc == fc) {
                return &protos[i];
            }
        }
        return nullptr;
    }

    // Find a free slot
    ProtoStore* findFreeSlot() {
        for(size_t i = 0; i < MAX_PROTOS; i++) {
            if(protos[i].fc == NULL_FC) {
                return &protos[i];
            }
        }
        return nullptr;
    }

    // Compare 2 strings
    static bool strEqual(const char* a, const char* b) {
        return strcmp(a, b) == 0;
    }

};