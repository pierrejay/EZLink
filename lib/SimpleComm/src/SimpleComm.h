#pragma once
#include <Arduino.h>
#include <functional>
#include <type_traits>
#include "ScrollBuffer.h"

// Uncomment to enable debug
// #define SIMPLECOMM_DEBUG
// #define SIMPLECOMM_DEBUG_TAG "DBG"

// TODO:
// 1. Implement no-lock SPSC buffer
// 2. Decouple communication layer w/ byte input/output ("expert mode")
// 3. Test with STM32F030
// Later ?: Support validation of the full message structure (hash ?)

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

// Constants that must stay out of the class as they
// cannot be inlined in C++11

namespace SimpleCommDfs {

// Frame format
static constexpr size_t MAX_FRAME_SIZE = 32;      // Defines max RX buffer size
static constexpr uint8_t START_OF_FRAME = 0xAA;   // Start Of Frame marker
static constexpr size_t FRAME_OVERHEAD = 5;       // SOF + LEN + ID + CRC*2
// ID constants
static constexpr uint8_t NULL_FC = 0;             // ID=0 reserved/invalid
static constexpr uint8_t MAX_PROTOS = 10;         // Maximum number of protos registered overall
static constexpr uint8_t FC_RESPONSE_BIT = 0x80;  // Bit 7 set for responses
static constexpr uint8_t FC_MAX_USER = 0x7F;      // Max 127 user-defined ID (1-127)
// Default timeouts
static constexpr uint32_t DEFAULT_RESPONSE_TIMEOUT_MS = 500;   // Wait max 500ms for a response

} // namespace SimpleCommDfs

class SimpleComm {
    
public:

    // Proto types
    enum class ProtoType {
        MESSAGE,  // No response expected
        MESSAGE_ACK,     // Echo expected
        REQUEST,          // Expect a specific response
        RESPONSE          // Is a response to a request
    };

    // Result of an operation
    enum Status {
        SUCCESS = 0,
        // Poll result
        NOTHING_TO_DO = 1,
        // RegisterProto errors
        ERR_FC_ALREADY_REGISTERED = 10,     // ID already registered
        ERR_NAME_ALREADY_REGISTERED = 11,   // Name already registered
        ERR_TOO_MANY_PROTOS = 12,           // Too many protos registered
        ERR_INVALID_NAME = 13,              // Name is null or empty
        ERR_REG_INVALID_FC = 14,            // ID not registered
        ERR_REG_PROTO_MISMATCH = 15,        // Inconsistency between expected and registered proto
        // Poll & sendMsg errors
        ERR_BUSY_RECEIVING = 20,            // Frame capture pending, we must wait for the end of the frame to send a new message
        // RX errors
        ERR_RCV_MIN = 30,                   // Dummy error code
        ERR_RCV_INVALID_FC = 31,            // ID not registered
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
        ERR_SND_INVALID_FC = 41,            // ID not registered
        ERR_SND_EMPTY_DATA = 42,            // Empty data to send
        ERR_SND_OVERFLOW = 43,              // Buffer too small
        ERR_SND_PROTO_MISMATCH = 44,        // Inconsistency between expected and sent proto
        ERR_SND_MAX = 45                    // Dummy error code
    };

    // Complete result of a reception operation
    struct Result {
        Status status;
        uint8_t id;  // ID of the processed message (if status == SUCCESS)

        // Overload operators for direct comparison with Status
        bool operator==(Status s) const { return status == s; }
        bool operator!=(Status s) const { return status != s; }
    };
    // Helpers to create results
    static constexpr Result Error(Status status, uint8_t id = SimpleCommDfs::NULL_FC) { return Result{status, id}; }
    static constexpr Result Success(uint8_t id) { return Result{SUCCESS, id}; }
    static constexpr Result NoData() { return Result{NOTHING_TO_DO, SimpleCommDfs::NULL_FC}; }

    // Constructor
    explicit SimpleComm(HardwareSerial* serial 
                       , uint32_t responseTimeoutMs = SimpleCommDfs::DEFAULT_RESPONSE_TIMEOUT_MS
                       #ifdef SIMPLECOMM_DEBUG
                       , Stream* debugStream = nullptr
                       , const char* instanceName = ""  // Nom de l'instance pour les logs
                       #endif
                       ) 
        : serial(serial)
        , responseTimeoutMs(responseTimeoutMs)
        #ifdef SIMPLECOMM_DEBUG
        , debugStream(debugStream)
        , instanceName(instanceName)
        #endif
    {}

    // Make cleanupRxBuffer public to allow manual cleanup if needed
    void cleanupRxBuffer() {
        // Clear the hardware buffer first
        while (serial->available()) {
            serial->read();
        }
        // Clear our internal RX buffer
        rxBuffer.clear();
        frameCapturePending = false;
        
        #ifdef SIMPLECOMM_DEBUG
        debugPrint("RX buffer cleaned");
        #endif
    }

    // Initialize the communication
    void begin() {
        // Clean any residual data that might be in the hardware buffer
        cleanupRxBuffer();
        
        #ifdef SIMPLECOMM_DEBUG
        debugPrint("Instance initialized");
        #endif
    }

    // Extract data from a message
    template<typename T>
    void extractData(const T& msg, uint8_t* outData, size_t& outDataLen) {
        if (outData) {
            outDataLen = sizeof(T);
            memcpy(outData, &msg, outDataLen);
        }
    }

    // Register REQUEST prototype
    template<typename T>
    Result registerRequest() {
        static_assert(T::type == ProtoType::REQUEST || 
                     T::type == ProtoType::MESSAGE || 
                     T::type == ProtoType::MESSAGE_ACK,
                     "You tried to register a RESPONSE with registerRequest() - use registerResponse() for RESPONSE types");
        static_assert(T::id != SimpleCommDfs::NULL_FC, "ID=0 is reserved/invalid");
        static_assert((T::id & SimpleCommDfs::FC_RESPONSE_BIT) == 0, "Request ID must be <= 127");
        static_assert(T::name != nullptr, "Message must have a name");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(sizeof(T) + SimpleCommDfs::FRAME_OVERHEAD <= SimpleCommDfs::MAX_FRAME_SIZE, "Message too large");
        return registerProtoInternal<T>(T::id);
    }

    // Register RESPONSE prototype
    template<typename T>
    Result registerResponse() {
        static_assert(T::type == ProtoType::RESPONSE, 
                     "Wrong message type - registerResponse() only works with RESPONSE types");
        static_assert(T::id != SimpleCommDfs::NULL_FC, "ID=0 is reserved/invalid");
        static_assert((T::id & SimpleCommDfs::FC_RESPONSE_BIT) == 0, "Response ID must be <= 127");
        static_assert(T::name != nullptr, "Message must have a name");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(sizeof(T) + SimpleCommDfs::FRAME_OVERHEAD <= SimpleCommDfs::MAX_FRAME_SIZE, "Message too large");

        // Vérifier qu'une requête avec ce ID est enregistrée
        ProtoStore* requestProto = findProto(T::id);  // Chercher avec le ID original
        if (requestProto == nullptr) {
            return Error(ERR_REG_INVALID_FC, T::id);  // Pas de requête correspondante
        }

        // On remplace le ID par son complément (ID | 0x80)
        return registerProtoInternal<T>(T::id | SimpleCommDfs::FC_RESPONSE_BIT);
    }

    // Handler for MESSAGE and MESSAGE_ACK messages
    template<typename T>
    Result onReceive(std::function<void(const T&)> handler) {
        static_assert(T::type == ProtoType::MESSAGE || 
                     T::type == ProtoType::MESSAGE_ACK,
                     "onReceive only works with MESSAGE or MESSAGE_ACK messages");
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
            // Appeler le handler
            handler(req, resp);

            // Envoyer la réponse automatiquement
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

    // Type-safe send for MESSAGE messages
    template<typename T>
    Result sendMsg(const T& msg) {
        static_assert(T::type == ProtoType::MESSAGE, "Wrong message type, sendMsg() only works with MESSAGE messages");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert((T::id & SimpleCommDfs::FC_RESPONSE_BIT) == 0, "ID must be <= 127");
        static_assert(T::id != SimpleCommDfs::NULL_FC, "ID must not be NULL (0)");  // Check at compilation
        return sendMsgInternal(msg, false);
    }

    // Send message and wait for acknowledgement (same message echoed back)
    template<typename T>
    Result sendMsgAck(const T& msg) {
        static_assert(T::type == ProtoType::MESSAGE_ACK, "Wrong message type, sendMsgAck() only works with MESSAGE_ACK messages");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert((T::id & SimpleCommDfs::FC_RESPONSE_BIT) == 0, "ID must be <= 127");
        static_assert(T::id != SimpleCommDfs::NULL_FC, "ID must not be NULL (0)");  // Check at compilation
        
        // Clean the RX buffer before sending
        cleanupRxBuffer();
        
        // Send message first
        Result result = sendMsgInternal(msg, false);
        if (result != SUCCESS) {
            return Error(result.status, T::id);
        }
        
        // Wait for echo with timeout
        uint8_t frame[SimpleCommDfs::MAX_FRAME_SIZE];
        size_t frameLen;
        
        unsigned long startTime = millis();
        while(millis() - startTime < responseTimeoutMs) {
            // We expect a response with a complement of the message ID (ID | 0x80)
            result = captureFrame(T::id | SimpleCommDfs::FC_RESPONSE_BIT, frame, &frameLen);
            if(result == NOTHING_TO_DO) {
                continue;
            }
            if(result != SUCCESS) {
                return result;
            }
            
            // Vérifier que la réponse fait la même taille que la requête
            if(frameLen - SimpleCommDfs::FRAME_OVERHEAD != sizeof(T)) {
                return Error(ERR_RCV_ACK_MISMATCH);
            }
            
            // Vérifier que le contenu est identique
            if(memcmp(&frame[3], &msg, sizeof(T)) != 0) {
                return Error(ERR_RCV_ACK_MISMATCH);
            }
            
            return Success(T::id);
        }
        
        frameCapturePending = false; // Reset frame capture - protects against late responses
        return Error(ERR_RCV_TIMEOUT);
    }

    // Send request and wait for specific response type
    template<typename REQ, typename RESP>
    Result sendRequest(const REQ& req, RESP& resp) {
        static_assert(REQ::type == ProtoType::REQUEST, "Wrong message type, sendRequest() only works with REQUEST messages");
        static_assert(RESP::type == ProtoType::RESPONSE, "Response must be a RESPONSE type");
        static_assert(std::is_same<RESP, typename REQ::ResponseType>::value, "Response type doesn't match request's ResponseType");
        static_assert(std::is_standard_layout<REQ>::value, "Request type must be POD/standard-layout");
        static_assert(std::is_standard_layout<RESP>::value, "Response type must be POD/standard-layout");
        static_assert(REQ::id != SimpleCommDfs::NULL_FC, "ID must not be NULL (0)");
        static_assert((REQ::id & SimpleCommDfs::FC_RESPONSE_BIT) == 0, "Request ID must be <= 127");
        static_assert(REQ::id == RESP::id, "Response ID must match Request ID"); 
        
        // Clean the RX buffer before sending
        cleanupRxBuffer();
        
        // Send request
        Result result = sendMsgInternal(req);
        if (result != SUCCESS) {
            return Error(result.status, REQ::id);
        }
        
        // Wait for response with timeout
        uint8_t frame[SimpleCommDfs::MAX_FRAME_SIZE];
        size_t frameLen;
        
        unsigned long startTime = millis();
        while(millis() - startTime < responseTimeoutMs) {
            // We expect a response with a complement of the request ID (ID | 0x80)
            result = captureFrame(RESP::id | SimpleCommDfs::FC_RESPONSE_BIT, frame, &frameLen);  // Attendre le ID avec bit de réponse
            if(result == NOTHING_TO_DO) {
                continue;
            }
            if(result != SUCCESS) {
                return result;
            }
            
            // Vérifier que la réponse fait la bonne taille
            if(frameLen - SimpleCommDfs::FRAME_OVERHEAD != sizeof(RESP)) {
                return Error(ERR_RCV_RESP_MISMATCH);
            }
            
            // Frame capturée, on la copie dans la réponse
            memcpy(&resp, &frame[3], sizeof(RESP));
            return Success(RESP::id);
        }
        
        frameCapturePending = false; // Reset frame capture - protects against late responses
        return Error(ERR_RCV_TIMEOUT);
    }

    // Process received messages with optional frame capture
    Result poll() {
        uint8_t rxBuffer[SimpleCommDfs::MAX_FRAME_SIZE];  // Buffer local
        size_t frameLen;

        // Tente de capturer une frame dans le buffer local
        Result result = captureFrame(0, rxBuffer, &frameLen);
        
        // Si la capture n'a pas réussi, on retourne l'erreur obtenue
        if(result != SUCCESS && result != NOTHING_TO_DO) {
            return result;
        }
        
        // Si la capture a réussi, vérifier que ce n'est pas une réponse
        if(result == SUCCESS) {
            
            // Rejeter les réponses (bit 7 setté)
            if((result.id & SimpleCommDfs::FC_RESPONSE_BIT) != 0) {
                return Error(ERR_RCV_UNEXPECTED_RESPONSE, result.id);
            }

            // Trouver le proto et vérifier la taille
            ProtoStore* proto = findProto(result.id);
            if(proto) {
                // Vérifier que la taille correspond au proto
                if(frameLen - SimpleCommDfs::FRAME_OVERHEAD != proto->size) {
                    return Error(ERR_RCV_PROTO_MISMATCH, result.id);
                }
                
                // Appeler le handler si enregistré
                if(proto->callback) {
                    proto->callback(&rxBuffer[3]);
                }

                // Si c'est un message MESSAGE_ACK, envoyer l'ACK automatiquement,
                // en flippant le ID
                uint8_t ackId = result.id | SimpleCommDfs::FC_RESPONSE_BIT;
                if(proto->type == ProtoType::MESSAGE_ACK) {
                    sendFrame(ackId, &rxBuffer[3], proto->size);
                }
            }
            return result;
        }
        
        // Si on n'a rien capturé, on retourne NOTHING_TO_DO
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

    // Utility function to calculate CRC8
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

    // Utility function to calculate CRC16
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
    // Structure to store message prototypes
    struct ProtoStore {
        ProtoType type = ProtoType::MESSAGE;
        const char* name = nullptr;
        uint8_t id = SimpleCommDfs::NULL_FC;
        size_t size = 0;
        std::function<void(const void*)> callback = nullptr;
    };

    // Attributes
    HardwareSerial* serial;         // Port de communication
    uint32_t responseTimeoutMs;
    ProtoStore protos[SimpleCommDfs::MAX_PROTOS];
    bool frameCapturePending = false;

    #ifdef SIMPLECOMM_DEBUG
    Stream* debugStream;            // Port de debug (optionnel)
    const char* instanceName;       // Nom de l'instance pour les logs

    // Helper pour les logs de debug
    void debugPrint(const char* msg) {
        if (debugStream) {
            debugStream->print("[");
            debugStream->print(SIMPLECOMM_DEBUG_TAG);
            debugStream->print("_");
            debugStream->print(instanceName);
            debugStream->print("]: ");
            debugStream->print(msg);
            debugStream->print("\n");  // Un seul \n explicite
        }
    }
    
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

    // Helper pour les logs de debug
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
        
        // Afficher l'analyse rapide si c'est une frame
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
        
        // Afficher le hexdump
        for(size_t i = 0; i < len; i++) {
            if(data[i] < 0x10) debugStream->print("0");
            debugStream->print(data[i], HEX);
            debugStream->print(" ");
        }
        debugStream->println();
    }
    #endif

    // Register message prototype
    template<typename T>
    Result registerProtoInternal(uint8_t id) {
        // Proto validation is already done in registerRequest() and registerResponse()
        
        // Invalid names (nullptr or "") are blocked at compilation
        if(T::name == nullptr || T::name[0] == '\0') {
            return Error(ERR_INVALID_NAME, T::id);
        }
        
        // Check if already registered (by ID or by name)
        for(size_t i = 0; i < SimpleCommDfs::MAX_PROTOS; i++) {
            if(protos[i].id != SimpleCommDfs::NULL_FC) {
                if(protos[i].id == id) {  // On compare avec le ID modifié !
                    return Error(ERR_FC_ALREADY_REGISTERED, T::id);
                }
                if(strEqual(protos[i].name, T::name)) {
                    return Error(ERR_NAME_ALREADY_REGISTERED, T::id);
                }
            }
        }
        
        // Find a free slot
        ProtoStore* slot = findFreeSlot();
        if(slot == nullptr) {
            return Error(ERR_TOO_MANY_PROTOS, T::id);
        }
        
        // Register the proto
        slot->id = id;  // Utiliser le ID modifié passé en paramètre
        slot->type = T::type;
        slot->size = sizeof(T);
        slot->name = T::name;  // Store the name
        
        return Success(T::id);
    }

    // Check if proto is registered and matches
    // If outProto is not null, it will be set to the proto found
    template<typename T>
    Result checkProto(uint8_t id, ProtoStore** outProto = nullptr) {
        ProtoStore* proto = findProto(id);
        if (proto == nullptr) {
            return Error(ERR_SND_INVALID_FC, id);
        }
        if (!strEqual(proto->name, T::name)) {
            return Error(ERR_SND_PROTO_MISMATCH, id);
        }
        if(outProto) {
            *outProto = proto;
        }
        return Success(id);
    }

    // Send message
    template<typename T>
    Result sendMsgInternal(const T& msg, bool flipId = false) {

        // Define ID, flip if needed (for responses)
        uint8_t id = msg.id;
        if (flipId) id |= SimpleCommDfs::FC_RESPONSE_BIT;

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

    // Send message
    Result sendFrame(uint8_t id, uint8_t* data, size_t dataLen) {
        if (!data || dataLen == 0) {
            return Error(ERR_SND_EMPTY_DATA);
        }

        // Check if we are already capturing a frame
        if(frameCapturePending) {
            return Error(ERR_BUSY_RECEIVING);
        }
        
        // Check if we have enough space
        size_t frameSize = dataLen + SimpleCommDfs::FRAME_OVERHEAD; // Only includes non-static fields (data) + overhead
        if(frameSize > serial->availableForWrite()) {
            return Error(ERR_SND_OVERFLOW);
        }

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
        // Log la frame envoyée
            debugHexDump("TX frame", frame, frameSize);
        #endif

        // Send frame (now we are sure we have enough space)
        serial->write(frame, frameSize);
        serial->flush();

        return Success(id);
    }

    // "Scrolling" ring buffer for reception
    ScrollBuffer<uint8_t, SimpleCommDfs::MAX_FRAME_SIZE> rxBuffer;

    Result captureFrame(uint8_t expectedId = 0, uint8_t* outFrame = nullptr, size_t* outLen = nullptr) {

        const uint8_t sof = SimpleCommDfs::START_OF_FRAME;

        // D'abord on lit tout ce qui est disponible sur le port série
        while (serial->available() && rxBuffer.size() < SimpleCommDfs::MAX_FRAME_SIZE) {
            rxBuffer.push(serial->read());
        }

        // Pas assez de données
        if (rxBuffer.size() == 0) {
            return NoData();
        }

        // Si pas de capture en cours, on cherche un SOF valide
        if (!frameCapturePending) {
            // Pas de données
            if (rxBuffer.size() == 0) {
                return NoData();
            }
            
            // L'octet courant doit être un SOF
            if (rxBuffer.peek(0) != sof) {
                // Log le SOF invalide
                uint8_t invalidSof = rxBuffer.peek(0);
                #ifdef SIMPLECOMM_DEBUG
                debugHexDump("RX invalid SOF", &invalidSof, 1);
                #endif
                
                // Sinon on essaye de glisser jusqu'au prochain SOF
                rxBuffer.scrollTo(sof); 
                return Error(ERR_RCV_INVALID_SOF);
            }
            
            // On a maintenant un SOF valide, on commence la capture
            frameCapturePending = true;
        }

        // Capture en cours, on doit avoir au moins 2 octets pour lire le LEN
        if (rxBuffer.size() < 2) {
            return NoData();
        }

        // Vérifier LEN
        uint8_t frameSize = rxBuffer.peek(1);
        if (frameSize < SimpleCommDfs::FRAME_OVERHEAD || frameSize > SimpleCommDfs::MAX_FRAME_SIZE) {
            // Log SOF + LEN invalide
            #ifdef SIMPLECOMM_DEBUG
            uint8_t header[2];
            header[0] = rxBuffer.peek(0);
            header[1] = rxBuffer.peek(1);
            debugHexDump("RX invalid LEN", header, 2);
            #endif
            
            // Si LEN invalide, on jette le SOF
            rxBuffer.dump(1);
            // On essaye de glisser jusqu'au prochain SOF
            rxBuffer.scrollTo(sof);
            return Error(ERR_RCV_INVALID_LEN);
        }

        // Attendre d'avoir la frame complète
        if (rxBuffer.size() < frameSize) {
            return NoData();
        }

        // On a une frame complète, on la copie pour le CRC
        uint8_t tempFrame[SimpleCommDfs::MAX_FRAME_SIZE];
        for (uint8_t i = 0; i < frameSize; i++) {
            tempFrame[i] = rxBuffer.peek(i);
        }

        // Valider CRC en premier avant de vérifier le contenu
        uint16_t receivedCRC = ((uint16_t)tempFrame[frameSize-2] << 8) | tempFrame[frameSize-1];
        uint16_t calculatedCRC = calculateCRC16(tempFrame, frameSize-2);
        if (receivedCRC != calculatedCRC) {
            frameCapturePending = false;
            // CRC invalide, peut être une frame tronquée suivie d'une frame valide, 
            // on jette le SOF et on essaie de glisser jusqu'au prochain.
            // - Si c'était une frame tronquée, la suivante sera traitée au prochain poll().
            // - Si ce n'était pas une frame tronquée, on peut tomber sur un SOF présent
            //   dans les données de la frame invalide, il sera rejeté.
            // Dans tous les cas, on ne jette aucune frame valide, tant qu'on appelle
            // régulièrement poll() on finira par tomber sur le SOF d'une frame valide.
            #ifdef SIMPLECOMM_DEBUG
            debugHexDump("RX invalid CRC", tempFrame, frameSize);
            #endif
            rxBuffer.dump(1);
            rxBuffer.scrollTo(sof);
            return Error(ERR_RCV_CRC);
        }

        // Le CRC est valide, on peut maintenant vérifier le ID si nécessaire
        uint8_t id = rxBuffer.peek(2);
        if (expectedId > 0 && id != expectedId) {
            frameCapturePending = false;
            // ID invalide, on jette la frame entière et on essaie de glisser jusqu'au prochain SOF
            #ifdef SIMPLECOMM_DEBUG
            debugHexDump("RX invalid ID", tempFrame, frameSize);
            #endif
            rxBuffer.dump(frameSize);
            rxBuffer.scrollTo(sof);
            return Error(ERR_RCV_INVALID_FC, id);
        }

        #ifdef SIMPLECOMM_DEBUG
        debugHexDump("RX valid frame", tempFrame, frameSize);
        #endif

        // Frame valide !
        if (outFrame && outLen) {
            memcpy(outFrame, tempFrame, frameSize);
            *outLen = frameSize;
        }

        // On consomme la frame
        rxBuffer.dump(frameSize);
        frameCapturePending = false;

        return Success(id);
    }

    // Generic handler (internal usage only)
    template<typename T>
    Result onMessage(std::function<void(const T&)> handler) {
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(T::id < 256, "ID must be < 256");
        
        ProtoStore* proto = findProto(T::id);
        if(proto == nullptr) {
            return Error(ERR_REG_INVALID_FC, T::id);
        }
        if (!strEqual(proto->name, T::name)) {
            return Error(ERR_REG_PROTO_MISMATCH, T::id);
        }
        
        proto->callback = [handler](const void* data) {
            handler(*static_cast<const T*>(data));
        };
        
        return Success(T::id);
    }

    // Find a proto by its ID
    ProtoStore* findProto(uint8_t id) {
        for(size_t i = 0; i < SimpleCommDfs::MAX_PROTOS; i++) {
            if(protos[i].id == id) {
                return &protos[i];
            }
        }
        return nullptr;
    }

    // Find a free slot
    ProtoStore* findFreeSlot() {
        for(size_t i = 0; i < SimpleCommDfs::MAX_PROTOS; i++) {
            if(protos[i].id == SimpleCommDfs::NULL_FC) {
                return &protos[i];
            }
        }
        return nullptr;
    }

    // Compare 2 strings
    static bool strEqual(const char* a, const char* b) {
        return strcmp(a, b) == 0;
    }

    // Fonction pour envoyer automatiquement un ACK
    void autoSendAck(uint8_t originalId, const void* payload, size_t payloadSize) {
        // Vérifier qu'on n'est pas déjà en train de capturer une frame
        if(frameCapturePending) {
            return;  // On ne peut pas envoyer d'ACK pendant une capture
        }
        
        // Create the ACK ID (bit 7 = 1)
        uint8_t ackId = originalId | SimpleCommDfs::FC_RESPONSE_BIT;
        
        // Vérifier qu'on a assez d'espace dans le buffer TX
        size_t frameSize = payloadSize + SimpleCommDfs::FRAME_OVERHEAD;
        if(frameSize > serial->availableForWrite()) {
            return;  // Buffer TX plein, on ne peut pas envoyer l'ACK
        }
        
        // Construire la frame d'ACK
        uint8_t frame[SimpleCommDfs::MAX_FRAME_SIZE];
        frame[0] = SimpleCommDfs::START_OF_FRAME;
        frame[1] = frameSize;
        frame[2] = ackId;
        memcpy(&frame[3], payload, payloadSize);
        
        // Calculer et ajouter le CRC
        uint16_t crc = calculateCRC16(frame, frameSize - 2);
        frame[frameSize-2] = (uint8_t)(crc >> 8);
        frame[frameSize-1] = (uint8_t)(crc & 0xFF);
        
        // Envoyer la frame
        serial->write(frame, frameSize);
        serial->flush();
    }

#ifdef UNIT_TESTING
public:
    // Méthode pour les tests uniquement
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