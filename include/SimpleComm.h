#pragma once
#include <Arduino.h>
#include <functional>
#include <type_traits>

// TODO FOR NEXT VERSIONS:
// Add support for custom communication layer (UART, SPI, ... => simple byte input/output ?)
// Decouple processing messages from the communication layer (queue + custom buffers or handlers ?)  
// Support validation of the full message structure (hash ?)
// Add more CRC options

/**
 * SimpleComm - Simple communication protocol
 * 
 * Message types:
 * - FIRE_AND_FORGET: Simple message without response (FC 1-127)
 * - ACK_REQUIRED: Message requiring echo confirmation (FC 1-127, response FC = request FC | 0x80)
 * - REQUEST: Message requiring specific response (FC 1-127)
 * - RESPONSE: Response to a REQUEST (FC = request FC | 0x80)
 * 
 * FC (Function Code) rules:
 * - Request/Command FC must be between 1-127
 * - Response FC are automatically set to request FC | 0x80 (128-255)
 * - FC 0 is reserved (invalid)
 * 
 * Usage:
 * - Register requests with registerRequest<T>()
 * - Register responses with registerResponse<T>()
 * - Responses are automatically rejected by poll() to prevent late response handling
* 
 * Safety & Robustness features:
 * 1. Buffer management
 *    - Buffer-driven approach (no interbyte timeout)
 *    - Automatic buffer cleanup on invalid frames
 *    - Frame capture state tracking
 * 
 * 2. Protocol safety
 *    - Request/Response distinction using FC MSB
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
 *    - FC range validation (1-127 requests, 128-255 responses)
 *    - Request/Response type matching
 * 
 * Error handling ensures proper cleanup and state reset in all cases,
 * making the protocol suitable for both master/slave and bidirectional
 * communication patterns.
 */

class SimpleComm {
public:
    // Buffer/frame size
    static constexpr size_t MAX_FRAME_SIZE = 32;
    // Frame format
    static constexpr uint8_t START_OF_FRAME = 0xAA;
    static constexpr uint8_t FRAME_HEADER_SIZE = 2;  // SOF + LEN
    static constexpr size_t FRAME_OVERHEAD = 4;      // SOF + LEN + FC + CRC
    // FC constants
    static constexpr uint8_t NULL_FC = 0;  // FC=0 reserved/invalid
    static constexpr uint8_t MAX_PROTOS = 10;  // Maximum number of protos
    static constexpr uint8_t FC_RESPONSE_BIT = 0x80;  // Bit 7 set pour les réponses
    static constexpr uint8_t FC_MAX_USER = 0x7F;      // 127 FC utilisateur max (0-127)
    // Default timeouts
    static constexpr uint32_t DEFAULT_RESPONSE_TIMEOUT_MS = 500;   // Wait max 500ms for a response
    
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
        ERR_BUSY_RECEIVING,      // Frame capture pending, we must wait for the end of the frame to send a new message
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
                       uint32_t responseTimeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS) 
        : serial(serial)
        , responseTimeoutMs(responseTimeoutMs)
    {}

    // Register REQUEST prototype
    template<typename T>
    Result registerRequest() {
        static_assert(T::type == ProtoType::REQUEST || 
                     T::type == ProtoType::FIRE_AND_FORGET || 
                     T::type == ProtoType::ACK_REQUIRED,
                     "Wrong message type - use registerResponse() for RESPONSE types");
        static_assert(T::fc != NULL_FC, "FC=0 is reserved/invalid");
        static_assert((T::fc & FC_RESPONSE_BIT) == 0, "Request FC must be <= 127");
        static_assert(T::name != nullptr, "Message must have a name");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(sizeof(T) + FRAME_OVERHEAD <= MAX_FRAME_SIZE, "Message too large");
        return registerProtoInternal<T>(T::fc);
    }

    // Register RESPONSE prototype
    template<typename T>
    Result registerResponse() {
        static_assert(T::type == ProtoType::RESPONSE, 
                     "Wrong message type - registerResponse() only works with RESPONSE types");
        static_assert(T::fc != NULL_FC, "FC=0 is reserved/invalid");
        static_assert((T::fc & FC_RESPONSE_BIT) == FC_RESPONSE_BIT, "Response FC must be >= 128");
        static_assert(T::name != nullptr, "Message must have a name");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(sizeof(T) + FRAME_OVERHEAD <= MAX_FRAME_SIZE, "Message too large");

        // We replace the FC by its complement (FC | 0x80)
        return registerProtoInternal<T>(T::fc | FC_RESPONSE_BIT);
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
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert((REQ::fc & FC_RESPONSE_BIT) == 0, "FC must be <= 127");
        static_assert(T::fc != NULL_FC, "FC must not be NULL (0)");  // Check at compilation
        return sendMsgInternal(msg);
    }

    // Send message and wait for acknowledgement (same message echoed back)
    template<typename T>
    Result sendMsgAck(const T& msg) {
        static_assert(T::type == ProtoType::ACK_REQUIRED, "Wrong message type");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert((REQ::fc & FC_RESPONSE_BIT) == 0, "FC must be <= 127");
        static_assert(T::fc != NULL_FC, "FC must not be NULL (0)");  // Check at compilation
        
        // Send message first
        Result result = sendMsgInternal(msg);
        if (result != SUCCESS) {
            return Error(result.status, T::fc);
        }
        
        // Wait for echo with timeout
        uint8_t frame[MAX_FRAME_SIZE];
        size_t frameLen;
        T echo;
        
        unsigned long startTime = millis();
        while(millis() - startTime < responseTimeoutMs) {
            result = captureFrame(T::fc | FC_RESPONSE_BIT, frame, &frameLen);
            if(result == NOTHING_TO_DO) {
                continue;
            }
            if(result != SUCCESS) {
                return result;
            }
            
            // Frame capturée, on vérifie qu'elle correspond
            memcpy(&echo, &frame[3], sizeof(T));
            if(memcmp(&msg, &echo, sizeof(T)) != 0) {
                return Error(ERR_PROTO_MISMATCH, T::fc);
            }
            
            return Success(T::fc);
        }
        
        frameCapturePending = false; // Reset frame capture - protects against late responses
        return Error(ERR_TIMEOUT);
    }

    // Send request and wait for specific response type
    template<typename REQ, typename RESP>
    Result sendRequest(const REQ& req, RESP& resp) {
        static_assert(REQ::type == ProtoType::REQUEST, "Wrong message type");
        static_assert(RESP::type == ProtoType::RESPONSE, "Response must be a RESPONSE type");
        static_assert(std::is_same<RESP, typename REQ::ResponseType>::value, "Response type doesn't match request's ResponseType");
        static_assert(std::is_standard_layout<REQ>::value, "Request type must be POD/standard-layout");
        static_assert(std::is_standard_layout<RESP>::value, "Response type must be POD/standard-layout");
        static_assert(REQ::fc != NULL_FC, "FC must not be NULL (0)");
        static_assert((REQ::fc & FC_RESPONSE_BIT) == 0, "Request FC must be <= 127");
        static_assert((RESP::fc & FC_RESPONSE_BIT) == FC_RESPONSE_BIT, "Response FC must be >= 128");
        
        // Send request
        Result result = sendMsgInternal(req);
        if (result != SUCCESS) {
            return Error(result.status, REQ::fc);
        }
        
        // Wait for response with timeout
        uint8_t frame[MAX_FRAME_SIZE];
        size_t frameLen;
        
        unsigned long startTime = millis();
        while(millis() - startTime < responseTimeoutMs) {
            result = captureFrame(RESP::fc, frame, &frameLen);
            if(result == NOTHING_TO_DO) {
                continue;
            }
            if(result != SUCCESS) {
                return result;
            }
            
            // Frame capturée, on la copie dans la réponse
            memcpy(&resp, &frame[3], sizeof(RESP));
            return Success(RESP::fc);
        }
        
        frameCapturePending = false; // Reset frame capture - protects against late responses
        return Error(ERR_TIMEOUT);
    }

    // Process received messages
    Result poll() {
        // Tente de capturer une frame
        Result result = captureFrame();
        
        // Si la capture n'a pas réussi, on retourne l'erreur obtenue
        if(result != SUCCESS && result != NOTHING_TO_DO) {
            return result;
        }
        
        // Si la capture a réussi, vérifier que ce n'est pas une réponse
        if(result == SUCCESS) {
            // Rejeter les réponses (bit 7 setté)
            if((result.fc & FC_RESPONSE_BIT) != 0) {
                return Error(ERR_INVALID_FC, result.fc);
            }

            // Appeler le handler si enregistré
            ProtoStore* proto = findProto(result.fc);
            if(proto && proto->callback) {
                proto->callback(&rxBuffer[3]);
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
    uint32_t responseTimeoutMs;   // Timeout for waiting for a response
    ProtoStore protos[MAX_PROTOS];  // Indexed by FC
    uint8_t rxBuffer[MAX_FRAME_SIZE];
    bool frameCapturePending = false;  // État de la capture de frame


    // Helpers to create results
    static constexpr Result Error(Status status, uint8_t fc = NULL_FC) { return Result{status, fc}; }
    static constexpr Result Success(uint8_t fc) { return Result{SUCCESS, fc}; }
    static constexpr Result NoData() { return Result{NOTHING_TO_DO, NULL_FC}; }

    // Register message prototype
    template<typename T>
    Result registerProtoInternal(uint8_t fc) {
        // Proto validation is already done in registerRequest() and registerResponse()
        
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

    // Send message
    template<typename T>
    Result sendMsgInternal(const T& msg) {

        // Check if we are already capturing a frame
        if(frameCapturePending) {
            return Error(ERR_BUSY_RECEIVING, T::fc);
        }
        
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

    // Flush RX buffer until SOF is found
    void flushRxBufferUntilSOF() {
        while(serial->available()) {
            if(serial->peek() == START_OF_FRAME) {
                return; // SOF trouvé mais pas consommé
            }
            serial->read(); // Consomme uniquement les octets non-SOF
        }
    }

    // Nouvelle fonction de capture de frame
    Result captureFrame(uint8_t expectedFc = 0, uint8_t* outFrame = nullptr, size_t* outLen = nullptr) {
        // Si pas de SOF, chercher le prochain
        if(!frameCapturePending) {
            if(!serial->available()) {
                return NoData();
            }
            
            uint8_t byte = serial->read();
            if(byte != START_OF_FRAME) {
                return Error(ERR_INVALID_SOF);
            }
            frameCapturePending = true;
        }

        // À ce stade on a capturé un SOF valide
        if(serial->available() < 1) {
            return NoData(); // Pas assez de données pour LEN
        }

        // Vérifier LEN sans le consommer
        uint8_t frameSize = serial->peek();
        if(frameSize < FRAME_OVERHEAD) {
            frameCapturePending = false;
            flushRxBufferUntilSOF(); // Frame invalide, on vide le buffer
            return Error(ERR_INVALID_LEN);
        }

        // Attendre d'avoir assez de données
        if(serial->available() < frameSize - 1) { // -1 car SOF déjà consommé
            return NoData();
        }

        // Capturer la frame complète
        uint8_t tempFrame[MAX_FRAME_SIZE];
        tempFrame[0] = START_OF_FRAME;
        tempFrame[1] = serial->read(); // LEN

        for(size_t i = 2; i < frameSize; i++) {
            if(serial->peek() == START_OF_FRAME && i < frameSize-1) {
                frameCapturePending = false;
                flushRxBufferUntilSOF(); // Frame tronquée, on vide le buffer
                return Error(ERR_INVALID_LEN);
            }
            tempFrame[i] = serial->read();
        }

        // Valider FC
        uint8_t fc = tempFrame[2];
        if(expectedFc > 0 && fc != expectedFc) {
            frameCapturePending = false;
            flushRxBufferUntilSOF(); // Frame invalide, on vide le buffer
            return Error(ERR_INVALID_FC, fc);
        }

        // Valider taille proto si FC attendu
        if(expectedFc > 0) {
            ProtoStore* proto = findProto(fc);
            if(!proto || proto->size != frameSize - FRAME_OVERHEAD) {
                frameCapturePending = false;
                flushRxBufferUntilSOF(); // Frame invalide, on vide le buffer
                return Error(ERR_PROTO_MISMATCH, fc);
            }
        }

        // Valider CRC
        uint8_t receivedCRC = tempFrame[frameSize-1];
        uint8_t calculatedCRC = calculateCRC(tempFrame, frameSize-1);
        if(receivedCRC != calculatedCRC) {
            frameCapturePending = false;
            flushRxBufferUntilSOF(); // Frame invalide, on vide le buffer
            return Error(ERR_CRC, fc);
        }

        // Copier frame si demandé
        if(outFrame && outLen) {
            memcpy(outFrame, tempFrame, frameSize);
            *outLen = frameSize;
        }

        frameCapturePending = false;
        return Success(fc);
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