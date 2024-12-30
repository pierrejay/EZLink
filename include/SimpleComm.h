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
    static constexpr uint8_t NULL_FC = 0;  // FC=0 réservé/invalide
    static constexpr uint8_t MAX_PROTOS = 10;  // Nombre maximum de protos
    
    // Timeouts par défaut
    static constexpr uint32_t DEFAULT_INTERBYTE_TIMEOUT_MS = 10;   // 10ms entre bytes
    static constexpr uint32_t DEFAULT_RESPONSE_TIMEOUT_MS = 500;   // 500ms pour une réponse
    
    // Types de protocoles
    enum class ProtoType {
        FIRE_AND_FORGET,  // Pas de réponse attendue
        ACK_REQUIRED,     // Echo attendu
        REQUEST,          // Attend une réponse spécifique
        RESPONSE          // Est une réponse à une requête
    };

    // Résultat d'une opération
    enum Status {
        SUCCESS = 0,
        // Résultat de registerProto
        ERR_FC_ALREADY_REGISTERED, // FC déjà enregistré
        // Résultat de poll
        NOTHING_TO_DO,
        // Résultats de poll & sendMsg
        ERR_INVALID_FC,     // FC non enregistré
        ERR_INVALID_SOF,    // SOF invalide
        ERR_INVALID_LEN,    // Taille non valide
        ERR_FC_MISMATCH,    // Incohérence entre taille reçue et taille attendue par le FC
        ERR_TIMEOUT,        // Pas de réponse
        ERR_CRC,           // CRC invalide
        ERR_OVERFLOW,       // Buffer trop petit
    };

    // Résultat complet d'une opération de réception
    struct Result {
        Status status;
        uint8_t fc;  // FC du message traité (si status == SUCCESS)

        // Surcharge des opérateurs pour comparaison directe avec Status
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
    {
        for(uint16_t i = 0; i < MAX_PROTOS; i++) {
            protos[i].size = 0;
            protos[i].callback = nullptr;
        }
    }


    // Register message prototype
    template<typename T>
    Result registerProto() {
        static_assert(T::fc != NULL_FC, "FC=0 is reserved/invalid");
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(T::fc < 256, "FC must be < 256");
        static_assert(sizeof(T) + FRAME_OVERHEAD <= MAX_FRAME_SIZE, "Message too large");
        
        if(protos[T::fc].size != 0) {
            return Error(ERR_FC_ALREADY_REGISTERED, T::fc);
        }
        
        protos[T::fc].type = T::type;  // Stocke le type
        protos[T::fc].size = sizeof(T);
        
        return Success(T::fc);
    }

    // Handler pour messages FIRE_AND_FORGET et ACK_REQUIRED
    template<typename T>
    Result onReceive(std::function<void(const T&)> handler) {
        static_assert(T::type == ProtoType::FIRE_AND_FORGET || 
                     T::type == ProtoType::ACK_REQUIRED,
                     "onReceive only works with FIRE_AND_FORGET or ACK_REQUIRED messages");
        return onMessage<T>(handler);
    }

    // Handler pour messages REQUEST avec réponse automatique
    template<typename REQ>
    Result onRequest(std::function<void(const REQ&, typename REQ::ResponseType&)> handler) {
        static_assert(REQ::type == ProtoType::REQUEST, 
                     "onRequest only works with REQUEST messages");
        static_assert(REQ::ResponseType::type == ProtoType::RESPONSE, 
                     "Response type must be RESPONSE");
        
        return onMessage<REQ>([this, handler](const REQ& req) {
            typename REQ::ResponseType resp;
            handler(req, resp);
            sendMsg(resp);  // Envoi automatique de la réponse
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
            return Error(ERR_FC_MISMATCH, T::fc);
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
        
        // 1. Wait for START_OF_FRAME
        if(!waitByte(byte)) {
            return Error(ERR_TIMEOUT);
        }
        if(byte != START_OF_FRAME) {
            flushRxBuffer();
            return Error(ERR_INVALID_SOF);
        }
        
        // 2. Get length
        if(!waitByte(byte)) {
            flushRxBuffer();
            return Error(ERR_TIMEOUT);
        }
        uint8_t frameLen = byte;
        
        // Validate frame length
        if(frameLen < FRAME_OVERHEAD || frameLen > MAX_FRAME_SIZE) {
            flushRxBuffer();
            return Error(ERR_INVALID_LEN);
        }
        
        // 3. Get FC
        if(!waitByte(byte)) {
            flushRxBuffer();
            return Error(ERR_TIMEOUT);
        }
        uint8_t fc = byte;
        
        // Validate FC and expected message size
        if(protos[fc].size == 0) {
            flushRxBuffer();
            return Error(ERR_INVALID_FC, fc);
        }
        if (protos[fc].size != frameLen - FRAME_OVERHEAD) {
            flushRxBuffer();
            return Error(ERR_FC_MISMATCH, fc);
        }
        
        // 4. Get data + CRC
        size_t remainingBytes = frameLen - FRAME_HEADER_SIZE - 1;
        if(!waitBytes(rxBuffer + FRAME_HEADER_SIZE + 1, remainingBytes)) {
            flushRxBuffer();
            return Error(ERR_TIMEOUT, fc);
        }
        
        // 5. Reconstruct full frame for CRC check
        rxBuffer[0] = START_OF_FRAME;
        rxBuffer[1] = frameLen;
        rxBuffer[2] = fc;
        
        // 6. Validate CRC
        uint8_t receivedCrc = rxBuffer[frameLen-1];
        uint8_t calculatedCrc = calculateCrc(rxBuffer, frameLen-1);
        if(receivedCrc != calculatedCrc) {
            flushRxBuffer();
            return Error(ERR_CRC, fc);
        }
        
        // 7. Call handler if registered
        if(protos[fc].callback) {
            protos[fc].callback(&rxBuffer[3]);
            return Success(fc);
        }
        
        return Success(fc);  // Message valide même sans handler
    }

    // Setters pour les timeouts
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


private:

    // Structure de stockage des prototypes de messages
    struct ProtoStore {
        ProtoType type = ProtoType::FIRE_AND_FORGET;
        size_t size = 0;
        std::function<void(const void*)> callback = nullptr;
    };

    // Attributs
    HardwareSerial* serial;
    uint32_t interbyteTimeoutMs;  // Timeout entre bytes d'une même frame
    uint32_t responseTimeoutMs;   // Timeout pour attendre une réponse
    ProtoStore protos[MAX_PROTOS];  // Indexé par FC
    uint8_t rxBuffer[MAX_FRAME_SIZE];


    // Helpers pour créer des résultats
    static constexpr Result Error(Status status, uint8_t fc = NULL_FC) { return Result{status, fc}; }
    static constexpr Result Success(uint8_t fc) { return Result{SUCCESS, fc}; }
    static constexpr Result NoData() { return Result{NOTHING_TO_DO, NULL_FC}; }

// Send message
    template<typename T>
    Result sendMsgInternal(const T& msg) {
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(T::fc < 256, "FC must be < 256");
        // Check if proto is registered
        if(protos[T::fc].size == 0 || protos[T::fc].size != sizeof(T)) {
            return Error(ERR_INVALID_FC);
        }

        // Prepare frame
        uint8_t frame[MAX_FRAME_SIZE];
        size_t frameSize = sizeof(T) + FRAME_OVERHEAD;

        frame[0] = START_OF_FRAME;
        frame[1] = frameSize;
        frame[2] = T::fc;
        memcpy(&frame[3], &msg, sizeof(T));
        frame[frameSize-1] = calculateCrc(frame, frameSize-1);

        // Send frame
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

   uint8_t calculateCrc(const uint8_t* data, size_t size) {
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

    // Helper for receiving specific message type
    template<typename T>
    Result receiveMsg(T& msg, uint8_t expectedFc, bool checkFc = false) {
        uint32_t startTime = millis();
        
        while (millis() - startTime < responseTimeoutMs) {  // Timeout long
            Result result = poll();  // Utilise timeout court en interne
            
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

    // Handler générique (usage interne uniquement)
    template<typename T>
    Result onMessage(std::function<void(const T&)> handler) {
        static_assert(std::is_standard_layout<T>::value, "Message type must be POD/standard-layout");
        static_assert(T::fc < 256, "FC must be < 256");
        
        if(protos[T::fc].size == 0) {
            return Error(ERR_INVALID_FC);
        }
        
        protos[T::fc].callback = [handler](const void* data) {
            handler(*static_cast<const T*>(data));
        };
        
        return Success(T::fc);
    }
};