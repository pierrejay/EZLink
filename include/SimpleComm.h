#pragma once
#include <Arduino.h>
#include <functional>
#include <type_traits>  // Pour is_standard_layout

class SimpleComm {
public:
    // Protocol constants
    static constexpr uint8_t START_OF_FRAME = 0xAA;
    static constexpr uint8_t FRAME_HEADER_SIZE = 2;  // SOF + LEN
    static constexpr size_t FRAME_OVERHEAD = 4;      // SOF + LEN + FC + CRC
    static constexpr size_t MAX_FRAME_SIZE = 32;
    static constexpr uint32_t DEFAULT_TIMEOUT_MS = 100;
    static constexpr uint8_t NULL_FC = 0;  // FC=0 réservé/invalide

    enum Result {
        SUCCESS = 0,
        NOTHING_TO_DO,
        ERR_INVALID_FC,     // FC non enregistré
        ERR_INVALID_SOF,    // SOF invalide
        ERR_INVALID_LEN,    // Taille non valide
        ERR_FC_MISMATCH,    // Incohérence entre taille reçue et taille attendue par le FC
        ERR_TIMEOUT,        // Pas de réponse
        ERR_CRC,           // CRC invalide
        ERR_OVERFLOW       // Buffer trop petit
    };

    // Résultat complet d'une opération de réception
    struct CommResult {
        Result status;
        uint8_t fc;  // FC du message traité (si status == SUCCESS)

        // Surcharge des opérateurs pour comparaison directe avec Status
        bool operator==(Result s) const { return status == s; }
        bool operator!=(Result s) const { return status != s; }


    };

       // Helpers pour créer des résultats
        static constexpr CommResult Error(Result status, uint8_t fc = NULL_FC) {
            return CommResult{status, fc};
        }
        
        static constexpr CommResult Success(uint8_t fc) {
            return CommResult{SUCCESS, fc};
        }
        
        static constexpr CommResult NoData() {
            return CommResult{NOTHING_TO_DO, NULL_FC};
        }

private:
    // Structure de stockage des prototypes de messages
    struct ProtoStore {
        size_t size = 0;
        std::function<void(const void*)> callback = nullptr;
    };

    // Attributs
    HardwareSerial* serial;
    uint32_t timeoutMs;
    ProtoStore protos[256];  // Indexé par FC
    uint8_t rxBuffer[MAX_FRAME_SIZE];

private:
   // Reception methods
   Result waitByte(uint8_t& byte) {
       uint32_t startTime = millis();
       
       while(!serial->available()) {
           if(millis() - startTime > timeoutMs) {
               return ERR_TIMEOUT;
           }
       }
       
       byte = serial->read();
       return SUCCESS;
   }

   Result waitBytes(uint8_t* buffer, size_t size) {
       uint32_t startTime = millis();
       size_t bytesRead = 0;
       
       while(bytesRead < size) {
           if(millis() - startTime > timeoutMs) {
               return ERR_TIMEOUT;
           }
           
           if(serial->available()) {
               buffer[bytesRead] = serial->read();
               bytesRead++;
               startTime = millis(); // Reset timeout on each byte received
           }
       }
       
       return SUCCESS;
   }

   void flushRxBuffer() {
       uint32_t startTime = millis();
       
       // Continue reading until no more data or timeout
       while(millis() - startTime < timeoutMs) {
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

public:
   // Constructor
   explicit SimpleComm(HardwareSerial* serial, uint32_t timeoutMs = DEFAULT_TIMEOUT_MS) 
       : serial(serial)
       , timeoutMs(timeoutMs)
   {
       for(uint16_t i = 0; i < 256; i++) {
           protos[i].size = 0;
           protos[i].callback = nullptr;
       }
   }

   // Register message prototype
   template<typename T>
   Result registerProto() {
       static_assert(T::fc != NULL_FC, "FC=0 is reserved/invalid");
       static_assert(std::is_standard_layout_v<T>, "Message type must be POD/standard-layout");
       static_assert(T::fc < 256, "FC must be < 256");
       // Validate message size
       if(sizeof(T) + FRAME_OVERHEAD > MAX_FRAME_SIZE) {
           return ERR_OVERFLOW;
       }
       protos[T::fc].size = sizeof(T);
       return SUCCESS;
   }

   // Register message handler
   template<typename T>
   Result onMessage(std::function<void(const T&)> handler) {
       static_assert(std::is_standard_layout_v<T>, "Message type must be POD/standard-layout");
       static_assert(T::fc < 256, "FC must be < 256");
       // Verify that proto is registered
       if(protos[T::fc].size == 0) {
           return ERR_INVALID_FC;
       }
       
       // Wrap typed handler into void* handler
       protos[T::fc].callback = [handler](const void* data) {
           handler(*static_cast<const T*>(data));
       };
       
       return SUCCESS;
   }

   // Send message
   template<typename T>
   Result sendMsg(const T& msg) {
       static_assert(std::is_standard_layout_v<T>, "Message type must be POD/standard-layout");
       static_assert(T::fc < 256, "FC must be < 256");
       // Check if proto is registered
       if(protos[T::fc].size == 0 || protos[T::fc].size != sizeof(T)) {
           return ERR_INVALID_FC;
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

       return SUCCESS;
   }

    // Process received messages
    CommResult processRx() {
        uint8_t byte;
        
        // Check if there's data available
        if(!serial->available()) {
            return NoData();
        }
        
        // 1. Wait for START_OF_FRAME
        if(waitByte(byte) != SUCCESS) {
            return Error(ERR_TIMEOUT);
        }
        if(byte != START_OF_FRAME) {
            flushRxBuffer();
            return Error(ERR_INVALID_SOF);
        }
        
        // 2. Get length
        if(waitByte(byte) != SUCCESS) {
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
        if(waitByte(byte) != SUCCESS) {
            flushRxBuffer();
            return Error(ERR_TIMEOUT);
        }
        uint8_t fc = byte;
        
        // Validate FC and expected message size
        if(protos[fc].size == 0) {
            flushRxBuffer();
            return Error(ERR_INVALID_FC);
        }
        if (protos[fc].size != frameLen - FRAME_OVERHEAD) {
            flushRxBuffer();
            return Error(ERR_FC_MISMATCH, fc);
        }
        
        // 4. Get data + CRC
        size_t remainingBytes = frameLen - FRAME_HEADER_SIZE - 1;
        if(waitBytes(rxBuffer + FRAME_HEADER_SIZE + 1, remainingBytes) != SUCCESS) {
            flushRxBuffer();
            return Error(ERR_TIMEOUT);
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
};