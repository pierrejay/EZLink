#pragma once
#include <Arduino.h>

class SimpleComm {
public:
    // Proto constants
    static constexpr uint8_t START_OF_FRAME = 0xAA;
    static constexpr uint8_t FRAME_HEADER_SIZE = 2;  // SOF + LEN
    static constexpr size_t FRAME_OVERHEAD = 4;      // SOF + LEN + FC + CRC
    static constexpr size_t MAX_FRAME_SIZE = 32;
    static constexpr uint32_t DEFAULT_TIMEOUT_MS = 100;

    enum Result {
        SUCCESS = 0,
        ERR_INVALID_FC,     // FC non enregistré
        ERR_INVALID_LEN,    // Taille incohérente
        ERR_TIMEOUT,        // Pas de réponse
        ERR_CRC,           // CRC invalide
        ERR_OVERFLOW       // Buffer trop petit
    };

    // Types for callbacks
    template<typename T>
    using MessageHandler = void(*)(const T& msg);

    // Constructor
    explicit SimpleComm(HardwareSerial* serial, uint32_t timeoutMs = DEFAULT_TIMEOUT_MS);

    // Proto registration
    template<typename T>
    Result registerProto() {
        return registerProto(T::fc, sizeof(T));
    }

    // Message handler registration
    template<typename T>
    Result onMessage(MessageHandler<T> handler) {
        return onMessage(T::fc, reinterpret_cast<void*>(handler));
    }

    // Send message
    template<typename T>
    Result sendMsg(const T& msg) {
        return sendMsg(T::fc, &msg, sizeof(T));
    }

    // Receive and process messages
    void processRx();

private:
    struct Proto {
        size_t size = 0;
        void* handler = nullptr;
    };

    // Internal methods
    Result registerProto(uint8_t fc, size_t size);
    Result onMessage(uint8_t fc, void* handler);
    Result sendMsg(uint8_t fc, const void* data, size_t size);
    
    // Reception methods
    Result waitByte(uint8_t& byte);
    Result waitBytes(uint8_t* buffer, size_t size);
    void flushRxBuffer();
    uint8_t calculateCrc(const uint8_t* data, size_t size);

    // Attributes
    HardwareSerial* serial;
    uint32_t timeoutMs;
    Proto protos[256];  // Indexed by FC
    uint8_t rxBuffer[MAX_FRAME_SIZE];
};

