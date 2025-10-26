/* @file EZLinkHAL-UART_Arduino.hpp */

#pragma once

#include <Arduino.h>
#include <Stream.h>
#include <cstdint>
#include <cstring>
#include <cstdarg>

#ifdef EZLINK_RTOS_POLL_TASK
    #include <freertos/FreeRTOS.h>
    #include <freertos/semphr.h>
#endif

#ifndef EZLINK_UART_MAX_FRAME_SIZE
    #define EZLINK_UART_MAX_FRAME_SIZE 32 // Total UART frame size limit
#endif
#ifndef EZLINK_UART_MAX_CHUNK_SIZE
    #define EZLINK_UART_MAX_CHUNK_SIZE 64 // Max TX chunk size (must be <= TX buffer size)
#endif

// Logging macros
#ifdef EZLINK_DEBUG
    #define EZL_LOG_UART(...) printLog("[EZLinkHAL-UART] " __VA_ARGS__)
    #define EZL_LOG_UART_HEX(prefix, buf, len) hexdump("[EZLinkHAL-UART] " prefix, buf, len)
#else
    #define EZL_LOG_UART(...) do{}while(0)
    #define EZL_LOG_UART_HEX(prefix, buf, len) do{}while(0)
#endif

namespace ezlink {
namespace hal {

/* @brief Circular buffer for UART communication
 * @note Used to store data received from UART and
 *       identify valid or invalid frames.
 */
template<typename T, size_t SIZE>
class ScrollBuffer {

public:
    ScrollBuffer() : head(0), tail(0), count(0) {}

    // Flush the whole buffer
    void clear() {
        count = 0;
        head = tail = 0;
    }

    // Dump n bytes from the buffer
    void dump(size_t n) {
        if (n > count) {
            n = count;
        }
        tail = (tail + n) % SIZE;
        count -= n;
    }

    // Dump until an element is found (exclusive)
    bool dumpUntil(const T* element) {
        if (!element) return false;
        
        // Check if the element is inside our buffer
        if (element < &buffer[0] || element >= &buffer[SIZE]) {
            return false;
        }
        
        size_t offset = element - &buffer[0];
        size_t distance = (offset - tail + SIZE) % SIZE;
        
        if (distance >= count) {
            return false;  // Element outside valid zone
        }
        
        dump(distance);
        return true;
    }

    // Slide to the next occurrence of marker, clear the whole buffer if not found
    void scrollTo(const T& marker) {
        // If marker found, slide to it
        if (!dumpUntil(find(marker))) {
            // If no marker found, clear the buffer
            clear();
        }
    }

    // Push a byte into the buffer
    void push(T byte) {
        if (count < SIZE) {
            buffer[head] = byte;
            head = (head + 1) % SIZE;
            count++;
        }
    }

    // Pop a byte from the buffer
    // Never used in our implementation, because we want to prevent against
    // truncated frames: potentially valid chunks cannot be re-processed 
    // once they are consumed.
    // We always want to peek() into the buffer, scrollTo() if we get an
    // invalid chunk, and dump() once process is done or if we have garbage.
    T pop() {
        if (count > 0) {
            T byte = buffer[tail];
            tail = (tail + 1) % SIZE;
            count--;
            return byte;
        }
        return 0;
    }

    // Peek a byte from the buffer
    T peek(size_t offset = 0) const {
        if (offset < count) {
            return buffer[(tail + offset) % SIZE];
        }
        return 0;
    }

    // Check if the buffer is empty
    bool isEmpty() const {
        return count == 0;
    }

    // Find the position of the next occurrence of a byte in the buffer,
    // and return a pointer to it (or nullptr if not found).
    T* find(const T& next) {
        for (size_t i = 0; i < count; i++) {
            if (peek(i) == next) {
                return &buffer[(tail + i) % SIZE];
            }
        }
        return nullptr;
    }

    // Find the position of the first occurrence of a byte in the buffer,
    // and return its index (or SIZE if not found).
    size_t indexOf(const T* element) const {
        if (!element) return SIZE; // valid index cannot be SIZE since index is 0-based
        return (element - &buffer[0]) % SIZE;
    }

    // Return the number of bytes in the buffer
    size_t size() const {
        return count;
    }

private:
    T buffer[SIZE];
    size_t head;
    size_t tail;
    size_t count;
}; 

/* @brief UART HAL implementation for Arduino
 * @note Implements the EZLink HAL interface for Arduino
 */
class UART {
public:
    // Configuration
    static constexpr size_t MAX_FRAME_SIZE = EZLINK_UART_MAX_FRAME_SIZE;
    static constexpr size_t MAX_CHUNK_SIZE = EZLINK_UART_MAX_CHUNK_SIZE;

    // Frame format: SOF + LEN + ID + DATA + CRC16
    static constexpr uint8_t START_OF_FRAME = 0xAA;

    // LEN field size depends on MAX_FRAME_SIZE
    static constexpr size_t LEN_FIELD_SIZE =
        (MAX_FRAME_SIZE <= 255) ? 1 :
        (MAX_FRAME_SIZE <= 65535) ? 2 :
        4;
    static constexpr size_t FRAME_OVERHEAD = 1 + LEN_FIELD_SIZE + 2;  // SOF + LEN + CRC(2)
    static constexpr size_t MAX_PAYLOAD_SIZE = MAX_FRAME_SIZE - FRAME_OVERHEAD;  // Max payload (msgId + data)
    static constexpr size_t PAYLOAD_OFFSET = 1 + LEN_FIELD_SIZE;  // Payload starts here in frame

    static_assert(MAX_FRAME_SIZE >= FRAME_OVERHEAD, 
        "MAX_FRAME_SIZE must be greater than FRAME_OVERHEAD");

    UART(Stream* linkStream, Stream* logStream = nullptr)
    : _linkStream(linkStream)
    , _logStream(logStream) {
        _rxBuffer.clear();
        #ifdef EZLINK_RTOS_POLL_TASK
            _responseSemaphore = xSemaphoreCreateBinaryStatic(&_responseSemaphoreBuffer);
            _txMutex = xSemaphoreCreateMutexStatic(&_txMutexBuffer);
            // Static allocation should never fail, but defensive check
            configASSERT(_responseSemaphore);
            configASSERT(_txMutex);
        #endif
    }

    UART(HardwareSerial& linkSerial, Stream* logStream = nullptr)
    : _linkStream(&linkSerial)
    , _logStream(logStream) {
        _rxBuffer.clear();
        #ifdef EZLINK_RTOS_POLL_TASK
            _responseSemaphore = xSemaphoreCreateBinaryStatic(&_responseSemaphoreBuffer);
            _txMutex = xSemaphoreCreateMutexStatic(&_txMutexBuffer);
            // Static allocation should never fail, but defensive check
            configASSERT(_responseSemaphore);
            configASSERT(_txMutex);
        #endif
    }

    void begin() {
        // Clear any pending data
        while (_linkStream->available()) {
            _linkStream->read();
        }
        _rxBuffer.clear();
    }

    // Send frame with payload (msgId + data)
    int sendFrame(const uint8_t* payload, size_t len, uint32_t timeoutMs) {
        if (!payload || len == 0 || len > MAX_PAYLOAD_SIZE) {
            return -1; // Invalid parameters
        }

        #ifdef EZLINK_RTOS_POLL_TASK
            // Protect TX against concurrent access from multiple threads
            if (_txMutex && xSemaphoreTake(_txMutex, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) {
                EZL_LOG_UART("TX ERROR: Failed to acquire mutex (timeout=%ums)", timeoutMs);
                return 0; // Failed to acquire mutex
            }
        #endif

        size_t frameSize = len + FRAME_OVERHEAD;

        // Build frame
        uint8_t frame[MAX_FRAME_SIZE];
        frame[0] = START_OF_FRAME;

        // Write variable-length frame size
        writeFrameLength(frame, frameSize);

        // Copy entire payload (msgId + data) at correct offset
        memcpy(&frame[PAYLOAD_OFFSET], payload, len);

        // Add CRC at end
        uint16_t crc = calculateCRC16(frame, frameSize - 2);
        frame[frameSize - 2] = (crc >> 8);
        frame[frameSize - 1] = (crc & 0xFF);

        // Send with chunking (handles all frame sizes uniformly)
        size_t bytesSent = sendFrameChunked(frame, frameSize, timeoutMs);

        #ifdef EZLINK_RTOS_POLL_TASK
            // Release mutex
            if (_txMutex) {
                xSemaphoreGive(_txMutex);
            }
        #endif

        // All or nothing: only return success if complete frame sent
        if (bytesSent == frameSize) {
            EZL_LOG_UART_HEX("TX FRAME: ", frame, frameSize);
            return len;  // Complete frame sent = success
        } else {
            return -1;   // Partial frame = error (will be rejected by receiver)
        }
    }

    // Receive frame with payload (msgId + data)
    int recvFrame(uint8_t* payload, size_t maxLen, uint32_t timeoutMs) {
        if (!payload || maxLen == 0) {
            return -1; // Invalid parameters
        }

        uint32_t start = millis();

        do {
            // Read available data from serial
            processRx();

            // Try to find a valid frame
            uint8_t frame[MAX_FRAME_SIZE];
            size_t frameLen = 0;

            if (findNextFrame(frame, frameLen)) {
                // Extract entire payload (msgId + data)
                size_t payloadLen = frameLen - FRAME_OVERHEAD;
                if (payloadLen > maxLen) {
                    return -1;  // Buffer too small
                }

                // Copy entire payload from frame
                memcpy(payload, &frame[PAYLOAD_OFFSET], payloadLen);

                return payloadLen;
            }

            // For non-blocking mode, return immediately
            if (timeoutMs == 0) {
                return 0;
            }

            yield();

        } while (millis() - start < timeoutMs);

        return 0;  // Timeout
    }

    uint32_t getTimestampMs() const {
        return millis();
    }

    void yield() {
        ::yield();
    }

    bool takeSemaphore(uint32_t timeoutMs) {
        #ifdef EZLINK_RTOS_POLL_TASK
            if (_responseSemaphore) {
                return xSemaphoreTake(_responseSemaphore, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
            }
        #endif
        return false;  // Single-thread mode fallback
    }

    void giveSemaphore() {
        #ifdef EZLINK_RTOS_POLL_TASK
            if (_responseSemaphore) {
                xSemaphoreGive(_responseSemaphore);
            }
        #endif
    }

    // Perfect forwarding template for zero-copy logging
    template<typename... Args>
    void printLog(const char* format, Args&&... args) {
        if (_logStream) {
            _logStream->printf(format, std::forward<Args>(args)...);
            _logStream->println();  // Add newline
        }
    }

    // Hexdump utility for debugging (builds complete string before logging)
    void hexdump(const char* prefix, const uint8_t* data, size_t len) {
        if (_logStream) {
            char buf[512];  // Static buffer for hexdump
            int offset = snprintf(buf, sizeof(buf), "%s", prefix);

            for (size_t i = 0; i < len && offset < sizeof(buf) - 4; i++) {
                if (i > 0 && i % 16 == 0) {
                    offset += snprintf(buf + offset, sizeof(buf) - offset, "\n%*s", (int)strlen(prefix), "");
                }
                offset += snprintf(buf + offset, sizeof(buf) - offset, "%02X ", data[i]);
            }

            _logStream->println(buf);  // Single println at the end
        }
    }

    // CRC16 calculation (public for testing)
    static uint16_t calculateCRC16(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;

        for (size_t i = 0; i < len; i++) {
            crc ^= ((uint16_t)data[i]) << 8;
            for (int j = 0; j < 8; j++) {
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
    Stream* _linkStream;
    Stream* _logStream{nullptr};
    ScrollBuffer<uint8_t, MAX_FRAME_SIZE> _rxBuffer;

    #ifdef EZLINK_RTOS_POLL_TASK
        StaticSemaphore_t _responseSemaphoreBuffer;
        SemaphoreHandle_t _responseSemaphore{nullptr};
        StaticSemaphore_t _txMutexBuffer;
        SemaphoreHandle_t _txMutex{nullptr};
    #endif

    // Write frame length to buffer (1, 2, or 4 bytes)
    inline void writeFrameLength(uint8_t* buffer, size_t frameSize) {
        if constexpr (LEN_FIELD_SIZE == 1) {
            buffer[1] = static_cast<uint8_t>(frameSize);
        } else if constexpr (LEN_FIELD_SIZE == 2) {
            buffer[1] = static_cast<uint8_t>(frameSize >> 8);    // MSB
            buffer[2] = static_cast<uint8_t>(frameSize & 0xFF);  // LSB
        } else { // LEN_FIELD_SIZE == 4
            buffer[1] = static_cast<uint8_t>(frameSize >> 24);   // MSB
            buffer[2] = static_cast<uint8_t>(frameSize >> 16);
            buffer[3] = static_cast<uint8_t>(frameSize >> 8);
            buffer[4] = static_cast<uint8_t>(frameSize & 0xFF);  // LSB
        }
    }

    // Read frame length from buffer (1, 2, or 4 bytes)
    inline size_t readFrameLength() {
        if constexpr (LEN_FIELD_SIZE == 1) {
            return _rxBuffer.peek(1);
        } else if constexpr (LEN_FIELD_SIZE == 2) {
            return (static_cast<size_t>(_rxBuffer.peek(1)) << 8) | _rxBuffer.peek(2);
        } else { // LEN_FIELD_SIZE == 4
            return (static_cast<size_t>(_rxBuffer.peek(1)) << 24) |
                   (static_cast<size_t>(_rxBuffer.peek(2)) << 16) |
                   (static_cast<size_t>(_rxBuffer.peek(3)) << 8) |
                   _rxBuffer.peek(4);
        }
    }


    // Send frame in chunks (respects global timeout)
    // Returns number of bytes actually sent (0 to frameSize)
    size_t sendFrameChunked(const uint8_t* frame, size_t frameSize, uint32_t timeoutMs) {
        uint32_t start = millis();
        size_t sent = 0;

        while (sent < frameSize) {
            // Check global timeout before each chunk
            if (millis() - start >= timeoutMs) {
                break;  // Global timeout - return what we managed to send
            }

            // Calculate chunk size
            size_t remaining = frameSize - sent;
            size_t chunkSize = (remaining < MAX_CHUNK_SIZE) ? remaining : MAX_CHUNK_SIZE;

            // Wait for TX buffer space (with remaining timeout)
            uint32_t remainingTimeout = timeoutMs - (millis() - start);
            uint32_t chunkStart = millis();

            while (_linkStream->availableForWrite() < chunkSize) {
                if (millis() - chunkStart >= remainingTimeout) {
                    return sent;  // Timeout waiting for buffer space
                }
                yield();
            }

            // Send chunk
            size_t written = _linkStream->write(&frame[sent], chunkSize);
            if (written == 0) {
                break;  // Send error - return what we managed to send
            }

            sent += written;

            // If partial write, we're done (buffer full or error)
            if (written < chunkSize) {
                break;
            }
        }

        // Only flush if we sent the complete frame
        if (sent == frameSize) {
            _linkStream->flush();
        }

        return sent;
    }

    // Read available bytes from serial
    void processRx() {
        while (_linkStream->available() && _rxBuffer.size() < MAX_FRAME_SIZE) {
            _rxBuffer.push(_linkStream->read());
        }
    }

    // Find and extract next valid frame from buffer
    bool findNextFrame(uint8_t* outFrame, size_t& outLen) {
        // Not enough data for minimum frame
        if (_rxBuffer.size() == 0) {
            return false;
        }

        // Look for SOF - if not found, slide to next SOF or clear
        if (_rxBuffer.peek(0) != START_OF_FRAME) {
            _rxBuffer.scrollTo(START_OF_FRAME);
            return false;
        }

        // Need at least SOF + LEN field to proceed
        if (_rxBuffer.size() < (1 + LEN_FIELD_SIZE)) {
            return false;
        }

        // Get frame length using helper
        size_t frameLen = readFrameLength();

        // Validate frame length
        if (frameLen < FRAME_OVERHEAD || frameLen > MAX_FRAME_SIZE) {
            #ifdef EZLINK_DEBUG
                uint8_t debugBuf[MAX_FRAME_SIZE];
                size_t dumpLen = _rxBuffer.size();
                for (size_t i = 0; i < dumpLen; i++) {
                    debugBuf[i] = _rxBuffer.peek(i);
                }
                char prefix[128];
                snprintf(prefix, sizeof(prefix), "[EZLinkHAL-UART] RX ERROR: Invalid length=%u (min=%u, max=%u), buf_size=%u: ",
                         frameLen, FRAME_OVERHEAD, MAX_FRAME_SIZE, _rxBuffer.size());
                hexdump(prefix, debugBuf, dumpLen);
            #endif
            // Invalid length - discard SOF and try again
            _rxBuffer.dump(1);
            _rxBuffer.scrollTo(START_OF_FRAME);
            return false;
        }

        // Wait for complete frame
        if (_rxBuffer.size() < frameLen) {
            return false;  // Need more data
        }

        // Copy frame for validation
        uint8_t tempFrame[MAX_FRAME_SIZE];
        for (size_t i = 0; i < frameLen; i++) {
            tempFrame[i] = _rxBuffer.peek(i);
        }

        // Validate CRC
        uint16_t calcCrc = calculateCRC16(tempFrame, frameLen - 2);
        uint16_t rxCrc = (tempFrame[frameLen - 2] << 8) | tempFrame[frameLen - 1];

        if (calcCrc != rxCrc) {
            #ifdef EZLINK_DEBUG
                char prefix[128];
                snprintf(prefix, sizeof(prefix), "[EZLinkHAL-UART] RX ERROR: CRC mismatch, expected=0x%04X, got=0x%04X: ", calcCrc, rxCrc);
                hexdump(prefix, tempFrame, frameLen);
            #endif
            // CRC error - discard SOF and try again
            _rxBuffer.dump(1);
            _rxBuffer.scrollTo(START_OF_FRAME);
            return false;
        }

        // Valid frame found!
        if (outFrame) {
            memcpy(outFrame, tempFrame, frameLen);
            outLen = frameLen;
        }

        EZL_LOG_UART_HEX("RX FRAME: ", tempFrame, frameLen);

        // Remove frame from buffer
        _rxBuffer.dump(frameLen);
        return true;
    }
};

} // namespace hal
} // namespace ezlink