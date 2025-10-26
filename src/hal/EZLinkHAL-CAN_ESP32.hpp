/* @file EZLinkHAL-CAN_ESP32.hpp 
 *  
 * Requires the ESP32-TWAI-CAN library:
 * https://github.com/handmade0octopus/ESP32-TWAI-CAN.git 
 * (or available in the /external folder)
 * 
 */

#pragma once

#include "../EZLink.hpp"
#include <ESP32-TWAI-CAN.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdarg>  // For va_list

#ifdef EZLINK_DEBUG
    #define EZL_LOG_CAN(...) printLog("[EZLinkHAL-CAN] " __VA_ARGS__)
#else
    #define EZL_LOG_CAN(...) do{}while(0)
#endif

namespace ezlink {
namespace hal {

/* @brief CAN HAL implementation for ESP32
 * @note Implements the EZLink HAL interface for ESP32-TWAI-CAN
 */
class CAN_ESP32 {
public:
    // Internal CAN HAL constants
    static constexpr size_t MAX_DATA_SIZE = 8; // CAN 2.0 classic limit
    static constexpr size_t MSGID_SIZE = 1; // MsgId is 1 byte
    static constexpr size_t DATA_OFFSET = MSGID_SIZE; // Data starts here in payload (after msgId)

    // Mandatory EZLink HAL API constants
    static constexpr size_t MAX_PAYLOAD_SIZE = MAX_DATA_SIZE + MSGID_SIZE; // We store msgId in the CAN ID

    CAN_ESP32(uint16_t baseCanId = 0x100)
        : baseCanId(baseCanId)
    {
        responseSemaphore = xSemaphoreCreateBinaryStatic(&responseSemaphoreBuffer);
    }

    // Set useHwFilter=true to enable hardware filter (requires baseCanId aligned to 0x100)
    // Hardware filter only works if baseCanId is aligned on 256 (e.g., 0x000, 0x100, 0x200, etc.)
    // Default is false for safety - software filtering in recvFrame() always works
    bool begin(TwaiSpeed speed, int8_t txPin, int8_t rxPin, uint16_t txQueueSize = 0xFFFF, uint16_t rxQueueSize = 0xFFFF, bool useHwFilter = false) {
        if (useHwFilter) {
            // Hardware filter: accept only CAN IDs in range [baseCanId, baseCanId + 255]
            // LIMITATION: baseCanId must be aligned on 256 (lower 8 bits = 0)
            // TWAI filter semantics (IDF 4.x): mask bit=1 means "don't care", bit=0 means "must match"

            if ((baseCanId & 0xFF) != 0) {
                // baseCanId not aligned - cannot represent exact range with single mask
                // Reject initialization to avoid unexpected behavior
                #ifdef EZLINK_DEBUG
                    printLog("[EZLinkHAL-CAN] ERROR: Hardware filter requires baseCanId aligned to 0x100. Got 0x%03X", baseCanId);
                    printLog("[EZLinkHAL-CAN] Valid aligned values: 0x%03X..0x%03X", baseCanId & ~0xFF, (baseCanId & ~0xFF) + 0xFF);
                #endif
                return false;
            }

            // Standard 11-bit CAN ID is placed in bits [28:18] of 32-bit register (SJA1000-like layout)
            const uint32_t base_shifted = (uint32_t)(baseCanId & 0x7FF) << 21;

            // We want to ignore lower 8 bits of ID → bits [28:21] in register
            // Mask bit = 1 means "ignore this bit"
            const uint32_t MASK_IGNORE_8LSB = 0xFFu << 21;

            twai_filter_config_t filter = {
                .acceptance_code = base_shifted & ~MASK_IGNORE_8LSB,  // Base ID with lower 8 bits zeroed
                .acceptance_mask = MASK_IGNORE_8LSB,                  // Ignore lower 8 bits (set to 1)
                .single_filter = true
            };
            return ESP32Can.begin(speed, txPin, rxPin, txQueueSize, rxQueueSize, &filter, nullptr, nullptr);
        } else {
            // No hardware filter (accept all, software filtering in recvFrame)
            return ESP32Can.begin(speed, txPin, rxPin, txQueueSize, rxQueueSize);
        }
    }

    bool end() {
        return ESP32Can.end();
    }

    int sendFrame(const uint8_t* payload, size_t len, uint32_t timeoutMs) {
        if (!payload || len < DATA_OFFSET || len > MAX_PAYLOAD_SIZE) {
            return -1; // Invalid parameters
        }

        uint8_t msgId = payload[0];
        const uint8_t* data = payload + DATA_OFFSET;
        size_t dataLen = len - DATA_OFFSET;

        CanFrame frame;
        frame.identifier = baseCanId + msgId;  // Convert EZLink msgId to CAN ID
        frame.data_length_code = dataLen;
        frame.flags = TWAI_MSG_FLAG_NONE;
        frame.rtr = 0;
        memcpy(frame.data, data, dataLen);

        bool success = ESP32Can.writeFrame(frame, timeoutMs);
        return success ? len : -1;
    }

    int recvFrame(uint8_t* payload, size_t maxLen, uint32_t timeoutMs) {
        if (!payload || maxLen < DATA_OFFSET) {
            return -1; // Invalid parameters
        }

        CanFrame frame;

        if (!ESP32Can.readFrame(frame, timeoutMs)) {
            return 0; // Nothing available
        }

        // Extract EZLink message ID from CAN ID
        if (frame.identifier < baseCanId || frame.identifier >= baseCanId + 256) {
            return 0; // Not our message - HAL filters it out, act as timeout
        }

        const uint8_t msgId = frame.identifier - baseCanId;
        uint8_t* data = payload + DATA_OFFSET;
        const size_t maxDataLen = maxLen - DATA_OFFSET; // Reserve 1 byte for msgId at the beginning

        // Copy msgId & data into payload buffer
        payload[0] = msgId;
        const size_t dataLen = frame.data_length_code < maxDataLen ? frame.data_length_code : maxDataLen;
        if (dataLen > 0 && data) {
            memcpy(data, frame.data, dataLen);
        }
        const size_t copyLen = MSGID_SIZE + dataLen; // We copied msgId + data

        return copyLen;
    }

    uint32_t getTimestampMs() const {
        return millis();
    }

    void yield() {
        vTaskDelay(1);  // Minimum delay to yield in FreeRTOS
    }

    // Perfect forwarding template for zero-copy logging
    template<typename... Args>
    void printLog(const char* format, Args&&... args) {
        // Forward fmt/args to printf for ESP32
        printf(format, std::forward<Args>(args)...);
        printf("\n");  // Add newline
    }

    bool takeSemaphore(uint32_t timeoutMs) {
        return xSemaphoreTake(responseSemaphore, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
    }

    void giveSemaphore() {
        xSemaphoreGive(responseSemaphore);
    }

    // DIAGNOSTICS - CAN bus health monitoring

    bool isBusOff() const {
        return ESP32Can.canState() == TWAI_STATE_BUS_OFF;
    }

    bool hasErrors() const {
        return ESP32Can.txErrorCounter() > 0 || ESP32Can.rxErrorCounter() > 0;
    }

    bool hasWarning() const {
        return ESP32Can.txErrorCounter() >= 96 || ESP32Can.rxErrorCounter() >= 96;
    }

    bool recover() {
        return ESP32Can.recover();
    }

    struct BusStatus {
        uint32_t state;
        uint32_t txErrors;
        uint32_t rxErrors;
        uint32_t txFailed;
        uint32_t rxMissed;
        uint32_t busErrors;
    };

    BusStatus getStatus() const {
        return {
            .state = ESP32Can.canState(),
            .txErrors = ESP32Can.txErrorCounter(),
            .rxErrors = ESP32Can.rxErrorCounter(),
            .txFailed = ESP32Can.txFailedCounter(),
            .rxMissed = ESP32Can.rxMissedCounter(),
            .busErrors = ESP32Can.busErrCounter()
        };
    }
    

private:
    uint16_t baseCanId;
    SemaphoreHandle_t responseSemaphore;  // Binary semaphore for RTOS response signaling
    StaticSemaphore_t responseSemaphoreBuffer;  // Static storage for semaphore
};

} // namespace hal
} // namespace ezlink