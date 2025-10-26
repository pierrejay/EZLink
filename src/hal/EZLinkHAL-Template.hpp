/* @file EZLinkHAL-Template.hpp */

#pragma once

#include "../EZLink.hpp"

namespace ezlink {
namespace hal {

// Example HAL boilerplate - replace with your transport layer implementation
class IHAL {
public:
    /* @brief Maximum payload size for your protocol & HAL
     * @note Does not include your own framing overhead, only EZLink's Message ID (1B) + data bytes!
     * @note Must be strictly > 1 (to accomodate for Message ID + 1 data byte)
    */
    static constexpr size_t MAX_PAYLOAD_SIZE = 256;

    /* @brief Constructor */
    IHAL() = default;

    /* @brief Send frame
     * @param payload Payload to send (Message ID + data bytes)
     * @param len Length of payload to send
     * @param timeoutMs Timeout in milliseconds
     * @return Bytes sent, <=0 on timeout/error
     * @note EZLink does not handle retries: the whole data must be sent in one call 
     *       (manage chunking inside sendFrame() if necessary)
     * @note You are free to define your timeout policy: strict abort when reached, 
     *       or complete the current operation if TX has already been initiated
     */
    int sendFrame(const uint8_t* payload, size_t len, uint32_t timeoutMs) {
        return -1; // Placeholder
    }

    /* @brief Receive frame
     * @param payload Payload to receive
     * @param maxLen Maximum length of data to receive
     * @param timeoutMs Timeout in milliseconds
     * @return Bytes received, 0 on no data, <0 on error
     * @note This function must be NON-BLOCKING if called with timeoutMs = 0
     * @note EZLink does not handle chunked messages: the whole data must be written in one call. 
     *       Your implementation must handle this if required.
     */
    int recvFrame(uint8_t* payload, size_t maxLen, uint32_t timeoutMs) {
        return -1; // Placeholder
    }

    /* @brief Get current timestamp in milliseconds
     * @return current timestamp in milliseconds
     */
    uint32_t getTimestampMs() const {
        return 0; // Placeholder
    }

    /* @brief Give time to scheduler (OPTIONAL)
     * @note In single thread mode, avoids blocking the program while waiting for a response.
     *       Useful in an RTOS environment (e.g. ESP32) if you need to service a watchdog task, network 
     *       task, etc. but still keep all EZLink operations (send + poll) in a single loop.
     */
    void yield() {
        return; // Placeholder
    }

    /* @brief Give & take semaphore (OPTIONAL)
     * @note Allows EZLink to synchronize response reception in an RTOS environment when poll() is
     *       called from a separate loop/task/thread as sendRequest().
     * @note Implement these methods ONLY if you are supporting RTOS multi-threading (sendFrame() must be thread-safe)
     * @note You don't need to manage the semaphore yourself, EZLink takes care of that internally.
     *       Just create it and give EZLink access to it through these wrappers.
     * @note Implementation requirements:
     *       - Must use a binary semaphore (not counting)
     *       - takeSemaphore(uint32_t timeoutMs) blocks for up to timeoutMs and return true if taken, false on timeout
     *       - giveSemaphore() signals the semaphore
     * @warning If not supporting RTOS: Simply DON'T implement these methods (remove them entirely).
     */
    // bool takeSemaphore(uint32_t timeoutMs);  // Uncomment & implement for RTOS support
    // void giveSemaphore();                    // Uncomment & implement for RTOS support

    /* @brief Debug logging (optional)
     * @note Implement this to enable debug output from EZLink when EZLINK_DEBUG flag is defined
     * @note Logs are sent without CRLF at the end, you must implement newline yourself if needed
     */
    void printLog(const char* format, ...) {
        // Placeholder - implement your logging here
        // Example: route to printf, Serial.printf, or custom logger
    }
};

} // namespace hal
} // namespace ezlink