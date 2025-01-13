#include <Arduino.h>
#include "EZLink.h"
#include "EZLink_Proto.h"

// Communication pins
#define UART1_RX D5
#define UART1_TX D6
#define UART2_RX D7
#define UART2_TX D8

// Serial ports
#define UART1 Serial1
#define UART2 Serial2
#define USB Serial

// Delays
#define MASTER_DELAY_MS 500  // 500ms between each test

// Log queue
QueueHandle_t logQueue;
#define LOG_QUEUE_SIZE 32
#define MAX_LOG_MSG_SIZE 256

// Log message structure
struct LogMessage {
    char buffer[MAX_LOG_MSG_SIZE];
};

// Log task polling log queue & sending to USB
void logTask(void* parameter) {
    LogMessage msg;
    while(true) {
        if(xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
            USB.print(msg.buffer);
            USB.flush();
        }
    }
}

// Log helpers sending to log queue
void log(const char* message) {
    LogMessage msg;
    snprintf(msg.buffer, sizeof(msg.buffer), "%s\n", message);
    msg.buffer[MAX_LOG_MSG_SIZE-1] = '\0';
    xQueueSend(logQueue, &msg, portMAX_DELAY);
}
void logf(const char* format, ...) {
    LogMessage msg;
    va_list args;
    va_start(args, format);
    vsnprintf(msg.buffer, sizeof(msg.buffer)-2, format, args);
    strcat(msg.buffer, "\n");
    va_end(args);
    xQueueSend(logQueue, &msg, portMAX_DELAY);
}

// Custom stream that uses our thread-safe log functions to catch hexdumps from EZLink
class ThreadSafeLogStream : public Stream {
private:
    static constexpr size_t BUFFER_SIZE = 256;
    char buffer[BUFFER_SIZE];
    size_t bufferIndex = 0;

    // Flush the buffer
    void flushBuffer() {
        if (bufferIndex > 0) {
            buffer[bufferIndex] = '\0';
            log(buffer);
            bufferIndex = 0;
        }
    }

public:
    // Write single byte - required by Stream
    size_t write(uint8_t c) override {
        if (bufferIndex >= BUFFER_SIZE - 1) {
            flushBuffer();
        }
        buffer[bufferIndex++] = c;
        if (c == '\n') {
            buffer[bufferIndex-1] = '\0';
            flushBuffer();
        }
        return 1;
    }

    // Write buffer - optimized for writing all at once
    size_t write(const uint8_t *data, size_t size) override {
        // If the new content doesn't fit, flush first
        if (bufferIndex + size >= BUFFER_SIZE - 1) {
            flushBuffer();
        }
        
        // If the size is too large for our buffer, send directly
        if (size >= BUFFER_SIZE - 1) {
            char temp[BUFFER_SIZE];
            size_t len = min(size, BUFFER_SIZE-1);
            memcpy(temp, data, len);
            temp[len] = '\0';
            log(temp);
            return len;
        }

        // Otherwise, accumulate in the buffer
        memcpy(buffer + bufferIndex, data, size);
        bufferIndex += size;
        
        // If we find a \n, flush
        for (size_t i = 0; i < size; i++) {
            if (data[i] == '\n') {
                buffer[bufferIndex-1] = '\0';
                flushBuffer();
                break;
            }
        }
        return size;
    }

    // Print formatted - uses logf directly
    size_t printf(const char *format, ...) {
        va_list args;
        va_start(args, format);
        char temp[BUFFER_SIZE];
        size_t len = vsnprintf(temp, sizeof(temp), format, args);
        va_end(args);
        log(temp);
        return len;
    }

    // Required by Stream but not used for logging
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override { flushBuffer(); }
};

// Create a global instance
ThreadSafeLogStream threadSafeLog;

// Communication instances
#ifdef EZLINK_DEBUG
EZLink master(&UART1, &threadSafeLog, "DBG_MASTER");
EZLink slave(&UART2, &threadSafeLog, "DBG_SLAVE");
#else
EZLink master(&UART1);
EZLink slave(&UART2);
#endif

// Test state
enum TestState {
    TEST_FIRE_AND_FORGET,
    TEST_ACK_REQUIRED,
    TEST_REQUEST_RESPONSE,
    TEST_DONE
};

volatile TestState currentTest = TEST_FIRE_AND_FORGET;
volatile bool testInProgress = false;

// Separate stats for master and slave
struct Stats {
    // Message counters
    uint32_t messagesSent = 0;     // Master only
    uint32_t messagesReceived = 0;  // Slave only
    uint32_t acksSent = 0;         // Master only  
    uint32_t acksReceived = 0;     // Slave only
    uint32_t requestsSent = 0;     // Master only
    uint32_t requestsReceived = 0;  // Slave only
    
    // Error counters
    struct {
        uint32_t rxErrors = 0;    // RX errors
        uint32_t txErrors = 0;    // TX errors
    } masterErrors;
    
    struct {
        uint32_t rxErrors = 0;  // RX errors
        uint32_t txErrors = 0;  // TX errors
    } slaveErrors;
    
    void print() {
        logf("\nStats Master:\n"
             "Messages sent: %lu\n"
             "ACKs sent: %lu\n"
             "Requests sent: %lu\n"
             "Errors: TX=%lu, RX=%lu", 
             messagesSent,
             acksSent,
             requestsSent,
             masterErrors.txErrors,
             masterErrors.rxErrors);
            
        logf("\nStats Slave:\n"
             "Messages received: %lu\n"
             "ACKs received: %lu\n"
             "Requests received: %lu\n"
             "Errors: RX=%lu, TX=%lu",
             messagesReceived,
             acksReceived,
             requestsReceived,
             slaveErrors.rxErrors,
             slaveErrors.txErrors);
    }
} txRxStats;

// Master task that executes tests sequentially
void masterTask(void* parameter) {
    // Wait for slave to be fully started
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms initial delay
    
    while(true) {
        switch(currentTest) {
            case TEST_FIRE_AND_FORGET: {
                log("\n=== Test FIRE_AND_FORGET ===");
                SetLedMsg msg{.state = 1};
                logf("MASTER: Attempting to send LED message, state=%d", msg.state);
                
                testInProgress = true;
                auto result = master.sendMsg(msg);
                if(result != EZLink::SUCCESS) {
                    logf("MASTER: Error sending LED message, code=%d", result.status);
                    txRxStats.masterErrors.txErrors++;
                } else {
                    txRxStats.messagesSent++;
                }
                
                // Wait before next test
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                currentTest = TEST_ACK_REQUIRED;
                break;
            }
            
            case TEST_ACK_REQUIRED: {
                log("\n=== Test ACK_REQUIRED ===");
                SetPwmMsg msg{.pin = 1, .freq = 1000};
                logf("MASTER: Attempting to send PWM message, pin=%d, freq=%lu", msg.pin, msg.freq);
                
                testInProgress = true;
                unsigned long startTime = millis();  // Capture time before sending
                auto result = master.sendMsgAck(msg);
                unsigned long responseTime = millis() - startTime;  // Calculate response time
                
                int errorCode = (int)result.status;
                if(result == EZLink::ERR_RCV_TIMEOUT) {
                    log("MASTER: Timeout waiting for ACK");
                    txRxStats.masterErrors.rxErrors++;
                }
                else if (errorCode > EZLink::ERR_SND_MIN && errorCode < EZLink::ERR_SND_MAX) {
                    logf("MASTER: TX error, code=%d", errorCode);
                    txRxStats.masterErrors.txErrors++;
                }
                else if (errorCode > EZLink::ERR_RCV_MIN && errorCode < EZLink::ERR_RCV_MAX) {
                    logf("MASTER: RX error, code=%d", errorCode);
                    txRxStats.masterErrors.rxErrors++;
                }
                else if(result != EZLink::SUCCESS) {
                    logf("MASTER: TX/RX error, code=%d", errorCode);
                    txRxStats.masterErrors.txErrors++;
                } else {
                    logf("MASTER: ACK received for PWM, pin=%d, freq=%lu (response in %lu ms)", 
                         msg.pin, msg.freq, responseTime);
                    txRxStats.acksSent++;
                }
                
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                currentTest = TEST_REQUEST_RESPONSE;
                break;
            }
            
            case TEST_REQUEST_RESPONSE: {
                log("\n=== Test REQUEST/RESPONSE ===");
                GetStatusMsg req{};
                StatusResponseMsg resp{};
                
                log("MASTER: Attempting to send status request");
                
                testInProgress = true;
                unsigned long startTime = millis();  // Capture time before sending
                auto result = master.sendRequest(req, resp);
                unsigned long responseTime = millis() - startTime;  // Calculate response time
                
                int errorCode = (int)result.status;
                if(result == EZLink::ERR_RCV_TIMEOUT) {
                    log("MASTER: Timeout waiting for response");
                    txRxStats.masterErrors.rxErrors++;
                }
                else if (errorCode > EZLink::ERR_SND_MIN && errorCode < EZLink::ERR_SND_MAX) {
                    logf("MASTER: TX error, code=%d", errorCode);
                    txRxStats.masterErrors.txErrors++;
                }
                else if (errorCode > EZLink::ERR_RCV_MIN && errorCode < EZLink::ERR_RCV_MAX) {
                    logf("MASTER: RX error, code=%d", errorCode);
                    txRxStats.masterErrors.rxErrors++;
                }
                else if(result != EZLink::SUCCESS) {
                    logf("MASTER: TX/RX error, code=%d", errorCode);
                    txRxStats.masterErrors.txErrors++;
                } else {
                    txRxStats.requestsSent++;
                    logf("MASTER: Response received: state=%d, uptime=%lu (response in %lu ms)", 
                        resp.state, resp.uptime, responseTime);
                }
                
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                currentTest = TEST_FIRE_AND_FORGET;
                
                // Display txRxStats after a full cycle
                txRxStats.print();
                break;
            }
            
            default:
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                break;
        }
    }
}

// Slave task that polls continuously
void slaveTask(void* parameter) {
    while(true) {
        auto result = slave.poll();
        if(result == EZLink::SUCCESS) {
            // At this stage, the message is already processed and the response already sent!
            vTaskDelay(1);  // This delay does not impact the response time
        }
        else if(result != EZLink::NOTHING_TO_DO) {
            int errorCode = (int)result.status;
            if (errorCode > EZLink::ERR_SND_MIN && errorCode < EZLink::ERR_SND_MAX) {
                logf("SLAVE: TX error, code=%d", errorCode);
                txRxStats.slaveErrors.txErrors++;
            }
            else if (errorCode > EZLink::ERR_RCV_MIN && errorCode < EZLink::ERR_RCV_MAX) {
                logf("SLAVE: RX error, code=%d", errorCode);
                txRxStats.slaveErrors.rxErrors++;
            }
            else {
                logf("SLAVE: TX/RX error, code=%d", errorCode);
                txRxStats.slaveErrors.rxErrors++; // Default to RX error
            }
        }
        taskYIELD(); // Poll as frequently as possible
    }
}

void setup() {
    // Blink 5 times to indicate that the program is running
    pinMode(LED_BUILTIN, OUTPUT);
    for(int i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(500);
    }
    digitalWrite(LED_BUILTIN, LOW);

    // Create log queue
    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));
    if(logQueue == NULL) {
        while(1) {
            USB.println("Error creating log queue");
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(100);
        }
    }

    // Debug port
    USB.begin(115200);
    log("\nStarting sequential test...");
    
    // Communication ports
    UART1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
    UART2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);

    // Let the UARTs stabilize
    delay(10);
    
    // Initialize communication instances
    master.begin();
    slave.begin();
    
    // Register protos
    master.registerRequest<SetLedMsg>();
    master.registerRequest<SetPwmMsg>();
    master.registerRequest<GetStatusMsg>();
    master.registerResponse<StatusResponseMsg>();
    
    slave.registerRequest<SetLedMsg>();
    slave.registerRequest<SetPwmMsg>();
    slave.registerRequest<GetStatusMsg>();
    slave.registerResponse<StatusResponseMsg>();
    
    // Setup handlers
    slave.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
        logf("SLAVE: Received LED message, state=%d", msg.state);
        txRxStats.messagesReceived++;
    });
    
    slave.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        logf("SLAVE: Received PWM message, pin=%d, freq=%lu", msg.pin, msg.freq);
        txRxStats.acksReceived++;
    });
    
    slave.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        logf("SLAVE: Received status request");
        txRxStats.requestsReceived++;
        resp.state = 1;
        resp.uptime = millis();
    });
    
    // Create tasks - they will start automatically after setup()
    xTaskCreatePinnedToCore(
        masterTask,
        "masterTask",
        10000,
        NULL,
        1,
        NULL,
        0  // Core 0
    );
    
    xTaskCreatePinnedToCore(
        slaveTask,
        "slaveTask",
        10000,
        NULL,
        1,
        NULL,
        1  // Core 1
    );
    
    // Create log task
    xTaskCreatePinnedToCore(
        logTask,
        "logTask",
        10000,
        NULL,
        1,
        NULL,
        0  // Core 0 with masterTask
    );
    
    log("Configuration completed, starting tests...\n");
}

void loop() {
    // Main loop does nothing, everything happens in tasks
    vTaskDelete(NULL);
} 