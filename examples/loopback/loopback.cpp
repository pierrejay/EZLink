#include <Arduino.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

// Communication pins
#define UART1_RX D4
#define UART1_TX D5
#define UART2_RX D8
#define UART2_TX D9

// Serial ports
#define USB Serial
#define UART1 Serial1
#define UART2 Serial2

// Task configuration
#define MASTER_DELAY_US 1000  // 1ms between messages
#define SLAVE_DELAY_US  100   // 100us between polls
#define STATS_PERIOD_MS 1000  // Print stats every second

// Extended stats with timing
struct Stats {
    // Message counters
    uint32_t messagesReceived = 0;
    uint32_t ackReceived = 0;
    uint32_t requestsReceived = 0;
    uint32_t errors = 0;
    
    // Performance metrics
    uint32_t messagesSent = 0;
    uint32_t messagesPerSecond = 0;
    uint32_t maxLatencyUs = 0;
    uint32_t minLatencyUs = UINT32_MAX;
    uint32_t totalLatencyUs = 0;  // For average calculation
    
    void updateRate() {
        static uint32_t lastUpdate = 0;
        static uint32_t lastCount = 0;
        
        if (millis() - lastUpdate >= STATS_PERIOD_MS) {
            messagesPerSecond = ((messagesSent - lastCount) * 1000) / STATS_PERIOD_MS;
            lastCount = messagesSent;
            lastUpdate = millis();
            
            // Print stats
            USB.printf("\nStats after %lu seconds:\n", millis()/1000);
            USB.printf("Messages: Sent=%lu/s, Received=%lu, Acks=%lu, Reqs=%lu, Errs=%lu\n",
                messagesPerSecond, messagesReceived, ackReceived, requestsReceived, errors);
            USB.printf("Latency: Min=%lu us, Max=%lu us, Avg=%lu us\n",
                minLatencyUs, maxLatencyUs, totalLatencyUs/messagesSent);
            
            // Reset some metrics
            maxLatencyUs = 0;
            minLatencyUs = UINT32_MAX;
            totalLatencyUs = 0;
        }
    }
} stats;

// Communication instances
SimpleComm master(&UART1);
SimpleComm slave(&UART2);

// Task configuration (in ticks, assuming configTICK_RATE_HZ = 1000)
#define MASTER_DELAY_TICKS (MASTER_DELAY_US/1000)  // Convert µs to ms
#define SLAVE_DELAY_TICKS  (SLAVE_DELAY_US/1000)   // Convert µs to ms

// Master task running on core 0
void masterTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(MASTER_DELAY_TICKS);
    
    while(true) {
        uint32_t startTime = micros();
        
        // 1. FIRE_AND_FORGET
        SetLedMsg ledMsg{.state = (uint8_t)(stats.messagesSent & 1)};
        auto result = master.sendMsg(ledMsg);
        if(result != SimpleComm::SUCCESS) {
            stats.errors++;
        }
        stats.messagesSent++;
        
        // 2. ACK_REQUIRED (every 10th message)
        if((stats.messagesSent % 10) == 0) {
            SetPwmMsg pwmMsg{.pin = 1, .freq = 1000};
            result = master.sendMsgAck(pwmMsg);
            if(result != SimpleComm::SUCCESS) {
                stats.errors++;
            }
        }
        
        // 3. REQUEST/RESPONSE (every 100th message)
        if((stats.messagesSent % 100) == 0) {
            GetStatusMsg req{};
            StatusResponseMsg resp{};
            result = master.sendRequest(req, resp);
            if(result != SimpleComm::SUCCESS) {
                stats.errors++;
            }
        }
        
        // Update timing stats
        uint32_t latency = micros() - startTime;
        stats.maxLatencyUs = max(stats.maxLatencyUs, latency);
        stats.minLatencyUs = min(stats.minLatencyUs, latency);
        stats.totalLatencyUs += latency;
        
        // Precise timing using vTaskDelayUntil
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Slave task running on core 1
void slaveTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SLAVE_DELAY_TICKS);
    
    while(true) {
        auto result = slave.poll();
        if(result != SimpleComm::SUCCESS && result != SimpleComm::NOTHING_TO_DO) {
            stats.errors++;
        }
        
        stats.updateRate();
        
        // Precise timing using vTaskDelayUntil
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    // Debug port
    USB.begin(115200);
    USB.println("\nStarting stress test...");
    USB.printf("Configuration: Master=%uus, Slave=%uus\n", 
        MASTER_DELAY_US, SLAVE_DELAY_US);
    
    // Communication ports
    UART1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
    UART2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);
    
    // Register protos
    master.registerProto<SetLedMsg>();
    master.registerProto<SetPwmMsg>();
    master.registerProto<GetStatusMsg>();
    master.registerProto<StatusResponseMsg>();
    
    slave.registerProto<SetLedMsg>();
    slave.registerProto<SetPwmMsg>();
    slave.registerProto<GetStatusMsg>();
    slave.registerProto<StatusResponseMsg>();
    
    // Setup handlers
    slave.onReceive<SetLedMsg>([](const SetLedMsg& msg) {
        stats.messagesReceived++;
    });
    
    slave.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        stats.ackReceived++;
    });
    
    slave.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        stats.requestsReceived++;
        resp.state = 1;
        resp.uptime = millis();
    });
    
    // Create tasks
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
    
    USB.println("Setup complete, starting stress test...");
}

void loop() {
    // Main loop does nothing, everything happens in tasks
    vTaskDelete(NULL);
} 