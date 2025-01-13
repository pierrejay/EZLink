#include <Arduino.h>
#include <unity.h>
#include <vector>
#include "EZLink.h"
#include "EZLink_Proto.h"

// Communication pins
#define UART1_RX D5
#define UART1_TX D6
#define UART2_RX D7
#define UART2_TX D8

// Polling delay for slave
#define SLAVE_DELAY_MS 10

#ifdef EZLINK_DEBUG
EZLink master(&Serial1, &Serial, "DBG_MASTER");
EZLink slave(&Serial2, &Serial, "DBG_SLAVE");
#else
EZLink master(&Serial1);
EZLink slave(&Serial2);
#endif

TaskHandle_t slaveTask = NULL;

// Message with maximum payload
struct MaxPayloadMsg {
    static constexpr MsgType type = MsgType::MESSAGE;
    static constexpr uint8_t id = 10;
    uint8_t data[EZLinkDfs::MAX_FRAME_SIZE - EZLinkDfs::FRAME_OVERHEAD];  // Maximum possible
} __attribute__((packed));

// Message with empty payload
struct EmptyMsg {
    static constexpr MsgType type = MsgType::MESSAGE;
    static constexpr uint8_t id = 11;
    // No data !
} __attribute__((packed));

// Message for burst test
struct BurstMsg {
    static constexpr MsgType type = MsgType::MESSAGE_ACK;  // Use ACK to verify reception
    static constexpr uint8_t id = 12;
    uint32_t sequence;  // Sequence number to verify order
} __attribute__((packed));

// Messages for bidirectional test
struct PongMsg {
    static constexpr MsgType type = MsgType::RESPONSE;
    static constexpr uint8_t id = 13;
    uint32_t echo_timestamp;
    uint32_t response_timestamp;
} __attribute__((packed));

struct PingMsg {
    static constexpr MsgType type = MsgType::REQUEST;
    static constexpr uint8_t id = 13;
    uint32_t timestamp;
    using ResponseType = PongMsg;
} __attribute__((packed));

// Utility function to send a custom frame
void sendCustomFrame(HardwareSerial* serial, uint8_t sof, uint8_t len, uint8_t id, const std::vector<uint8_t>& payload, uint16_t crc = 0) {
    // Build frame without CRC
    std::vector<uint8_t> frame;
    frame.push_back(sof);
    frame.push_back(len);
    frame.push_back(id);
    frame.insert(frame.end(), payload.begin(), payload.end());

    // If crc == 0, calculate it on the full frame
    if(crc == 0) {
        crc = EZLink::calculateCRC16(frame.data(), frame.size());
    }
    
    // Add CRC
    frame.push_back(static_cast<uint8_t>(crc >> 8));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));

    serial->write(frame.data(), frame.size());
    serial->flush();
}

void testsInit() {
    // Debug port
    Serial.begin(115200);
    
    // Setup hardware
    Serial1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
    Serial2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);
    delay(10);  // Let UARTs stabilize

    // Setup communication
    master.begin();
    slave.begin();

    // Register basic messages
    master.registerRequest<SetLedMsg>();
    master.registerRequest<SetPwmMsg>();
    master.registerRequest<GetStatusMsg>();
    master.registerResponse<StatusResponseMsg>();
    slave.registerRequest<SetLedMsg>();
    slave.registerRequest<SetPwmMsg>();
    slave.registerRequest<GetStatusMsg>();
    slave.registerResponse<StatusResponseMsg>();
}

// Slave task that polls continuously (to be started on purpose if required)
void startSlaveTask() {
    xTaskCreatePinnedToCore(
        [](void* parameter) {
            Serial.println("Slave task started");
            while(true) {
                auto result = slave.poll();
                if(result == EZLink::SUCCESS) {
                    Serial.println("Slave: message processed");
                }
                else if(result != EZLink::NOTHING_TO_DO) {
                    Serial.printf("Slave: poll error %d\n", result.status);
                }
                vTaskDelay(pdMS_TO_TICKS(SLAVE_DELAY_MS));
            }
        },
        "slaveTask",
        10000,
        NULL,
        1,
        &slaveTask,
        1
    );
}

void setUp(void) {
    // Reset all RX buffers before each test
    master.begin();
    slave.begin();
}

void tearDown(void) {
    // Stop slave task after each testif running
    if (slaveTask != NULL) {
        vTaskDelete(slaveTask);
        slaveTask = NULL;
    }
}

void test_basic_communication() {
    bool messageReceived = false;
    
    // Setup handler
    slave.onReceive<SetLedMsg>([&messageReceived](const SetLedMsg& msg) {
        messageReceived = true;
        TEST_ASSERT_EQUAL(1, msg.state);
    });
    
    // Send message
    SetLedMsg msg{.state = 1};
    auto result = master.sendMsg(msg);
    
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    // Give some time for reception
    delay(50);
    
    // Process message
    result = slave.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    TEST_ASSERT_TRUE(messageReceived);
}

void test_ack_required() {
    
    bool messageReceived = false;
    
    // Setup handler
    slave.onReceive<SetPwmMsg>([&messageReceived](const SetPwmMsg& msg) {
        messageReceived = true;
        TEST_ASSERT_EQUAL(1000, msg.freq);
    });
    
    // Send message with ACK
    SetPwmMsg msg{.pin = 1, .freq = 1000};
    auto result = master.sendMsgAck(msg);
    
    // Give some time for reception
    delay(50);
    
    // Process message - IMPORTANT !
    result = slave.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    TEST_ASSERT_TRUE(messageReceived);
}

void test_request_response() {
    startSlaveTask(); // Start slave task
    
    slave.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        Serial.println("Handler GetStatus appelé");
        resp.state = 1;
        resp.uptime = 1000;
    });

    // Wait for slave task to be well started and stable
    delay(50);

    GetStatusMsg req;
    StatusResponseMsg resp;
    auto result = master.sendRequest(req, resp);
    
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    TEST_ASSERT_EQUAL(1, resp.state);
    TEST_ASSERT_EQUAL(1000, resp.uptime);

}

// Robustness tests
void test_max_payload() {
    Serial.println("Début test_max_payload");
    
    // Register messages for this test
    master.registerRequest<MaxPayloadMsg>();
    slave.registerRequest<MaxPayloadMsg>();
    
    startSlaveTask();  // Start slave task
    
    bool messageReceived = false;
    MaxPayloadMsg msg;
    
    // Fill payload with a recognizable pattern
    for(size_t i = 0; i < sizeof(msg.data); i++) {
        msg.data[i] = i & 0xFF;
    }
    
    // Setup handler
    slave.onReceive<MaxPayloadMsg>([&messageReceived, &msg](const MaxPayloadMsg& received) {
        messageReceived = true;
        // Verify pattern is intact
        for(size_t i = 0; i < sizeof(msg.data); i++) {
            TEST_ASSERT_EQUAL(i & 0xFF, received.data[i]);
        }
    });
    
    auto result = master.sendMsg(msg);
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    delay(50);  // Give some time for slave to process
    TEST_ASSERT_TRUE(messageReceived);
}

void test_empty_payload() {
    Serial.println("Début test_empty_payload");
    
    // Register messages for this test
    master.registerRequest<EmptyMsg>();
    slave.registerRequest<EmptyMsg>();
    
    startSlaveTask();
    
    bool messageReceived = false;
    EmptyMsg msg;
    
    slave.onReceive<EmptyMsg>([&messageReceived](const EmptyMsg& received) {
        messageReceived = true;
    });
    
    auto result = master.sendMsg(msg);
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    delay(50);
    TEST_ASSERT_TRUE(messageReceived);
}

void test_burst_messages() {
    Serial.println("Début test_burst_messages");
    
    // Register messages for this test
    master.registerRequest<BurstMsg>();
    slave.registerRequest<BurstMsg>();
    
    startSlaveTask();
    
    const int NUM_MESSAGES = 10;
    int receivedCount = 0;
    
    slave.onReceive<BurstMsg>([&receivedCount](const BurstMsg& msg) {
        TEST_ASSERT_EQUAL(receivedCount, msg.sequence);
        receivedCount++;
    });
    
    // Send messages in burst
    for(int i = 0; i < NUM_MESSAGES; i++) {
        BurstMsg msg;
        msg.sequence = i;
        auto result = master.sendMsgAck(msg);  // Use ACK to ensure reception
        TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    }
    
    delay(100);  // Give some time for processing
    TEST_ASSERT_EQUAL(NUM_MESSAGES, receivedCount);
}

void test_bidirectional() {
    Serial.println("Début test_bidirectional");
    
    // Register messages for this test - attention to order !
    master.registerRequest<PingMsg>();
    master.registerResponse<PongMsg>();
    slave.registerRequest<PingMsg>();
    slave.registerResponse<PongMsg>();

    
    startSlaveTask();
    
    // Setup slave handler
    slave.onRequest<PingMsg>([](const PingMsg& req, PongMsg& resp) {
        resp.echo_timestamp = req.timestamp;
        resp.response_timestamp = millis();
    });
    
    const int NUM_PINGS = 5;
    for(int i = 0; i < NUM_PINGS; i++) {
        // Send a ping
        PingMsg ping;
        ping.timestamp = millis();
        PongMsg pong;
        
        auto result = master.sendRequest(ping, pong);
        TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
        uint32_t request_time = ping.timestamp;
        uint32_t response_time = pong.response_timestamp;
        TEST_ASSERT_GREATER_THAN(0, request_time);
        TEST_ASSERT_GREATER_THAN(request_time, response_time);
        
        delay(10);  // Small delay between pings
    }
}

void test_response_timeout() {
    Serial.println("Début test_response_timeout");
    
    // Don't start slave task to simulate timeout
    GetStatusMsg req;
    StatusResponseMsg resp;
    auto result = master.sendRequest(req, resp);
    
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_TIMEOUT, result.status);
}

void test_corrupted_crc() {
    Serial.println("Début test_corrupted_crc");
    
    // Master sends a message with invalid CRC to slave
    std::vector<uint8_t> data = {0x01};  // Example data
    uint16_t invalidCRC = 0xFFFF;  // Invalid CRC
    sendCustomFrame(&Serial1, EZLinkDfs::START_OF_FRAME, data.size() + EZLinkDfs::FRAME_OVERHEAD, data.size(), data, invalidCRC);
    
    // Slave must detect error
    delay(20);  // Give some time for UART transmission
    auto result = slave.poll();
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_CRC, result.status);
}

void test_truncated_message() {
    Serial.println("Début test_truncated_message");
    
    // Register message for this test
    master.registerRequest<SetLedMsg>();
    slave.registerRequest<SetLedMsg>();
    
    // 1. Send a truncated message: announce a length of 12 (0x0C) but send less
    std::vector<uint8_t> truncated_data = {0x01, 0x42};  // ID + payload
    sendCustomFrame(&Serial1, EZLinkDfs::START_OF_FRAME, 0x0C, 0x01, truncated_data, 0);
    
    // 2. Send a valid message (SetLedMsg)
    SetLedMsg msg{.state = 1};
    master.sendMsg(msg);
    
    delay(20);
    
    // 3. First poll() must detect CRC error (it will read data from the next message)
    auto result = slave.poll();
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_CRC, result.status);
    
    // 4. Continue polling until all messages are processed
    bool success_found = false;
    while(true) {
        result = slave.poll();
        if(result.status == EZLink::SUCCESS) {
            success_found = true;
        }
        else if(result.status == EZLink::NOTHING_TO_DO) {
            break;
        }
        // Ignore other potential errors (invalid SOF etc)
    }
    
    // We must have found at least one valid message
    TEST_ASSERT_TRUE(success_found);
}

void test_invalid_sof() {
    Serial.println("Début test_invalid_sof");
    
    // Send a message with invalid SOF
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};  // Example data
    uint16_t crc = EZLink::calculateCRC16(data.data(), data.size());
    sendCustomFrame(&Serial1, 0x55, data.size() + EZLinkDfs::FRAME_OVERHEAD, data.size(), data, crc);  // SOF incorrect
    
    // Poll to check error
    delay(20);  // Give some time for UART transmission
    auto result = slave.poll();
    bool got_invalid_sof = (result.status == EZLink::ERR_RCV_INVALID_SOF);
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_INVALID_SOF, result.status);
}

void test_invalid_length() {
    Serial.println("Début test_invalid_length");
    
    // Send a message with invalid length
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};  // Example data
    uint16_t crc = EZLink::calculateCRC16(data.data(), data.size());
    sendCustomFrame(&Serial1, EZLinkDfs::START_OF_FRAME, 255, data.size(), data, crc);  // Longueur incorrecte
    
    // Poll to check error
    delay(20);  // Give some time for UART transmission
    auto result = slave.poll();
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_INVALID_LEN, result.status);
}

void test_unexpected_response() {
    Serial.println("Début test_unexpected_response");
    
    // Here it's different: slave sends an unexpected response to master
    std::vector<uint8_t> payload = {0x42};  // Any data
    sendCustomFrame(&Serial2, EZLinkDfs::START_OF_FRAME, 6, 0x81, payload, 0);  // ID with response bit
    
    // Master must detect error
    delay(20);  // Give some time for UART transmission
    auto result = master.poll();
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_UNEXPECTED_RESPONSE, result.status);
}

void test_oversized_message() {
    Serial.println("Début test_oversized_message");
    
    // Register message for this test
    master.registerRequest<SetLedMsg>();
    slave.registerRequest<SetLedMsg>();
    
    // 1. Send a message longer than its announced LEN
    std::vector<uint8_t> oversized_data = {0x01, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48};  // 8 bytes of data
    sendCustomFrame(&Serial1, EZLinkDfs::START_OF_FRAME, 0x05, 0x01, oversized_data, 0);  // LEN = 5 mais données plus longues
    
    // 2. Send a valid message (SetLedMsg)
    SetLedMsg msg{.state = 1};
    master.sendMsg(msg);
    
    delay(20);
    
    // 3. First poll() must detect CRC error
    auto first_result = slave.poll();
    bool got_crc_error = (first_result.status == EZLink::ERR_RCV_CRC);

    // 4. Continue polling until all messages are processed
    bool success_found = false;
    while(true) {
        auto result = slave.poll();
        if(result.status == EZLink::SUCCESS) {
            success_found = true;
        }
        else if(result.status == EZLink::NOTHING_TO_DO) {
            break;
        }
        // yield();
        // We can have several CRC errors before finding the valid message
    }
    
    // Make assertions AFTER the loop
    TEST_ASSERT_TRUE(got_crc_error);
    TEST_ASSERT_TRUE(success_found);
}

// Simple circular buffer to simulate hardware UART
class MockUartBuffer {
private:
    static constexpr size_t SIZE = 256;
    uint8_t buffer[SIZE];
    volatile size_t head = 0;
    volatile size_t tail = 0;

public:
    size_t write(const uint8_t* data, size_t len) {
        size_t written = 0;
        for(size_t i = 0; i < len; i++) {
            size_t next = (head + 1) % SIZE;
            if(next != tail) {
                buffer[head] = data[i];
                head = next;
                written++;
            }
        }
        return written;
    }

    size_t read(uint8_t* data, size_t maxLen) {
        size_t read = 0;
        while(read < maxLen && tail != head) {
            data[read++] = buffer[tail];
            tail = (tail + 1) % SIZE;
        }
        return read;
    }
};

void test_custom_callbacks() {
    Serial.println("Début test_custom_callbacks");

    static MockUartBuffer masterToSlave;
    static MockUartBuffer slaveToMaster;
    TaskHandle_t callbackSlaveTask = NULL;  // Handle local to this test

    // Create master and slave instances
    EZLink master(
        [](const uint8_t* data, size_t len) {
            return masterToSlave.write(data, len);
        },
        [](uint8_t* data, size_t maxLen) {
            return slaveToMaster.read(data, maxLen);
        }
    );

    EZLink slave(
        [](const uint8_t* data, size_t len) {
            return slaveToMaster.write(data, len);
        },
        [](uint8_t* data, size_t maxLen) {
            return masterToSlave.read(data, maxLen);
        }
    );

    // Register messages on both instances
    master.registerRequest<SetLedMsg>();
    master.registerRequest<SetPwmMsg>();
    master.registerRequest<GetStatusMsg>();
    master.registerResponse<StatusResponseMsg>();

    slave.registerRequest<SetLedMsg>();
    slave.registerRequest<SetPwmMsg>();
    slave.registerRequest<GetStatusMsg>();
    slave.registerResponse<StatusResponseMsg>();

    // Configure slave handlers
    bool messageLedReceived = false;
    slave.onReceive<SetLedMsg>([&messageLedReceived](const SetLedMsg& msg) {
        messageLedReceived = true;
        TEST_ASSERT_EQUAL(1, msg.state);
    });

    slave.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        TEST_ASSERT_EQUAL(1000, msg.freq);
    });

    slave.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        resp.state = 1;
        resp.uptime = 1000;
    });

    // Start slave task specific to this test
    xTaskCreatePinnedToCore(
        [](void* parameter) {
            EZLink* slaveInstance = (EZLink*)parameter;
            while(true) {
                auto result = slaveInstance->poll();
                if(result == EZLink::SUCCESS) {
                    Serial.println("Callback slave: message processed");
                }
                else if(result != EZLink::NOTHING_TO_DO) {
                    Serial.printf("Callback slave: poll error %d\n", result.status);
                }
                vTaskDelay(pdMS_TO_TICKS(SLAVE_DELAY_MS));
            }
        },
        "callbackSlaveTask",
        10000,
        &slave,  // Pass local instance
        1,
        &callbackSlaveTask,
        1
    );

    // Test 1: simple MESSAGE
    SetLedMsg ledMsg{.state = 1};
    auto resultSendLed = master.sendMsg(ledMsg);
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, resultSendLed.status);

    delay(50);  // Give some time for slave to process
    TEST_ASSERT_TRUE(messageLedReceived);

    // Test 2: MESSAGE_ACK
    SetPwmMsg pwmMsg{.pin = 1, .freq = 1000};
    auto resultSendPwm = master.sendMsgAck(pwmMsg);
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, resultSendPwm.status);

    // Test 3: REQUEST/RESPONSE
    GetStatusMsg req;
    StatusResponseMsg resp;
    auto resultRequest = master.sendRequest(req, resp);
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, resultRequest.status);
    TEST_ASSERT_EQUAL(1, resp.state);
    TEST_ASSERT_EQUAL(1000, resp.uptime);

    // Cleanup specific to this test
    if (callbackSlaveTask != NULL) {
        vTaskDelete(callbackSlaveTask);
        callbackSlaveTask = NULL;
    }
}

void setup() {
    delay(2000);  // Give monitor time to connect
    testsInit(); // Initialize hardware
    
    UNITY_BEGIN();
    
    // Basic tests
    RUN_TEST(test_basic_communication);
    RUN_TEST(test_ack_required);
    RUN_TEST(test_request_response);
    
    // Robustness tests
    RUN_TEST(test_max_payload);
    RUN_TEST(test_empty_payload);
    RUN_TEST(test_burst_messages);
    RUN_TEST(test_bidirectional);
    
    // Advanced tests
    RUN_TEST(test_response_timeout);
    RUN_TEST(test_corrupted_crc);
    RUN_TEST(test_truncated_message);
    RUN_TEST(test_invalid_sof);
    RUN_TEST(test_invalid_length);
    RUN_TEST(test_unexpected_response);
    RUN_TEST(test_oversized_message);

    // Tests with custom callbacks
    RUN_TEST(test_custom_callbacks);
    UNITY_END();
    
    // Final cleanup
    if (slaveTask != NULL) {
        vTaskDelete(slaveTask);
        slaveTask = NULL;
    }
}

void loop() {
    // Nothing to do
    Serial.println("Idle...");
    delay(1000);
} 