#include <unity.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

// Test messages
struct BorderlineMsg {
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "BORDERLINE";
    static constexpr uint8_t fc = 10;
    uint8_t data[SimpleComm::MAX_FRAME_SIZE - SimpleComm::FRAME_OVERHEAD];
};

struct DuplicateNameMsg {
    static constexpr uint8_t fc = 20;
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "SET_LED";  // Same name as SetLedMsg
    uint8_t value;
};

struct SameFcMsg {
    static constexpr uint8_t fc = SetLedMsg::fc;  // Same FC
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "DIFFERENT";
    uint8_t value;
};

// Not used (test blocked at compilation => OK)
struct NullNameMsg {
    static constexpr uint8_t fc = 30;
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = nullptr;
    uint8_t value;
};

// Not used (test blocked at compilation => OK)
struct EmptyNameMsg {
    static constexpr uint8_t fc = 31;
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "";  // Empty string
    uint8_t value;
};

struct ComplexMsg {
    static constexpr uint8_t fc = 32;
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "COMPLEX";
    uint32_t array[4];
    int16_t coordinates[3];
    uint8_t flags;
};

struct MinimalMsg {
    static constexpr uint8_t fc = 33;
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "MINIMAL";
    uint8_t dummy;  // Minimal message of one byte
};



// Mock for tests
class MockSerial : public HardwareSerial {
private:
    static constexpr size_t BUFFER_SIZE = 256;
    static constexpr uint32_t BYTE_READ_TIME_MS = 1;
    
    uint8_t txBuffer[BUFFER_SIZE];
    uint8_t rxBuffer[BUFFER_SIZE];
    size_t txCount;
    size_t rxCount;
    size_t rxIndex;
    
    // For response simulation
    uint8_t lastMessageFC;
    uint8_t autoResponseBuffer[BUFFER_SIZE];
    size_t autoResponseSize;
    int availableForWriteValue;

    void prepareAutoResponse() {
        // We don't prepare auto-response for FIRE_AND_FORGET messages
        if (lastMessageFC == SetLedMsg::fc) {
            return;  // SetLedMsg is FIRE_AND_FORGET
        }
        
        if (lastMessageFC == SetPwmMsg::fc) {
            // For ACK_REQUIRED, we return exactly the same message
            memcpy(autoResponseBuffer, txBuffer, txCount);
            autoResponseSize = txCount;
        }
        else if (lastMessageFC == GetStatusMsg::fc) {
            // For REQUEST/RESPONSE, we build a response
            StatusResponseMsg resp{.state = 1, .uptime = 1000};
            
            // Build the response frame
            autoResponseBuffer[0] = SimpleComm::START_OF_FRAME;
            autoResponseBuffer[1] = sizeof(StatusResponseMsg) + SimpleComm::FRAME_OVERHEAD;
            autoResponseBuffer[2] = StatusResponseMsg::fc;
            memcpy(&autoResponseBuffer[3], &resp, sizeof(StatusResponseMsg));
            autoResponseBuffer[sizeof(StatusResponseMsg) + SimpleComm::FRAME_OVERHEAD - 1] = 
                SimpleComm::calculateCRC(autoResponseBuffer, sizeof(StatusResponseMsg) + SimpleComm::FRAME_OVERHEAD - 1);
            
            autoResponseSize = sizeof(StatusResponseMsg) + SimpleComm::FRAME_OVERHEAD;
        }
    }

public:
    MockSerial() : txCount(0), rxCount(0), rxIndex(0), lastMessageFC(0), autoResponseSize(0), availableForWriteValue(255) {
        memset(txBuffer, 0, BUFFER_SIZE);
        memset(rxBuffer, 0, BUFFER_SIZE);
        memset(autoResponseBuffer, 0, BUFFER_SIZE);
    }

    size_t write(uint8_t byte) override {
        if (txCount < BUFFER_SIZE) {
            txBuffer[txCount++] = byte;
            return 1;
        }
        return 0;
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        // Store the message
        size_t written = 0;
        while (written < size && txCount < BUFFER_SIZE) {
            txBuffer[txCount++] = buffer[written++];
        }
        
        // Extract the FC (3rd byte of the frame)
        if (size >= 3) {
            lastMessageFC = buffer[2];
            
            // Prepare a response only for messages that expect one
            if (lastMessageFC == SetPwmMsg::fc || lastMessageFC == GetStatusMsg::fc) {
                prepareAutoResponse();
                
                // Copy auto-response into the reception buffer
                memcpy(rxBuffer, autoResponseBuffer, autoResponseSize);
                rxCount = autoResponseSize;
                rxIndex = 0;
            }
        }
        
        return written;
    }

    int available() override { 
        return rxCount - rxIndex; 
    }

    int read() override { 
        TimeManager::advanceTime(BYTE_READ_TIME_MS); // Simulate reading time
        return rxIndex < rxCount ? rxBuffer[rxIndex++] : -1; 
    }

    int peek() override {
        TimeManager::advanceTime(BYTE_READ_TIME_MS); // Simulate reading time
        return rxIndex < rxCount ? rxBuffer[rxIndex] : -1;
    }

    // Specific methods for tests
    void reset() {
        // Reset buffers
        memset(txBuffer, 0, BUFFER_SIZE);
        memset(rxBuffer, 0, BUFFER_SIZE);
        memset(autoResponseBuffer, 0, BUFFER_SIZE);
        
        // Reset counters
        txCount = 0;
        rxCount = 0;
        rxIndex = 0;
        
        // Reset states
        lastMessageFC = 0;
        autoResponseSize = 0;
        availableForWriteValue = 255;
        
        // Reset simulated time
        TimeManager::reset();
    }

    const uint8_t* getTxBuffer() const { 
        return txBuffer; 
    }
    
    size_t getTxCount() const { 
        return txCount; 
    }

    void injectData(const uint8_t* data, size_t len) {
        if (rxCount + len <= BUFFER_SIZE) {
            memcpy(rxBuffer + rxCount, data, len);
            rxCount += len;
        }
    }

    // Always return a large value to simulate an empty write buffer
    int availableForWrite() override {
        return availableForWriteValue;
    }

    bool setAvailableForWrite(int value) override {
        availableForWriteValue = value;
        return true;
    }
};

void setUp(void) {
    TimeManager::reset();  // Reset time at the beginning of each test
}

void tearDown(void) {
    // Cleanup after each test
}

void test_register_proto(void) {
    MockSerial serial;
    SimpleComm comm(&serial);

    // Test 1: Normal registration
    auto result = comm.registerProto<SetLedMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);

    // Test 2: Double registration
    result = comm.registerProto<SetLedMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_FC_ALREADY_REGISTERED, result.status);
}

void test_send_msg(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Register the proto first
    comm.registerProto<SetLedMsg>();
    
    // Normal send test
    SetLedMsg msg{.state = 1};
    auto result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Send test without proto registered
    SetPwmMsg pwm{.pin = 1, .freq = 1000};
    result = comm.sendMsgAck(pwm);  // Use sendMsgAck for ACK_REQUIRED
    TEST_ASSERT_EQUAL(SimpleComm::ERR_INVALID_FC, result.status);
}

void test_on_receive(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    bool handlerCalled = false;
    
    // Register the proto and the handler
    comm.registerProto<SetLedMsg>();
    comm.onReceive<SetLedMsg>([&handlerCalled](const SetLedMsg& msg) {
        handlerCalled = true;
        TEST_ASSERT_EQUAL(1, msg.state);
    });
    
    // Build a valid frame
    uint8_t frame[] = {
        SimpleComm::START_OF_FRAME,  // SOF
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,  // LEN
        SetLedMsg::fc,               // FC
        1,                          // state = 1
        0                           // CRC (will be calculated below)
    };
    
    // Calculate CRC on the entire frame except the last byte
    frame[sizeof(frame)-1] = SimpleComm::calculateCRC(frame, sizeof(frame)-1);
    
    // Inject the frame
    serial.injectData(frame, sizeof(frame));
    
    // Process the frame
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    TEST_ASSERT_TRUE(handlerCalled);
}

void test_timeout(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Inject an incomplete frame start
    uint8_t start = SimpleComm::START_OF_FRAME;
    serial.injectData(&start, 1);
    
    // The timeout should naturally occur because each byte read
    // increments the time
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_TIMEOUT, result.status);
}

void test_send_msg_with_ack(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Register the proto
    comm.registerProto<SetPwmMsg>();
    
    // Test with auto-response from mock
    SetPwmMsg msg{.pin = 5, .freq = 1000};
    auto result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status); // Must succeed because the mock automatically responds
}

void test_request_response(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Register the protos
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
    
    // Configure the request handler that will generate the response
    comm.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        resp.state = 1;
        resp.uptime = 1000;
    });
    
    // Send a request
    GetStatusMsg req{};
    StatusResponseMsg resp{};
    
    // The response will be automatically generated by the mock
    auto result = comm.sendRequest(req, resp);
    
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    TEST_ASSERT_EQUAL(1, resp.state);
    TEST_ASSERT_EQUAL(1000, resp.uptime);
}

void test_error_cases(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Test short frame
    uint8_t shortFrame[] = {SimpleComm::START_OF_FRAME, 2};
    serial.injectData(shortFrame, sizeof(shortFrame));
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_INVALID_LEN, result.status);
    
    // Reset for the next test
    serial.reset();
    
    // Register the proto for the CRC test
    comm.registerProto<SetLedMsg>();
    
    // Test bad CRC
    uint8_t badCrcFrame[] = {
        SimpleComm::START_OF_FRAME,
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,
        SetLedMsg::fc,
        1,
        0xFF  // Bad CRC
    };
    serial.injectData(badCrcFrame, sizeof(badCrcFrame));
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_CRC, result.status);
}

void test_buffer_limits(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Test with a message at the limit (must pass)
    auto result = comm.registerProto<BorderlineMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Test write buffer full
    comm.registerProto<SetLedMsg>();  // Must register the proto first !
    serial.setAvailableForWrite(2);   // Simulate almost full buffer
    SetLedMsg msg{.state = 1};
    result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_OVERFLOW, result.status);
}

void test_timeouts(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Test custom timeout
    comm.setInterbyteTimeout(100);
    comm.setResponseTimeout(500);
    TEST_ASSERT_EQUAL(100, comm.getInterbyteTimeout());
    TEST_ASSERT_EQUAL(500, comm.getResponseTimeout());
    
    // Test invalid timeout (0)
    TEST_ASSERT_FALSE(comm.setInterbyteTimeout(0));
    TEST_ASSERT_FALSE(comm.setResponseTimeout(0));
}

void test_name_collisions(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    comm.registerProto<SetLedMsg>();
    auto result = comm.registerProto<DuplicateNameMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_NAME_ALREADY_REGISTERED, result.status);
}

void test_proto_mismatch(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    comm.registerProto<SetLedMsg>();
    
    // Try to use a message with the same FC but a different name
    SameFcMsg msg{.value = 1};
    auto result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_PROTO_MISMATCH, result.status);
}

void test_handler_wrong_name(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    comm.registerProto<SetLedMsg>();
    
    // Try to register a handler for a message with the same FC but different name
    auto result = comm.onReceive<SameFcMsg>([](const SameFcMsg&) {});
    TEST_ASSERT_EQUAL(SimpleComm::ERR_PROTO_MISMATCH, result.status);
}

void test_invalid_names(void) {
    // ... removed because it's impossible to test (blocked at compilation => OK)
}

void test_complex_data(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    ComplexMsg msg = {
        .array = {0xDEADBEEF, 0xCAFEBABE, 0x12345678, 0x87654321},
        .coordinates = {-100, 200, -300},
        .flags = 0xAA
    };
    
    comm.registerProto<ComplexMsg>();
    
    // Test send
    auto result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Check that the data is correctly transmitted
    const uint8_t* txBuffer = serial.getTxBuffer();
    ComplexMsg received;
    memcpy(&received, &txBuffer[3], sizeof(ComplexMsg));
    
    TEST_ASSERT_EQUAL_MEMORY(msg.array, received.array, sizeof(msg.array));
    TEST_ASSERT_EQUAL_MEMORY(msg.coordinates, received.coordinates, sizeof(msg.coordinates));
    TEST_ASSERT_EQUAL(msg.flags, received.flags);
}

void test_size_limits(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Test message minimal
    auto result = comm.registerProto<MinimalMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Test with different buffer sizes available
    serial.setAvailableForWrite(sizeof(MinimalMsg) + SimpleComm::FRAME_OVERHEAD - 1);
    MinimalMsg msg{.dummy = 42};
    result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_OVERFLOW, result.status);
    
    serial.setAvailableForWrite(sizeof(MinimalMsg) + SimpleComm::FRAME_OVERHEAD);
    result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
}

void test_malformed_frames(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Must register the proto first !
    comm.registerProto<SetLedMsg>();
    
    // Frame with wrong length
    uint8_t wrongLenFrame[] = {
        SimpleComm::START_OF_FRAME,
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD + 1,  // Too long
        SetLedMsg::fc,
        1,
        0  // CRC
    };
    wrongLenFrame[4] = SimpleComm::calculateCRC(wrongLenFrame, 4);
    serial.injectData(wrongLenFrame, sizeof(wrongLenFrame));
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_PROTO_MISMATCH, result.status);
    
    serial.reset();  // Important to reset between tests !
    
    // Truncated frame
    uint8_t truncatedFrame[] = {
        SimpleComm::START_OF_FRAME,
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,
        SetLedMsg::fc
    };
    serial.injectData(truncatedFrame, sizeof(truncatedFrame));
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_TIMEOUT, result.status);
}

void test_stress(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    comm.registerProto<SetLedMsg>();
    
    // Test 1: Back-to-back frames
    uint8_t frame[] = {
        SimpleComm::START_OF_FRAME,
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,
        SetLedMsg::fc,
        1,
        0  // CRC
    };
    frame[4] = SimpleComm::calculateCRC(frame, 4);
    
    // Inject two back-to-back frames
    serial.injectData(frame, sizeof(frame));
    serial.injectData(frame, sizeof(frame));
    
    // Both must be processed
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Test 2: Corrupted data between two frames
    uint8_t garbage[] = {0xFF, 0x00, 0xFF};
    serial.injectData(frame, sizeof(frame));
    serial.injectData(garbage, sizeof(garbage));
    serial.injectData(frame, sizeof(frame));
    
    // First valid frame
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // The garbage should cause an error
    result = comm.poll();
    TEST_ASSERT_NOT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // The second frame should pass
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Test 3: Frame with SOF in the data (must pass !)
    uint8_t frameWithSOF[] = {
        SimpleComm::START_OF_FRAME,  // SOF initial
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,
        SetLedMsg::fc,
        SimpleComm::START_OF_FRAME,  // SOF in the data - perfectly valid !
        0  // CRC
    };
    frameWithSOF[4] = SimpleComm::calculateCRC(frameWithSOF, 4);
    
    serial.injectData(frame, sizeof(frame));          // Normal frame
    serial.injectData(frameWithSOF, sizeof(frameWithSOF));  // Frame with SOF in the data
    serial.injectData(frame, sizeof(frame));          // Normal frame
    
    // All frames must pass
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);  // The frame with SOF must pass !
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
}

void test_mixed_message_types(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Register different types of messages
    comm.registerProto<SetLedMsg>();        // FIRE_AND_FORGET
    comm.registerProto<SetPwmMsg>();        // ACK_REQUIRED
    comm.registerProto<GetStatusMsg>();     // REQUEST
    comm.registerProto<StatusResponseMsg>(); // RESPONSE
    
    // Test a mixed sequence
    SetLedMsg led{.state = 1};
    SetPwmMsg pwm{.pin = 2, .freq = 1000};
    GetStatusMsg status{};
    StatusResponseMsg resp{};
    
    auto result = comm.sendMsg(led);        // FIRE_AND_FORGET
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    result = comm.sendMsgAck(pwm);         // ACK_REQUIRED
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    result = comm.sendRequest(status, resp); // REQUEST/RESPONSE
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_register_proto);
    RUN_TEST(test_send_msg);
    RUN_TEST(test_on_receive);
    RUN_TEST(test_timeout);
    RUN_TEST(test_send_msg_with_ack);
    RUN_TEST(test_request_response);
    RUN_TEST(test_error_cases);
    RUN_TEST(test_buffer_limits);
    RUN_TEST(test_timeouts);
    RUN_TEST(test_name_collisions);
    RUN_TEST(test_proto_mismatch);
    RUN_TEST(test_handler_wrong_name);
    RUN_TEST(test_complex_data);
    RUN_TEST(test_size_limits);
    RUN_TEST(test_malformed_frames);
    RUN_TEST(test_stress);
    RUN_TEST(test_mixed_message_types);
    return UNITY_END();
} 