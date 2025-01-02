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

// Structures pour les tests de FC Request/Response
struct TestResponseMsg;  // Forward declaration nécessaire car TestRequestMsg y fait référence

struct TestRequestMsg {
    static constexpr ProtoType type = ProtoType::REQUEST;
    static constexpr const char* name = "TEST_REQ";
    static constexpr uint8_t fc = 42;  // FC arbitraire entre 1-127
    using ResponseType = TestResponseMsg;  // Utilise TestResponseMsg qui doit être déclaré avant
    uint8_t data;
} __attribute__((packed));

struct TestResponseMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr const char* name = "TEST_RESP";
    static constexpr uint8_t fc = 42;  // Même FC que la requête
    uint8_t data;
} __attribute__((packed));

struct WrongResponseMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr const char* name = "WRONG_RESP";
    static constexpr uint8_t fc = 43;  // Different de TestRequestMsg::fc !
    uint8_t data;
} __attribute__((packed));

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
    bool autoResponseEnabled = true;  // Par défaut on répond

    void prepareAutoResponse() {
        // We don't prepare auto-response for FIRE_AND_FORGET messages
        if (lastMessageFC == SetLedMsg::fc) {
            return;  // SetLedMsg is FIRE_AND_FORGET
        }
        
        if (lastMessageFC == SetPwmMsg::fc) {
            // Trouver la taille du message SetPwmMsg
            size_t msgSize = sizeof(SetPwmMsg) + SimpleComm::FRAME_OVERHEAD;
            
            // Trouver l'offset du message SetPwmMsg dans le buffer
            size_t offset = 0;
            while (offset < txCount) {
                if (txBuffer[offset] == SimpleComm::START_OF_FRAME && 
                    offset + 2 < txCount && 
                    txBuffer[offset + 2] == SetPwmMsg::fc) {
                    break;
                }
                offset++;
            }
            
            printf("\nMOCK: Preparing ACK response (message size: %zu, offset: %zu)\n", msgSize, offset);
            
            // 1. Copier uniquement le message SetPwmMsg depuis le bon offset
            memcpy(autoResponseBuffer, txBuffer + offset, msgSize - 1);
            printf("MOCK: Original message (without CRC):");
            for(size_t i = 0; i < msgSize-1; i++) {
                printf(" %02X", autoResponseBuffer[i]);
            }
            printf("\n");
            
            // 2. Mettre le bon FC
            autoResponseBuffer[2] = SetPwmMsg::fc | SimpleComm::FC_RESPONSE_BIT;
            printf("MOCK: Modified message (before CRC):");
            for(size_t i = 0; i < msgSize-1; i++) {
                printf(" %02X", autoResponseBuffer[i]);
            }
            printf("\n");
            
            // 3. Calculer le CRC sur le buffer modifié
            uint8_t crc = SimpleComm::calculateCRC(autoResponseBuffer, msgSize-1);
            printf("MOCK: Calculated CRC: %02X\n", crc);
            autoResponseBuffer[msgSize-1] = crc;
            
            printf("MOCK: Final message:");
            for(size_t i = 0; i < msgSize; i++) {
                printf(" %02X", autoResponseBuffer[i]);
            }
            printf("\n");
            
            autoResponseSize = msgSize;  // Important : utiliser la taille correcte
        }
        else if (lastMessageFC == GetStatusMsg::fc) {
            // Pour REQUEST/RESPONSE, on construit une réponse avec FC | FC_RESPONSE_BIT
            StatusResponseMsg resp{.state = 1, .uptime = 1000};
            
            // Build the response frame
            autoResponseBuffer[0] = SimpleComm::START_OF_FRAME;
            autoResponseBuffer[1] = sizeof(StatusResponseMsg) + SimpleComm::FRAME_OVERHEAD;
            autoResponseBuffer[2] = StatusResponseMsg::fc | SimpleComm::FC_RESPONSE_BIT;  // FC de réponse
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
        printf("MOCK: Writing frame, size=%zu\n", size);
        printf("MOCK: FC=%d (0x%02X)\n", buffer[2], buffer[2]);
        
        // Store the message
        size_t written = 0;
        while (written < size && txCount < BUFFER_SIZE) {
            txBuffer[txCount++] = buffer[written++];
        }
        
        // Extract the FC (3rd byte of the frame)
        if (size >= 3) {
            lastMessageFC = buffer[2];
            printf("MOCK: Stored lastMessageFC=%d (0x%02X)\n", lastMessageFC, lastMessageFC);
            
            // Prepare a response only for messages that expect one AND if enabled
            if (autoResponseEnabled && 
                (lastMessageFC == SetPwmMsg::fc || lastMessageFC == GetStatusMsg::fc)) {
                prepareAutoResponse();
                
                printf("MOCK: Prepared auto-response, size=%zu\n", autoResponseSize);
                printf("MOCK: Response FC=%d (0x%02X)\n", autoResponseBuffer[2], autoResponseBuffer[2]);
                
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

        // Reset enableAutoResponse
        enableAutoResponse();
        
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

    void disableAutoResponse() { autoResponseEnabled = false; }
    void enableAutoResponse() { autoResponseEnabled = true; }
};

// Variable globale pour le mock (accessible dans setUp/tearDown)
static MockSerial* currentMock = nullptr;

void setUp(void) {
    TimeManager::reset();
}

void tearDown(void) {
}

void test_register_proto(void) {
    MockSerial serial;
    serial.reset();  // Stockage du mock
    SimpleComm comm(&serial);

    // Test 1: Normal registration
    auto result = comm.registerRequest<SetLedMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);

    // Test 2: Double registration
    result = comm.registerRequest<SetLedMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_FC_ALREADY_REGISTERED, result.status);
}

void test_send_msg(void) {
    MockSerial serial;
    serial.reset();  // Stockage du mock
    SimpleComm comm(&serial);
    
    // Register the proto first
    comm.registerRequest<SetLedMsg>();
    
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
    serial.reset();
    SimpleComm comm(&serial);
    
    bool handlerCalled = false;
    
    // Register the proto and the handler
    comm.registerRequest<SetLedMsg>();
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
    serial.reset();
    SimpleComm comm(&serial);
    
    comm.registerRequest<SetPwmMsg>();
    
    // Désactiver l'auto-réponse
    serial.disableAutoResponse();
    
    // Envoyer un message qui nécessite une réponse
    SetPwmMsg msg{.pin = 1, .freq = 1000};
    
    // Test 1: Pas de réponse du tout
    auto result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_TIMEOUT, result.status);
    
    // Test 2: Réponse partielle
    uint8_t partialResponse[] = {
        SimpleComm::START_OF_FRAME,
        sizeof(SetPwmMsg) + SimpleComm::FRAME_OVERHEAD,
        SetPwmMsg::fc | SimpleComm::FC_RESPONSE_BIT,
        // Manque les données et le CRC
    };
    serial.injectData(partialResponse, sizeof(partialResponse));
    
    result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_TIMEOUT, result.status);
}

void test_send_msg_with_ack(void) {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    // Register the proto
    comm.registerRequest<SetPwmMsg>();
    
    // Test with auto-response from mock
    SetPwmMsg msg{.pin = 5, .freq = 1000};
    auto result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status); // Must succeed because the mock automatically responds
}

void test_request_response(void) {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    // Register the protos
    comm.registerRequest<GetStatusMsg>();
    comm.registerResponse<StatusResponseMsg>();
    
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
    serial.reset();
    SimpleComm comm(&serial);
    
    // Test short frame
    uint8_t shortFrame[] = {SimpleComm::START_OF_FRAME, 2};
    serial.injectData(shortFrame, sizeof(shortFrame));
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_INVALID_LEN, result.status);
    
    // Reset for the next test
    serial.reset();
    
    // Register the proto for the CRC test
    comm.registerRequest<SetLedMsg>();
    
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
    serial.reset();
    SimpleComm comm(&serial);
    
    // Test with a message at the limit (must pass)
    auto result = comm.registerRequest<BorderlineMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Test write buffer full
    comm.registerRequest<SetLedMsg>();  // Must register the proto first !
    serial.setAvailableForWrite(2);   // Simulate almost full buffer
    SetLedMsg msg{.state = 1};
    result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_OVERFLOW, result.status);
}

void test_set_timeout(void) {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    // Test custom timeout
    comm.setResponseTimeout(500);
    TEST_ASSERT_EQUAL(500, comm.getResponseTimeout());
    
    // Test invalid timeout (0)
    TEST_ASSERT_FALSE(comm.setResponseTimeout(0));
}

void test_name_collisions(void) {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    comm.registerRequest<SetLedMsg>();
    auto result = comm.registerRequest<DuplicateNameMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_NAME_ALREADY_REGISTERED, result.status);
}

void test_proto_mismatch(void) {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    comm.registerRequest<SetLedMsg>();
    
    // Try to use a message with the same FC but a different name
    SameFcMsg msg{.value = 1};
    auto result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_SND_PROTO_MISMATCH, result.status);
}

void test_handler_wrong_name(void) {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    comm.registerRequest<SetLedMsg>();
    
    // Try to register a handler for a message with the same FC but different name
    auto result = comm.onReceive<SameFcMsg>([](const SameFcMsg&) {});
    TEST_ASSERT_EQUAL(SimpleComm::ERR_REG_PROTO_MISMATCH, result.status);
}

void test_invalid_names(void) {
    // ... removed because it's impossible to test (blocked at compilation => OK)
}

void test_complex_data(void) {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    ComplexMsg msg = {
        .array = {0xDEADBEEF, 0xCAFEBABE, 0x12345678, 0x87654321},
        .coordinates = {-100, 200, -300},
        .flags = 0xAA
    };
    
    comm.registerRequest<ComplexMsg>();
    
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
    serial.reset();
    SimpleComm comm(&serial);
    
    // Test message minimal
    auto result = comm.registerRequest<MinimalMsg>();
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
    serial.reset();
    SimpleComm comm(&serial);
    
    // Must register the proto first !
    comm.registerRequest<SetLedMsg>();
    
    // Frame with wrong length
    uint8_t wrongLenFrame[] = {
        SimpleComm::START_OF_FRAME,
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD + 1,  // Too long
        SetLedMsg::fc,
        1,
        0,    // Données supplémentaires pour atteindre la taille annoncée
        0  // CRC
    };
    wrongLenFrame[4] = SimpleComm::calculateCRC(wrongLenFrame, 4);
    serial.injectData(wrongLenFrame, sizeof(wrongLenFrame));
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_RCV_PROTO_MISMATCH, result.status);
    
    serial.reset();  // Important to reset between tests !
}

void test_stress(void) {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    comm.registerRequest<SetLedMsg>();
    
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
    uint8_t captured[SimpleComm::MAX_FRAME_SIZE];
    size_t capturedLen;
    
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
    
    printf("\nTEST: Injecting normal frame\n");
    serial.injectData(frame, sizeof(frame));

    printf("\nTEST: Injecting frame with SOF in data\n");
    serial.injectData(frameWithSOF, sizeof(frameWithSOF));

    printf("\nTEST: Injecting normal frame\n");
    serial.injectData(frame, sizeof(frame));
    
    // All frames must pass
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
}

void test_mixed_message_types(void) {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    // Register different types of messages
    comm.registerRequest<SetLedMsg>();        // FIRE_AND_FORGET
    comm.registerRequest<SetPwmMsg>();        // ACK_REQUIRED
    comm.registerRequest<GetStatusMsg>();     // REQUEST
    comm.registerResponse<StatusResponseMsg>(); // RESPONSE
    
    // Test a mixed sequence
    SetLedMsg led{.state = 1};
    SetPwmMsg pwm{.pin = 2, .freq = 1000};
    GetStatusMsg status{};
    StatusResponseMsg resp{};
    
    printf("\nTesting FIRE_AND_FORGET message...\n");
    auto result = comm.sendMsg(led);        
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    printf("\nTesting ACK_REQUIRED message...\n");
    printf("Original FC: 0x%02X\n", SetPwmMsg::fc);
    printf("Expected response FC: 0x%02X\n", SetPwmMsg::fc | SimpleComm::FC_RESPONSE_BIT);
    result = comm.sendMsgAck(pwm);         
    if (result != SimpleComm::SUCCESS) {
        printf("Failed with error: %d\n", result.status);
    }
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // ...
}

void test_response_fc_calculation() {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    // Enregistrer la paire request/response
    auto result = comm.registerRequest<TestRequestMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    result = comm.registerResponse<TestResponseMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    const auto* proto = comm.getProtoStore(TestRequestMsg::fc | SimpleComm::FC_RESPONSE_BIT);
    TEST_ASSERT_NOT_NULL(proto);
    TEST_ASSERT_EQUAL(TestResponseMsg::fc | SimpleComm::FC_RESPONSE_BIT, proto->fc);
}

void test_response_wrong_fc() {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    // La requête doit passer
    auto result = comm.registerRequest<TestRequestMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // La réponse avec mauvais FC doit être rejetée
    result = comm.registerResponse<WrongResponseMsg>();
    TEST_ASSERT_NOT_EQUAL(SimpleComm::SUCCESS, result.status);
}

// Those tests are intentionally commented out because they MUST fail at compilation
// They demonstrate that the library prevents compilation:
// 1. Registering a response as a request
// 2. Registering a request as a response
//
// void test_response_registered_as_request() {
//     MockSerial serial;
//     serial.reset();
//     SimpleComm comm(&serial);
//
//     // This test MUST fail at compilation because we cannot
//     // register a RESPONSE as a REQUEST (checked by static_assert)
//     auto result = comm.registerRequest<TestResponseMsg>();
//     TEST_ASSERT_NOT_EQUAL(SimpleComm::SUCCESS, result.status);
// }
//
// void test_request_registered_as_response() {
//     MockSerial serial;
//     serial.reset();   
//     SimpleComm comm(&serial);
//
//     // This test MUST fail at compilation because we cannot
//     // register a REQUEST as a RESPONSE (checked by static_assert)
//     auto result = comm.registerResponse<TestRequestMsg>();
//     TEST_ASSERT_NOT_EQUAL(SimpleComm::SUCCESS, result.status);
// }

void test_busy_receiving() {
    MockSerial serial;
    serial.reset();
    SimpleComm comm(&serial);
    
    // Enregistrer les protos nécessaires
    comm.registerRequest<SetLedMsg>();
    comm.registerRequest<SetPwmMsg>();
    
    // Préparer un message incomplet pour simuler une réception en cours
    uint8_t partialFrame[] = {
        SimpleComm::START_OF_FRAME,  // SOF
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,  // LEN
        SetLedMsg::fc,  // FC
        // On n'injecte pas tout le message pour simuler une réception partielle
    };
    
    // Injecter le début de frame
    serial.injectData(partialFrame, sizeof(partialFrame));
    
    // Premier poll() pour démarrer la capture
    printf("\nStarting frame capture...\n");
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::NOTHING_TO_DO, result.status);
    
    // Essayer d'envoyer un message pendant la capture
    printf("Trying to send while capturing...\n");
    SetPwmMsg msg{.pin = 1, .freq = 1000};
    result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_BUSY_RECEIVING, result.status);
    
    // Préparer la frame complète pour calculer le bon CRC
    uint8_t completeFrame[SimpleComm::MAX_FRAME_SIZE];
    memcpy(completeFrame, partialFrame, sizeof(partialFrame));
    completeFrame[sizeof(partialFrame)] = 0x01;  // state = 1
    
    // Calculer le CRC sur la frame complète (sans le CRC)
    uint8_t crc = SimpleComm::calculateCRC(completeFrame, sizeof(partialFrame) + 1);
    
    // Injecter le reste avec le bon CRC
    uint8_t remainingFrame[] = {
        0x01,  // state = 1
        crc    // CRC calculé sur la frame complète
    };
    
    serial.injectData(remainingFrame, sizeof(remainingFrame));
    
    // La capture doit se terminer correctement
    printf("Completing frame capture...\n");
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Maintenant on peut renvoyer le message
    printf("Sending after capture complete...\n");
    result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
}

int main(void) {
    UNITY_BEGIN();
    
    // Tests d'enregistrement et validation des protos
    RUN_TEST(test_register_proto);
    RUN_TEST(test_name_collisions);
    RUN_TEST(test_proto_mismatch);
    RUN_TEST(test_handler_wrong_name);
    
    // Tests de communication basique
    RUN_TEST(test_send_msg);
    RUN_TEST(test_on_receive);
    RUN_TEST(test_send_msg_with_ack);
    RUN_TEST(test_request_response);
    
    // Tests de gestion des erreurs et limites
    RUN_TEST(test_error_cases);
    RUN_TEST(test_buffer_limits);
    RUN_TEST(test_timeout);
    RUN_TEST(test_set_timeout);
    RUN_TEST(test_size_limits);
    RUN_TEST(test_malformed_frames);
    
    // Tests de données
    RUN_TEST(test_complex_data);
    
    // Tests de robustesse
    RUN_TEST(test_stress);
    RUN_TEST(test_mixed_message_types);
    
    // Tests spécifiques Request/Response
    RUN_TEST(test_response_fc_calculation);
    RUN_TEST(test_response_wrong_fc);
    
    // Tests de réception en cours
    RUN_TEST(test_busy_receiving);

    
    return UNITY_END();
} 