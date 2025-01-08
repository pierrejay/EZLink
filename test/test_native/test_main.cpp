#include <unity.h>
#include <Arduino.h>
#include "EZLink.h"
#include "EZLink_Proto.h"

// Test messages
struct BorderlineMsg {
    static constexpr MsgType type = MsgType::MESSAGE;
    static constexpr uint8_t id = 10;
    uint8_t data[EZLinkDfs::MAX_FRAME_SIZE - EZLinkDfs::FRAME_OVERHEAD];
} __attribute__((packed));

struct DuplicateNameMsg {
    static constexpr MsgType type = MsgType::MESSAGE;
    static constexpr uint8_t id = 20;
    uint8_t value;
} __attribute__((packed));

// Not used (test blocked at compilation => OK)
struct NullNameMsg {
    static constexpr MsgType type = MsgType::MESSAGE;
    static constexpr uint8_t id = 30;
    uint8_t value;
} __attribute__((packed));

// Not used (test blocked at compilation => OK)
struct EmptyNameMsg {
    static constexpr MsgType type = MsgType::MESSAGE;
    static constexpr uint8_t id = 31;
    uint8_t value;
} __attribute__((packed));

struct ComplexMsg {
    static constexpr MsgType type = MsgType::MESSAGE;
    static constexpr uint8_t id = 32;
    uint32_t array[4];
    int16_t coordinates[3];
    uint8_t flags;
} __attribute__((packed));

struct MinimalMsg {
    static constexpr MsgType type = MsgType::MESSAGE;
    static constexpr uint8_t id = 33;
    uint8_t dummy;  // Minimal message of one byte
} __attribute__((packed));

// Structures pour les tests de ID Request/Response
struct TestResponseMsg;  // Forward declaration nécessaire car TestRequestMsg y fait référence

struct TestRequestMsg {
    static constexpr MsgType type = MsgType::REQUEST;
    static constexpr uint8_t id = 42;  // ID arbitraire entre 1-127
    using ResponseType = TestResponseMsg;  // Utilise TestResponseMsg qui doit être déclaré avant
    uint8_t data;
} __attribute__((packed));

struct TestResponseMsg {
    static constexpr MsgType type = MsgType::RESPONSE;
    static constexpr uint8_t id = 42;  // Même ID que la requête
    uint8_t data;
} __attribute__((packed));

struct WrongResponseMsg {
    static constexpr MsgType type = MsgType::RESPONSE;
    static constexpr uint8_t id = 43;  // Different de TestRequestMsg::id !
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
    uint8_t lastMessageID;
    uint8_t autoResponseBuffer[BUFFER_SIZE];
    size_t autoResponseSize;
    int availableForWriteValue;
    bool autoResponseEnabled = true;  // Par défaut on répond

    void prepareAutoResponse() {
        // We don't prepare auto-response for MESSAGE messages
        if (lastMessageID == SetLedMsg::id) {
            return;  // SetLedMsg is MESSAGE
        }
        
        if (lastMessageID == SetPwmMsg::id) {
            // Trouver la taille du message SetPwmMsg
            size_t msgSize = sizeof(SetPwmMsg) + EZLinkDfs::FRAME_OVERHEAD;
            
            // Trouver l'offset du message SetPwmMsg dans le buffer
            size_t offset = 0;
            while (offset < txCount) {
                if (txBuffer[offset] == EZLinkDfs::START_OF_FRAME && 
                    offset + 2 < txCount && 
                    txBuffer[offset + 2] == SetPwmMsg::id) {
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
            
            // 2. Mettre le bon ID
            autoResponseBuffer[2] = SetPwmMsg::id | EZLinkDfs::ID_RESPONSE_BIT;
            printf("MOCK: Modified message (before CRC):");
            for(size_t i = 0; i < msgSize-1; i++) {
                printf(" %02X", autoResponseBuffer[i]);
            }
            printf("\n");
            
            // 3. Calculer le CRC sur le buffer modifié
            uint16_t crc = EZLink::calculateCRC16(autoResponseBuffer, msgSize-2);
            autoResponseBuffer[msgSize-2] = (uint8_t)(crc >> 8);    // MSB
            autoResponseBuffer[msgSize-1] = (uint8_t)(crc & 0xFF);  // LSB
            
            printf("MOCK: Calculated CRC: %02X\n", crc);
            
            printf("MOCK: Final message:");
            for(size_t i = 0; i < msgSize; i++) {
                printf(" %02X", autoResponseBuffer[i]);
            }
            printf("\n");
            
            autoResponseSize = msgSize;
        }
        else if (lastMessageID == GetStatusMsg::id) {
            // Pour REQUEST/RESPONSE, on construit une réponse avec ID | EZLinkDfs::ID_RESPONSE_BIT
            StatusResponseMsg resp{.state = 1, .uptime = 1000};
            
            // Build the response frame
            autoResponseBuffer[0] = EZLinkDfs::START_OF_FRAME;
            autoResponseBuffer[1] = sizeof(StatusResponseMsg) + EZLinkDfs::FRAME_OVERHEAD;
            autoResponseBuffer[2] = StatusResponseMsg::id | EZLinkDfs::ID_RESPONSE_BIT;  // ID de réponse
            memcpy(&autoResponseBuffer[3], &resp, sizeof(StatusResponseMsg));
            uint16_t crc = EZLink::calculateCRC16(autoResponseBuffer, sizeof(StatusResponseMsg) + EZLinkDfs::FRAME_OVERHEAD - 2);
            autoResponseBuffer[sizeof(StatusResponseMsg) + EZLinkDfs::FRAME_OVERHEAD - 2] = (uint8_t)(crc >> 8);    // MSB
            autoResponseBuffer[sizeof(StatusResponseMsg) + EZLinkDfs::FRAME_OVERHEAD - 1] = (uint8_t)(crc & 0xFF);  // LSB
            
            autoResponseSize = sizeof(StatusResponseMsg) + EZLinkDfs::FRAME_OVERHEAD;
        }
    }

public:
    MockSerial() : txCount(0), rxCount(0), rxIndex(0), lastMessageID(0), autoResponseSize(0), availableForWriteValue(255) {
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
        printf("MOCK: ID=%d (0x%02X)\n", buffer[2], buffer[2]);
        
        // Store the message
        size_t written = 0;
        while (written < size && txCount < BUFFER_SIZE) {
            txBuffer[txCount++] = buffer[written++];
        }
        
        // Extract the ID (3rd byte of the frame)
        if (size >= 3) {
            lastMessageID = buffer[2];
            printf("MOCK: Stored lastMessageID=%d (0x%02X)\n", lastMessageID, lastMessageID);
            
            // Prepare a response only for messages that expect one AND if enabled
            if (autoResponseEnabled && 
                (lastMessageID == SetPwmMsg::id || lastMessageID == GetStatusMsg::id)) {
                prepareAutoResponse();
                
                printf("MOCK: Prepared auto-response, size=%zu\n", autoResponseSize);
                printf("MOCK: Response ID=%d (0x%02X)\n", autoResponseBuffer[2], autoResponseBuffer[2]);
                
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
        lastMessageID = 0;
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
    EZLink comm(&serial);

    // Test 1: Normal registration
    auto result = comm.registerRequest<SetLedMsg>();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);

    // Test 2: Double registration
    result = comm.registerRequest<SetLedMsg>();
    TEST_ASSERT_EQUAL(EZLink::ERR_ID_ALREADY_REGISTERED, result.status);
}

void test_send_msg(void) {
    MockSerial serial;
    serial.reset();  // Stockage du mock
    EZLink comm(&serial);
    
    // Register the proto first
    comm.registerRequest<SetLedMsg>();
    
    // Normal send test
    SetLedMsg msg{.state = 1};
    auto result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    // Send test without proto registered
    SetPwmMsg pwm{.pin = 1, .freq = 1000};
    result = comm.sendMsgAck(pwm);  // Use sendMsgAck for MESSAGE_ACK
    TEST_ASSERT_EQUAL(EZLink::ERR_SND_INVALID_ID, result.status);
}

void test_on_receive(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    bool handlerCalled = false;
    
    // Register the proto and the handler
    comm.registerRequest<SetLedMsg>();
    comm.onReceive<SetLedMsg>([&handlerCalled](const SetLedMsg& msg) {
        handlerCalled = true;
        TEST_ASSERT_EQUAL(1, msg.state);
    });
    
    // Build a valid frame
    uint8_t frame[] = {
        EZLinkDfs::START_OF_FRAME,  // SOF
        sizeof(SetLedMsg) + EZLinkDfs::FRAME_OVERHEAD,  // LEN
        SetLedMsg::id,               // ID
        1,                          // state = 1
        0,                           // CRC (will be calculated below)
        0                           // CRC (will be calculated below)
    };
    
    uint16_t crc = EZLink::calculateCRC16(frame, sizeof(frame)-2);
    frame[sizeof(frame)-2] = (uint8_t)(crc >> 8);    // MSB
    frame[sizeof(frame)-1] = (uint8_t)(crc & 0xFF);  // LSB
    
    // Inject the frame
    serial.injectData(frame, sizeof(frame));
    
    // Process the frame
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    TEST_ASSERT_TRUE(handlerCalled);
}

void test_timeout(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    comm.registerRequest<SetPwmMsg>();
    
    // Désactiver l'auto-réponse
    serial.disableAutoResponse();
    
    // Envoyer un message qui nécessite une réponse
    SetPwmMsg msg{.pin = 1, .freq = 1000};
    
    // Test 1: Pas de réponse du tout
    auto result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_TIMEOUT, result.status);
    
    // Test 2: Réponse partielle
    uint8_t partialResponse[] = {
        EZLinkDfs::START_OF_FRAME,
        sizeof(SetPwmMsg) + EZLinkDfs::FRAME_OVERHEAD,
        SetPwmMsg::id | EZLinkDfs::ID_RESPONSE_BIT,
        // Manque les données et le CRC
    };
    serial.injectData(partialResponse, sizeof(partialResponse));
    
    result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_TIMEOUT, result.status);
}

void test_send_msg_with_ack(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // Register the proto
    comm.registerRequest<SetPwmMsg>();
    
    // Test with auto-response from mock
    SetPwmMsg msg{.pin = 5, .freq = 1000};
    auto result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status); // Must succeed because the mock automatically responds
}

void test_request_response(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
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
    
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    TEST_ASSERT_EQUAL(1, resp.state);
    TEST_ASSERT_EQUAL(1000, resp.uptime);
}

void test_error_cases(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // Test short frame
    uint8_t shortFrame[] = {EZLinkDfs::START_OF_FRAME, 2};
    serial.injectData(shortFrame, sizeof(shortFrame));
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_INVALID_LEN, result.status);
    
    // Reset for the next test
    serial.reset();
    
    // Register the proto for the CRC test
    comm.registerRequest<SetLedMsg>();
    
    // Test bad CRC
    uint8_t badCrcFrame[] = {
        EZLinkDfs::START_OF_FRAME,
        sizeof(SetLedMsg) + EZLinkDfs::FRAME_OVERHEAD,
        SetLedMsg::id,
        1,
        0xFF, 0xFF  // Bad CRC
    };
    serial.injectData(badCrcFrame, sizeof(badCrcFrame));
    result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_CRC, result.status);
}

void test_buffer_limits(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // Test with a message at the limit (must pass)
    auto result = comm.registerRequest<BorderlineMsg>();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    // Test write buffer full
    comm.registerRequest<SetLedMsg>();  // Must register the proto first !
    serial.setAvailableForWrite(2);   // Simulate almost full buffer
    SetLedMsg msg{.state = 1};
    result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(EZLink::ERR_HW_TX_FAILED, result.status);
}

void test_set_timeout(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // Test custom timeout
    comm.setResponseTimeout(500);
    TEST_ASSERT_EQUAL(500, comm.getResponseTimeout());
    
    // Test invalid timeout (0)
    TEST_ASSERT_FALSE(comm.setResponseTimeout(0));
}

void test_complex_data(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    ComplexMsg msg = {
        .array = {0xDEADBEEF, 0xCAFEBABE, 0x12345678, 0x87654321},
        .coordinates = {-100, 200, -300},
        .flags = 0xAA
    };
    
    comm.registerRequest<ComplexMsg>();
    
    // Test send
    auto result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
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
    EZLink comm(&serial);
    
    // Test message minimal
    auto result = comm.registerRequest<MinimalMsg>();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    // Test with different buffer sizes available
    serial.setAvailableForWrite(sizeof(MinimalMsg) + EZLinkDfs::FRAME_OVERHEAD - 1);
    MinimalMsg msg{.dummy = 42};
    result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(EZLink::ERR_HW_TX_FAILED, result.status);
    
    serial.setAvailableForWrite(sizeof(MinimalMsg) + EZLinkDfs::FRAME_OVERHEAD);
    result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
}

void test_malformed_frames(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // Must register the proto first !
    comm.registerRequest<SetLedMsg>();
    
    // Frame with wrong length
    uint8_t wrongLenFrame[] = {
        EZLinkDfs::START_OF_FRAME,
        sizeof(SetLedMsg) + EZLinkDfs::FRAME_OVERHEAD + 1,  // Too long
        SetLedMsg::id,
        1,
        0,    // Données supplémentaires pour atteindre la taille annoncée
        0, 0  // CRC
    };
    uint16_t crc = EZLink::calculateCRC16(wrongLenFrame, sizeof(wrongLenFrame)-2);
    wrongLenFrame[sizeof(wrongLenFrame)-2] = (uint8_t)(crc >> 8); // MSB
    wrongLenFrame[sizeof(wrongLenFrame)-1] = (uint8_t)(crc & 0xFF); // LSB
    serial.injectData(wrongLenFrame, sizeof(wrongLenFrame));
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_PROTO_MISMATCH, result.status);
    
    serial.reset();  // Important to reset between tests !
}

void test_stress(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    comm.registerRequest<SetLedMsg>();
    
    // Test 1: Back-to-back frames
    uint8_t frame[] = {
        EZLinkDfs::START_OF_FRAME,
        sizeof(SetLedMsg) + EZLinkDfs::FRAME_OVERHEAD,
        SetLedMsg::id,
        1,
        0, 0  // CRC
    };
    uint16_t crc = EZLink::calculateCRC16(frame, sizeof(frame)-2);
    frame[sizeof(frame)-2] = (uint8_t)(crc >> 8); // MSB
    frame[sizeof(frame)-1] = (uint8_t)(crc & 0xFF); // LSB
    
    // Inject two back-to-back frames
    serial.injectData(frame, sizeof(frame));
    serial.injectData(frame, sizeof(frame));
    
    // Both must be processed
    uint8_t captured[EZLinkDfs::MAX_FRAME_SIZE];
    size_t capturedLen;
    
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    // Test 2: Corrupted data between two frames
    uint8_t garbage[] = {0xFF, 0x00, 0xFF};
    serial.injectData(frame, sizeof(frame));
    serial.injectData(garbage, sizeof(garbage));
    serial.injectData(frame, sizeof(frame));
    
    // First valid frame
    result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    // The garbage should cause an error
    result = comm.poll();
    TEST_ASSERT_NOT_EQUAL(EZLink::SUCCESS, result.status);
    
    // The second frame should pass
    result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    // Test 3: Frame with SOF in the data (must pass !)
    uint8_t frameWithSOF[] = {
        EZLinkDfs::START_OF_FRAME,  // SOF initial
        sizeof(SetLedMsg) + EZLinkDfs::FRAME_OVERHEAD,
        SetLedMsg::id,
        EZLinkDfs::START_OF_FRAME,  // SOF in the data - perfectly valid !
        0, 0  // CRC
    };
    crc = EZLink::calculateCRC16(frameWithSOF, sizeof(frameWithSOF)-2);
    frameWithSOF[sizeof(frameWithSOF)-2] = (uint8_t)(crc >> 8); // MSB
    frameWithSOF[sizeof(frameWithSOF)-1] = (uint8_t)(crc & 0xFF); // LSB
    
    printf("\nTEST: Injecting normal frame\n");
    serial.injectData(frame, sizeof(frame));

    printf("\nTEST: Injecting frame with SOF in data\n");
    serial.injectData(frameWithSOF, sizeof(frameWithSOF));

    printf("\nTEST: Injecting normal frame\n");
    serial.injectData(frame, sizeof(frame));
    
    // All frames must pass
    result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
}

void test_mixed_message_types(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // Register different types of messages
    comm.registerRequest<SetLedMsg>();        // MESSAGE
    comm.registerRequest<SetPwmMsg>();        // MESSAGE_ACK
    comm.registerRequest<GetStatusMsg>();     // REQUEST
    comm.registerResponse<StatusResponseMsg>(); // RESPONSE
    
    // Test a mixed sequence
    SetLedMsg led{.state = 1};
    SetPwmMsg pwm{.pin = 2, .freq = 1000};
    GetStatusMsg status{};
    StatusResponseMsg resp{};
    
    printf("\nTesting MESSAGE message...\n");
    auto result = comm.sendMsg(led);        
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    printf("\nTesting MESSAGE_ACK message...\n");
    printf("Original ID: 0x%02X\n", SetPwmMsg::id);
    printf("Expected response ID: 0x%02X\n", SetPwmMsg::id | EZLinkDfs::ID_RESPONSE_BIT);
    result = comm.sendMsgAck(pwm);         
    if (result != EZLink::SUCCESS) {
        printf("Failed with error: %d\n", result.status);
    }
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    // ...
}

void test_response_id_calculation() {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // Enregistrer la paire request/response
    auto result = comm.registerRequest<TestRequestMsg>();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    result = comm.registerResponse<TestResponseMsg>();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    const auto* proto = comm.getProtoStore(TestRequestMsg::id | EZLinkDfs::ID_RESPONSE_BIT);
    TEST_ASSERT_NOT_NULL(proto);
    TEST_ASSERT_EQUAL(TestResponseMsg::id | EZLinkDfs::ID_RESPONSE_BIT, proto->id);
}

void test_response_wrong_id() {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // La requête doit passer
    auto result = comm.registerRequest<TestRequestMsg>();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    
    // La réponse avec mauvais ID doit être rejetée
    result = comm.registerResponse<WrongResponseMsg>();
    TEST_ASSERT_NOT_EQUAL(EZLink::SUCCESS, result.status);
}

// Those tests are intentionally commented out because they MUST fail at compilation
// They demonstrate that the library prevents compilation:
// 1. Registering a response as a request
// 2. Registering a request as a response
//
// void test_response_registered_as_request() {
//     MockSerial serial;
//     serial.reset();
//     EZLink comm(&serial);
//
//     // This test MUST fail at compilation because we cannot
//     // register a RESPONSE as a REQUEST (checked by static_assert)
//     auto result = comm.registerRequest<TestResponseMsg>();
//     TEST_ASSERT_NOT_EQUAL(EZLink::SUCCESS, result.status);
// }
//
// void test_request_registered_as_response() {
//     MockSerial serial;
//     serial.reset();   
//     EZLink comm(&serial);
//
//     // This test MUST fail at compilation because we cannot
//     // register a REQUEST as a RESPONSE (checked by static_assert)
//     auto result = comm.registerResponse<TestRequestMsg>();
//     TEST_ASSERT_NOT_EQUAL(EZLink::SUCCESS, result.status);
// }

void test_busy_receiving(void) {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // Enregistrer les protos nécessaires
    comm.registerRequest<SetLedMsg>();  // MESSAGE
    
    // Préparer un message incomplet pour simuler une réception en cours
    uint8_t partialFrame[] = {
        EZLinkDfs::START_OF_FRAME,  // SOF
        sizeof(SetLedMsg) + EZLinkDfs::FRAME_OVERHEAD,  // LEN
        SetLedMsg::id,  // ID
        // On n'injecte pas tout le message pour simuler une réception partielle
    };
    
    // Injecter le début de frame
    printf("\nTEST: Injecting partial frame:");
    for(size_t i = 0; i < sizeof(partialFrame); i++) {
        printf(" %02X", partialFrame[i]);
    }
    printf("\n");
    serial.injectData(partialFrame, sizeof(partialFrame));
    
    // Premier poll() pour démarrer la capture
    printf("TEST: Starting frame capture...\n");
    auto result = comm.poll();
    printf("TEST: Poll result: status=%d, id=%d\n", result.status, result.id);
    
    // Essayer d'envoyer un message pendant la capture
    printf("TEST: Trying to send while capturing...\n");
    SetLedMsg msg{.state = 1};
    result = comm.sendMsg(msg);  // MESSAGE ne nettoie pas le buffer
    printf("TEST: Send result: status=%d, id=%d\n", result.status, result.id);
    TEST_ASSERT_EQUAL(EZLink::ERR_BUSY_RECEIVING, result.status);
}

void test_request_during_response_wait() {
    MockSerial serial;
    serial.reset();
    EZLink comm(&serial);
    
    // Enregistrer les messages
    comm.registerRequest<GetStatusMsg>();
    comm.registerResponse<StatusResponseMsg>();
    comm.registerRequest<SetLedMsg>();
    
    // Préparer les messages
    GetStatusMsg statusReq{};
    StatusResponseMsg statusResp{};
    
    // Désactiver l'auto-réponse pour contrôler la séquence
    serial.disableAutoResponse();
    
    // 1. Envoyer une requête de status
    printf("\nSending status request...\n");
    auto result = comm.sendRequest(statusReq, statusResp);  // Non bloquant car pas de réponse
    
    // 2. Pendant qu'on attend la réponse, on reçoit une requête LED
    uint8_t incomingRequest[] = {
        EZLinkDfs::START_OF_FRAME,
        sizeof(SetLedMsg) + EZLinkDfs::FRAME_OVERHEAD,
        SetLedMsg::id,
        0x01,  // state = 1
        0, 0   // CRC à calculer
    };
    uint16_t crc = EZLink::calculateCRC16(incomingRequest, sizeof(incomingRequest)-2);
    incomingRequest[sizeof(incomingRequest)-2] = (uint8_t)(crc >> 8); // MSB
    incomingRequest[sizeof(incomingRequest)-1] = (uint8_t)(crc & 0xFF); // LSB
    
    printf("Injecting LED request while waiting for status response...\n");
    serial.injectData(incomingRequest, sizeof(incomingRequest));
    
    // 3. Le poll() doit traiter la requête LED normalement
    result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::SUCCESS, result.status);
    TEST_ASSERT_EQUAL(SetLedMsg::id, result.id);
    
    // 4. Maintenant on reçoit la réponse de status (tardive)
    uint8_t lateResponse[] = {
        EZLinkDfs::START_OF_FRAME,
        sizeof(StatusResponseMsg) + EZLinkDfs::FRAME_OVERHEAD,
        StatusResponseMsg::id | EZLinkDfs::ID_RESPONSE_BIT,
        0x01,  // state = 1
        0x00, 0x00, 0x00, 0x00,  // uptime = 0
        0, 0   // CRC à calculer
    };
    crc = EZLink::calculateCRC16(lateResponse, sizeof(lateResponse)-2);
    lateResponse[sizeof(lateResponse)-2] = (uint8_t)(crc >> 8); // MSB
    lateResponse[sizeof(lateResponse)-1] = (uint8_t)(crc & 0xFF); // LSB
    
    printf("Injecting late status response...\n");
    serial.injectData(lateResponse, sizeof(lateResponse));
    
    // 5. Le poll() doit rejeter la réponse tardive
    result = comm.poll();
    TEST_ASSERT_EQUAL(EZLink::ERR_RCV_UNEXPECTED_RESPONSE, result.status);
}

int main(void) {
    UNITY_BEGIN();
    
    // Tests d'enregistrement et validation des protos
    RUN_TEST(test_register_proto);
    
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
    RUN_TEST(test_busy_receiving);

    // Tests spécifiques Request/Response
    RUN_TEST(test_response_id_calculation);
    RUN_TEST(test_response_wrong_id);
    RUN_TEST(test_request_during_response_wait);

    return UNITY_END();
} 