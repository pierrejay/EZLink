#include <unity.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

// Messages de test
struct BorderlineMsg {
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "BORDERLINE";
    static constexpr uint8_t fc = 10;
    uint8_t data[SimpleComm::MAX_FRAME_SIZE - SimpleComm::FRAME_OVERHEAD];
};

struct DuplicateNameMsg {
    static constexpr uint8_t fc = 20;
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "SET_LED";  // Même nom que SetLedMsg
    uint8_t value;
};

struct SameFcMsg {
    static constexpr uint8_t fc = SetLedMsg::fc;  // Même FC
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "DIFFERENT";
    uint8_t value;
};

// Non utilisé (test bloqué à la compilation => OK)
struct NullNameMsg {
    static constexpr uint8_t fc = 30;
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = nullptr;
    uint8_t value;
};

// Non utilisé (test bloqué à la compilation => OK)
struct EmptyNameMsg {
    static constexpr uint8_t fc = 31;
    static constexpr ProtoType type = ProtoType::FIRE_AND_FORGET;
    static constexpr const char* name = "";  // Chaîne vide
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
    uint8_t dummy;  // Message minimal d'un byte
};



// Mock pour les tests
class MockSerial : public HardwareSerial {
private:
    static constexpr size_t BUFFER_SIZE = 256;
    static constexpr uint32_t BYTE_READ_TIME_MS = 1;
    
    uint8_t txBuffer[BUFFER_SIZE];
    uint8_t rxBuffer[BUFFER_SIZE];
    size_t txCount;
    size_t rxCount;
    size_t rxIndex;
    
    // Pour la simulation de réponses
    uint8_t lastMessageFC;
    uint8_t autoResponseBuffer[BUFFER_SIZE];
    size_t autoResponseSize;
    int availableForWriteValue;

    void prepareAutoResponse() {
        // On ne prépare pas de réponse auto pour les messages FIRE_AND_FORGET
        if (lastMessageFC == SetLedMsg::fc) {
            return;  // SetLedMsg est FIRE_AND_FORGET
        }
        
        if (lastMessageFC == SetPwmMsg::fc) {
            // Pour ACK_REQUIRED, on renvoie exactement le même message
            memcpy(autoResponseBuffer, txBuffer, txCount);
            autoResponseSize = txCount;
        }
        else if (lastMessageFC == GetStatusMsg::fc) {
            // Pour REQUEST/RESPONSE, on construit une réponse
            StatusResponseMsg resp{.state = 1, .uptime = 1000};
            
            // Construire la frame de réponse
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
        // Stocker le message
        size_t written = 0;
        while (written < size && txCount < BUFFER_SIZE) {
            txBuffer[txCount++] = buffer[written++];
        }
        
        // Extraire le FC (3ème byte de la frame)
        if (size >= 3) {
            lastMessageFC = buffer[2];
            
            // Ne préparer une réponse que pour les messages qui en attendent une
            if (lastMessageFC == SetPwmMsg::fc || lastMessageFC == GetStatusMsg::fc) {
                prepareAutoResponse();
                
                // Copier la réponse auto dans le buffer de réception
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
        TimeManager::advanceTime(BYTE_READ_TIME_MS); // Simule le temps de lecture
        return rxIndex < rxCount ? rxBuffer[rxIndex++] : -1; 
    }

    int peek() override {
        TimeManager::advanceTime(BYTE_READ_TIME_MS); // Simule le temps de lecture
        return rxIndex < rxCount ? rxBuffer[rxIndex] : -1;
    }

    // Méthodes spécifiques pour les tests
    void reset() {
        // Reset des buffers
        memset(txBuffer, 0, BUFFER_SIZE);
        memset(rxBuffer, 0, BUFFER_SIZE);
        memset(autoResponseBuffer, 0, BUFFER_SIZE);
        
        // Reset des compteurs
        txCount = 0;
        rxCount = 0;
        rxIndex = 0;
        
        // Reset des états
        lastMessageFC = 0;
        autoResponseSize = 0;
        availableForWriteValue = 255;
        
        // Reset du temps simulé
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

    // Retourne toujours une grande valeur pour simuler un buffer d'écriture vide
    int availableForWrite() override {
        return availableForWriteValue;
    }

    bool setAvailableForWrite(int value) override {
        availableForWriteValue = value;
        return true;
    }
};

void setUp(void) {
    TimeManager::reset();  // Reset le temps au début de chaque test
}

void tearDown(void) {
    // Cleanup après chaque test
}

void test_register_proto(void) {
    MockSerial serial;
    SimpleComm comm(&serial);

    // Test 1: Enregistrement normal
    auto result = comm.registerProto<SetLedMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);

    // Test 2: Double enregistrement
    result = comm.registerProto<SetLedMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_FC_ALREADY_REGISTERED, result.status);
}

void test_send_msg(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Enregistrer le proto d'abord
    comm.registerProto<SetLedMsg>();
    
    // Test envoi normal
    SetLedMsg msg{.state = 1};
    auto result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Test envoi sans proto enregistré
    SetPwmMsg pwm{.pin = 1, .freq = 1000};
    result = comm.sendMsgAck(pwm);  // Utiliser sendMsgAck pour ACK_REQUIRED
    TEST_ASSERT_EQUAL(SimpleComm::ERR_INVALID_FC, result.status);
}

void test_on_receive(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    bool handlerCalled = false;
    
    // Enregistrer le proto et le handler
    comm.registerProto<SetLedMsg>();
    comm.onReceive<SetLedMsg>([&handlerCalled](const SetLedMsg& msg) {
        handlerCalled = true;
        TEST_ASSERT_EQUAL(1, msg.state);
    });
    
    // Construire une frame valide
    uint8_t frame[] = {
        SimpleComm::START_OF_FRAME,  // SOF
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,  // LEN
        SetLedMsg::fc,               // FC
        1,                          // state = 1
        0                           // CRC (sera calculé ci-dessous)
    };
    
    // Calculer le CRC sur toute la frame sauf le dernier byte
    frame[sizeof(frame)-1] = SimpleComm::calculateCRC(frame, sizeof(frame)-1);
    
    // Injecter la frame
    serial.injectData(frame, sizeof(frame));
    
    // Traiter la frame
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    TEST_ASSERT_TRUE(handlerCalled);
}

void test_timeout(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Injecter un début de frame incomplet
    uint8_t start = SimpleComm::START_OF_FRAME;
    serial.injectData(&start, 1);
    
    // Le timeout devrait se produire naturellement car chaque lecture 
    // de byte incrémente le temps
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_TIMEOUT, result.status);
}

void test_send_msg_with_ack(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Enregistrer le proto
    comm.registerProto<SetPwmMsg>();
    
    // Test avec auto-réponse du mock
    SetPwmMsg msg{.pin = 5, .freq = 1000};
    auto result = comm.sendMsgAck(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status); // Doit réussir car le mock répond automatiquement
}

void test_request_response(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Enregistrer les protos
    comm.registerProto<GetStatusMsg>();
    comm.registerProto<StatusResponseMsg>();
    
    // Configurer le handler de requête qui va générer la réponse
    comm.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        resp.state = 1;
        resp.uptime = 1000;
    });
    
    // Envoyer une requête
    GetStatusMsg req{};
    StatusResponseMsg resp{};
    
    // La réponse sera automatiquement générée par le mock
    auto result = comm.sendRequest(req, resp);
    
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    TEST_ASSERT_EQUAL(1, resp.state);
    TEST_ASSERT_EQUAL(1000, resp.uptime);
}

void test_error_cases(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Test frame trop courte
    uint8_t shortFrame[] = {SimpleComm::START_OF_FRAME, 2};
    serial.injectData(shortFrame, sizeof(shortFrame));
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_INVALID_LEN, result.status);
    
    // Reset pour le prochain test
    serial.reset();
    
    // Enregistrer le proto pour le test CRC
    comm.registerProto<SetLedMsg>();
    
    // Test mauvais CRC
    uint8_t badCrcFrame[] = {
        SimpleComm::START_OF_FRAME,
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,
        SetLedMsg::fc,
        1,
        0xFF  // Mauvais CRC
    };
    serial.injectData(badCrcFrame, sizeof(badCrcFrame));
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_CRC, result.status);
}

void test_buffer_limits(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Test avec un message à la limite (doit passer)
    auto result = comm.registerProto<BorderlineMsg>();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Test buffer d'écriture plein
    comm.registerProto<SetLedMsg>();  // Il faut d'abord enregistrer le proto !
    serial.setAvailableForWrite(2);   // Simuler buffer presque plein
    SetLedMsg msg{.state = 1};
    result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_OVERFLOW, result.status);
}

void test_timeouts(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Test timeout personnalisé
    comm.setInterbyteTimeout(100);
    comm.setResponseTimeout(500);
    TEST_ASSERT_EQUAL(100, comm.getInterbyteTimeout());
    TEST_ASSERT_EQUAL(500, comm.getResponseTimeout());
    
    // Test timeout invalide (0)
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
    
    // Tenter d'utiliser un message avec le même FC mais un nom différent
    SameFcMsg msg{.value = 1};
    auto result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_PROTO_MISMATCH, result.status);
}

void test_handler_wrong_name(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    comm.registerProto<SetLedMsg>();
    
    // Tenter d'enregistrer un handler pour un message avec le même FC mais nom différent
    auto result = comm.onReceive<SameFcMsg>([](const SameFcMsg&) {});
    TEST_ASSERT_EQUAL(SimpleComm::ERR_PROTO_MISMATCH, result.status);
}

void test_invalid_names(void) {
    // ... supprimé car impossible à tester (bloqué à la compilation => OK)
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
    
    // Test envoi
    auto result = comm.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Vérifier que les données sont correctement transmises
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
    
    // Test avec différentes tailles de buffer disponible
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
    
    // Il faut d'abord enregistrer le proto !
    comm.registerProto<SetLedMsg>();
    
    // Frame avec longueur incorrecte
    uint8_t wrongLenFrame[] = {
        SimpleComm::START_OF_FRAME,
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD + 1,  // Longueur trop grande
        SetLedMsg::fc,
        1,
        0  // CRC
    };
    wrongLenFrame[4] = SimpleComm::calculateCRC(wrongLenFrame, 4);
    serial.injectData(wrongLenFrame, sizeof(wrongLenFrame));
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_PROTO_MISMATCH, result.status);
    
    serial.reset();  // Important de reset entre les tests !
    
    // Frame tronquée
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
    
    // Test 1: Messages collés
    uint8_t frame[] = {
        SimpleComm::START_OF_FRAME,
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,
        SetLedMsg::fc,
        1,
        0  // CRC
    };
    frame[4] = SimpleComm::calculateCRC(frame, 4);
    
    // Injecter deux frames collées
    serial.injectData(frame, sizeof(frame));
    serial.injectData(frame, sizeof(frame));
    
    // Les deux doivent être traitées
    auto result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Test 2: Données corrompues entre deux frames
    uint8_t garbage[] = {0xFF, 0x00, 0xFF};
    serial.injectData(frame, sizeof(frame));
    serial.injectData(garbage, sizeof(garbage));
    serial.injectData(frame, sizeof(frame));
    
    // Première frame valide
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Le garbage devrait provoquer une erreur
    result = comm.poll();
    TEST_ASSERT_NOT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // La deuxième frame devrait passer
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Test 3: Frame avec SOF dans les données (doit passer !)
    uint8_t frameWithSOF[] = {
        SimpleComm::START_OF_FRAME,  // SOF initial
        sizeof(SetLedMsg) + SimpleComm::FRAME_OVERHEAD,
        SetLedMsg::fc,
        SimpleComm::START_OF_FRAME,  // SOF dans les données - parfaitement valide !
        0  // CRC
    };
    frameWithSOF[4] = SimpleComm::calculateCRC(frameWithSOF, 4);
    
    serial.injectData(frame, sizeof(frame));          // Frame normale
    serial.injectData(frameWithSOF, sizeof(frameWithSOF));  // Frame avec SOF dans les données
    serial.injectData(frame, sizeof(frame));          // Frame normale
    
    // Toutes les frames doivent passer
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);  // La frame avec SOF doit passer !
    result = comm.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
}

void test_mixed_message_types(void) {
    MockSerial serial;
    SimpleComm comm(&serial);
    
    // Enregistrer différents types de messages
    comm.registerProto<SetLedMsg>();        // FIRE_AND_FORGET
    comm.registerProto<SetPwmMsg>();        // ACK_REQUIRED
    comm.registerProto<GetStatusMsg>();     // REQUEST
    comm.registerProto<StatusResponseMsg>(); // RESPONSE
    
    // Tester une séquence mixte
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