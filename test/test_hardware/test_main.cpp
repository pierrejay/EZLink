#include <Arduino.h>
#include <unity.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

// Communication pins
#define UART1_RX D5
#define UART1_TX D6
#define UART2_RX D7
#define UART2_TX D8

// Délai de poll pour le slave
#define SLAVE_DELAY_MS 10

SimpleComm master(&Serial1, 500, &Serial, "MASTER");
SimpleComm slave(&Serial2, 500, &Serial, "SLAVE");

TaskHandle_t slaveTask = NULL;

// Message avec payload maximum
struct MaxPayloadMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE;
    static constexpr const char* name = "MAX_PAYLOAD";
    static constexpr uint8_t id = 10;
    uint8_t data[SimpleCommDfs::MAX_FRAME_SIZE - SimpleCommDfs::FRAME_OVERHEAD];  // Taille maximale possible
} __attribute__((packed));

// Message avec payload vide
struct EmptyMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE;
    static constexpr const char* name = "EMPTY";
    static constexpr uint8_t id = 11;
    // Pas de données !
} __attribute__((packed));

// Message pour test en rafale
struct BurstMsg {
    static constexpr ProtoType type = ProtoType::MESSAGE_ACK;  // On utilise ACK pour vérifier la réception
    static constexpr const char* name = "BURST";
    static constexpr uint8_t id = 12;
    uint32_t sequence;  // Numéro de séquence pour vérifier l'ordre
} __attribute__((packed));

// Messages pour test bidirectionnel
struct PongMsg {
    static constexpr ProtoType type = ProtoType::RESPONSE;
    static constexpr const char* name = "PONG";
    static constexpr uint8_t id = 13;
    uint32_t echo_timestamp;
    uint32_t response_timestamp;
} __attribute__((packed));

struct PingMsg {
    static constexpr ProtoType type = ProtoType::REQUEST;
    static constexpr const char* name = "PING";
    static constexpr uint8_t id = 13;
    uint32_t timestamp;
    using ResponseType = PongMsg;
} __attribute__((packed));

// Fonction utilitaire pour envoyer une frame personnalisée
void sendCustomFrame(HardwareSerial* serial, uint8_t sof, uint8_t len, uint8_t id, const std::vector<uint8_t>& payload, uint16_t crc = 0) {
    // Construire la frame sans CRC
    std::vector<uint8_t> frame;
    frame.push_back(sof);
    frame.push_back(len);
    frame.push_back(id);
    frame.insert(frame.end(), payload.begin(), payload.end());

    // Si crc == 0, on le calcule sur la frame complète
    if(crc == 0) {
        crc = SimpleComm::calculateCRC16(frame.data(), frame.size());
    }
    
    // Ajouter le CRC
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
            Serial.println("Tâche slave démarrée");
            while(true) {
                auto result = slave.poll();
                if(result == SimpleComm::SUCCESS) {
                    Serial.println("Slave: message traité");
                }
                else if(result != SimpleComm::NOTHING_TO_DO) {
                    Serial.printf("Slave: erreur poll %d\n", result.status);
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
    
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    // Give some time for reception
    delay(50);
    
    // Process message
    result = slave.poll();
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
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
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
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
    
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    TEST_ASSERT_EQUAL(1, resp.state);
    TEST_ASSERT_EQUAL(1000, resp.uptime);

}

// Les tests de robustesse
void test_max_payload() {
    Serial.println("Début test_max_payload");
    
    // Enregistrer les messages pour ce test
    master.registerRequest<MaxPayloadMsg>();
    slave.registerRequest<MaxPayloadMsg>();
    
    startSlaveTask();  // Démarrer la tâche slave
    
    bool messageReceived = false;
    MaxPayloadMsg msg;
    
    // Remplir le payload avec un pattern reconnaissable
    for(size_t i = 0; i < sizeof(msg.data); i++) {
        msg.data[i] = i & 0xFF;
    }
    
    // Setup handler
    slave.onReceive<MaxPayloadMsg>([&messageReceived, &msg](const MaxPayloadMsg& received) {
        messageReceived = true;
        // Vérifier que le pattern est intact
        for(size_t i = 0; i < sizeof(msg.data); i++) {
            TEST_ASSERT_EQUAL(i & 0xFF, received.data[i]);
        }
    });
    
    auto result = master.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    delay(50);  // Laisser le temps au slave de traiter
    TEST_ASSERT_TRUE(messageReceived);
}

void test_empty_payload() {
    Serial.println("Début test_empty_payload");
    
    // Enregistrer les messages pour ce test
    master.registerRequest<EmptyMsg>();
    slave.registerRequest<EmptyMsg>();
    
    startSlaveTask();
    
    bool messageReceived = false;
    EmptyMsg msg;
    
    slave.onReceive<EmptyMsg>([&messageReceived](const EmptyMsg& received) {
        messageReceived = true;
    });
    
    auto result = master.sendMsg(msg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    
    delay(50);
    TEST_ASSERT_TRUE(messageReceived);
}

void test_burst_messages() {
    Serial.println("Début test_burst_messages");
    
    // Enregistrer les messages pour ce test
    master.registerRequest<BurstMsg>();
    slave.registerRequest<BurstMsg>();
    
    startSlaveTask();
    
    const int NUM_MESSAGES = 10;
    int receivedCount = 0;
    
    slave.onReceive<BurstMsg>([&receivedCount](const BurstMsg& msg) {
        TEST_ASSERT_EQUAL(receivedCount, msg.sequence);
        receivedCount++;
    });
    
    // Envoyer les messages en rafale
    for(int i = 0; i < NUM_MESSAGES; i++) {
        BurstMsg msg;
        msg.sequence = i;
        auto result = master.sendMsgAck(msg);  // On utilise ACK pour s'assurer de la réception
        TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    }
    
    delay(100);  // Laisser le temps au traitement
    TEST_ASSERT_EQUAL(NUM_MESSAGES, receivedCount);
}

void test_bidirectional() {
    Serial.println("Début test_bidirectional");
    
    // Enregistrer les messages pour ce test - attention à l'ordre !
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
        // Envoyer un ping
        PingMsg ping;
        ping.timestamp = millis();
        PongMsg pong;
        
        auto result = master.sendRequest(ping, pong);
        TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
        uint32_t request_time = ping.timestamp;
        uint32_t response_time = pong.response_timestamp;
        TEST_ASSERT_GREATER_THAN(0, request_time);
        TEST_ASSERT_GREATER_THAN(request_time, response_time);
        
        delay(10);  // Petit délai entre les pings
    }
}

void test_response_timeout() {
    Serial.println("Début test_response_timeout");
    
    // Ne pas démarrer la tâche slave pour simuler un timeout
    GetStatusMsg req;
    StatusResponseMsg resp;
    auto result = master.sendRequest(req, resp);
    
    TEST_ASSERT_EQUAL(SimpleComm::ERR_RCV_TIMEOUT, result.status);
}

void test_corrupted_crc() {
    Serial.println("Début test_corrupted_crc");
    
    // Le master envoie un message avec CRC invalide au slave
    std::vector<uint8_t> data = {0x01};  // Exemple de données
    uint16_t invalidCRC = 0xFFFF;  // CRC invalide
    sendCustomFrame(&Serial1, SimpleCommDfs::START_OF_FRAME, data.size() + SimpleCommDfs::FRAME_OVERHEAD, data.size(), data, invalidCRC);
    
    // Le slave doit détecter l'erreur
    delay(20);  // Laisser le temps à la transmission UART
    auto result = slave.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_RCV_CRC, result.status);
}

void test_truncated_message() {
    Serial.println("Début test_truncated_message");
    
    // Enregistrer le message pour le test valide
    master.registerRequest<SetLedMsg>();
    slave.registerRequest<SetLedMsg>();
    
    // 1. Envoyer un message tronqué : on annonce une longueur de 12 (0x0C) mais on envoie moins
    std::vector<uint8_t> truncated_data = {0x01, 0x42};  // ID + payload
    sendCustomFrame(&Serial1, SimpleCommDfs::START_OF_FRAME, 0x0C, 0x01, truncated_data, 0);
    
    // 2. Envoyer un message valide (SetLedMsg)
    SetLedMsg msg{.state = 1};
    master.sendMsg(msg);
    
    delay(20);
    
    // 3. Premier poll() doit détecter l'erreur CRC (car il lira des données du message suivant)
    auto result = slave.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_RCV_CRC, result.status);
    
    // 4. Continuer à poller jusqu'à avoir traité tous les messages
    bool success_found = false;
    while(true) {
        result = slave.poll();
        if(result.status == SimpleComm::SUCCESS) {
            success_found = true;
        }
        else if(result.status == SimpleComm::NOTHING_TO_DO) {
            break;
        }
        // On ignore les autres erreurs potentielles (invalid SOF etc)
    }
    
    // On doit avoir trouvé au moins un message valide
    TEST_ASSERT_TRUE(success_found);
}

void test_invalid_sof() {
    Serial.println("Début test_invalid_sof");
    
    // Envoyer un message avec un SOF incorrect
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};  // Exemple de données
    uint16_t crc = SimpleComm::calculateCRC16(data.data(), data.size());
    sendCustomFrame(&Serial1, 0x55, data.size() + SimpleCommDfs::FRAME_OVERHEAD, data.size(), data, crc);  // SOF incorrect
    
    // Poller pour vérifier l'erreur
    delay(20);  // Laisser le temps à la transmission UART
    auto result = slave.poll();
    bool got_invalid_sof = (result.status == SimpleComm::ERR_RCV_INVALID_SOF);
    TEST_ASSERT_EQUAL(SimpleComm::ERR_RCV_INVALID_SOF, result.status);
}

void test_invalid_length() {
    Serial.println("Début test_invalid_length");
    
    // Envoyer un message avec une longueur incorrecte
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};  // Exemple de données
    uint16_t crc = SimpleComm::calculateCRC16(data.data(), data.size());
    sendCustomFrame(&Serial1, SimpleCommDfs::START_OF_FRAME, 255, data.size(), data, crc);  // Longueur incorrecte
    
    // Poller pour vérifier l'erreur
    delay(20);  // Laisser le temps à la transmission UART
    auto result = slave.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_RCV_INVALID_LEN, result.status);
}

void test_unexpected_response() {
    Serial.println("Début test_unexpected_response");
    
    // Ici c'est différent : le slave envoie une réponse inattendue au master
    std::vector<uint8_t> payload = {0x42};  // Données quelconques
    sendCustomFrame(&Serial2, SimpleCommDfs::START_OF_FRAME, 6, 0x81, payload, 0);  // ID avec bit de réponse
    
    // Le master doit détecter l'erreur
    delay(20);  // Laisser le temps à la transmission UART
    auto result = master.poll();
    TEST_ASSERT_EQUAL(SimpleComm::ERR_RCV_UNEXPECTED_RESPONSE, result.status);
}

void test_oversized_message() {
    Serial.println("Début test_oversized_message");
    
    // Enregistrer le message pour le test valide
    master.registerRequest<SetLedMsg>();
    slave.registerRequest<SetLedMsg>();
    
    // 1. Envoyer un message plus long que sa LEN indiquée
    std::vector<uint8_t> oversized_data = {0x01, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48};  // 8 bytes de données
    sendCustomFrame(&Serial1, SimpleCommDfs::START_OF_FRAME, 0x05, 0x01, oversized_data, 0);  // LEN = 5 mais données plus longues
    
    // 2. Envoyer un message valide (SetLedMsg)
    SetLedMsg msg{.state = 1};
    master.sendMsg(msg);
    
    delay(20);
    
    // 3. Premier poll() doit détecter l'erreur CRC
    auto first_result = slave.poll();
    bool got_crc_error = (first_result.status == SimpleComm::ERR_RCV_CRC);

    // 4. Continuer à poller jusqu'à avoir traité tous les messages
    bool success_found = false;
    while(true) {
        auto result = slave.poll();
        if(result.status == SimpleComm::SUCCESS) {
            success_found = true;
        }
        else if(result.status == SimpleComm::NOTHING_TO_DO) {
            break;
        }
        // yield();
        // On peut avoir plusieurs erreurs CRC avant de trouver le message valide
    }
    
    // Faire les assertions APRÈS la boucle
    TEST_ASSERT_TRUE(got_crc_error);
    TEST_ASSERT_TRUE(success_found);
}

// Buffer circulaire simple pour simuler un UART hardware
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
    TaskHandle_t callbackSlaveTask = NULL;  // Handle local au test

    // Créer les instances master et slave
    SimpleComm master(
        [](const uint8_t* data, size_t len) {
            return masterToSlave.write(data, len);
        },
        [](uint8_t* data, size_t maxLen) {
            return slaveToMaster.read(data, maxLen);
        }
    );

    SimpleComm slave(
        [](const uint8_t* data, size_t len) {
            return slaveToMaster.write(data, len);
        },
        [](uint8_t* data, size_t maxLen) {
            return masterToSlave.read(data, maxLen);
        }
    );

    // Enregistrer les messages sur les deux instances
    master.registerRequest<SetLedMsg>();
    master.registerRequest<SetPwmMsg>();
    master.registerRequest<GetStatusMsg>();
    master.registerResponse<StatusResponseMsg>();

    slave.registerRequest<SetLedMsg>();
    slave.registerRequest<SetPwmMsg>();
    slave.registerRequest<GetStatusMsg>();
    slave.registerResponse<StatusResponseMsg>();

    // Configurer les handlers sur le slave
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

    // Démarrer la tâche slave spécifique à ce test
    xTaskCreatePinnedToCore(
        [](void* parameter) {
            SimpleComm* slaveInstance = (SimpleComm*)parameter;
            while(true) {
                auto result = slaveInstance->poll();
                if(result == SimpleComm::SUCCESS) {
                    Serial.println("Callback slave: message traité");
                }
                else if(result != SimpleComm::NOTHING_TO_DO) {
                    Serial.printf("Callback slave: erreur poll %d\n", result.status);
                }
                vTaskDelay(pdMS_TO_TICKS(SLAVE_DELAY_MS));
            }
        },
        "callbackSlaveTask",
        10000,
        &slave,  // Passe l'instance locale
        1,
        &callbackSlaveTask,
        1
    );

    // Test 1: MESSAGE simple
    SetLedMsg ledMsg{.state = 1};
    auto resultSendLed = master.sendMsg(ledMsg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, resultSendLed.status);

    delay(50);  // Laisser le temps au slave de traiter
    TEST_ASSERT_TRUE(messageLedReceived);

    // Test 2: MESSAGE_ACK
    SetPwmMsg pwmMsg{.pin = 1, .freq = 1000};
    auto resultSendPwm = master.sendMsgAck(pwmMsg);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, resultSendPwm.status);

    // Test 3: REQUEST/RESPONSE
    GetStatusMsg req;
    StatusResponseMsg resp;
    auto resultRequest = master.sendRequest(req, resp);
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, resultRequest.status);
    TEST_ASSERT_EQUAL(1, resp.state);
    TEST_ASSERT_EQUAL(1000, resp.uptime);

    // Cleanup spécifique à ce test
    if (callbackSlaveTask != NULL) {
        vTaskDelete(callbackSlaveTask);
        callbackSlaveTask = NULL;
    }
}

void setup() {
    delay(2000);  // Give monitor time to connect
    testsInit(); // Initialize hardware
    
    UNITY_BEGIN();
    
    // Tests de base
    RUN_TEST(test_basic_communication);
    RUN_TEST(test_ack_required);
    RUN_TEST(test_request_response);
    
    // Tests de robustesse
    RUN_TEST(test_max_payload);
    RUN_TEST(test_empty_payload);
    RUN_TEST(test_burst_messages);
    RUN_TEST(test_bidirectional);
    
    // Tests avancés
    RUN_TEST(test_response_timeout);
    RUN_TEST(test_corrupted_crc);
    RUN_TEST(test_truncated_message);
    RUN_TEST(test_invalid_sof);
    RUN_TEST(test_invalid_length);
    RUN_TEST(test_unexpected_response);
    RUN_TEST(test_oversized_message);

    // Tests avec callbacks custom
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