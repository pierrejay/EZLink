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

void testsInit() {
    // Debug port
    Serial.begin(115200);
    
    // Setup hardware
    Serial1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
    Serial2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);
    delay(10);  // Let UARTs stabilize

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
    // Reset all RX buffers
    master.begin();
    slave.begin();
}

void tearDown(void) {
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
    Serial.println("Début test_request_response");
    startSlaveTask(); // Start slave task
    
    slave.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        Serial.println("Handler GetStatus appelé");
        resp.state = 1;
        resp.uptime = 1000;
    });

    // Attendre que la tâche slave soit bien démarrée et stable
    delay(50);  // Ajouter un délai plus long

    Serial.println("Envoi requête");
    GetStatusMsg req;
    StatusResponseMsg resp;
    auto result = master.sendRequest(req, resp);
    
    Serial.println("Requête terminée");
    TEST_ASSERT_EQUAL(SimpleComm::SUCCESS, result.status);
    TEST_ASSERT_EQUAL(1, resp.state);
    TEST_ASSERT_EQUAL(1000, resp.uptime);

    Serial.println("Fin test_request_response");
}

void setup() {
    delay(2000);  // Give monitor time to connect
    testsInit(); // Initialize hardware
    
    UNITY_BEGIN();
    
    RUN_TEST(test_basic_communication);
    RUN_TEST(test_ack_required);
    RUN_TEST(test_request_response);
    
    UNITY_END();
    
    // Cleanup final
    if (slaveTask != NULL) {
        vTaskDelete(slaveTask);
        slaveTask = NULL;
    }
}

void loop() {
    // Nothing to do
} 