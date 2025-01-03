#include <Arduino.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

// Communication pins
#define UART1_RX 16
#define UART1_TX 17
#define UART2_RX 18
#define UART2_TX 19

// Serial ports
#define UART1 Serial1
#define UART2 Serial2
#define USB Serial

// Délais et périodes
#define MASTER_DELAY_MS 2000  // 2s entre chaque test
#define SLAVE_DELAY_MS 10     // 10ms entre les polls
#define TEST_TIMEOUT_MS 5000  // Timeout global pour chaque test

// Communication instances
SimpleComm master(&UART1);
SimpleComm slave(&UART2);

// État du test en cours
enum TestState {
    TEST_FIRE_AND_FORGET,
    TEST_ACK_REQUIRED,
    TEST_REQUEST_RESPONSE,
    TEST_DONE
};

volatile TestState currentTest = TEST_FIRE_AND_FORGET;
volatile bool testInProgress = false;

// Statistiques séparées pour master et slave
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
        uint32_t sendErrors = 0;    // Erreurs d'envoi
        uint32_t timeoutErrors = 0; // Timeouts sur ACK/Response
        uint32_t protocolErrors = 0; // Erreurs de protocole
    } masterErrors;
    
    struct {
        uint32_t receiveErrors = 0;  // Erreurs de réception
        uint32_t protocolErrors = 0; // Erreurs de protocole (CRC, format...)
        uint32_t handlerErrors = 0;  // Erreurs dans les handlers
    } slaveErrors;
    
    void print() {
        USB.printf("\nStats Master:\n");
        USB.printf("Messages envoyés: %lu\n", messagesSent);
        USB.printf("ACKs envoyés: %lu\n", acksSent);
        USB.printf("Requêtes envoyées: %lu\n", requestsSent);
        USB.printf("Erreurs: send=%lu, timeout=%lu, proto=%lu\n", 
            masterErrors.sendErrors,
            masterErrors.timeoutErrors,
            masterErrors.protocolErrors);
            
        USB.printf("\nStats Slave:\n"); 
        USB.printf("Messages reçus: %lu\n", messagesReceived);
        USB.printf("ACKs reçus: %lu\n", acksReceived);
        USB.printf("Requêtes reçues: %lu\n", requestsReceived);
        USB.printf("Erreurs: recv=%lu, proto=%lu, handler=%lu\n",
            slaveErrors.receiveErrors,
            slaveErrors.protocolErrors,
            slaveErrors.handlerErrors);
    }
} stats;

// Tâche maître qui exécute les tests séquentiellement
void masterTask(void* parameter) {
    while(true) {
        switch(currentTest) {
            case TEST_FIRE_AND_FORGET: {
                USB.println("\n=== Test FIRE_AND_FORGET ===");
                SetLedMsg msg{.state = 1};
                USB.printf("MASTER: Tentative envoi message LED, état=%d\n", msg.state);
                
                testInProgress = true;
                auto result = master.sendMsg(msg);
                if(result != SimpleComm::SUCCESS) {
                    USB.printf("MASTER: Erreur envoi LED, code=%d\n", result.status);
                    stats.masterErrors.sendErrors++;
                } else {
                    stats.messagesSent++;
                }
                
                // Attendre avant le prochain test
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                currentTest = TEST_ACK_REQUIRED;
                break;
            }
            
            case TEST_ACK_REQUIRED: {
                USB.println("\n=== Test ACK_REQUIRED ===");
                SetPwmMsg msg{.pin = 1, .freq = 1000};
                USB.printf("MASTER: Tentative envoi message PWM, pin=%d, freq=%lu\n", msg.pin, msg.freq);
                
                testInProgress = true;
                auto result = master.sendMsgAck(msg);
                if(result == SimpleComm::ERR_TIMEOUT) {
                    USB.printf("MASTER: Timeout attente ACK\n");
                    stats.masterErrors.timeoutErrors++;
                }
                else if(result != SimpleComm::SUCCESS) {
                    USB.printf("MASTER: Erreur envoi PWM, code=%d\n", result.status);
                    stats.masterErrors.sendErrors++;
                } else {
                    stats.acksSent++;
                }
                
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                currentTest = TEST_REQUEST_RESPONSE;
                break;
            }
            
            case TEST_REQUEST_RESPONSE: {
                USB.println("\n=== Test REQUEST/RESPONSE ===");
                GetStatusMsg req{};
                StatusResponseMsg resp{};
                
                USB.println("MASTER: Tentative envoi requête status");
                
                testInProgress = true;
                auto result = master.sendRequest(req, resp);
                if(result == SimpleComm::ERR_TIMEOUT) {
                    USB.printf("MASTER: Timeout attente réponse\n");
                    stats.masterErrors.timeoutErrors++;
                }
                else if(result != SimpleComm::SUCCESS) {
                    USB.printf("MASTER: Erreur envoi requête, code=%d\n", result.status);
                    stats.masterErrors.sendErrors++;
                } else {
                    stats.requestsSent++;
                    USB.printf("MASTER: Réponse reçue: state=%d, uptime=%lu\n", 
                        resp.state, resp.uptime);
                }
                
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                currentTest = TEST_FIRE_AND_FORGET;
                
                // Afficher les stats après un cycle complet
                stats.print();
                break;
            }
            
            default:
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                break;
        }
    }
}

// Tâche esclave qui poll en continu
void slaveTask(void* parameter) {
    while(true) {
        auto result = slave.poll();
        if(result != SimpleComm::SUCCESS && result != SimpleComm::NOTHING_TO_DO) {
            // Classifier l'erreur selon son type
            switch(result.status) {
                case SimpleComm::ERR_CRC:
                case SimpleComm::ERR_INVALID_SOF:
                case SimpleComm::ERR_INVALID_LEN:
                    stats.slaveErrors.protocolErrors++;
                    USB.printf("SLAVE: Erreur protocole, code=%d\n", result.status);
                    break;
                    
                case SimpleComm::ERR_BUSY_RECEIVING:
                case SimpleComm::ERR_OVERFLOW:
                    stats.slaveErrors.receiveErrors++;
                    USB.printf("SLAVE: Erreur réception, code=%d\n", result.status);
                    break;
                    
                default:
                    stats.slaveErrors.handlerErrors++;
                    USB.printf("SLAVE: Erreur handler, code=%d\n", result.status);
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SLAVE_DELAY_MS));
    }
}

void setup() {
    // Debug port
    USB.begin(115200);
    USB.println("\nDémarrage du test séquentiel...");
    
    // Communication ports
    UART1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
    UART2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);
    
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
        USB.printf("SLAVE: Message LED reçu, état=%d\n", msg.state);
        stats.messagesReceived++;
    });
    
    slave.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        USB.printf("SLAVE: Message PWM reçu, pin=%d, freq=%lu\n", msg.pin, msg.freq);
        stats.acksReceived++;
    });
    
    slave.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        USB.printf("SLAVE: Requête status reçue\n");
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
    
    USB.println("Configuration terminée, début des tests...\n");
}

void loop() {
    // Main loop does nothing, everything happens in tasks
    vTaskDelete(NULL);
} 