#include <Arduino.h>
#include "SimpleComm.h"
#include "SimpleComm_Proto.h"

// Communication pins
#define UART1_RX D5
#define UART1_TX D6
#define UART2_RX D7
#define UART2_TX D8

// Serial ports
#define UART1 Serial1
#define UART2 Serial2
#define USB Serial

// Délais et périodes
#define MASTER_DELAY_MS 2000  // 2s entre chaque test
#define SLAVE_DELAY_MS 10     // 10ms entre les polls
#define TEST_TIMEOUT_MS 5000  // Timeout global pour chaque test

// Queue pour les messages de log
QueueHandle_t logQueue;
#define LOG_QUEUE_SIZE 32
#define MAX_LOG_MSG_SIZE 256

// Structure pour un message de log
struct LogMessage {
    char buffer[MAX_LOG_MSG_SIZE];
};

// Tâche dédiée à l'affichage des logs
void logTask(void* parameter) {
    LogMessage msg;
    while(true) {
        if(xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
            USB.print(msg.buffer);
            USB.flush();
        }
    }
}

// Helpers pour les logs protégés par mutex
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

// Stream personnalisé qui utilise nos fonctions de log thread-safe
class ThreadSafeLogStream : public Stream {
private:
    static constexpr size_t BUFFER_SIZE = 256;
    char buffer[BUFFER_SIZE];
    size_t bufferIndex = 0;

    // Flush le buffer
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
            flushBuffer();
        }
        return 1;
    }

    // Write buffer - optimisé pour écrire tout d'un coup
    size_t write(const uint8_t *data, size_t size) override {
        // Si le nouveau contenu ne rentre pas, on flush d'abord
        if (bufferIndex + size >= BUFFER_SIZE - 1) {
            flushBuffer();
        }
        
        // Si la taille est trop grande pour notre buffer, on envoie directement
        if (size >= BUFFER_SIZE - 1) {
            char temp[BUFFER_SIZE];
            size_t len = min(size, BUFFER_SIZE-1);
            memcpy(temp, data, len);
            temp[len] = '\0';
            log(temp);
            return len;
        }

        // Sinon on accumule dans le buffer
        memcpy(buffer + bufferIndex, data, size);
        bufferIndex += size;
        
        // Si on trouve un \n, on flush
        for (size_t i = 0; i < size; i++) {
            if (data[i] == '\n') {
                flushBuffer();
                break;
            }
        }
        return size;
    }

    // Print formatted - utilise directement logf
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

// Créer une instance globale
ThreadSafeLogStream threadSafeLog;

// Communication instances
SimpleComm master(&UART1, DEFAULT_RESPONSE_TIMEOUT_MS, &threadSafeLog, "MASTER");
SimpleComm slave(&UART2, DEFAULT_RESPONSE_TIMEOUT_MS, &threadSafeLog, "SLAVE");

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
        logf("\nStats Master:\n"
             "Messages envoyes: %lu\n"
             "ACKs envoyes: %lu\n"
             "Requetes envoyees: %lu\n"
             "Erreurs: send=%lu, timeout=%lu, proto=%lu", 
             messagesSent,
             acksSent,
             requestsSent,
             masterErrors.sendErrors,
             masterErrors.timeoutErrors,
             masterErrors.protocolErrors);
            
        logf("\nStats Slave:\n"
             "Messages recus: %lu\n"
             "ACKs recus: %lu\n"
             "Requetes recues: %lu\n"
             "Erreurs: recv=%lu, proto=%lu, handler=%lu",
             messagesReceived,
             acksReceived,
             requestsReceived,
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
                log("\n=== Test FIRE_AND_FORGET ===");
                SetLedMsg msg{.state = 1};
                logf("MASTER: Tentative envoi message LED, etat=%d", msg.state);
                
                testInProgress = true;
                auto result = master.sendMsg(msg);
                if(result != SimpleComm::SUCCESS) {
                    logf("MASTER: Erreur envoi LED, code=%d", result.status);
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
                log("\n=== Test ACK_REQUIRED ===");
                SetPwmMsg msg{.pin = 1, .freq = 1000};
                logf("MASTER: Tentative envoi message PWM, pin=%d, freq=%lu", msg.pin, msg.freq);
                
                testInProgress = true;
                auto result = master.sendMsgAck(msg);
                if(result == SimpleComm::ERR_TIMEOUT) {
                    log("MASTER: Timeout attente ACK");
                    stats.masterErrors.timeoutErrors++;
                }
                else if(result != SimpleComm::SUCCESS) {
                    logf("MASTER: Erreur envoi PWM, code=%d", result.status);
                    stats.masterErrors.sendErrors++;
                } else {
                    logf("MASTER: ACK recu pour PWM, pin=%d, freq=%lu", msg.pin, msg.freq);
                    stats.acksSent++;
                }
                
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                currentTest = TEST_REQUEST_RESPONSE;
                break;
            }
            
            case TEST_REQUEST_RESPONSE: {
                log("\n=== Test REQUEST/RESPONSE ===");
                GetStatusMsg req{};
                StatusResponseMsg resp{};
                
                log("MASTER: Tentative envoi requete status");
                
                testInProgress = true;
                auto result = master.sendRequest(req, resp);
                if(result == SimpleComm::ERR_TIMEOUT) {
                    log("MASTER: Timeout attente reponse");
                    stats.masterErrors.timeoutErrors++;
                }
                else if(result != SimpleComm::SUCCESS) {
                    logf("MASTER: Erreur envoi requete, code=%d", result.status);
                    stats.masterErrors.sendErrors++;
                } else {
                    stats.requestsSent++;
                    logf("MASTER: Reponse recue: state=%d, uptime=%lu", 
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
        if(result == SimpleComm::SUCCESS) {
            // On a traité un message avec succès
            // Donnons un peu de temps au système
            vTaskDelay(1);  // Juste 1 tick pour laisser respirer le système
        }
        else if(result != SimpleComm::NOTHING_TO_DO) {
            // Classifier l'erreur selon son type
            switch(result.status) {
                case SimpleComm::ERR_CRC:
                case SimpleComm::ERR_INVALID_SOF:
                case SimpleComm::ERR_INVALID_LEN:
                    stats.slaveErrors.protocolErrors++;
                    logf("SLAVE: Erreur protocole, code=%d", result.status);
                    break;
                    
                case SimpleComm::ERR_BUSY_RECEIVING:
                case SimpleComm::ERR_OVERFLOW:
                    stats.slaveErrors.receiveErrors++;
                    logf("SLAVE: Erreur reception, code=%d", result.status);
                    break;
                    
                default:
                    stats.slaveErrors.handlerErrors++;
                    logf("SLAVE: Erreur handler, code=%d", result.status);
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SLAVE_DELAY_MS));
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

    // Créer la queue de logs
    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));
    if(logQueue == NULL) {
        while(1) {
            USB.println("Erreur creation de la queue");
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(100);
        }
    }

    // Debug port
    USB.begin(115200);
    log("\nDemarrage du test sequentiel...");
    
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
        logf("SLAVE: Message LED recu, etat=%d", msg.state);
        stats.messagesReceived++;
    });
    
    slave.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        logf("SLAVE: Message PWM recu, pin=%d, freq=%lu", msg.pin, msg.freq);
        stats.acksReceived++;
    });
    
    slave.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        logf("SLAVE: Requete status recue");
        stats.requestsReceived++;
        resp.state = 1;
        resp.uptime = millis();
    });
    
    // Create tasks - elles démarreront automatiquement après setup()
    xTaskCreatePinnedToCore(
        masterTask,
        "masterTask",
        10000,
        NULL,
        1,  // Priorité normale
        NULL,
        0  // Core 0
    );
    
    xTaskCreatePinnedToCore(
        slaveTask,
        "slaveTask",
        10000,
        NULL,
        1,  // Priorité normale
        NULL,
        1  // Core 1
    );
    
    // Créer la tâche de log
    xTaskCreatePinnedToCore(
        logTask,
        "logTask",
        10000,
        NULL,
        2,  // Priorité plus élevée pour traiter les logs en priorité
        NULL,
        0  // Core 0 avec masterTask
    );
    
    log("Configuration terminee, debut des tests...\n");
}

void loop() {
    // Main loop does nothing, everything happens in tasks
    vTaskDelete(NULL);
} 