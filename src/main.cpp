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
#define MASTER_DELAY_MS 500  // 2s entre chaque test
#define SLAVE_DELAY_MS 10     // 10ms entre les polls

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
            buffer[bufferIndex-1] = '\0';
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
                buffer[bufferIndex-1] = '\0';
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
#ifdef SIMPLECOMM_DEBUG
SimpleComm master(&UART1, DEFAULT_RESPONSE_TIMEOUT_MS, &threadSafeLog, "MASTER");
SimpleComm slave(&UART2, DEFAULT_RESPONSE_TIMEOUT_MS, &threadSafeLog, "SLAVE");
#else
SimpleComm master(&UART1, DEFAULT_RESPONSE_TIMEOUT_MS);
SimpleComm slave(&UART2, DEFAULT_RESPONSE_TIMEOUT_MS);
#endif

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
        uint32_t rxErrors = 0;    // Erreurs de réception
        uint32_t txErrors = 0; // Erreurs d'envoi
    } masterErrors;
    
    struct {
        uint32_t rxErrors = 0;  // Erreurs de réception
        uint32_t txErrors = 0; // Erreurs d'envoi
    } slaveErrors;
    
    void print() {
        logf("\nStats Master:\n"
             "Messages envoyes: %lu\n"
             "ACKs envoyes: %lu\n"
             "Requetes envoyees: %lu\n"
             "Erreurs: TX=%lu, RX=%lu", 
             messagesSent,
             acksSent,
             requestsSent,
             masterErrors.txErrors,
             masterErrors.rxErrors);
            
        logf("\nStats Slave:\n"
             "Messages recus: %lu\n"
             "ACKs recus: %lu\n"
             "Requetes recues: %lu\n"
             "Erreurs: RX=%lu, TX=%lu",
             messagesReceived,
             acksReceived,
             requestsReceived,
             slaveErrors.rxErrors,
             slaveErrors.txErrors);
    }
} txRxStats;

// Tâche maître qui exécute les tests séquentiellement
void masterTask(void* parameter) {
    // Attendre que le slave soit bien démarré
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms de délai initial
    
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
                    txRxStats.masterErrors.txErrors++;
                } else {
                    txRxStats.messagesSent++;
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
                unsigned long startTime = millis();  // Capture du temps avant envoi
                auto result = master.sendMsgAck(msg);
                unsigned long responseTime = millis() - startTime;  // Calcul du temps de réponse
                
                int errorCode = (int)result.status;
                if(result == SimpleComm::ERR_RCV_TIMEOUT) {
                    log("MASTER: Timeout attente ACK");
                    txRxStats.masterErrors.rxErrors++;
                }
                else if (errorCode > SimpleComm::ERR_SND_MIN && errorCode < SimpleComm::ERR_SND_MAX) {
                    logf("MASTER: Erreur envoi PWM, code=%d", errorCode);
                    txRxStats.masterErrors.txErrors++;
                }
                else if (errorCode > SimpleComm::ERR_RCV_MIN && errorCode < SimpleComm::ERR_RCV_MAX) {
                    logf("MASTER: Erreur reception PWM, code=%d", errorCode);
                    txRxStats.masterErrors.txErrors++;
                }
                else if(result != SimpleComm::SUCCESS) {
                    logf("MASTER: Erreur envoi/reception PWM, code=%d", errorCode);
                    txRxStats.masterErrors.txErrors++;
                } else {
                    logf("MASTER: ACK recu pour PWM, pin=%d, freq=%lu (reponse en %lu ms)", 
                         msg.pin, msg.freq, responseTime);
                    txRxStats.acksSent++;
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
                unsigned long startTime = millis();  // Capture du temps avant envoi
                auto result = master.sendRequest(req, resp);
                unsigned long responseTime = millis() - startTime;  // Calcul du temps de réponse
                
                int errorCode = (int)result.status;
                if(result == SimpleComm::ERR_RCV_TIMEOUT) {
                    log("MASTER: Timeout attente reponse");
                    txRxStats.masterErrors.rxErrors++;
                }
                else if (errorCode > SimpleComm::ERR_SND_MIN && errorCode < SimpleComm::ERR_SND_MAX) {
                    logf("MASTER: Erreur envoi requete, code=%d", errorCode);
                    txRxStats.masterErrors.txErrors++;
                }
                else if (errorCode > SimpleComm::ERR_RCV_MIN && errorCode < SimpleComm::ERR_RCV_MAX) {
                    logf("MASTER: Erreur reception reponse, code=%d", errorCode);
                    txRxStats.masterErrors.rxErrors++;
                }
                else if(result != SimpleComm::SUCCESS) {
                    logf("MASTER: Erreur envoi/reception requete, code=%d", errorCode);
                    txRxStats.masterErrors.txErrors++;
                } else {
                    txRxStats.requestsSent++;
                    logf("MASTER: Reponse recue: state=%d, uptime=%lu (reponse en %lu ms)", 
                        resp.state, resp.uptime, responseTime);
                }
                
                vTaskDelay(pdMS_TO_TICKS(MASTER_DELAY_MS));
                currentTest = TEST_FIRE_AND_FORGET;
                
                // Afficher les txRxStats après un cycle complet
                txRxStats.print();
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
            // À ce stade, le message est déjà traité et la réponse déjà envoyée !
            vTaskDelay(1);  // Ce délai n'impacte pas le temps de réponse
        }
        else if(result != SimpleComm::NOTHING_TO_DO) {
            int errorCode = (int)result.status;
            if (errorCode > SimpleComm::ERR_SND_MIN && errorCode < SimpleComm::ERR_SND_MAX) {
                logf("SLAVE: Erreur envoi, code=%d", errorCode);
                txRxStats.slaveErrors.txErrors++;
            }
            else if (errorCode > SimpleComm::ERR_RCV_MIN && errorCode < SimpleComm::ERR_RCV_MAX) {
                logf("SLAVE: Erreur reception, code=%d", errorCode);
                txRxStats.slaveErrors.rxErrors++;
            }
            else {
                logf("SLAVE: Erreur, code=%d", errorCode);
                txRxStats.slaveErrors.rxErrors++; // Par défaut on considère une erreur RX
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

    // Let the UARTs stabilize
    delay(10);
    
    // Initialize communication instances
    master.begin();
    slave.begin();
    
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
        txRxStats.messagesReceived++;
    });
    
    slave.onReceive<SetPwmMsg>([](const SetPwmMsg& msg) {
        logf("SLAVE: Message PWM recu, pin=%d, freq=%lu", msg.pin, msg.freq);
        txRxStats.acksReceived++;
    });
    
    slave.onRequest<GetStatusMsg>([](const GetStatusMsg& req, StatusResponseMsg& resp) {
        logf("SLAVE: Requete status recue");
        txRxStats.requestsReceived++;
        resp.state = 1;
        resp.uptime = millis();
    });
    
    // Create tasks - elles démarreront automatiquement après setup()
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
    
    // Créer la tâche de log
    xTaskCreatePinnedToCore(
        logTask,
        "logTask",
        10000,
        NULL,
        1,
        NULL,
        0  // Core 0 avec masterTask
    );
    
    log("Configuration terminee, debut des tests...\n");
}

void loop() {
    // Main loop does nothing, everything happens in tasks
    vTaskDelete(NULL);
} 