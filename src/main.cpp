#include <Arduino.h>

// Désactiver le CRC pour le debug
#define DISABLE_CRC
// Ajout du build flag pour le mode binaire strict
#define BINARY_ONLY_MODE

#define LED_PIN PA4
#define SERIAL_TX PA9 
#define SERIAL_RX PA10
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 10  // Timeout en millisecondes
#define CMD_BUFFER_SIZE 32

// Nombre de sorties
#define NUM_OUTPUTS 4

// Constantes PWM
#define MIN_FREQ 1
#define MAX_FREQ 10000
#define MAX_DUTY 100

// Ajout des définitions pour la pin d'activation
#define ENABLE_PIN PA3

// Réduction des messages de debug et optimisation des chaînes
#define DBG_CMD "dbg:cmd="    // Plus court que "DBG: Got command: "
#define ERR_CMD "err:cmd\n" // Plus court que "error:invalid_command\n"
#define ERR_FREQ "err:freq\n"  // Plus court que "error:invalid_frequency\n"
#define ERR_DUTY "err:duty\n"  // Plus court que "error:invalid_duty\n"

// Optimisation de setState en utilisant des defines pour les états
#define STATE_OFF 0
#define STATE_LOW 1
#define STATE_HIGH 2
#define STATE_PWM 3

// Ajout des définitions pour le protocole binaire
#define BINARY_MARKER 0xAA
#define BINARY_HEADER_SIZE 3  // SOF + LEN + CMD
#define MAX_BINARY_SIZE 32

// Pins de sortie (output) du driver
struct OutputPins {
    uint8_t control;    // Pin de contrôle (PWM)
    uint8_t enable;     // Pin d'activation
};

static const OutputPins DRIVER_PINS[4] = {
    {PA3, PA2},  // DO1, DO1EN
    {PA1, PA0},  // DO2, DO2EN
    {PA5, PA4},  // DO3, DO3EN
    {PA7, PA6}   // DO4, DO4EN
};

// Commandes binaires (comme dans FlexDriver)
enum UartCommand : uint8_t {
    SET_SINGLE_OUTPUT = 0x01,
    SET_ALL_OUTPUTS = 0x02    // Pour plus tard
};

// Réponses binaires (comme dans FlexDriver)
enum UartResponse : uint8_t {
    OK = 0x00,
    INVALID_COMMAND = 0x01,
    INVALID_PIN = 0x02,
    INVALID_FREQUENCY = 0x03,
    INVALID_DUTY = 0x04,
    CRC_ERROR = 0x06
};

// Fonction pour calculer le CRC (même algo que FlexDriver)
uint8_t calculateCRC(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

// Fonction pour envoyer une réponse binaire
void sendBinaryResponse(UartResponse response) {
    uint8_t frame[4];  // SOF + LEN + RESPONSE + CRC
    frame[0] = BINARY_MARKER;
    frame[1] = sizeof(frame);
    frame[2] = static_cast<uint8_t>(response);
    #ifndef DISABLE_CRC
    frame[3] = calculateCRC(frame, 3);
    #else
    frame[3] = 0;  // CRC nul en mode debug
    #endif
    
    delay(1);  // Petit délai avant la réponse
    Serial.write(frame, sizeof(frame));
    Serial.flush();  // S'assurer que tout est envoyé
}

void setupPWM() {
    // Activer l'horloge du GPIOA et TIM14
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();
    
    // Configuration du pin en mode alternatif
    GPIOA->MODER &= ~GPIO_MODER_MODER4_Msk;  // Clear mode
    GPIOA->MODER |= GPIO_MODER_MODER4_1;     // Set alternate function mode (0b10)
    
    // Configuration de l'alternate function
    GPIOA->AFR[0] &= ~(0xF << (4 * 4));  // Clear AF pour PA4
    GPIOA->AFR[0] |= (4 << (4 * 4));     // AF4 pour TIM14_CH1
    
    // Configuration du timer
    TIM14->CR1 = 0;                    // Reset CR1
    TIM14->PSC = 47;                   // 48MHz/48 = 1MHz
    TIM14->ARR = 999;                  // 1MHz/1000 = 1kHz par défaut
    TIM14->CCR1 = TIM14->ARR;         // Duty cycle initial à 100% (LED éteinte)
    
    // Configuration PWM
    TIM14->CCMR1 = 0;                  // Reset CCMR1
    TIM14->CCMR1 |= (6 << 4);          // PWM mode 1 sur canal 1 (6 = 0b110)
    TIM14->CCMR1 |= TIM_CCMR1_OC1PE;   // Preload enable
    TIM14->CR1 |= TIM_CR1_ARPE;        // Auto-reload preload enable
    TIM14->CCER |= TIM_CCER_CC1E;      // Enable output
    
    // Démarrer le timer
    TIM14->CR1 |= TIM_CR1_CEN;
}

void setPWM(uint32_t freq, uint8_t duty) {
    // Pour freq=0 : mode statique
    if (freq == 0) {
        // Ne pas toucher à la fréquence, juste régler le duty
        TIM14->CCR1 = (duty > 0) ? 0 : TIM14->ARR;  // 0% = LED éteinte, 100% = LED allumée
        return;
    }
    
    // Calcul des paramètres PWM
    uint32_t timerClock = 48000000;  // 48MHz
    uint32_t period = 0xFFFF;        // On vise la période max pour la meilleure résolution
    uint32_t prescaler = (timerClock / freq / period);
    
    // Ajuster si prescaler trop petit
    if (prescaler < 1) {
        prescaler = 1;
        period = timerClock / freq;
    }
    
    // Configurer le timer
    TIM14->CR1 &= ~TIM_CR1_CEN;     // Arrêter le timer
    TIM14->PSC = prescaler - 1;      // Prescaler
    TIM14->ARR = period - 1;         // Période
    // Inverser le duty cycle car logique négative
    TIM14->CCR1 = period - ((period * duty) / 100);
    TIM14->CR1 |= TIM_CR1_CEN;      // Redémarrer le timer
}

// Nouvelle fonction pour gérer les états
void setState(uint8_t state, uint32_t freq = 100, uint8_t duty = 0) {
    // Configurer PWM avant enable pour éviter les transitions
    switch(state) {
        case STATE_OFF:
            setPWM(freq, 0);
            digitalWrite(ENABLE_PIN, HIGH);
            break;
        case STATE_LOW:
            setPWM(freq, 0);
            digitalWrite(ENABLE_PIN, LOW);
            break;
        case STATE_HIGH:
            setPWM(freq, 100);
            digitalWrite(ENABLE_PIN, LOW);
            break;
        case STATE_PWM:
            setPWM(freq, duty);
            digitalWrite(ENABLE_PIN, LOW);
            break;
    }
}

// Fonction utilitaire pour parser un nombre
inline uint32_t parseNumber(const char** str) {
    uint32_t num = 0;
    while (**str == ' ') (*str)++; // Skip spaces
    while (**str >= '0' && **str <= '9') {
        num = num * 10 + (**str - '0');
        (*str)++;
    }
    return num;
}

void processCommand(const char* cmd) {
    Serial.write(DBG_CMD);
    Serial.write(cmd);
    Serial.write('\n');
    
    if (strncmp(cmd, "set 0 ", 6) == 0) {
        const char* params = cmd + 6;
        
        // États simples - vérifier qu'il n'y a pas de paramètres supplémentaires
        if (strcmp(params, "off") == 0 || 
            strcmp(params, "low") == 0 || 
            strcmp(params, "high") == 0) {
            
            // Vérifier qu'il n'y a pas de caractères après la commande
            const char* end = params;
            while (*end != ' ' && *end != '\0') end++;
            if (*end != '\0') {
                Serial.write(ERR_CMD);
                return;
            }
            
            if (params[0] == 'o') setState(STATE_OFF);
            else if (params[0] == 'l') setState(STATE_LOW);
            else setState(STATE_HIGH);
            
            Serial.write("ok\n");
            return;
        }

        // PWM - Format attendu: "pwm 1000 50"
        if (strncmp(params, "pwm ", 4) == 0) {
            params += 4;  // Skip "pwm "
            
            // Sauvegarder la position initiale pour vérifier qu'on a bien lu deux nombres
            const char* start = params;
            
            uint32_t freq = parseNumber(&params);
            uint32_t duty = parseNumber(&params);
            
            // Vérifier qu'on a bien lu deux nombres et qu'il n'y a rien après
            while (*params == ' ') params++;  // Skip trailing spaces
            if (params == start || *params != '\0') {
                Serial.write(ERR_CMD);
                return;
            }
            
            if (freq < MIN_FREQ || freq > MAX_FREQ) {
                Serial.write(ERR_FREQ);
                return;
            }
            if (duty > MAX_DUTY) {
                Serial.write(ERR_DUTY);
                return;
            }
            
            setState(STATE_PWM, freq, duty);
            Serial.write("ok\n");
            return;
        }
    }
    Serial.write(ERR_CMD);
}

// Fonction utilitaire pour vider le buffer
void flushReceiveBuffer() {
    unsigned long startTime = millis();
    
    // On continue à vider tant qu'on reçoit des données ou jusqu'au timeout
    while ((millis() - startTime) < SERIAL_TIMEOUT) {
        if (Serial.available()) {
            Serial.read();
            startTime = millis();  // Reset du timeout si on reçoit des données
        }
    }
}

void processBinaryFrame() {
    // Attendre la longueur
    while (!Serial.available());
    uint8_t length = Serial.read();
    
    // Vérifier la taille minimale AVANT de lire les données
    if (length < 5) {  // SOF + LEN + CMD + CONTROL + CRC minimum
        flushReceiveBuffer();  // Important !
        sendBinaryResponse(UartResponse::INVALID_COMMAND);
        return;
    }
    
    if (length > MAX_BINARY_SIZE) {
        flushReceiveBuffer();
        sendBinaryResponse(UartResponse::INVALID_COMMAND);
        return;
    }
    
    // Lire les données avec timeout
    uint8_t buffer[MAX_BINARY_SIZE];
    uint8_t idx = 0;
    unsigned long startTime = millis();
    
    while (idx < length) {
        if (Serial.available()) {
            buffer[idx++] = Serial.read();
        }
        // Timeout si on ne reçoit pas tous les octets
        if (millis() - startTime > SERIAL_TIMEOUT) {
            flushReceiveBuffer();
            sendBinaryResponse(UartResponse::INVALID_COMMAND);
            return;
        }
    }
    
    // Vérifier le CRC
    #ifndef DISABLE_CRC
    uint8_t receivedCRC = buffer[length-1];
    uint8_t calculatedCRC = calculateCRC(buffer, length-1);
    if (receivedCRC != calculatedCRC) {
        sendBinaryResponse(UartResponse::CRC_ERROR);
        return;
    }
    #endif

    // Vérifier la commande
    UartCommand cmd = static_cast<UartCommand>(buffer[0]);
    if (cmd != SET_SINGLE_OUTPUT) {  // Pour l'instant on ne gère que SET_SINGLE_OUTPUT
        sendBinaryResponse(UartResponse::INVALID_COMMAND);
        return;
    }

    // Parser les données de la commande
    uint8_t state = (buffer[1] >> 5) & 0x07;
    uint8_t pin = (buffer[1] >> 2) & 0x07;
    uint16_t freq = ((buffer[1] & 0x03) << 8) | buffer[2];
    uint8_t duty = buffer[3];
    
    // Validation du pin
    if (pin != 0) {
        sendBinaryResponse(UartResponse::INVALID_PIN);
        return;
    }
    
    // Validation stricte des paramètres selon l'état
    switch (state) {
        case 0:  // HIGH_Z
        case 1:  // LOW
            if (freq != 0 || duty != 0) {
                sendBinaryResponse(UartResponse::INVALID_COMMAND);
                return;
            }
            break;
            
        case 2:  // HIGH
            if (freq != 0 || duty != 100) {
                sendBinaryResponse(UartResponse::INVALID_COMMAND);
                return;
            }
            break;
            
        case 3:  // PWM
            if (freq < MIN_FREQ || freq > MAX_FREQ) {
                sendBinaryResponse(UartResponse::INVALID_FREQUENCY);
                return;
            }
            if (duty > MAX_DUTY) {
                sendBinaryResponse(UartResponse::INVALID_DUTY);
                return;
            }
            break;
            
        default:
            sendBinaryResponse(UartResponse::INVALID_COMMAND);
            return;
    }
    
    // Appliquer la commande
    switch (state) {
        case 0: setState(STATE_OFF); break;
        case 1: setState(STATE_LOW); break;
        case 2: setState(STATE_HIGH); break;
        case 3: setState(STATE_PWM, freq, duty); break;
        default:
            sendBinaryResponse(UartResponse::INVALID_COMMAND);
            return;
    }
    
    sendBinaryResponse(UartResponse::OK);
}

void setup() {
    Serial.setRx(SERIAL_RX);
    Serial.setTx(SERIAL_TX);
    Serial.begin(SERIAL_BAUD);
    
    // Configuration de la pin d'activation
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);  // OFF par défaut (logique négative)
    
    setupPWM();
    
    Serial.write("dbg:reset\n");
}

void loop() {
    static char cmd[CMD_BUFFER_SIZE];
    static uint8_t idx = 0;
    static unsigned long lastCharTime = 0;
    
    if (Serial.available() > 0) {
        char c = Serial.read();
        
        #ifdef BINARY_ONLY_MODE
        // En mode binaire strict
        if (c != BINARY_MARKER) {
            flushReceiveBuffer();
            sendBinaryResponse(UartResponse::INVALID_COMMAND);
            return;
        }
        processBinaryFrame();
        return;
        #else
        // Code ASCII/Binaire existant
        if (idx == 0 && c == BINARY_MARKER) {
            processBinaryFrame();
            return;
        }
        
        // Mode ASCII existant
        lastCharTime = millis();
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                cmd[idx] = '\0';
                processCommand(cmd);
                idx = 0;
            }
        }
        else if (idx < CMD_BUFFER_SIZE-1) {
            cmd[idx] = c;
            idx++;
            Serial.write("dbg:read=");
            Serial.write(c);
            Serial.write("\n");
        }
        #endif
    }
    
    #ifndef BINARY_ONLY_MODE
    // Vérification du timeout (uniquement pour le mode ASCII)
    if (idx > 0 && (millis() - lastCharTime) > SERIAL_TIMEOUT) {
        cmd[idx] = '\0';
        processCommand(cmd);
        idx = 0;
    }
    #endif
}