#include <Arduino.h>

#define LED_PIN PA4
#define SERIAL_TX PA9 
#define SERIAL_RX PA10
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 10  // Timeout en millisecondes
#define CMD_BUFFER_SIZE 32

// Constantes PWM
#define MIN_FREQ 1
#define MAX_FREQ 10000
#define MAX_DUTY 100

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
    
    // Vérifier les limites
    if (prescaler > 0xFFFF) {
        Serial.write("DBG: Frequency too low!\n");
        return;
    }
    if (period > 0xFFFF) {
        Serial.write("DBG: Period overflow!\n");
        return;
    }
    
    // Configurer le timer
    TIM14->CR1 &= ~TIM_CR1_CEN;     // Arrêter le timer
    TIM14->PSC = prescaler - 1;      // Prescaler
    TIM14->ARR = period - 1;         // Période
    // Inverser le duty cycle car logique négative
    TIM14->CCR1 = period - ((period * duty) / 100);
    TIM14->CR1 |= TIM_CR1_CEN;      // Redémarrer le timer
}

// Fonction pour traiter la commande
void processCommand(const char* cmd) {
    Serial.write("DBG: Got command: ");
    Serial.write(cmd);
    Serial.write("\n");
    
    if (strncmp(cmd, "set 0 ", 6) == 0) {
        if (strcmp(cmd + 6, "on") == 0) {
            setPWM(0, 100);  // 100% = LED allumée
            Serial.write("ok:on\n");
        }
        else if (strcmp(cmd + 6, "off") == 0) {
            setPWM(0, 0);    // 0% = LED éteinte
            Serial.write("ok:off\n");
        }
        else if (strncmp(cmd + 6, "pwm ", 4) == 0) {
            char *params = (char *)(cmd + 10);
            char *space = strchr(params, ' ');
            if (!space) {
                Serial.write("error:invalid_command\n");
                return;
            }
            *space = '\0';
            int freq = atoi(params);
            int duty = atoi(space + 1);
            
            char debug[32];
            snprintf(debug, sizeof(debug), "DBG: freq=%d duty=%d\n", freq, duty);
            Serial.write(debug);
            
            if (freq < MIN_FREQ || freq > MAX_FREQ) {
                Serial.write("error:invalid_frequency\n");
                return;
            }
            if (duty < 0 || duty > MAX_DUTY) {
                Serial.write("error:invalid_duty\n");
                return;
            }
            
            setPWM(freq, duty);
            Serial.write("ok:pwm\n");
        }
        else {
            Serial.write("error:invalid_command\n");
        }
    }
    else {
        Serial.write("error:invalid_command\n");
    }
}

void setup() {
    Serial.setRx(SERIAL_RX);
    Serial.setTx(SERIAL_TX);
    Serial.begin(SERIAL_BAUD);
    
    setupPWM();
    
    Serial.write("=== RESET ===\n");
}

void loop() {
    static char cmd[CMD_BUFFER_SIZE];
    static uint8_t idx = 0;
    static unsigned long lastCharTime = 0;
    
    // Vérification du timeout
    if (idx > 0 && (millis() - lastCharTime) > SERIAL_TIMEOUT) {
        cmd[idx] = '\0';
        processCommand(cmd);
        idx = 0;
    }
    
    // Si on a un caractère disponible
    if (Serial.available() > 0) {
        char c = Serial.read();
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
            Serial.write("DBG: Added char to buffer: ");
            Serial.write(c);
            Serial.write("\n");
        }
    }
}