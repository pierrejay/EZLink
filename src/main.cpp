#include <Arduino.h>

#define LED_PIN PA4
#define SERIAL_TX PA9 
#define SERIAL_RX PA10

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
    TIM14->CCR1 = 0;                   // Duty cycle initial à 0
    
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
    if (freq == 0) {
        if (duty > 0) {
            // Mode ON : PWM à 0% (logique inversée)
            TIM14->CCR1 = 0;
            TIM14->CR1 |= TIM_CR1_CEN;
        } else {
            // Mode OFF : PWM à 100% (logique inversée)
            TIM14->CCR1 = TIM14->ARR;
            TIM14->CR1 |= TIM_CR1_CEN;
        }
        return;
    }
    
    // Calcul des paramètres PWM
    uint32_t timerClock = 48000000;  // 48MHz
    uint32_t prescaler = 48;         // Division fixe pour avoir 1MHz
    uint32_t period = (timerClock / prescaler) / freq;
    
    if (period > 0xFFFF) {
        return;
    }
    
    // Configurer le timer
    TIM14->CR1 &= ~TIM_CR1_CEN;  // Arrêter le timer
    TIM14->PSC = prescaler - 1;   // Prescaler
    TIM14->ARR = period - 1;      // Période
    // Inverser le duty cycle pour la logique négative
    TIM14->CCR1 = period - ((period * duty) / 100);
    TIM14->CR1 |= TIM_CR1_CEN;   // Redémarrer le timer
}

void setup() {
    // Ne pas configurer LED_PIN en OUTPUT ici
    // car il sera configuré en mode AF par setupPWM()
    
    Serial.setRx(SERIAL_RX);
    Serial.setTx(SERIAL_TX);
    Serial.begin(9600);
    
    setupPWM();
    Serial.write("Ready\n");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');  // Lecture simple d'une ligne complète
        
        // Debug de la commande complète
        Serial.write("Debug cmd: ");
        Serial.write(cmd.c_str());
        Serial.write("\n");
        
        // Traiter la commande
        if (cmd.startsWith("set 0 ")) {
            String action = cmd.substring(6);
            
            // Debug de l'action
            Serial.write("Debug action: ");
            Serial.write(action.c_str());
            Serial.write("\n");
            
            if (action == "on") {
                setPWM(0, 100);
                Serial.write("ok:on\n");
            }
            else if (action == "off") {
                setPWM(0, 0);
                Serial.write("ok:off\n");
            }
            else if (action.startsWith("pwm ")) {
                String params = action.substring(4);
                
                // Debug des paramètres
                Serial.write("Debug params: ");
                Serial.write(params.c_str());
                Serial.write("\n");
                
                int spacePos = params.indexOf(' ');
                if (spacePos == -1) {
                    Serial.write("error:invalid_command\n");
                    return;
                }
                
                int freq = params.substring(0, spacePos).toInt();
                int duty = params.substring(spacePos + 1).toInt();
                
                // Debug des valeurs
                char debug[32];
                snprintf(debug, sizeof(debug), "Debug freq=%d duty=%d\n", freq, duty);
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
}