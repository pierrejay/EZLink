#include <Arduino.h>
#include "SimpleComm.h"

#define LED_PIN PA4
#define SERIAL_TX PA9 
#define SERIAL_RX PA10
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 10  // Timeout en millisecondes
#define CMD_BUFFER_SIZE 32

void setup() {
    Serial.setRx(SERIAL_RX);
    Serial.setTx(SERIAL_TX);
    Serial.begin(SERIAL_BAUD);
    
    // Configuration de la pin d'activation
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // OFF par défaut (logique négative)
    
    Serial.write("dbg:reset\n");
}

void loop() {
    
}