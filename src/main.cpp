#include <Arduino.h>

#define LED_PIN PA4
#define SERIAL_TX PA9 
#define SERIAL_RX PA10
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 10  // Timeout en millisecondes
#define CMD_BUFFER_SIZE 32

// Fonction pour traiter la commande
void processCommand(const char* cmd) {
  // Debug de la commande complète
  Serial.write("DBG: Got command: ");
  Serial.write(cmd);
  Serial.write("\n");
  
  // Traitement de la commande
  if (strncmp(cmd, "set 0 ", 6) == 0) {
    if (strcmp(cmd + 6, "on") == 0) {
      digitalWrite(LED_PIN, LOW);
      Serial.write("ok:on\n");
    }
    else if (strcmp(cmd + 6, "off") == 0) {
      digitalWrite(LED_PIN, HIGH);
      Serial.write("ok:off\n");
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
  pinMode(LED_PIN, OUTPUT);
  Serial.setRx(SERIAL_RX);
  Serial.setTx(SERIAL_TX);
  Serial.begin(SERIAL_BAUD);
  
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
    lastCharTime = millis();  // Mise à jour du timestamp du dernier caractère
    
    // On continue à accepter \n et \r pour la compatibilité
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        cmd[idx] = '\0';
        processCommand(cmd);
        idx = 0;
      }
    }
    // Sinon on accumule le caractère s'il n'est pas une fin de ligne
    else if (idx < CMD_BUFFER_SIZE-1) {
      cmd[idx] = c;
      idx++;
      Serial.write("DBG: Added char to buffer: ");
      Serial.write(c);
      Serial.write("\n");
    }
  }
}