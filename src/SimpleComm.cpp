#include "SimpleComm.h"

SimpleComm::SimpleComm(HardwareSerial* serial, uint32_t timeoutMs)
    : serial(serial)
    , timeoutMs(timeoutMs)
{
    // Initialize protos array
    for(uint16_t i = 0; i < 256; i++) {
        protos[i].size = 0;
        protos[i].handler = nullptr;
    }
}

SimpleComm::Result SimpleComm::registerProto(uint8_t fc, size_t size) {
    // Validate size
    if(size + FRAME_OVERHEAD > MAX_FRAME_SIZE) {
        return ERR_OVERFLOW;
    }
    
    // Register proto
    protos[fc].size = size;
    
    return SUCCESS;
}

SimpleComm::Result SimpleComm::onMessage(uint8_t fc, void* handler) {
    // Verify that proto is registered
    if(protos[fc].size == 0) {
        return ERR_INVALID_FC;
    }
    
    // Register handler
    protos[fc].handler = handler;
    
    return SUCCESS;
}

SimpleComm::Result SimpleComm::sendMsg(uint8_t fc, const void* data, size_t size) {
   // Check if proto is registered
   if(protos[fc].size == 0 || protos[fc].size != size) {
       return ERR_INVALID_FC;
   }

   // Prepare frame
   uint8_t frame[MAX_FRAME_SIZE];
   size_t frameSize = size + FRAME_OVERHEAD;

   frame[0] = START_OF_FRAME;
   frame[1] = frameSize;
   frame[2] = fc;
   memcpy(&frame[3], data, size);
   frame[frameSize-1] = calculateCrc(frame, frameSize-1);

   // Send frame
   serial->write(frame, frameSize);
   serial->flush();

   return SUCCESS;
}

uint8_t SimpleComm::calculateCrc(const uint8_t* data, size_t size) {
   uint8_t crc = 0;
   for (size_t i = 0; i < size; i++) {
       uint8_t inbyte = data[i];
       for (uint8_t j = 0; j < 8; j++) {
           uint8_t mix = (crc ^ inbyte) & 0x01;
           crc >>= 1;
           if (mix) {
               crc ^= 0x8C;
           }
           inbyte >>= 1;
       }
   }
   return crc;
}

SimpleComm::Result SimpleComm::waitByte(uint8_t& byte) {
   uint32_t startTime = millis();
   
   while(!serial->available()) {
       if(millis() - startTime > timeoutMs) {
           return ERR_TIMEOUT;
       }
   }
   
   byte = serial->read();
   return SUCCESS;
}

SimpleComm::Result SimpleComm::waitBytes(uint8_t* buffer, size_t size) {
   uint32_t startTime = millis();
   size_t bytesRead = 0;
   
   while(bytesRead < size) {
       if(millis() - startTime > timeoutMs) {
           return ERR_TIMEOUT;
       }
       
       if(serial->available()) {
           buffer[bytesRead] = serial->read();
           bytesRead++;
           startTime = millis(); // Reset timeout quand on reçoit un byte
       }
   }
   
   return SUCCESS;
}

void SimpleComm::flushRxBuffer() {
   uint32_t startTime = millis();
   
   // Continue à vider tant qu'il y a des données ou jusqu'au timeout
   while(millis() - startTime < timeoutMs) {
       if(serial->available()) {
           serial->read();
           startTime = millis(); // Reset timeout si on reçoit des données
       }
   }
}

void SimpleComm::processRx() {
   uint8_t byte;
   
   // 1. Wait for START_OF_FRAME
   if(waitByte(byte) != SUCCESS) {
       return;
   }
   if(byte != START_OF_FRAME) {
       flushRxBuffer();
       return;
   }
   
   // 2. Get length
   if(waitByte(byte) != SUCCESS) {
       flushRxBuffer();
       return;
   }
   uint8_t frameLen = byte;
   
   // Validate frame length
   if(frameLen < FRAME_OVERHEAD || frameLen > MAX_FRAME_SIZE) {
       flushRxBuffer();
       return;
   }
   
   // 3. Get FC
   if(waitByte(byte) != SUCCESS) {
       flushRxBuffer();
       return;
   }
   uint8_t fc = byte;
   
   // Validate FC and expected message size
   if(protos[fc].size == 0 || 
      protos[fc].size != frameLen - FRAME_OVERHEAD) {
       flushRxBuffer();
       return;
   }
   
   // 4. Get data + CRC
   size_t remainingBytes = frameLen - FRAME_HEADER_SIZE - 1; // -1 for FC already read
   if(waitBytes(rxBuffer + FRAME_HEADER_SIZE + 1, remainingBytes) != SUCCESS) {
       flushRxBuffer();
       return;
   }
   
   // 5. Reconstruct full frame for CRC check
   rxBuffer[0] = START_OF_FRAME;
   rxBuffer[1] = frameLen;
   rxBuffer[2] = fc;
   
   // 6. Validate CRC
   uint8_t receivedCrc = rxBuffer[frameLen-1];
   uint8_t calculatedCrc = calculateCrc(rxBuffer, frameLen-1);
   if(receivedCrc != calculatedCrc) {
       flushRxBuffer();
       return;
   }
   
   // 7. Call handler if registered
   if(protos[fc].handler != nullptr) {
       // Cast handler back to correct type and call it
       void* data = &rxBuffer[3];  // Skip SOF, LEN, FC
       auto handler = reinterpret_cast<MessageHandler<void>>(protos[fc].handler);
       handler(data);
   }
}

