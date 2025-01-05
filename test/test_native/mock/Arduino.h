#pragma once
#include <stdint.h>
#include <string.h>
#include <chrono>
#include <thread>

#define ARDUINO

// Simulated time management
class TimeManager {
private:
    static std::chrono::steady_clock::time_point startTime;
    static uint32_t additionalTime; // For manual time advances

public:
    static void reset() {
        startTime = std::chrono::steady_clock::now();
        additionalTime = 0;
    }

    static uint32_t getMillis() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
        return elapsed + additionalTime;
    }

    static void advanceTime(uint32_t ms) {
        additionalTime += ms;
    }
};

std::chrono::steady_clock::time_point TimeManager::startTime = std::chrono::steady_clock::now();
uint32_t TimeManager::additionalTime = 0;

// Base interface for serial communication
class Stream {
protected:
    Stream() {}
public:
    virtual ~Stream() = default;
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t* buffer, size_t size) = 0;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() {}
    virtual int availableForWrite() = 0;
    virtual bool setAvailableForWrite(int value) = 0;
};

// Hardware serial interface
class HardwareSerial : public Stream {
protected:
    HardwareSerial() {}
public:
    virtual ~HardwareSerial() = default;
    virtual void begin(unsigned long) {}
    virtual void begin(unsigned long, uint8_t) {}
};

// Basic Arduino functions
inline uint32_t millis() { return TimeManager::getMillis(); }
inline void delay(uint32_t ms) { 
    std::this_thread::sleep_for(std::chrono::milliseconds(ms)); 
}

// Arduino constants
#define HIGH 1
#define LOW 0 