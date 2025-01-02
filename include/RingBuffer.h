#pragma once
#include <cstddef>
#include <limits>

template<typename T, size_t SIZE>
class RingBuffer {
    // The compiler will check that SIZE can fit in S (or whether it's incompatible with T)

public:
    RingBuffer() : head(0), tail(0), count(0) {}

    // Flush the whole buffer
    void clear() {
        count = 0;
        head = tail = 0;
    }

    // Dump n bytes from the buffer
    void dump(size_t n) {
        if (n > count) {
            n = count;
        }
        tail = (tail + n) % SIZE;
        count -= n;
    }

    // Dump until the element is found (exclusive)
    void dumpUntil(const T* element) {
        if (!element) return;
        
        // Calculer la distance entre tail et l'élément
        size_t offset = element - &buffer[0];
        size_t distance = (offset - tail + SIZE) % SIZE;
        
        // Vérifier que l'élément est bien dans notre buffer
        if (distance < count) {
            dump(distance);
        }
    }

    // Push a byte into the buffer
    void push(T byte) {
        if (count < SIZE) {
            buffer[head] = byte;
            head = (head + 1) % SIZE;
            count++;
        }
    }

    // Pop a byte from the buffer
    T pop() {
        if (count > 0) {
            T byte = buffer[tail];
            tail = (tail + 1) % SIZE;
            count--;
            return byte;
        }
        return 0;
    }

    // Peek a byte from the buffer
    T peek(size_t offset = 0) const {
        if (offset < count) {
            return buffer[(tail + offset) % SIZE];
        }
        return 0;
    }

    // Check if the buffer is empty
    bool isEmpty() const {
        return count == 0;
    }

    // Find the position of the first occurrence of a byte in the buffer
    T* find(const T& next) {
        for (size_t i = 0; i < count; i++) {
            if (peek(i) == next) {
                return &buffer[(tail + i) % SIZE];
            }
        }
        return nullptr;
    }

    // Find the position of the first occurrence of a byte in the buffer
    size_t indexOf(const T* element) const {
        if (!element) return SIZE; // valid index cannot be SIZE since index is 0-based
        return (element - &buffer[0]) % SIZE;
    }

    size_t size() const {
        return count;
    }

private:
    T buffer[SIZE];
    size_t head;
    size_t tail;
    size_t count;
}; 