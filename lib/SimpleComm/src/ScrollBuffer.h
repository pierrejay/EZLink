#pragma once
#include <cstddef>

template<typename T, size_t SIZE>
class ScrollBuffer {

public:
    ScrollBuffer() : head(0), tail(0), count(0) {}

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

    // Dump until an element is found (exclusive)
    bool dumpUntil(const T* element) {
        if (!element) return false;
        
        // Check if the element is inside our buffer
        if (element < &buffer[0] || element >= &buffer[SIZE]) {
            return false;
        }
        
        size_t offset = element - &buffer[0];
        size_t distance = (offset - tail + SIZE) % SIZE;
        
        if (distance >= count) {
            return false;  // Element outside valid zone
        }
        
        dump(distance);
        return true;
    }

    // Slide to the next occurrence of marker, clear the whole buffer if not found
    void scrollTo(const T& marker) {
        // If marker found, slide to it
        if (!dumpUntil(find(marker))) {
            // If no marker found, clear the buffer
            clear();
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
    // Never used in our implementation, because we want to prevent against
    // truncated frames: potentially valid chunks cannot be re-processed 
    // once they are consumed.
    // We always want to peek() into the buffer, scrollTo() if we get an
    // invalid chunk, and dump() once process is done or if we have garbage.
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

    // Find the position of the next occurrence of a byte in the buffer,
    // and return a pointer to it (or nullptr if not found).
    T* find(const T& next) {
        for (size_t i = 0; i < count; i++) {
            if (peek(i) == next) {
                return &buffer[(tail + i) % SIZE];
            }
        }
        return nullptr;
    }

    // Find the position of the first occurrence of a byte in the buffer,
    // and return its index (or SIZE if not found).
    size_t indexOf(const T* element) const {
        if (!element) return SIZE; // valid index cannot be SIZE since index is 0-based
        return (element - &buffer[0]) % SIZE;
    }

    // Return the number of bytes in the buffer
    size_t size() const {
        return count;
    }

private:
    T buffer[SIZE];
    size_t head;
    size_t tail;
    size_t count;
}; 