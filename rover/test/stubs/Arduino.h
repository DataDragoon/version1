// Minimal Arduino API stubs. Exist ONLY so rover/rover.ino can be type-checked
// with g++ on a machine that has no Arduino toolchain -- see test/build_check.sh.
// They are never compiled into the firmware.
//
// The guard is shared with test/netstubs/Arduino.h, which is a scriptable
// replacement for this file. Both are reached as "Arduino.h" from the other
// stub headers, so #pragma once cannot deduplicate them -- whichever the
// include path finds first wins and the other must fall away.
#pragma once
#ifndef ROVER_ARDUINO_STUB_H
#define ROVER_ARDUINO_STUB_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 0; }
inline void delay(unsigned long) {}
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline void noInterrupts() {}
inline void interrupts() {}
// CMSIS, available on the RA4M1 core through the real Arduino headers.
inline void NVIC_SystemReset() {}

struct IPAddress;

struct SerialStub {
    void begin(unsigned long) {}
    void print(const char*) {}
    void print(int) {}
    void print(long) {}
    void print(unsigned long) {}
    void print(float) {}
    void print(double) {}
    void println(const char*) {}
    void println(int) {}
    void println(long) {}
    void println(unsigned long) {}
    void println(float) {}
    void println(double) {}
    void println() {}
    // The real core prints anything Printable, IPAddress included.
    void print(const struct IPAddress&) {}
    void println(const struct IPAddress&) {}
};
extern SerialStub Serial;

struct IPAddress {
    uint8_t a, b, c, d;
    IPAddress() : a(0), b(0), c(0), d(0) {}
    IPAddress(uint8_t a_, uint8_t b_, uint8_t c_, uint8_t d_) : a(a_), b(b_), c(c_), d(d_) {}
    bool operator==(const IPAddress& o) const { return a==o.a && b==o.b && c==o.c && d==o.d; }
    bool operator!=(const IPAddress& o) const { return !(*this == o); }
};
inline void __ip_printable(const IPAddress&) {}

#endif  // ROVER_ARDUINO_STUB_H
