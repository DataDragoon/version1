// Scriptable Arduino stubs for rover/test/test_net.cpp.
//
// Distinct from test/stubs/Arduino.h, which exists only to make the sketch
// type-check and therefore hardcodes every answer. These ones are DRIVEN: the
// clock advances, delay() advances it too, and NVIC_SystemReset() throws so a
// harness can observe a reset instead of vanishing into it.
//
// Put this directory FIRST on the include path and test/stubs SECOND, so the
// libraries the network ladder does not touch (EEPROM, FspTimer) still come
// from the plain stubs.
#pragma once
#ifndef ROVER_ARDUINO_STUB_H
#define ROVER_ARDUINO_STUB_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <string>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

// ── the fake clock ──────────────────────────────────────────────────────────
extern unsigned long g_ms;
inline unsigned long millis() { return g_ms; }
inline unsigned long micros() { return g_ms * 1000UL; }
// Every blocking wait in the firmware goes through delay(), so advancing the
// clock here is what makes connectWiFi()'s association and DHCP timeouts real
// rather than instantaneous.
inline void delay(unsigned long ms) { g_ms += ms; }

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 0; }
inline void noInterrupts() {}
inline void interrupts() {}

// Thrown by NVIC_SystemReset so the harness can see the reset and unwind out of
// loop(), which is as close as a host build gets to the board rebooting.
struct BoardReset {};
[[noreturn]] inline void NVIC_SystemReset() { throw BoardReset{}; }

struct IPAddress;

// Captures the serial log so the harness can assert on what the firmware said,
// which for this state machine is most of the observable behaviour.
extern std::string g_serial;
extern bool g_serial_echo;
void _log(const std::string& s);

struct SerialStub {
    void begin(unsigned long) {}
    void print(const char* s) { _log(s); }
    void print(int v) { _log(std::to_string(v)); }
    void print(long v) { _log(std::to_string(v)); }
    void print(unsigned long v) { _log(std::to_string(v)); }
    void print(float v) { _log(std::to_string(v)); }
    void print(double v) { _log(std::to_string(v)); }
    void println(const char* s) { _log(s); _log("\n"); }
    void println(int v) { print(v); _log("\n"); }
    void println(long v) { print(v); _log("\n"); }
    void println(unsigned long v) { print(v); _log("\n"); }
    void println(float v) { print(v); _log("\n"); }
    void println(double v) { print(v); _log("\n"); }
    void println() { _log("\n"); }
    void print(const struct IPAddress&) { _log("<ip>"); }
    void println(const struct IPAddress&) { _log("<ip>\n"); }
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
