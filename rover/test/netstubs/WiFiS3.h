// A scriptable model of the WiFi modem, for rover/test/test_net.cpp.
//
// The point of the model is the ONE thing the real modem does that broke the
// firmware: `latched`. When an AP disappears without a clean deauth, the modem
// can keep reporting WL_CONNECTED with the last-known address for as long as it
// likes. Set `ap_present = false` while leaving `latched = true` and this stub
// reproduces exactly that -- status() and localIP() both lie, and nothing but a
// full WiFi.end() clears them.
#pragma once
#include "Arduino.h"

#define WL_CONNECTED     3
#define WL_IDLE_STATUS   0
#define WL_DISCONNECTED  6

struct WiFiStub {
    // ── what the world is actually doing ────────────────────────────────────
    bool ap_present   = true;    // is the SSID on the air
    bool ap_accepts   = true;    // will it complete association for this MAC
    bool dhcp_answers = true;    // will it hand out a lease
    bool gateway_up   = true;    // does the gateway answer a ping

    // ── modem state ─────────────────────────────────────────────────────────
    bool associated = false;
    bool has_ip     = false;
    // The lie. Survives disconnect() (which is what makes the old firmware
    // deadlock) and is cleared only by end().
    bool latched    = false;

    // ── instrumentation ─────────────────────────────────────────────────────
    int begins = 0, disconnects = 0, ends = 0, scans = 0, pings = 0;

    void begin(const char*, const char*) {
        ++begins;
        if (!ap_present || !ap_accepts) return;
        associated = true;
        has_ip = dhcp_answers;
    }
    void disconnect() {
        ++disconnects;
        associated = false;
        has_ip = false;
        // Deliberately does NOT clear `latched`.
    }
    void end() {
        ++ends;
        associated = false;
        has_ip = false;
        latched = false;              // only a full teardown clears the lie
    }
    void config(IPAddress) {}
    void config(IPAddress, IPAddress, IPAddress, IPAddress) {}

    int status() const { return (associated || latched) ? WL_CONNECTED
                                                        : WL_DISCONNECTED; }
    IPAddress localIP() const {
        return (has_ip || latched) ? IPAddress(192, 168, 1, 77)
                                   : IPAddress(0, 0, 0, 0);
    }
    IPAddress gatewayIP() const {
        return (has_ip || latched) ? IPAddress(192, 168, 1, 1)
                                   : IPAddress(0, 0, 0, 0);
    }
    long RSSI() const { return -50; }
    uint8_t* macAddress(uint8_t* mac) { for (int i = 0; i < 6; ++i) mac[i] = (uint8_t)i; return mac; }

    int scanNetworks() { ++scans; g_ms += 2000; return ap_present ? 1 : 0; }
    const char* SSID(int) { return "test-network"; }
    long RSSI(int) { return -55; }

    // A ping only succeeds if the radio is genuinely on a working network. The
    // latch alone is not enough -- which is the whole reason the ladder asks.
    int ping(IPAddress, uint8_t ttl = 128) {
        (void)ttl;
        ++pings;
        g_ms += 100;
        return (associated && has_ip && gateway_up) ? 3 : -1;
    }
};
extern WiFiStub WiFi;
