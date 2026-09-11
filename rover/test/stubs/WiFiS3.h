#pragma once
#include "Arduino.h"
#define WL_CONNECTED 3
#define WL_IDLE_STATUS 0
struct WiFiStub {
    void disconnect() {}
    void begin(const char*, const char*) {}
    void config(IPAddress) {}
    void config(IPAddress, IPAddress, IPAddress, IPAddress) {}
    void end() {}
    uint8_t* macAddress(uint8_t* mac) { for (int i = 0; i < 6; ++i) mac[i] = 0; return mac; }
    int status() { return WL_CONNECTED; }
    IPAddress localIP() { return IPAddress(1,2,3,4); }
    IPAddress gatewayIP() { return IPAddress(1,2,3,1); }
    long RSSI() { return -50; }
    // Scan + ping, used by the network recovery ladder. Signatures follow
    // WiFiS3's CWifi: scanNetworks() returns a count, SSID(i) a C string,
    // RSSI(i) the level, ping() the round trip in ms or a negative error.
    int scanNetworks() { return 0; }
    const char* SSID(int) { return ""; }
    long RSSI(int) { return -50; }
    int ping(IPAddress, uint8_t ttl = 128) { (void)ttl; return 1; }
};
extern WiFiStub WiFi;
inline void _serial_print_ip(SerialStub& s, const IPAddress&) { (void)s; }
