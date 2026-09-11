// A scriptable model of arduinoWebSockets' client, for rover/test/test_net.cpp.
//
// `pi_up` is whether rover_server.py is listening; `tcp_writes_fail` models a
// HALF-OPEN connection -- the state a Pi losing power abruptly leaves behind,
// where the library still believes it is connected and only the write fails.
#pragma once
#include "Arduino.h"

enum WStype_t {
    WStype_ERROR, WStype_DISCONNECTED, WStype_CONNECTED, WStype_TEXT,
    WStype_BIN, WStype_PING, WStype_PONG,
};
typedef void (*WSEvent)(WStype_t, uint8_t*, size_t);

// Set by the harness; needed because a socket can only connect when the radio
// is genuinely on a working network, which is a property of the WiFi model.
extern bool ws_network_usable();

struct WebSocketsClient {
    bool pi_up = true;
    bool tcp_writes_fail = false;
    bool connected = false;
    WSEvent cb = nullptr;
    bool pending = false;
    int begins = 0, disconnects = 0, attempts = 0, sends = 0, failed_sends = 0;

    void onEvent(WSEvent e) { cb = e; }
    void setReconnectInterval(unsigned long) {}
    void enableHeartbeat(uint32_t, uint32_t, uint8_t) {}

    // begin() only ARMS a connection. The real client opens the TCP socket from
    // loop(), not from begin(), and the difference matters: a sketch that
    // registers its event handler after calling begin() still works on hardware
    // for exactly that reason, so a model that connected inside begin() would
    // flag a non-bug and -- worse -- would leave the OLD firmware unable to
    // connect at all, making any before/after comparison meaningless.
    void begin(const char*, uint16_t, const char*) {
        ++begins;
        if (connected) { connected = false; fire(WStype_DISCONNECTED); }
        pending = true;
    }
    void disconnect() {
        ++disconnects;
        pending = false;
        if (connected) { connected = false; fire(WStype_DISCONNECTED); }
    }
    // One connection attempt per begin(). The real library also retries on its
    // own every reconnectInterval; the model deliberately does not, so that a
    // recovery observed here is the FIRMWARE's doing and not the library's.
    void loop() {
        if (!pending || connected) return;
        pending = false;
        ++attempts;
        if (pi_up && ws_network_usable()) {
            connected = true;
            fire(WStype_CONNECTED);
        }
    }

    bool sendTXT(const char*) {
        if (!connected) { ++failed_sends; return false; }
        if (tcp_writes_fail) { ++failed_sends; return false; }
        ++sends;
        return true;
    }

private:
    void fire(WStype_t t) { if (cb) cb(t, nullptr, 0); }
};
