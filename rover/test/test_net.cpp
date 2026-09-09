// Drives the REAL network recovery ladder against a scriptable modem.
//
// WHY THIS EXISTS
// ---------------
// The fault it was written for -- "the board never comes back, only a router
// restart fixes it" -- is a state-machine deadlock, not an arithmetic mistake,
// and no amount of reading catches it: the old code looked perfectly reasonable
// because each layer was individually correct and only their arrangement was
// wrong. The property that matters is a liveness property ("from every bad
// state, some action eventually restores the link"), and the only way to check
// one is to put the machine in each bad state and run it.
//
// It includes rover.ino rather than reimplementing serviceNetwork(), so it
// exercises the shipped function and its shipped statics. A retyped copy would
// have been free to drift, which for this file is the whole ballgame.
//
// The modem model (test/netstubs/WiFiS3.h) reproduces the one behaviour that
// caused the bug: WiFi.status() and WiFi.localIP() can go on reporting a
// healthy link after the AP is gone, and only WiFi.end() clears that.
//
// Build:  see rover/test/build_check.sh

#include <cstdio>
#include <string>

unsigned long g_ms = 0;
std::string g_serial;
bool g_serial_echo = false;
void _log(const std::string& s) {
    g_serial += s;
    if (g_serial_echo) fputs(s.c_str(), stdout);
}

#include "Arduino.h"
#include "WiFiS3.h"
#include "EEPROM.h"

SerialStub Serial;
WiFiStub WiFi;
EEPROMStub EEPROM;

// A socket can only come up when the radio is genuinely on a working network.
// The latch alone must not be enough, or the model would hide the bug.
bool ws_network_usable() {
    return WiFi.associated && WiFi.has_ip && WiFi.gateway_up;
}

// The sketch itself. Everything below reaches into its statics on purpose.
#include "../rover.ino"

// ── harness ─────────────────────────────────────────────────────────────────

static int checks = 0, failures = 0;

static void ok(bool cond, const char* what) {
    ++checks;
    if (!cond) {
        ++failures;
        printf("  FAIL: %s   (t=%lu ms)\n", what, g_ms);
    }
}

// One pass of loop(), with the ISR's millisecond clock kept in step with the
// fake wall clock -- the firmware reads that one for status pacing, not millis().
static void tick() {
    isrMs = (uint32_t)g_ms;
    try {
        loop();
    } catch (const BoardReset&) {
        throw;                       // the caller decides what a reset means
    }
    g_ms += 1;
}

// Runs for `ms` of simulated time. Returns true if the board reset.
static bool run_for(unsigned long ms) {
    const unsigned long until = g_ms + ms;
    while (g_ms < until) {
        try {
            tick();
        } catch (const BoardReset&) {
            return true;
        }
    }
    return false;
}

// Put the firmware in a known-good, connected state.
static void boot_connected() {
    g_ms = 0;
    g_serial.clear();
    WiFi = WiFiStub{};
    webSocket = WebSocketsClient{};
    linkUp = false;
    txFails = 0;
    estopLatched = false;
    movePending = false;
    qCount = 0;
    positionValid = true;
    // serviceNetwork()'s statics persist across sections, which is realistic --
    // the board does not get to reinitialise them either. Every section below
    // starts from a connected state and lets them settle, so none of them
    // inherits a half-climbed ladder.
    setup();
    run_for(2000);
}

static bool logged(const char* needle) {
    return g_serial.find(needle) != std::string::npos;
}

// ── sections ────────────────────────────────────────────────────────────────

// The everyday good state. A ladder that fires here would reset a working rig.
static void test_steady_state() {
    printf("steady state: a healthy link is left alone\n");
    boot_connected();
    ok(linkUp, "connected after setup");
    const int begins = webSocket.begins, ends = WiFi.ends;
    const bool reset = run_for(15UL * 60UL * 1000UL);
    ok(!reset, "no reset in 15 minutes of a healthy link");
    ok(webSocket.begins == begins, "socket never restarted while up");
    ok(WiFi.ends == ends, "radio never recycled while up");
    ok(webSocket.sends > 10000, "status frames flowing (20 Hz)");
}

// The Pi being switched off is an ORDINARY state -- it happens on every
// rover_server restart. It must never be escalated: not a radio recycle, and
// above all not a reset, however long it lasts.
static void test_pi_off_never_escalates() {
    printf("pi off: retried forever, never escalated\n");
    boot_connected();
    const int ends = WiFi.ends;
    webSocket.pi_up = false;
    webSocket.disconnect();

    const bool reset = run_for(20UL * 60UL * 1000UL);
    ok(!reset, "20 minutes with the Pi off does NOT reset the board");
    ok(WiFi.ends == ends, "radio never recycled just because the Pi is off");
    ok(webSocket.begins > 30, "socket retried repeatedly meanwhile");
    ok(WiFi.pings > 0, "the gateway was asked, which is what excused the Pi");
    ok(logged("gateway answers"), "and the log says so");

    // ...and it comes straight back when the Pi returns.
    webSocket.pi_up = true;
    run_for(30000);
    ok(linkUp, "reconnects as soon as the Pi is listening again");
}

// THE REPORTED FAULT. The AP has gone (or the board was off while it went), and
// the modem is still reporting WL_CONNECTED with its old address. The old
// firmware returned at ensureNetwork()'s first line for ever here.
static void test_latched_modem_recovers() {
    printf("latched modem: the AP is gone but status() still says connected\n");
    boot_connected();
    const int ends = WiFi.ends;

    // The AP vanishes. The modem keeps the lie; the socket dies.
    WiFi.ap_present = false;
    WiFi.gateway_up = false;
    WiFi.latched = true;
    WiFi.associated = false;
    WiFi.has_ip = false;
    webSocket.disconnect();

    ok(linkReady(), "linkReady() is TRUE while the network is gone (the lie)");

    run_for(90000);
    ok(WiFi.ends > ends, "the radio was recycled despite status() claiming OK");
    ok(logged("not to be trusted"), "and the log names the reason");

    // The AP comes back. Nothing external touches the board.
    WiFi.ap_present = true;
    WiFi.gateway_up = true;
    const bool reset = run_for(120000);
    ok(!reset, "recovered without needing the reset");
    ok(linkUp, "back on the Pi with no operator intervention");
}

// The same lie, but permanent -- the modem never recovers however often it is
// recycled. This is the state nothing in the old firmware could clear, and the
// one the operator was clearing by restarting the router.
static void test_unrecoverable_modem_resets() {
    printf("wedged modem: reset is the last rung\n");
    boot_connected();

    // Associates happily every time and passes no traffic at all. If the reset
    // check sat after the escalation branches instead of before them, this case
    // would loop for ever and never reach it.
    WiFi.ap_present = true;
    WiFi.ap_accepts = true;
    WiFi.dhcp_answers = true;
    WiFi.gateway_up = false;         // the network is dead behind the AP
    webSocket.pi_up = false;
    webSocket.disconnect();

    const unsigned long t0 = g_ms;
    const bool early = run_for(NET_REBOOT_MS - 60000);
    ok(!early, "does NOT reset before the threshold");
    const bool reset = run_for(4UL * NET_REBOOT_MS);
    ok(reset, "resets once nothing else has worked");
    ok(g_ms - t0 < 2UL * NET_REBOOT_MS, "and does it promptly after the threshold");
    ok(logged("RESETTING the board"), "the log says why");
}

// A reset must not silently clear a latched E-stop or re-validate a position
// the operator was told not to trust.
static void test_reset_refused_when_not_parked() {
    printf("reset is refused unless the rig is parked\n");

    boot_connected();
    WiFi.gateway_up = false;
    webSocket.pi_up = false;
    webSocket.disconnect();
    estopLatched = true;
    bool reset = run_for(4UL * NET_REBOOT_MS);
    ok(!reset, "never resets while the E-stop is latched");
    ok(logged("is not parked"), "and says it is holding off");

    // Clearing it lets the ladder finish its job.
    estopLatched = false;
    reset = run_for(4UL * NET_REBOOT_MS);
    ok(reset, "resets once the E-stop is cleared");

    // The other two conditions are checked at the predicate rather than end to
    // end. There is no ISR in this build, so an injected queue entry is
    // dispatched and "completes" on the very next tick -- the rig really is
    // parked by then, and an end-to-end assertion would be testing the harness
    // rather than the firmware.
    boot_connected();
    ok(boardParked(), "a clean idle rig is parked");
    movePending = true;
    ok(!boardParked(), "a move in flight is not parked");
    movePending = false;
    qCount = 1;
    ok(!boardParked(), "a queued move is not parked");
    qCount = 0;
    estopLatched = true;
    ok(!boardParked(), "a latched E-stop is not parked");
    estopLatched = false;
}

// A half-open TCP connection: the library still reports the client connected
// and every write fails. Without the sendTXT check the board sits here happily
// reporting a good link until the library's own heartbeat eventually notices.
static void test_half_open_socket_detected() {
    printf("half-open socket: a failing write is treated as a dead link\n");
    boot_connected();
    ok(linkUp, "starts connected");

    webSocket.tcp_writes_fail = true;
    // 40 status frames at 20 Hz is 2 s; give it a little room, then the ladder
    // needs one WS_RECONNECT_FORCE_MS window to act.
    run_for(4000);
    ok(txFails >= NET_TX_FAIL_LIMIT, "consecutive write failures counted");
    const int begins = webSocket.begins;
    run_for(WS_RECONNECT_FORCE_MS + 4000);
    ok(webSocket.begins > begins, "the socket was restarted on that evidence");

    // Clearing the fault reconnects.
    webSocket.tcp_writes_fail = false;
    run_for(WS_RECONNECT_FORCE_MS + 4000);
    ok(linkUp && txFails == 0, "recovers and the counter resets");
}

// An honestly-disconnected modem. The first retry is a plain re-associate --
// most dropouts are a brief AP hiccup -- and every one after it is a full
// teardown, because that is the only thing that clears a wedged modem.
static void test_honest_disconnect_escalates_teardown() {
    printf("honest disconnect: plain retry first, teardown after\n");
    boot_connected();
    const int ends = WiFi.ends;

    WiFi.ap_present = false;         // no lie this time: status() reports down
    WiFi.associated = false;
    WiFi.has_ip = false;
    WiFi.latched = false;
    webSocket.disconnect();
    ok(!linkReady(), "linkReady() correctly reports down");

    run_for(6000);
    ok(WiFi.begins > 0, "re-association attempted");
    ok(WiFi.ends == ends, "the FIRST attempt is a plain re-associate");

    run_for(120000);
    ok(WiFi.ends > ends, "later attempts tear the radio all the way down");
    ok(WiFi.scans > 0, "and it scanned, so the log says whether the AP is there");

    WiFi.ap_present = true;
    run_for(60000);
    ok(linkUp, "reconnects when the AP returns");
}

// Associating but never getting a lease -- the stale-DHCP-lease case this
// firmware has been bitten by before. It must keep retrying rather than sitting
// in the half-connected state.
static void test_no_dhcp_lease_keeps_retrying() {
    printf("no DHCP lease: half-connected is not accepted as connected\n");
    boot_connected();
    WiFi.dhcp_answers = false;
    WiFi.associated = false;
    WiFi.has_ip = false;
    WiFi.latched = false;
    webSocket.disconnect();

    run_for(90000);
    ok(!linkReady(), "an address-less association is not treated as ready");
    ok(WiFi.begins > 2, "kept retrying");
    ok(logged("NO DHCP LEASE"), "and named the failure");

    WiFi.dhcp_answers = true;
    run_for(90000);
    ok(linkUp, "connects as soon as a lease is offered");
}

int main() {
    test_steady_state();
    test_pi_off_never_escalates();
    test_latched_modem_recovers();
    test_unrecoverable_modem_resets();
    test_reset_refused_when_not_parked();
    test_half_open_socket_detected();
    test_honest_disconnect_escalates_teardown();
    test_no_dhcp_lease_keeps_retrying();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
