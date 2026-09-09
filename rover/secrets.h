// Copy to secrets.h and fill in. secrets.h is gitignored -- the credentials
// must not be committed. (The previous firmware had the WiFi password in
// main.ino, which put it in the repository history.)
#pragma once

#define WIFI_SSID     "sfr2.4G"
#define WIFI_PASSWORD "Teddy@123"

// Where the Pi's rover server is listening (pi/rover/rover_server.py, the
// Arduino-facing socket). The Arduino dials in; the Pi is the server.
#define PI_HOST "192.168.1.8"
#define PI_PORT 8765
