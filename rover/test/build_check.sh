#!/usr/bin/env bash
# Type-checks the firmware without an Arduino toolchain.
#
# There is no arduino-cli on the Pi this project is developed from, so rover.ino
# would otherwise reach the board having never been through a compiler. This
# compiles it as ordinary C++ against thin stubs of the four libraries it uses
# (test/stubs/), which catches every syntax error, typo and type mismatch in our
# own code. What it CANNOT catch is anything about the real libraries' actual
# signatures or the hardware -- treat a pass as "this will probably compile",
# not as "this works".
#
# Also builds and runs the core unit tests and the network-ladder harness, which
# are the real verification.
set -euo pipefail
cd "$(dirname "$0")/../.."

OUT=${TMPDIR:-/tmp}

# rover/secrets.h is gitignored, so a fresh checkout does not have one and both
# builds that include rover.ino cannot resolve its #include. Stand one in from
# the committed example when it is missing -- the credentials in it are never
# used, nothing here connects to anything.
INC_SECRETS=""
if [ ! -f rover/secrets.h ]; then
    mkdir -p "$OUT/rover_secrets"
    cp rover/secrets.example.h "$OUT/rover_secrets/secrets.h"
    INC_SECRETS="-I $OUT/rover_secrets"
    echo "note: rover/secrets.h is absent; using secrets.example.h for the build"
fi

echo "== core unit tests =="
g++ -std=c++17 -O2 -Wall -Wextra -o "$OUT/rover_test_core" rover/test/test_core.cpp
"$OUT/rover_test_core"

echo
echo "== network recovery ladder =="
# Drives the REAL serviceNetwork() -- test_net.cpp includes rover.ino -- against
# a scriptable modem, so the behaviour that wedged the board (WiFi.status()
# reporting a link that no longer exists) can actually be put in front of it.
# See the header of test_net.cpp.
#
# netstubs must come FIRST on the include path: it replaces Arduino.h, WiFiS3.h
# and WebSocketsClient.h with driveable versions and leaves EEPROM/FspTimer to
# the plain stubs.
g++ -std=c++17 -O1 -Wall -Wextra -o "$OUT/rover_test_net" \
    -I rover/test/netstubs $INC_SECRETS -I rover/test/stubs -I rover \
    rover/test/test_net.cpp
"$OUT/rover_test_net"

echo
echo "== .ino preprocessor hazards =="
# The Arduino IDE auto-generates a prototype for every function in a .ino and
# inserts them near the top, ABOVE any type defined in that file -- so a
# function taking such a type fails with "does not name a type". Compiling the
# .ino as plain C++ below does NOT reproduce this, because it skips the .ino
# preprocessing entirely, so the rule is checked textually instead.
if grep -nE '^[[:space:]]*(struct|class|enum|typedef)[[:space:]]' rover/rover.ino; then
    echo "ERROR: rover.ino defines a type (above). Move it to rover/types.h --"
    echo "       the IDE's generated prototypes are emitted before it and will"
    echo "       fail to compile. See the note at the top of types.h."
    exit 1
fi
echo "no types defined in rover.ino"

echo
echo "== firmware type check =="
# .ino is not a C++ extension the compiler recognises, so point it at the file
# explicitly with -x c++.
g++ -std=c++17 -fsyntax-only -Wall -Wextra \
    $INC_SECRETS -I rover/test/stubs -I rover \
    -x c++ rover/rover.ino
# Also with the gateway ping compiled out, which is the escape hatch if
# WiFi.ping() turns out to be unavailable on the installed core.
g++ -std=c++17 -fsyntax-only -Wall -Wextra -DNET_USE_PING=0 \
    $INC_SECRETS -I rover/test/stubs -I rover \
    -x c++ rover/rover.ino
echo "rover.ino type-checks clean (with and without NET_USE_PING)"
