char const VER_MAJ = 1; // not backwards compatible
char const VER_MIN = 1; // backwards compatible

#include "downstairs_pins.h"

void init_low(int pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

#define WLH(c, p) case c: digitalWrite(p, LOW); break; case (128+c): digitalWrite(p, HIGH); break;

byte msg;
void act() {
    switch (msg) {
        WLH(127, PIN_SCOPE)
        WLH(126, PIN_OLFDISP)
        WLH(125, PIN_FLOW)
        WLH(124, PIN_MIRROR)

    case 123:
        digitalWrite(PIN_OLFDISP, LOW);
        digitalWrite(PIN_SCOPE, LOW);
        digitalWrite(PIN_MIRROR, LOW);
        digitalWrite(PIN_FLOW, LOW);
        for (int i = 0; i < N_SOLENOID; i++) {
            digitalWrite(PIN_SOLENOID[i], LOW);
        }
        break;

    case 122:
        Serial.write(VER_MAJ);
        Serial.write(VER_MIN);
        break;

    default:
        if (msg < 128) digitalWrite(msg, LOW);
        else digitalWrite(msg-128, HIGH);
    }
}

void setup() {
    Serial.begin(9600);

    init_low(PIN_OLFDISP);
    init_low(PIN_SCOPE);
    init_low(PIN_MIRROR);
    init_low(PIN_FLOW);
    for (int i = 0; i < N_SOLENOID; i++) {
        init_low(PIN_SOLENOID[i]);
    }
}

void loop() {
    if (Serial.available()) {
        msg = Serial.read();
        act();
    }
}
