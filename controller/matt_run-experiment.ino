/*** START THIS SCRIPT 2-10s BEFORE THORSYNC!
 *** CHECKLIST: DID YOU...
 *** 1. ALIGN THE ODOR TUBE AND PID?
 *** 2. MAKE SURE TO USE A FILTER ON THE ODOR VIAL?
 *** 3. PLUG EVERYTHING IN?
 *** 4. SET N_TRIALS CORRECTLY?
 *** 5. MAKE SURE THORSYNC AND THORIMAGE ARE CONNECTED?
 *** 6. MAKE SURE THE PMTs ARE ON?
 *** 7. SET THE FLOW CONTROLLERS?
 */

/*** SET ME ***/
float const DT_BEGIN     = 10.0;
float const DT_ODORPULSE = 2.0;
float const DT_PREODOR   = 45.0;
float const DT_POSTALL   = 45.0;
int   const N_TRIALS     = 3;
/*** SET ME **/

int const PIN_OLFDISP    = 12;
int const PIN_SCOPE      = 13;
int const PIN_MIRROR     = A2;
int const PIN_FLOW       = 2;
int const PIN_SOLENOID[] = {9, 8, 11};
int const N_SOLENOID     = 3;

float const DT_MIRRORDELAY = 1.0;

void init_pin(int pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void delay_s(float dt) {
    delay((unsigned long)(dt*1000.0));
}

void write_all(int *pins, int n, int val) {
    for (int i = 0; i < n; i++) {
        digitalWrite(*(pins+i), val);
    }
}

float time_since(unsigned long t0) {
    unsigned long now = millis();
    return ((float)(now-t0))/1000.0;
}

void print_time_s(float s, bool nl=true) {
    Serial.print(s);
    Serial.print("s");
    if (nl) Serial.print("\n");
}

void setup() {
    Serial.begin(9600);
    Serial.print("RUNNING WITH:\n");
    Serial.print("  ODOR PULSE  = ");
    print_time_s(DT_ODORPULSE);
    Serial.print("  INTERPULSE  = ");
    print_time_s(DT_PREODOR);
    Serial.print("  # OF TRIALS = ");
    Serial.println(N_TRIALS);

    init_pin(PIN_OLFDISP);
    init_pin(PIN_SCOPE);
    init_pin(PIN_MIRROR);
    init_pin(PIN_FLOW);
    for (int i = 0; i < N_SOLENOID; i++) {
        init_pin(PIN_SOLENOID[i]);
    }

    delay_s(DT_BEGIN);

    digitalWrite(PIN_MIRROR, HIGH);
    delay_s(DT_MIRRORDELAY);
    digitalWrite(PIN_SCOPE, HIGH);
    unsigned long t0_scope = millis();
    for (int trial = 0; trial < N_TRIALS; trial++) {
        delay_s(DT_PREODOR);

        Serial.print("\nPRESENTATION ");
        Serial.print(trial+1);
        Serial.print("\n START = ");
        print_time_s(time_since(t0_scope));

        write_all(PIN_SOLENOID, N_SOLENOID, HIGH);
        digitalWrite(PIN_OLFDISP, HIGH);
        delay_s(DT_ODORPULSE);
        write_all(PIN_SOLENOID, N_SOLENOID, LOW);
        digitalWrite(PIN_OLFDISP, LOW);

        Serial.print(" END = ");
        print_time_s(time_since(t0_scope));
    }

    delay_s(DT_POSTALL);
    digitalWrite(PIN_SCOPE, LOW);
    digitalWrite(PIN_MIRROR, LOW);

    Serial.print("\nFINISHED\n");
}

void loop() {}
