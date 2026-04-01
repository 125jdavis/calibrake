/*
 * calibrake.ino
 *
 * Signal interceptor for the Bosch iBooster electronic brake booster.
 * Reads pedal travel sensors S2 (D2/INT0, falls with travel) and S4
 * (D3/INT1, rises with travel) as 1 kHz PWM signals using edge-triggered
 * interrupts, applies an EMA filter, maps each to a 0–100 % pedal travel
 * value using user-supplied calibration end-points, rescales through an
 * 11-entry lookup table, and drives two 1 kHz PWM outputs on D9 (OC1A,
 * S2 out) and D10 (OC1B, S4 out) via Timer1.
 *
 * All timing is non-blocking (millis()-based).  No delay() is used.
 *
 * Serial command interface (115200 baud):
 *   E<0-99>        – Set EMA coefficient (0 = unfiltered, 99 = heavy filter)
 *   A<0-1000>      – S2 no-travel  pulse width µs (pedal fully released)
 *   B<0-1000>      – S2 full-travel pulse width µs (pedal fully pressed)
 *   C<0-1000>      – S4 no-travel  pulse width µs
 *   D<0-1000>      – S4 full-travel pulse width µs
 *   L<0-10>,<0-100>– Set lookup table entry (index, output %)
 *   P              – Print current settings
 */

// ---------------------------------------------------------------------------
// Hardware constants
// ---------------------------------------------------------------------------
static const uint8_t S2_PIN     = 2;  // INT0 – S2 PWM input (falls with travel)
static const uint8_t S4_PIN     = 3;  // INT1 – S4 PWM input (rises with travel)

static const uint8_t S2_OUT_PIN = 9;  // OC1A – S2 PWM output
static const uint8_t S4_OUT_PIN = 10; // OC1B – S4 PWM output

// Timer1 ICR1 value for 1 kHz Fast PWM at 16 MHz / prescaler-8:
//   ICR1 = F_CPU / (prescaler * freq) - 1 = 16 000 000 / (8 * 1000) - 1 = 1999
static const uint16_t PWM_TOP = 1999;

// ---------------------------------------------------------------------------
// User-configurable parameters (defaults – overridable via serial)
// ---------------------------------------------------------------------------

// EMA coefficient: 0 = pass-through, 99 = maximum smoothing.
// Filtered = alpha * raw + (1 - alpha) * filtered,  alpha = (100 - coeff) / 100
int emaCoefficient = 10;

// S2 calibration pulse widths (µs). S2 FALLS as pedal travel increases.
int s2NoTravel   = 900; // µs when pedal is fully released
int s2FullTravel = 100; // µs when pedal is fully pressed

// S4 calibration pulse widths (µs). S4 RISES as pedal travel increases.
int s4NoTravel   = 100; // µs when pedal is fully released
int s4FullTravel = 900; // µs when pedal is fully pressed

// 11-entry rescaling lookup table.
// Index i  → input  pedal travel = i * 10 %
// Value    → output pedal travel (0–100 %)
// Default: unity (1 : 1) mapping.
int lookupTable[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

// ---------------------------------------------------------------------------
// PWM input state (written by ISRs, read in loop)
// ---------------------------------------------------------------------------
volatile uint32_t s2RiseUs  = 0;
volatile uint16_t s2PulseUs = 500; // last measured pulse width (µs)

volatile uint32_t s4RiseUs  = 0;
volatile uint16_t s4PulseUs = 500; // last measured pulse width (µs)

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
float s2Filtered = 500.0f;
float s4Filtered = 500.0f;

unsigned long lastSensorMs = 0;
unsigned long lastSerialMs = 0;

// ---------------------------------------------------------------------------
// ISR for S2 (D2 / INT0) — measures PWM pulse width on each edge.
// ---------------------------------------------------------------------------
void s2ISR()
{
    if (digitalRead(S2_PIN)) {          // rising edge: record start time
        s2RiseUs = micros();
    } else {                            // falling edge: compute pulse width
        uint32_t pw = micros() - s2RiseUs;
        if (pw <= 1000UL) {
            s2PulseUs = (uint16_t)pw;
        }
    }
}

// ---------------------------------------------------------------------------
// ISR for S4 (D3 / INT1) — measures PWM pulse width on each edge.
// ---------------------------------------------------------------------------
void s4ISR()
{
    if (digitalRead(S4_PIN)) {          // rising edge: record start time
        s4RiseUs = micros();
    } else {                            // falling edge: compute pulse width
        uint32_t pw = micros() - s4RiseUs;
        if (pw <= 1000UL) {
            s4PulseUs = (uint16_t)pw;
        }
    }
}

// ---------------------------------------------------------------------------
// Map an output travel % to a Timer1 OCR value for S2.
// S2 falls with travel: 0 % → s2NoTravel µs, 100 % → s2FullTravel µs.
// 1 timer tick = 0.5 µs at 16 MHz / prescaler-8, so OCR = pw_us * 2.
// ---------------------------------------------------------------------------
static uint16_t travelToS2Ocr(int travel)
{
    long pw = map(travel, 0, 100, s2NoTravel, s2FullTravel);
    pw = constrain(pw, 0, 1000);
    return (uint16_t)constrain(pw * 2L, 0L, (long)PWM_TOP);
}

// ---------------------------------------------------------------------------
// Map an output travel % to a Timer1 OCR value for S4.
// S4 rises with travel: 0 % → s4NoTravel µs, 100 % → s4FullTravel µs.
// ---------------------------------------------------------------------------
static uint16_t travelToS4Ocr(int travel)
{
    long pw = map(travel, 0, 100, s4NoTravel, s4FullTravel);
    pw = constrain(pw, 0, 1000);
    return (uint16_t)constrain(pw * 2L, 0L, (long)PWM_TOP);
}

// ---------------------------------------------------------------------------
// Lookup table: linear interpolation between the two nearest entries.
// input  : 0–100 (input pedal travel %)
// returns: 0–100 (rescaled output travel %)
// ---------------------------------------------------------------------------
static int lookupInterpolate(int input)
{
    input = constrain(input, 0, 100);
    int idx = input / 10;
    if (idx >= 10) {
        return lookupTable[10];
    }
    int rem = input % 10;
    if (rem == 0) {
        return lookupTable[idx];
    }
    // Linear interpolation between table[idx] and table[idx+1]
    return lookupTable[idx]
           + (lookupTable[idx + 1] - lookupTable[idx]) * rem / 10;
}

// ---------------------------------------------------------------------------
// Non-blocking serial command parser.
//
// Commands are single-character followed by integer argument(s).
// Serial.parseInt() returns when it sees a non-digit character, so commands
// can be sent as plain text lines, e.g.  "E20\n"  or  "L3,45\n".
// ---------------------------------------------------------------------------
static void processSerial()
{
    if (!Serial.available()) {
        return;
    }

    char cmd = (char)Serial.read();

    switch (cmd) {
    case 'E': {
        int val = Serial.parseInt();
        emaCoefficient = constrain(val, 0, 99);
        Serial.print(F("EMA coefficient: "));
        Serial.println(emaCoefficient);
        break;
    }
    case 'A': {
        int val = Serial.parseInt();
        s2NoTravel = constrain(val, 0, 1000);
        Serial.print(F("S2 no-travel us: "));
        Serial.println(s2NoTravel);
        break;
    }
    case 'B': {
        int val = Serial.parseInt();
        s2FullTravel = constrain(val, 0, 1000);
        Serial.print(F("S2 full-travel us: "));
        Serial.println(s2FullTravel);
        break;
    }
    case 'C': {
        int val = Serial.parseInt();
        s4NoTravel = constrain(val, 0, 1000);
        Serial.print(F("S4 no-travel us: "));
        Serial.println(s4NoTravel);
        break;
    }
    case 'D': {
        int val = Serial.parseInt();
        s4FullTravel = constrain(val, 0, 1000);
        Serial.print(F("S4 full-travel us: "));
        Serial.println(s4FullTravel);
        break;
    }
    case 'L': {
        int idx = Serial.parseInt();
        // Consume the comma separator
        while (Serial.available() && Serial.peek() == ',') {
            Serial.read();
        }
        int val = Serial.parseInt();
        if (idx >= 0 && idx <= 10) {
            lookupTable[idx] = constrain(val, 0, 100);
            Serial.print(F("LUT["));
            Serial.print(idx);
            Serial.print(F("] = "));
            Serial.println(lookupTable[idx]);
        } else {
            Serial.println(F("ERR: LUT index must be 0-10"));
        }
        break;
    }
    case 'P': {
        Serial.println(F("=== calibrake settings ==="));
        Serial.print(F("EMA coefficient : ")); Serial.println(emaCoefficient);
        Serial.print(F("S2 no-travel us : ")); Serial.println(s2NoTravel);
        Serial.print(F("S2 full-travel us: ")); Serial.println(s2FullTravel);
        Serial.print(F("S4 no-travel us : ")); Serial.println(s4NoTravel);
        Serial.print(F("S4 full-travel us: ")); Serial.println(s4FullTravel);
        Serial.print(F("LUT             : "));
        for (int i = 0; i <= 10; i++) {
            Serial.print(lookupTable[i]);
            if (i < 10) {
                Serial.print(',');
            }
        }
        Serial.println();
        Serial.println(F("=========================="));
        break;
    }
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    // PWM input pins — pull-ups guard against floating during power-up.
    pinMode(S2_PIN, INPUT_PULLUP);
    pinMode(S4_PIN, INPUT_PULLUP);

    // Seed EMA with no-travel calibration values before interrupts start.
    s2Filtered = (float)s2NoTravel;
    s4Filtered = (float)s4NoTravel;

    // Attach interrupts on both edges to measure pulse width.
    attachInterrupt(digitalPinToInterrupt(S2_PIN), s2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(S4_PIN), s4ISR, CHANGE);

    // Configure Timer1 for 1 kHz Fast PWM on OC1A (D9) and OC1B (D10).
    //   Fast PWM, ICR1 as TOP: WGM13:10 = 1110
    //   Non-inverting output on OC1A and OC1B: COM1A1=1, COM1B1=1
    //   Prescaler 8: CS11=1
    pinMode(S2_OUT_PIN, OUTPUT);
    pinMode(S4_OUT_PIN, OUTPUT);
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS11);
    ICR1   = PWM_TOP;

    // Drive outputs to the no-travel position at power-on.
    OCR1A = travelToS2Ocr(0);
    OCR1B = travelToS4Ocr(0);

    // Reduce Serial.parseInt() timeout so each parseInt() call blocks the loop
    // for at most 20 ms instead of the default 1000 ms.  Commands that call
    // parseInt() twice (e.g. 'L') may block up to ~40 ms total.
    Serial.setTimeout(20);

    Serial.println(F("calibrake ready"));
    Serial.println(F("Commands: E<ema> A<s2no> B<s2full> C<s4no> D<s4full> L<idx>,<val> P"));
}

// ---------------------------------------------------------------------------
// loop  — entirely non-blocking
// ---------------------------------------------------------------------------
void loop()
{
    unsigned long now = millis();

    // -- Non-blocking serial command handling --------------------------------
    processSerial();

    // -- 1 ms: read sensors and update PWM outputs ---------------------------
    // Unsigned subtraction handles millis() wrap-around correctly (~50-day rollover).
    if (now - lastSensorMs >= 1UL) {
        lastSensorMs = now;

        // Safely snapshot latest ISR-measured pulse widths.
        noInterrupts();
        uint16_t s2Raw = s2PulseUs;
        uint16_t s4Raw = s4PulseUs;
        interrupts();

        // EMA: alpha = (100 - coeff) / 100
        //   coeff = 0  → alpha = 1.0  → output == input  (no filtering)
        //   coeff = 99 → alpha = 0.01 → very slow response (heavy filtering)
        float alpha = (100.0f - (float)emaCoefficient) / 100.0f;
        s2Filtered = alpha * (float)s2Raw + (1.0f - alpha) * s2Filtered;
        s4Filtered = alpha * (float)s4Raw + (1.0f - alpha) * s4Filtered;

        // Map filtered pulse widths to 0–100 % pedal travel
        int s2Travel = constrain(
            map((int)s2Filtered, s2NoTravel, s2FullTravel, 0, 100), 0, 100);
        int s4Travel = constrain(
            map((int)s4Filtered, s4NoTravel, s4FullTravel, 0, 100), 0, 100);

        // Rescale through lookup table
        int s2Out = lookupInterpolate(s2Travel);
        int s4Out = lookupInterpolate(s4Travel);

        // Write to PWM output channels
        OCR1A = travelToS2Ocr(s2Out);
        OCR1B = travelToS4Ocr(s4Out);
    }

    // -- 200 ms: print filtered pulse widths to serial monitor ---------------
    if (now - lastSerialMs >= 200UL) {
        lastSerialMs = now;
        Serial.print(F("S2: "));
        Serial.print((int)s2Filtered);
        Serial.print(F(" us  S4: "));
        Serial.print((int)s4Filtered);
        Serial.println(F(" us"));
    }
}
