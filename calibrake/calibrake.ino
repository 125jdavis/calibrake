/*
 * calibrake.ino
 *
 * Signal interceptor for the Bosch iBooster electronic brake booster.
 * Reads pedal travel sensors S2 (A0, falls with travel) and S4 (A1, rises
 * with travel), applies an EMA filter, maps each to a 0–100 % pedal travel
 * value using user-supplied calibration end-points, rescales through an
 * 11-entry lookup table, and drives the two channels of an MCP4922 12-bit
 * dual DAC (SPI: D10/SS, D11/MOSI, D13/SCK).  A unity-gain MCP6002 op-amp
 * buffers each DAC output before it reaches the iBooster ECU.
 *
 * All timing is non-blocking (millis()-based).  No delay() is used.
 *
 * Serial command interface (115200 baud):
 *   E<0-99>        – Set EMA coefficient (0 = unfiltered, 99 = heavy filter)
 *   A<0-1023>      – S2 no-travel  ADC count  (pedal fully released)
 *   B<0-1023>      – S2 full-travel ADC count (pedal fully pressed)
 *   C<0-1023>      – S4 no-travel  ADC count
 *   D<0-1023>      – S4 full-travel ADC count
 *   L<0-10>,<0-100>– Set lookup table entry (index, output %)
 *   P              – Print current settings
 */

#include <SPI.h>

// ---------------------------------------------------------------------------
// Hardware constants
// ---------------------------------------------------------------------------
static const uint8_t DAC_SS_PIN = 10; // MCP4922 chip-select (active LOW)

// MCP4922 channel identifiers
static const uint8_t DAC_CH_A = 0; // S2 output
static const uint8_t DAC_CH_B = 1; // S4 output

// ---------------------------------------------------------------------------
// User-configurable parameters (defaults – overridable via serial)
// ---------------------------------------------------------------------------

// EMA coefficient: 0 = pass-through, 99 = maximum smoothing.
// Filtered = alpha * raw + (1 - alpha) * filtered,  alpha = (100 - coeff) / 100
int emaCoefficient = 10;

// S2 (A0) calibration: S2 FALLS as pedal travel increases.
int s2NoTravel   = 900; // ADC count when pedal is fully released
int s2FullTravel = 100; // ADC count when pedal is fully pressed

// S4 (A1) calibration: S4 RISES as pedal travel increases.
int s4NoTravel   = 100; // ADC count when pedal is fully released
int s4FullTravel = 900; // ADC count when pedal is fully pressed

// 11-entry rescaling lookup table.
// Index i  → input  pedal travel = i * 10 %
// Value    → output pedal travel (0–100 %)
// Default: unity (1 : 1) mapping.
int lookupTable[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
float s2Filtered = 512.0f;
float s4Filtered = 512.0f;

unsigned long lastSensorMs = 0;
unsigned long lastSerialMs = 0;

// ---------------------------------------------------------------------------
// MCP4922 DAC write
// channel : DAC_CH_A or DAC_CH_B
// value   : 12-bit DAC code (0–4095)
//
// MCP4922 16-bit command word:
//   [15]   A/B  channel select  (0 = A, 1 = B)
//   [14]   BUF  Vref buffer     (0 = unbuffered — external MCP6002 used)
//   [13]   /GA  output gain     (1 = 1×)
//   [12]   SHDN shutdown        (1 = active)
//   [11:0] data
// ---------------------------------------------------------------------------
static void dacWrite(uint8_t channel, uint16_t value)
{
    value = constrain(value, 0, 4095);

    uint16_t cmd = 0x3000; // BUF=0, GA=1x, SHDN=active
    if (channel == DAC_CH_B) {
        cmd |= 0x8000;     // Channel B
    }
    cmd |= (value & 0x0FFF);

    digitalWrite(DAC_SS_PIN, LOW);
    SPI.transfer((uint8_t)(cmd >> 8));
    SPI.transfer((uint8_t)(cmd & 0xFF));
    digitalWrite(DAC_SS_PIN, HIGH);
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
// Map an output travel % to a 12-bit DAC code for S2.
// S2 falls with travel: 0 % travel → s2NoTravel voltage,
//                      100 % travel → s2FullTravel voltage.
// ---------------------------------------------------------------------------
static uint16_t travelToS2Dac(int travel)
{
    long adcVal = map(travel, 0, 100, s2NoTravel, s2FullTravel);
    adcVal = constrain(adcVal, 0, 1023);
    return (uint16_t)((adcVal * 4095L) / 1023L);
}

// ---------------------------------------------------------------------------
// Map an output travel % to a 12-bit DAC code for S4.
// S4 rises with travel: 0 % travel → s4NoTravel voltage,
//                      100 % travel → s4FullTravel voltage.
// ---------------------------------------------------------------------------
static uint16_t travelToS4Dac(int travel)
{
    long adcVal = map(travel, 0, 100, s4NoTravel, s4FullTravel);
    adcVal = constrain(adcVal, 0, 1023);
    return (uint16_t)((adcVal * 4095L) / 1023L);
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
        s2NoTravel = constrain(val, 0, 1023);
        Serial.print(F("S2 no-travel ADC: "));
        Serial.println(s2NoTravel);
        break;
    }
    case 'B': {
        int val = Serial.parseInt();
        s2FullTravel = constrain(val, 0, 1023);
        Serial.print(F("S2 full-travel ADC: "));
        Serial.println(s2FullTravel);
        break;
    }
    case 'C': {
        int val = Serial.parseInt();
        s4NoTravel = constrain(val, 0, 1023);
        Serial.print(F("S4 no-travel ADC: "));
        Serial.println(s4NoTravel);
        break;
    }
    case 'D': {
        int val = Serial.parseInt();
        s4FullTravel = constrain(val, 0, 1023);
        Serial.print(F("S4 full-travel ADC: "));
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
        Serial.print(F("S2 no-travel    : ")); Serial.println(s2NoTravel);
        Serial.print(F("S2 full-travel  : ")); Serial.println(s2FullTravel);
        Serial.print(F("S4 no-travel    : ")); Serial.println(s4NoTravel);
        Serial.print(F("S4 full-travel  : ")); Serial.println(s4FullTravel);
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

    // MCP4922 SPI chip-select — idle HIGH
    pinMode(DAC_SS_PIN, OUTPUT);
    digitalWrite(DAC_SS_PIN, HIGH);

    // SPI: MCP4922 supports up to 20 MHz; 4 MHz is conservative and safe.
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000UL, MSBFIRST, SPI_MODE0));

    // Reduce Serial.parseInt() timeout so each parseInt() call blocks the loop
    // for at most 20 ms instead of the default 1000 ms.  Commands that call
    // parseInt() twice (e.g. 'L') may block up to ~40 ms total.
    Serial.setTimeout(20);

    // Seed EMA filters with the current sensor readings
    s2Filtered = (float)analogRead(A0);
    s4Filtered = (float)analogRead(A1);

    // Drive DAC outputs to the no-travel position at power-on
    dacWrite(DAC_CH_A, travelToS2Dac(0));
    dacWrite(DAC_CH_B, travelToS4Dac(0));

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

    // -- 1 ms: read sensors and update DAC outputs ---------------------------
    // Unsigned subtraction handles millis() wrap-around correctly (~50-day rollover).
    if (now - lastSensorMs >= 1UL) {
        lastSensorMs = now;

        int s2Raw = analogRead(A0);
        int s4Raw = analogRead(A1);

        // EMA: alpha = (100 - coeff) / 100
        //   coeff = 0  → alpha = 1.0  → output == input  (no filtering)
        //   coeff = 99 → alpha = 0.01 → very slow response (heavy filtering)
        float alpha = (100.0f - (float)emaCoefficient) / 100.0f;
        s2Filtered = alpha * (float)s2Raw + (1.0f - alpha) * s2Filtered;
        s4Filtered = alpha * (float)s4Raw + (1.0f - alpha) * s4Filtered;

        // Map filtered ADC values to 0–100 % pedal travel
        int s2Travel = constrain(
            map((int)s2Filtered, s2NoTravel, s2FullTravel, 0, 100), 0, 100);
        int s4Travel = constrain(
            map((int)s4Filtered, s4NoTravel, s4FullTravel, 0, 100), 0, 100);

        // Rescale through lookup table
        int s2Out = lookupInterpolate(s2Travel);
        int s4Out = lookupInterpolate(s4Travel);

        // Write to DAC channels
        dacWrite(DAC_CH_A, travelToS2Dac(s2Out));
        dacWrite(DAC_CH_B, travelToS4Dac(s4Out));
    }

    // -- 200 ms: print filtered sensor values to serial monitor --------------
    if (now - lastSerialMs >= 200UL) {
        lastSerialMs = now;
        Serial.print(F("S2: "));
        Serial.print((int)s2Filtered);
        Serial.print(F("  S4: "));
        Serial.println((int)s4Filtered);
    }
}
