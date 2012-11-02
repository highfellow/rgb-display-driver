// sketch to drive rgb LED ribbon


// Uses clock in normal mode (no PWM)

#include <avr/interrupt.h>
#include <math.h>

#define UNITY 4096 // fixed point unity value.
#define UNITY_BITS 12 // same expressed as a power of 2.
#define IN_POINTS 2 // binary points to add to input values.
#define OUT_POINTS 4 // binary points to remove from output values.
#define MODE_PIN 5 // mode switch button pin.
#define DIAG_PIN 6 // diagnostic output pin.
#define OUTPUT_BASE 2 // output pin to count from.
#define ANALOG_BASE 0 // analog pin to count from.
#define ANALOG_SIG 4 // analog pin for audio signal.
#define ANALOG_GND 5 // analog pin for false ground.
#define MODES 3 // number of driver modes.
#define MAX_TIME 15000 // maximum fade time in 30000ths of a second
#define LEVEL_SAMPS 300000 // samples to average level over.

// defines for the meanings of the channels in non-RGB colour mode.
#define VAL_RED 0 // red channel
#define VAL_GREEN 1 // green channel
#define VAL_BLUE 2 // blue channel
#define VAL_VALUE 0 // hsv value
#define VAL_SAT 1 // hsv saturation
#define VAL_HUE 2 // hsv hue
#define VAL_SPEED 1 // colour cycle speed.
#define VAL_RANDOM 2 // colour cycle randomness

volatile int16_t out[] = {0, 0, 0}; // rgb output value.
volatile int16_t count = 0; // PWM count.
char mode = 0; // colour changing mode.
volatile char sampleNow = false;
int16_t sigGround = 0;
volatile char printNow = false; // cycles to 0 every 4 secs.

void setup() {
  PORTD = 0; // reset output port
  for (char i=0; i<3; i++)
    pinMode(OUTPUT_BASE+i, OUTPUT); // set control pins to output.
  pinMode(MODE_PIN, INPUT); // mode switch button.
  pinMode(DIAG_PIN, OUTPUT); // diagnostic
  PORTD |= 1 << MODE_PIN; // set pullup resistor on pin 5.
  // Increase the analog sample rate to 76 kHz.
  ADCSRA &= ~( _BV(ADPS0) | _BV(ADPS1) ); // clear bits 0 and 1.
  ADCSRA |= _BV(ADPS2); // set bit 2.
  // Set up the timer.
  TCCR2A = _BV(WGM21); // normal mode
  TCCR2B = _BV(CS21); // 1/8 prescaler;
  OCR2A = 128; // interrupt at 15 Khz;
  TCNT2 = 0; // reset counter.
  // Enable interrupt on compare match
  TIMSK2 |= _BV(OCIE2A);
  // find the false ground signal level.
  delay(500); // wait for it to settle.
  sigGround = analogRead(ANALOG_GND);
  // Start the USB Serial.
  Serial.begin(115200);
}

uint32_t intExp(uint32_t power) {
  // returns 2 ^ power (approx.)
  uint8_t prevPower;
  uint8_t nextPower;
  uint32_t rem;
  uint32_t ret;
  prevPower = power >> UNITY_BITS;
  nextPower = prevPower + 1;
  rem = power % UNITY;
  ret = ((UNITY - rem) << prevPower) + (rem << nextPower);
  /*
  Serial.println((long)prevPower);
  Serial.println((long)nextPower);
  Serial.println((long)rem);
  Serial.println((long)ret);
  */
  return ret;
}

ISR(TIMER2_COMPA_vect) {
  // Interrupt service routine for timer 2 overflow.
  static char print = 0;
  //PORTD |= 1 << DIAG_PIN;
  count++;
  for (int16_t chan = 0; chan < 3; chan++) {
    if (count < out[chan]) {
      PORTD |= 1 << (chan + OUTPUT_BASE); // turn pin on.
      } else {
      PORTD &= ~(1 << (chan + OUTPUT_BASE)); // turn pin off.
    }
  }
  if (count >= (UNITY >> OUT_POINTS)) {
    count = 0;
    //print++;
    //if (print % 64 == 0) printNow = true;
  }
  sampleNow = true; // sample at 15 kHz;
  //if (count % 2 == 0) sampleNow = true; // trigger a sample at 15kHz.
  //PORTD &= ~(1 << DIAG_PIN);
}

void setOutput(int16_t vals[3]) {
  int16_t val;
  /*if (printNow) {
    printNow = false;
    Serial.print("vals: ");
    for (int16_t chan = 0; chan < 3; chan++) {
      if (chan > 0) Serial.print(", ");
      Serial.print(vals[chan]);
    }
    Serial.println(" ");
  }*/
  for (int16_t chan = 0; chan < 3; chan++) {
    val = vals[chan];
    if (val < 0) val = 0;
    if (val > UNITY - 1) val = UNITY - 1;
    out[chan] = val >> OUT_POINTS;
  }
}

void setOnOff(int16_t vals[3], char on) {
  // sets output either on or off.
  int16_t val = on ? UNITY : 0;
  for (int16_t chan = 0; chan < 3; chan++)
    vals[chan] = val;
  setOutput(vals);
}

void setRGB(int16_t vals[3], int16_t sample) {
  uint64_t exp;
  for (int16_t chan = 0; chan < 3; chan++) {
    exp = intExp((uint32_t)vals[chan] * 10);
    //if (printNow && chan == 0)
      //Serial.println(vals[chan]);
    vals[chan] = exp >> 10;
  }
  setOutput(vals);
}

void setHSV(int16_t vals[3], int16_t sample) {
  // hsv to rgb from wikipedia algorithm.
  int64_t c;
  int64_t x;
  int16_t h_;
  int16_t hmod;
  int64_t m;
  //vals[VAL_SAT] = UNITY - (intExp((uint32_t) (vals[VAL_SAT]) * 4) >> 4);
  vals[VAL_VALUE] = intExp((uint32_t) vals[VAL_VALUE] * 10) >> 10;
  c = ((int64_t) vals[VAL_VALUE] * (int64_t) vals[VAL_SAT]) >> UNITY_BITS;
  h_ = vals[VAL_HUE] * 6;
  hmod = h_ % (UNITY << 1) - UNITY;
  if (hmod < 0) hmod = -hmod;
  x = (c * (UNITY - (int64_t) hmod)) >> UNITY_BITS;
  m = vals[VAL_VALUE] - c;
  switch (h_ >> UNITY_BITS) {
    case 0:
      vals[VAL_RED] = c + m;
      vals[VAL_GREEN] = x + m;
      vals[VAL_BLUE] = m;
      break;
    case 1:
      vals[VAL_RED] = x + m;
      vals[VAL_GREEN] = c + m;
      vals[VAL_BLUE] = m;
      break;
    case 2:
      vals[VAL_RED] = m;
      vals[VAL_GREEN] = c + m;
      vals[VAL_BLUE] = x + m;
      break;
    case 3:
      vals[VAL_RED] = m;
      vals[VAL_GREEN] = x + m;
      vals[VAL_BLUE] = c + m;
      break;
    case 4:
      vals[VAL_RED] = x + m;
      vals[VAL_GREEN] = m;
      vals[VAL_BLUE] = c + m;
      break;
    case 5:
      vals[VAL_RED] = c + m;
      vals[VAL_GREEN] = m;
      vals[VAL_BLUE] = x + m;
      break;
  }
  setOutput(vals);
}

void colourCycle(int16_t vals[3], int16_t sample) {
  static int16_t lastHue = 0;
  static int16_t nextHue = 0;
  static int16_t lastSat = UNITY;
  static int16_t nextSat = UNITY;
  static int32_t time = 0;
  static int32_t nextTime = 0;
  int32_t hue;
  int16_t nextHue_;
  int16_t hueStep;
  int32_t sat;
  int32_t frac;
  int32_t maxTime;
  int32_t minTime;
  if (time == nextTime) {
    maxTime = ((uint64_t) MAX_TIME * (intExp(4 * (uint32_t) vals[VAL_SPEED]) >> 4)) >> UNITY_BITS; // * powf(10, b);
    minTime = ((uint64_t) maxTime * (intExp(4 * (uint32_t) vals[VAL_RANDOM]) >> 4)) >> UNITY_BITS;
    time = 0;
    nextTime = random(minTime, maxTime); // * powf(10, a * (float) random(OUT_MAX) / (float) OUT_MAX);
    lastHue = nextHue;
    lastSat = nextSat;
    nextHue = random(UNITY);
    nextSat = random(UNITY);
  }
  // do circular hue change.
  hueStep = nextHue - lastHue;
  if (hueStep > (UNITY >> 1)) {
    nextHue_ = nextHue - UNITY;
  } else if (hueStep < -(UNITY >> 1)) {
    nextHue_ = nextHue + UNITY;
  } else {
    nextHue_ = nextHue;
  }
  frac = ((int64_t) time << UNITY_BITS) / (int64_t) nextTime;
  hue = ((UNITY - frac) * (int64_t) lastHue + frac * (int64_t) nextHue_) >> UNITY_BITS;
  if (hue > UNITY) hue -= UNITY;
  if (hue < 0) hue += UNITY;
  sat = ((UNITY - frac) * (int64_t) lastSat + frac * (int64_t) nextSat) >> UNITY_BITS;
  vals[VAL_SAT] = sat;
  vals[VAL_HUE] = hue;
  setHSV(vals, sample);
  time++;
}

/*void colourOrgan(float a, float b, float c, float sample) {
  static float level = 0;
  static float amp;
  amp = fabs(sample);
  if (amp > level) {
    level = amp;
  } else {
    level -= level / LEVEL_SAMPS;
  }
  amp = amp / level;
  setOutput(amp, 0, 0);
}*/

void doMode(int16_t potVals[3], int16_t sample) {
  static int16_t vals[3]; // values to pass to mode functions.
  if ((PIND & (1 << MODE_PIN)) == 0) {
    mode++;
    if (mode == MODES) mode = 0;
    delay(500);
    for (char i = 0; i < mode + 1; i++) {
      setOnOff(vals, true);
      delay(300);
      setOnOff(vals, false);
      delay(300);
    }
    delay(500);
  }
  for (int16_t chan = 0; chan < 3; chan++) 
    vals[chan] = potVals[chan];
  switch (mode) {
    case 0:
      setRGB(vals, sample);
      break;
    case 1:
      setHSV(vals, sample);
      break;
    case 2:
      colourCycle(vals, sample);
      break;
  }
}  

void loop() {
  static int16_t potVals[3]; // values read from pots
  static int16_t pot = 0; // potentiometer to read.
  int16_t sample; // audio sample value.
  while (!sampleNow); // wait for sample time.
  sampleNow = false;
  PORTD |= 1 << DIAG_PIN;
  //sample = (analogRead(ANALOG_SIG) - sigGround) << IN_POINTS;
  potVals[pot] = analogRead(ANALOG_BASE + pot) << IN_POINTS;
  pot++;
  if (pot > 2) pot = 0;
  doMode(potVals, sample);
  PORTD &= ~(1 << DIAG_PIN);
}


