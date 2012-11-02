// sketch to drive rgb LED ribbon


// Uses clock in normal mode (no PWM)

#include <avr/interrupt.h>
#include <math.h>

#define UNITY 4096 // fixed point unity value.
#define IN_POINTS 2 // binary points to add to input values.
#define OUT_POINTS 2 // binary points to remove from output values.
#define MODE_PIN 5 // mode switch button pin.
#define DIAG_PIN 6 // diagnostic output pin.
#define OUTPUT_BASE 2 // output pin to count from.
#define ANALOG_BASE 0 // analog pin to count from.
#define ANALOG_SIG 4 // analog pin for audio signal.
#define ANALOG_GND 5 // analog pin for false ground.
#define MODES 1 // number of driver modes.
#define MIN_TIME 200 // minimum fade time in 100ths of a second
#define LEVEL_SAMPS 300000 // samples to average level over.

unsigned int out[] = {0, 0, 0}; // rgb output value.
unsigned int count = 0; // PWM count.
char mode = 0; // colour changing mode.
volatile char sampleNow = false;
int sigGround = 0;

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
  TCCR2A = 0; // normal mode
  TCCR2B = _BV(CS20); // no prescaler;
  TCNT2 = 0; // reset counter.
  // Enable interrupt on overflow
  TIMSK2 |= _BV(TOIE2);
  // find the false ground signal level.
  delay(500); // wait for it to settle.
  sigGround = analogRead(ANALOG_GND);
  // Start the USB Serial.
  //Serial.begin(115200);
}

ISR(TIMER2_OVF_vect) {
  // Interrupt service routine for timer 2 overflow.
  count++;
  for (int chan = 0; chan < 3; chan++) {
    if (count < out[chan]) {
      PORTD |= 1 << (chan + OUTPUT_BASE); // turn pin on.
      } else {
      PORTD &= ~(1 << (chan + OUTPUT_BASE)); // turn pin off.
    }
  }
  if (count >= (UNITY >> OUT_POINTS)) count = 0;
  if (count % 2 == 0) sampleNow = true; // trigger a sample at 31kHz.
}

float ranFloat(float maxVal) {
  return random(65536)*maxVal/65536;
}

float logLevel(float level) {
  float ret = level*(powf(10,-3*(1-level)));
  return ret;
}

void setOutput(int vals[3]) {
  int val;
  for (int chan = 0; chan < 3; chan++) {
    val = vals[chan];
    if (val < 0) val = 0;
    if (val >= UNITY) val = UNITY;
    out[chan] = val >> OUT_POINTS;
  }
}

void setRGB(int vals[3], int sample) {
  setOutput(vals);
}

/*
void setHSV(float hue, float saturation, float value, float sample) {
  // hsv to rgb from wikipedia algorithm. hue,saturation,value are all floats 0<=h,s,v<=1.
  float c;
  float x;
  float h_;
  float m;
  value=logLevel(value);
  c=value*saturation;
  h_=hue*6;
  x=c*(1-abs(fmodf(h_,2)-1));
  m=value-c;
  switch ((char)h_) {
    case 0:
      setOutput(c+m,x+m,m);
      break;
    case 1:
      setOutput(x+m,c+m,m);
      break;
    case 2:
      setOutput(m,c+m,x+m);
      break;
    case 3:
      setOutput(m,x+m,c+m);
      break;
    case 4:
      setOutput(x+m,m,c+m);
      break;
    case 5:
      setOutput(c+m,m,x+m);
      break;
  }
}

void colourCycle(float a, float b, float c, float sample) {
  static float lastHue = 0;
  static float nextHue = 0;
  static float lastSat = 1;
  static float nextSat = 1;
  static int time = 0;
  static int nextTime = 0;
  float hue;
  float nextHue_;
  float hueStep;
  float sat;
  float frac;
  float minTime;
  if (time == nextTime) {
    minTime = MIN_TIME * powf(10, b);
    time = 0;
    nextTime = minTime * powf(10, a * (float) random(OUT_MAX) / (float) OUT_MAX);
    lastHue = nextHue;
    lastSat = nextSat;
    nextHue = (float) random(OUT_MAX) / (float) OUT_MAX;
    nextSat = (float) random(OUT_MAX) / (float) OUT_MAX;
  }
  // do circular hue change.
  hueStep = nextHue - lastHue;
  if (hueStep > 0.5) {
    nextHue_ = nextHue - 1;
  } else if (hueStep < -0.5) {
    nextHue_ = nextHue + 1;
  } else {
    nextHue_ = nextHue;
  }
  frac = (float) time / (float) nextTime;
  hue = (1 - frac) * lastHue + frac * nextHue_;
  if (hue > 1) hue -= 1;
  if (hue < 0) hue += 1;
  sat = (1 - frac) * lastSat + frac * nextSat; 
  setHSV(hue, sat, c, sample);
  time++;
}

void colourOrgan(float a, float b, float c, float sample) {
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

void doMode(int vals[3], int sample) {
  /*if (PORTD & (1 << MODE_PIN)) {
    mode++;
    if (mode == MODES) mode = 0;
    delay(500);
    for (char i = 0; i < mode + 1; i++) {
      setOutput(1.0, 1.0, 1.0);
      delay(300);
      setOutput(0.0, 0.0, 0.0);
      delay(300);
    }
    delay(500);
  }*/
  switch (mode) {
    case 0:
      setRGB(vals, sample);
      break;
    /*case 1:
      setHSV(a,b,c, sample);
      break;
    case 2:
      colourCycle(a,b,c, sample);
      break;*/
  }
}  

void loop() {
  static int potVals[3];
  static char pot = 0; // potentiometer to read.
  int sample; // audio sample value.
  while (!sampleNow); // wait for sample time.
  sampleNow = false;
  PORTD ^= 1 << DIAG_PIN; // toggle diagnostic output
  sample = (analogRead(ANALOG_SIG) - sigGround) << IN_POINTS;
  potVals[pot] = analogRead(ANALOG_BASE + pot) << IN_POINTS;
  pot++;
  if (pot > 2) pot = 0;
  doMode(potVals, sample);
}


