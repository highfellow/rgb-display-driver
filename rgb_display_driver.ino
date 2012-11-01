// sketch to drive rgb LED ribbon


// Uses clock in normal mode (no PWM)

#include <avr/interrupt.h>
#include <math.h>

#define OUT_MAX 1023 // maximum pwm output value.
#define OUTPUT_BASE 2 // output pin to count from.
#define ANALOG_BASE 0 // analog pin to count from.
#define MODES 3 // number of driver modes.
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
  pinMode(5, INPUT); // mode switch button.
  pinMode(6, OUTPUT); // diagnostic
  PORTD |= 0b100000; // set pullup resistor on pin 5.
  // Increase the analog sample rate to 76 kHz.
  ADCSRA &= ~( _BV(ADPS0) | _BV(ADPS1) ); // clear bits 0 and 1.
  ADCSRA |= _BV(ADPS2); // set bit 2.
  // Set up the timer.
  TCCR2A = 0; // normal mode
  TCCR2B = _BV(CS20); // no prescaler;
  TCNT2 = 0; // reset counter.
  // Enable interrupt on overflow
  TIMSK2 |= _BV(TOIE2);
  // find the false ground level.
  delay(1000);
  sigGround = analogRead(4);
  // Start the USB Serial.
  Serial.begin(115200);
}

ISR(TIMER2_OVF_vect) {
  // Interrupt service routine for timer 2 overflow.
  count++;
  for (char colour=0; colour < 3; colour++) {
    if (count < out[colour]) {
      PORTD |= 1 << (colour + OUTPUT_BASE); // turn pin on.
      } else {
      PORTD &= ~(1 << (colour + OUTPUT_BASE)); // turn pin off.
    }
  }
  if (count > OUT_MAX) count = 0;
  if (count % 2 == 0) sampleNow = true; // trigger a sample.
}

unsigned int valueToLevel(float value) {
  unsigned int level = value * OUT_MAX; //
  if (level < 0) level = 0;
  if (level > OUT_MAX) level = OUT_MAX;
//  Serial.println(level,DEC);
  return level;
}

float ranFloat(float maxVal) {
  return random(65536)*maxVal/65536;
}

float logLevel(float level) {
  float ret = level*(powf(10,-3*(1-level)));
  return ret;
}

/*
void chaosCycle(float gr, float bg, float rb, unsigned char rgb[]) {
  static float reds = 0;
  static float greens = 0;
  static float blues = 0;
  float growth = 0.01;
  float ran = 0.0001;
  float crowding = 0.01;
  reds += ranFloat(ran) + growth * (reds - reds * reds - reds * greens * gr);
  greens += ranFloat(ran) + growth * (greens - greens * greens - greens * blues * bg);
  blues += ranFloat(ran) + growth * (blues - blues * blues - blues * reds * rb);
  setRGB(reds/2, greens/2, blues/2, out);
}*/

void setOutput(float red, float green, float blue) {
  out[0]=valueToLevel(red);
  out[1]=valueToLevel(green);
  out[2]=valueToLevel(blue);
}

void setRGB(float red,float green, float blue, float sample) {
  setOutput(logLevel(red), logLevel(green), logLevel(blue));
}


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

void colourChange(float a, float b, float c, float sample) {
  if (digitalRead(5) == LOW) {
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
  }
  switch (mode) {
    case 0:
      setRGB(a,b,c, sample);
      break;
    case 1:
      setHSV(a,b,c, sample);
      break;
    case 2:
      colourCycle(a,b,c, sample);
      break;
  }
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
}


void loop() {
  static float potVals[3];
  static char pot = 0; // potentiometer to read.
  float sample;
  while (!sampleNow); // wait for sample time.
  PORTD ^= 0b1000000;
  sampleNow = false;
  sample = ((float) analogRead(3) - sigGround) / (float) sigGround;
  potVals[pot] = (float) analogRead(ANALOG_BASE + pot) / (float) 1024;
  pot++;
  if (pot > 2) pot = 0;
  colourChange(potVals[0], potVals[1], potVals[2], sample);
}


