// sketch to drive rgb LED ribbon


// Uses clock in normal mode (no PWM)

#include <avr/interrupt.h>
#include <math.h>

#define OUT_MAX 1023 // maximum pwm output value.
#define OUTPUT_BASE 2 // output pin to count from.
#define ANALOG_BASE 0 // analog pin to count from.
#define MODES 2 // number of driver modes.

unsigned int out[] = {0, 0, 0}; // rgb output value.
unsigned int count = 0; // PWM count.
char mode = 0; // colour changing mode.

void setup() {
  PORTD = 0; // reset output port
  for (char i=0; i<3; i++)
    pinMode(OUTPUT_BASE+i, OUTPUT); // set control pins to output.
  pinMode(5, OUTPUT); // mode switch button.
  PORTD |= 0b100000; // set pullup resistor on pin 5.
  // Set up the timer.
  TCCR2A = 0; // normal mode
  TCCR2B = _BV(CS20); // no prescaler;
  TCNT2 = 0; // reset counter.
  // Enable interrupt on overflow
  TIMSK2 |= _BV(TOIE2);
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

void setRGB(float red,float green, float blue) {
  setOutput(logLevel(red), logLevel(green), logLevel(blue));
}


void setHSV(float hue, float saturation, float value) {
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

void colourChange(float a, float b, float c) {
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
      setRGB(a,b,c);
      break;
    case 1:
      setHSV(a,b,c);
      break;
/*    case 2:
      chaosCycle(a,b,c,out);
      break;*/
  }
}  

void loop() {
  float a, b, c;
  a=(float)analogRead(ANALOG_BASE) / 1024;
  b=(float)analogRead(ANALOG_BASE + 1) / 1024;
  c=(float)analogRead(ANALOG_BASE + 2) / 1024;
  colourChange(a,b,c);
  delay(10);
}


