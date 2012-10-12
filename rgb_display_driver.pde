// sketch to drive rgb LED ribbon


// Uses clock in normal mode (no PWM)

#include <avr/interrupt.h>
#include <math.h>

char outputBase=2; // output pin to start counting from (<6).
char analogBase=0; // analog pin to start counting from (<6).
unsigned char out[]={127,127,127}; // rgb output value.
unsigned char count=0; // PWM count.
char mode=0; // colour changing mode.

void setup() {
  PORTD = 0; // reset output port
  for (char i=0; i<3; i++)
    pinMode(outputBase+i, OUTPUT); // set control pins to output.
  // Set up the timer.
  
  TCCR2A = 0; // normal mode;
  TCCR2B = _BV(CS20); // no prescaler;
  // Enable interrupt on overflow
  TIMSK2 |= _BV(TOIE2);
  TCNT2 = 0; // reset counter.
  // Start the USB Serial.
  Serial.begin(115200);
}

ISR(TIMER2_OVF_vect) {
  // Interrupt service routine for timer 2 overflow.
  count++;
  for (char colour=0; colour < 3; colour++) {
    if (count < out[colour]) {
      PORTD |= 1 << (colour + outputBase); // turn pin on.
      } else {
      PORTD &= ~(1 << (colour + outputBase)); // turn pin off.
    }
  }
}

unsigned char valueToLevel(float value) {
  if (value < 0) value = 0;
  if (value > 1) value = 1;
  unsigned char level=256*value;//256*(powf(10,-2*(1-value)));
//  Serial.println(level,DEC);
  return level;
}

float ranFloat(float maxVal) {
  return random(65536)*maxVal/65536;
}

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
}

void setRGB(float red,float green, float blue, unsigned char rgb[]) {
  rgb[0]=valueToLevel(red);
  rgb[1]=valueToLevel(green);
  rgb[2]=valueToLevel(blue);
}

void setHSV(float hue, float saturation, float value, unsigned char rgb[]) {
  // hsv to rgb from wikipedia algorithm. hue,saturation,value are all floats 0<=h,s,v<=1.
  float c=value*saturation;
  float h_=hue*6;
  float x=c*(1-abs(fmodf(h_,2)-1));
  float m=value-c;
  switch ((char)h_) {
    case 0:
      setRGB(c+m,x+m,m,rgb);
      break;
    case 1:
      setRGB(x+m,c+m,m,rgb);
      break;
    case 2:
      setRGB(m,c+m,x+m,rgb);
      break;
    case 3:
      setRGB(m,x+m,c+m,rgb);
      break;
    case 4:
      setRGB(x+m,m,c+m,rgb);
      break;
    case 5:
      setRGB(c+m,m,x+m,rgb);
      break;
  }
}

void colourChange(float a, float b, float c) {
  switch (mode) {
    case 0:
      setHSV(a,b,c,out);
      break;
    case 1:
      setRGB(a,b,c,out);
      break;
    case 2:
      chaosCycle(a,b,c,out);
      break;
  }
}  

void loop() {
  float a, b, c;
  a=(float)analogRead(analogBase)/1024;
  b=(float)analogRead(analogBase+1)/1024;
  c=(float)analogRead(analogBase+2)/1024;
  colourChange(a,b,c);
  delay(10);
}


