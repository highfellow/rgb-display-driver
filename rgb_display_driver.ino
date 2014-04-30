// sketch to drive rgb LED ribbon


// Uses clock in normal mode (no PWM)

// include the library code:
#include <LiquidCrystal.h>
#include <Rotary.h>
#include <LcdMenu.h>
#include <avr/interrupt.h>
#include <math.h>

#define UNITY 4096 // fixed point unity value.
#define UNITY_BITS 12 // same expressed as a power of 2.
#define IN_POINTS 2 // binary points to add to input values.
#define OUT_POINTS 4 // binary points to remove from output values.
#define DIAG_PIN 13 // diagnostic output pin.
#define OUTPUT_BASE 2 // output pin to count from.
#define CHAN_BASE 3 // analog pin of first frequency channel
#define MODES 4 // number of driver modes.
#define MAX_TIME 15000 // maximum fade time in 30000ths of a second
#define LEVEL_SAMPS 300000 // samples to average level over.
#define DECAY_MAX 4084 // max and min decay ratios for audio channels
#define DECAY_MIN 2048 // in units of UNITY
#define CHAN_MIN 200 // min and max values of channel decay towards this
                     // value over approx 5 mins
#define MINMAX_DECAY 4086 // decay ratio with 5 min half life when done every second.
#define RAN_SMOOTH 256 // used when smoothing colour fade to stop flicker.
#define RAN_SPEED 500 // maximum colour fade speed.

// clock rate: 15.625kHz
// refresh rate: ~61.0Hz
// analog read rate: ~244Hz

// defines for the meanings of the channels in non-RGB colour mode.
// RGB mode:
#define CHAN_RED 0 // red channel
#define CHAN_GREEN 1 // green channel
#define CHAN_BLUE 2 // blue channel
// HSV mode:
#define PARM_VALUE 0 // hsv value
#define PARM_SAT 1 // hsv saturation
#define PARM_HUE 2 // hsv hue
// Colour fade mode:
#define PARM_SPEED 1 // colour cycle speed.
// Colour organ mode:
#define PARM_DECAY 1 // speed of change under peak level.
#define PARM_PEAKY 2 // ratio of peaks to whole signal.

volatile int16_t out[] = {0, 0, 0}; // rgb output value.
volatile int16_t count = 0; // PWM count.
int16_t chanVals[3]; // values from audio filter channels.
char mode = 3; // colour changing mode.
volatile char doLoop = false;
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(5, 6, 7, 8, 9, 10);
// set up rotary encoder.
Rotary rot(11, 12);
LcdMenu *menu;

ISR(TIMER2_COMPA_vect) {
  // Interrupt service routine for timer 2 overflow.
  //PORTB |= 1 << DIAG_PIN - 8;
  count++;
  for (int16_t chan = 0; chan < 3; chan++) {
    if (count < out[chan]) {
      PORTD |= 1 << (chan + OUTPUT_BASE); // turn pin on.
      } else {
      PORTD &= ~(1 << (chan + OUTPUT_BASE)); // turn pin off.
    }
  }
  // this expression looks ugly but mostly should be evaluated at compile time.
  if ((count & (0xffff >> (16 - UNITY_BITS + OUT_POINTS + 2))) == 0) {
    doLoop = true; // tell the main loop to run again.
  }
  if (count >= (UNITY >> OUT_POINTS)) {
    count = 0;
  }
  menu->readEncoder();
  //PORTB &= ~(1 << DIAG_PIN - 8);
}

void getModeVals(MenuMode *mode, int16_t *parms, char count) {
  // get first <count> values from current mode into an array.
  MenuParamList *params;
  params = mode->getParams();
  for (unsigned char i = 0; i < count; i++) {
    parms[i] = (((int64_t)params->get(i)->getValue()->get() * 1023) / 1000) << IN_POINTS;
  }
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
  return ret;
}

void setOutput(int16_t vals[3]) {
  int16_t val;
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

void setRGB(int16_t vals[3]) {
  uint64_t exp;
  for (int16_t chan = 0; chan < 3; chan++) {
    exp = intExp((uint32_t)vals[chan] * 10);
    //if (printNow && chan == 0)
      //Serial.println(vals[chan]);
    vals[chan] = exp >> 10;
  }
  setOutput(vals);
}

void colourOrgan(MenuMode *mode) {
  static int16_t chanMins[] = {UNITY, UNITY, UNITY}; // minimum levels on channels
  static int16_t chanMaxes[] = {0, 0, 0}; // maximum levels on channels
  static int16_t chanBases[] = {0, 0, 0}; // levels of base (non-peak) parts of channels.
  static char second = 0;
  int16_t parms[3];
  int16_t decay, peaky, bright;
  getModeVals(mode, parms, 3);
  decay = intExp((uint32_t) parms[PARM_DECAY] * 10) >> 10;
  decay = DECAY_MAX - (((DECAY_MAX - DECAY_MIN) * (int64_t) decay) >> UNITY_BITS);
  // separate into fast peak part and decaying base value.
  peaky = parms[PARM_PEAKY];
  bright = intExp((uint32_t) parms[PARM_VALUE] * 10) >> 10;
  second++;
  if (second == 61) second = 0;
  for (char chan = 0; chan < 3; chan++) {
    int16_t val = chanVals[chan];
    int16_t peakVal, baseVal;
    // make min and max levels decay towards CHAN_MIN with a 5 minute half life.
    if (second == 0) {
      chanMins[chan] = CHAN_MIN + 
        ((((int64_t) chanMins[chan] - CHAN_MIN) * MINMAX_DECAY) >> UNITY_BITS);
      chanMaxes[chan] = CHAN_MIN + 
        ((((int64_t) chanMaxes[chan] - CHAN_MIN) * MINMAX_DECAY) >> UNITY_BITS);
    }
    if (val < chanMins[chan]) chanMins[chan] = val;
    if (val > chanMaxes[chan]) chanMaxes[chan] = val;
    val = ( ((int64_t) val - (int64_t) chanMins[chan]) << UNITY_BITS) /
      (chanMaxes[chan] - chanMins[chan]);
    peakVal = ((int64_t) val * (int64_t) peaky) >> UNITY_BITS;
    baseVal = ((int64_t) val * (UNITY - (int64_t) peaky)) >> UNITY_BITS;
    baseVal = (((int64_t) baseVal * (UNITY - (int64_t) decay)) + 
              ((int64_t) chanBases[chan] * (int64_t) decay)) >> UNITY_BITS;
    chanBases[chan] = baseVal;
    val = (((int64_t) peakVal + (int64_t) baseVal) * ((int64_t) bright)) >> UNITY_BITS;
    parms[chan] = intExp((uint32_t) val * 7) >> 7;
  }
  setOutput(parms);
}

void setHSV(MenuMode *mode) {
  // hsv to rgb from wikipedia algorithm.
  int16_t parms[3];
  int64_t c;
  int64_t x;
  int16_t h_;
  int16_t hmod;
  int64_t m;
  getModeVals(mode, parms, 3);
  parms[PARM_VALUE] = intExp((uint16_t) parms[PARM_VALUE] * 7) >> 7;
  c = ((int64_t) parms[PARM_VALUE] * (int64_t) parms[PARM_SAT]) >> UNITY_BITS;
  h_ = parms[PARM_HUE] * 6;
  hmod = h_ % (UNITY << 1) - UNITY;
  if (hmod < 0) hmod = -hmod;
  x = (c * (UNITY - (int64_t) hmod)) >> UNITY_BITS;
  m = parms[PARM_VALUE] - c;
  switch (h_ >> UNITY_BITS) {
    case 0:
      parms[CHAN_RED] = c + m;
      parms[CHAN_GREEN] = x + m;
      parms[CHAN_BLUE] = m;
      break;
    case 1:
      parms[CHAN_RED] = x + m;
      parms[CHAN_GREEN] = c + m;
      parms[CHAN_BLUE] = m;
      break;
    case 2:
      parms[CHAN_RED] = m;
      parms[CHAN_GREEN] = c + m;
      parms[CHAN_BLUE] = x + m;
      break;
    case 3:
      parms[CHAN_RED] = m;
      parms[CHAN_GREEN] = x + m;
      parms[CHAN_BLUE] = c + m;
      break;
    case 4:
      parms[CHAN_RED] = x + m;
      parms[CHAN_GREEN] = m;
      parms[CHAN_BLUE] = c + m;
      break;
    case 5:
      parms[CHAN_RED] = c + m;
      parms[CHAN_GREEN] = m;
      parms[CHAN_BLUE] = x + m;
      break;
  }
  setOutput(parms);
}

void colourCycle(MenuMode *mode) {
  static int16_t rgb[] = {UNITY, UNITY, UNITY}, smoothed[] = {UNITY, UNITY, UNITY};
  int16_t parms[3], outVals[3]; 
  int16_t delta, maxVal, abVal, norm, smooth;
  char maxChan;
  getModeVals(mode, parms, 3); // brightness, saturation, speed.
  parms[PARM_VALUE] = intExp((uint16_t) parms[PARM_VALUE] * 7) >> 7;
  parms[PARM_SPEED] = intExp((uint16_t) parms[PARM_SPEED] * 4) >> 4;
  delta = ((int64_t)parms[PARM_SPEED] * RAN_SPEED) >> UNITY_BITS;
  if (delta < 1) delta = 1;
  maxVal = 0;
  maxChan = -1;
  // add random delta and find max coord.
  for (char i = 0; i < 3; i++) {
    rgb[i] += random(-delta, delta); 
    abVal = abs(rgb[i]);
    if (abVal > maxVal) {
      maxVal = abVal;
      maxChan = i;
    }
  }
  // renormalise onto surface of unit cube.
  norm = ((int64_t)UNITY << UNITY_BITS) / maxVal;
  for (char i = 0; i < 3; i++) {
    rgb[i] = ( (int64_t)rgb[i] * norm) >> UNITY_BITS;
    // set output values. (smoothed to stop flicker at high speeds).
    smoothed[i] = ( ( ( (int64_t)smoothed[i] * (UNITY - RAN_SMOOTH) ) +
      ( (int64_t)abs(rgb[i]) * RAN_SMOOTH) ) )  >> UNITY_BITS;
    outVals[i] = ( (int64_t)smoothed[i] * parms[PARM_VALUE] ) >> UNITY_BITS;
  }
  setOutput(outVals);  
}

void setup() {
 // PORTD = 0; // reset output port
  for (char i=0; i<3; i++) {
    pinMode(OUTPUT_BASE+i, OUTPUT); // set control pins to output.
    digitalWrite(OUTPUT_BASE + i, LOW);
  }
  pinMode(DIAG_PIN, OUTPUT);
  analogReference(EXTERNAL);
  // Increase the analog sample rate to 76 kHz.
  //ADCSRA &= ~( _BV(ADPS0) | _BV(ADPS1) ); // clear bits 0 and 1.
  //ADCSRA |= _BV(ADPS2); // set bit 2.
  // Set up the timer.
  TCCR2A = _BV(WGM21); // normal mode
  TCCR2B = _BV(CS21); // 1/8 prescaler;
  OCR2A = 128; // interrupt at 15.625 Khz.
  TCNT2 = 0; // reset counter.
  // Enable interrupt on compare match
  TIMSK2 |= _BV(OCIE2A);
  // Start the USB Serial.
//  Serial.begin(115200);
  // set up the menu system.
  MenuModeList *modes;
  MenuMode *mode;
  MenuParamList *params;
  MenuParam *param;
  MenuParamValue *value;
  lcd.begin(16, 2);
  modes = new MenuModeList(3);
  params = new MenuParamList(3);
  value = new MenuParamValuePercent(1000);
  param = new MenuParam("Brightness", value);
  params->set(0, param);
  value = new MenuParamValuePercent(0);
  param = new MenuParam("Saturation", value);
  params->set(1, param);
  value = new MenuParamValuePercent(0);
  param = new MenuParam("Hue       ", value);
  params->set(2, param);
  value = new MenuParamValuePercent(0);
  mode = new MenuMode("Single Colour", params, &setHSV);
  modes->set(0, mode);
  params = new MenuParamList(2);
  value = new MenuParamValuePercent(1000);
  param = new MenuParam("Brightness", value);
  params->set(0, param);
  value = new MenuParamValuePercent(500);
  param = new MenuParam("Speed     ", value);
  params->set(1, param);
  mode = new MenuMode("Colour Fade  ", params, &colourCycle);
  modes->set(1, mode);
  params = new MenuParamList(4);
  value = new MenuParamValuePercent(1000);
  param = new MenuParam("Brightness", value);
  params->set(0, param);
  value = new MenuParamValuePercent(0);
  param = new MenuParam("Smoothing ", value);
  params->set(1, param);
  value = new MenuParamValuePercent(0);
  param = new MenuParam("Peakiness ", value);
  params->set(2, param);
  value = new MenuParamValueOption(0, 6);
  value->setOption(0, "rgb  ");
  value->setOption(1, "rbg  ");
  value->setOption(2, "grb  ");
  value->setOption(3, "gbr  ");
  value->setOption(4, "brg  ");
  value->setOption(5, "bgr  ");
  param = new MenuParam("Mapping   ", value);
  params->set(3, param);
  mode = new MenuMode("Colour Organ ", params, &colourOrgan);
  modes->set(2, mode);
  // Print a message to the LCD.
  menu = new LcdMenu(&lcd, &rot, 0, 1, modes);
  menu->print();
}

void loop() {
  static char chan = 0; // analog channel to read.
  static char doPrint = 0; // print menus when this is zero.
  while (!doLoop); // should be true at the refresh rate * 4
  //PORTB |= 1 << DIAG_PIN - 8;
  doLoop = false;
  // read audio channels quickly
  menu->process(); // process encoder steps and set menu state.
  if (chan < 3) {
    chanVals[chan] = analogRead(CHAN_BASE + chan) << IN_POINTS;
  } else {
    menu->runMode(); // run mode output callback.
    //setOutput(vals);
    doPrint++;
    if (doPrint > 5) {
      doPrint = 0;
      menu->print(); // print menu state.
    }
  }
  chan++;
  if (chan > 3) {
    chan = 0;
  }
  //PORTB &= ~(1 << DIAG_PIN - 8);
}


