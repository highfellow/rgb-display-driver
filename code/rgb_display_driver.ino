// sketch to drive rgb LED ribbon


// Uses clock in normal mode (no PWM)

// include the library code:
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <LiquidCrystal.h>
#include <Rotary.h>
#include <LcdMenu.h>

#define UNITY 4096 // fixed point unity value.
#define UNITY_BITS 12 // same expressed as a power of 2.
#define UNITY_SQR ((int32_t)1 << (UNITY_BITS * 2)) // unity squared.
#define IN_POINTS 2 // binary points to add to input values.
#define OUT_POINTS 4 // binary points to remove from output values.
#define ROTARY_PIN_1 12 // first pin for rotary encoder
#define ROTARY_PIN_2 13 // second pin for rotary encoder
#define OUT_PORT PORTD // port to use for PWM output.
#define OUT_DDR DDRD // data direction register for given port.
#define OUT_BASE 4 // offset of first PWM pin in given port.
#define CHAN_BASE 3 // analog pin of first frequency channel
#define MODES 4 // number of driver modes.
#define MAX_TIME 15000 // maximum fade time in 30000ths of a second
#define LEVEL_SAMPS 300000 // samples to average level over.
#define DECAY_MAX 4084 // max and min decay ratios for audio channels
#define DECAY_MIN 2048 // in units of UNITY
#define CHAN_MIN 100 // min values of channels decay towards this
                     // value over approx 5 mins
#define CHAN_SEP 50 // minimum separation between min and max
                    // channel values
#define MINMAX_DECAY 4086 // decay ratio with 5 min half life when done every second.
#define RAN_SMOOTH 256 // used when smoothing colour fade to stop flicker.
#define RAN_SPEED 500 // maximum colour fade speed.
#define ENABLE_PULLUPS // for the rotary library.
#define FLICKER_BASE_SINGLE 3 // base of flicker params for single 
  // colour mode
#define FLICKER_BASE_FADE 2 // ditto for colour fade
#define FLICKER_BASE_RATE 2 // number of base time steps in one
  // fast flicker.


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
// Flicker modes
#define PARM_FL_SPEED 0 // flicker speed
#define PARM_FL_FADE 1 // flicker fading
#define PARM_FL_DEPTH 2 // flicker depth

volatile int16_t out[] = {0, 0, 0}; // rgb output value.
volatile int16_t count = 0; // PWM count.
volatile unsigned char adcChan = 0; // adc channel
volatile int16_t chanVals[3]; // values from audio filter channels.
char mode = 1; // colour changing mode.
volatile char doLoop = false;
LcdMenu *menu;
LiquidCrystal *lcd;
Rotary *rot;

ISR(TIMER2_COMPA_vect) {
  // Interrupt service routine for timer 2 overflow.
  //PORTB |= 1 << DIAG_PIN - 8;
  uint8_t mask = 1 << OUT_BASE;
  count++;
  for (int16_t chan = 0; chan < 3; chan++) {
    if (count < out[chan]) {
      OUT_PORT |= mask; // turn pin on.
      } else {
      OUT_PORT &= ~(mask); // turn pin off.
    }
    mask <<= 1;
  }
  // this expression looks ugly but mostly should be evaluated at compile time.
  if ((count & (0xffff >> (16 - UNITY_BITS + OUT_POINTS + 2))) == 0) {
    // do ADC stuff.
    /*if (adcChan > 0) {
      // read result.
      chanVals[adcChan - 1] = ADCL | (ADCH << 8);
    }
    if (adcChan < 3) {
      // start conversion.
      ADMUX = adcChan; // set channel.
      ADCSRA |= _BV(ADSC); // start conversion.
    }
    adcChan++;
    adcChan &= 0b11;*/ // count from 0 to 3;
    doLoop = true; // tell the main loop to run again.
  }
  if (count >= (UNITY >> OUT_POINTS)) {
    //doLoop = true; // tell the main loop to run again.
    count = 0;
  }
  //PORTB &= ~(1 << DIAG_PIN - 8);
}

void encoderInterrupt() {
  menu->readEncoder();
}

void getModeVals(MenuMode *mode, int16_t *parms, char offset, 
  char count) {
  // get first <count> values from current mode into an array.
  MenuParamList *params;
  params = mode->getParams();
  for (unsigned char i = offset; i < offset + count; i++) {
    parms[i - offset] = (((int64_t)params->get(i)->getValue()->get() * 1023) / 1000) << IN_POINTS;
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

uint8_t lsfrBit() {
  static uint16_t lfsr = 1; // 16 bit linear feedback shift register
  uint16_t newBit;
  newBit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ 
    (lfsr >> 5) ) & 1;
  lfsr =  (lfsr >> 1) | (newBit << 15);
  return newBit;
}

int16_t addFlicker(MenuMode *mode, unsigned char parmBase) {
  static int32_t bright = UNITY_SQR - 1; // brightness
  static uint32_t flickerAcc = 0, fadeAcc = 0; // accumulators for
    // flicker and fading.
  static int32_t target = UNITY_SQR - 1; // target  brightness.
  int16_t parms[3];
  uint32_t rate, fadeRate;
  getModeVals(mode, parms, parmBase + 1, 3);
  rate = intExp( ((uint32_t)parms[PARM_FL_SPEED] * 6 + 
    4 * UNITY)) >> UNITY_BITS; // should go from 16 to 1024.
  fadeRate = rate * (intExp( (uint32_t)12 * UNITY -
    (uint32_t)parms[PARM_FL_FADE] * 8) >> UNITY_BITS);
    // should go from rate * 16 to rate * 4096
  flickerAcc += rate;
  fadeAcc += fadeRate;
  if (flickerAcc >= FLICKER_BASE_RATE * 1024) {
    flickerAcc -= FLICKER_BASE_RATE * 1024;
    if (lsfrBit() == 1) {
      target = UNITY_SQR - 1;
    } else {
      target = UNITY_SQR - 1 - 
        UNITY * (int32_t)parms[PARM_FL_DEPTH];
    }
  }
  if (fadeAcc >= FLICKER_BASE_RATE * 1024) {
    uint16_t compress = fadeAcc / (FLICKER_BASE_RATE * 1024);
    if (compress > 256) compress = 256;
    fadeAcc %= FLICKER_BASE_RATE * 1024;
    bright += (target - bright) * compress / 256;
  }
  return bright >> UNITY_BITS;
}

void colourOrgan(MenuMode *mode) {
  static int16_t chanMins[] = {UNITY, UNITY, UNITY};
    // minimum levels on channels
  static int16_t chanMaxes[] = {CHAN_SEP, CHAN_SEP, CHAN_SEP};
    // maximum levels on channels
  static int16_t chanBases[] = {0, 0, 0};
    // levels of base (non-peak) parts of channels.
  static char second = 0;
  int16_t parms[3];
  int16_t decay, peaky, bright;
  getModeVals(mode, parms, 0, 3);
  decay = intExp( (uint32_t)(UNITY - parms[PARM_DECAY]) * 10) >> 10;
  decay = DECAY_MAX - (((DECAY_MAX - DECAY_MIN) * (int64_t) decay)
    >> UNITY_BITS);
  // separate into fast peak part and smoothed base value.
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
        ((((int64_t) chanMins[chan] - CHAN_MIN) * MINMAX_DECAY)
        >> UNITY_BITS);
      chanMaxes[chan] = chanMins[chan] + CHAN_SEP +
        ((((int64_t) chanMaxes[chan] - chanMins[chan] - CHAN_SEP)
        * MINMAX_DECAY) >> UNITY_BITS);
    }
    if (val < chanMins[chan]) chanMins[chan] = val;
    if ((val > chanMins[chan] + CHAN_SEP) && 
      (val > chanMaxes[chan])) chanMaxes[chan] = val;
    val = ( ((int64_t) val - (int64_t) chanMins[chan]) << UNITY_BITS) /
      (chanMaxes[chan] - chanMins[chan]);
    peakVal = ((int64_t) val * (int64_t) peaky) >> UNITY_BITS;
    baseVal = ((int64_t) val * (UNITY - (int64_t) peaky)) >> UNITY_BITS;
    baseVal = (((int64_t) baseVal * (UNITY - (int64_t) decay)) + 
              ((int64_t) chanBases[chan] * (int64_t) decay)) >> UNITY_BITS;
    chanBases[chan] = baseVal;
    val = (((int64_t) peakVal + (int64_t) baseVal) * ((int64_t) bright)) >> UNITY_BITS;
    parms[chan] = intExp((uint32_t) val * 5) >> 5;
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
  getModeVals(mode, parms, 0, 3);
  parms[PARM_VALUE] = intExp((uint16_t) parms[PARM_VALUE] * 7) >> 7;
  parms[PARM_VALUE] = ((int32_t)parms[PARM_VALUE] * 
    addFlicker(mode, FLICKER_BASE_SINGLE)) >> UNITY_BITS;
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
  int16_t parms[2], outVals[3]; 
  int16_t delta, maxVal, abVal, norm, smooth;
  char maxChan;
  getModeVals(mode, parms, 0, 2); // brightness, saturation, speed.
  parms[PARM_VALUE] = intExp((uint16_t) parms[PARM_VALUE] * 7) >> 7;
  parms[PARM_VALUE] = ((int32_t)parms[PARM_VALUE] * 
    addFlicker(mode, FLICKER_BASE_FADE)) >> UNITY_BITS;
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

void addFlickerModes(MenuParamList *params, unsigned char parmBase) {
  MenuParam *param;
  MenuParamValue *value;
  value = new MenuParamValueOption(0, 2);
  value->setOption(0, F("off  "));
  value->setOption(1, F("on   "));
  param = new MenuParam(F("Flicker    "), value);
  params->set(parmBase, param);
  value = new MenuParamValuePercent(500);
  param = new MenuParam(F("Fl. speed  "), value);
  params->set(parmBase + 1, param);
  value = new MenuParamValuePercent(500);
  param = new MenuParam(F("Fl. fade   "), value);
  params->set(parmBase + 2, param);
  value = new MenuParamValuePercent(500);
  param = new MenuParam(F("Fl. depth  "), value);
  params->set(parmBase + 3, param);
}

void setup() {
  // set output pins to output mode.  
  for (char chan = OUT_BASE; chan < OUT_BASE + 3; chan++) {
    OUT_DDR |= 1 << chan;
  }
  //analogReference(EXTERNAL);
  // Increase the analog sample rate to 76 kHz.
  ADCSRA &= ~( _BV(ADPS0) | _BV(ADPS1) ); // clear bits 0 and 1.
  ADCSRA |= _BV(ADPS2); // set bit 2.
  // Set up the timer.
  TCCR2A = _BV(WGM21); // normal mode
  TCCR2B = _BV(CS21); // 1/8 prescaler;
  OCR2A = 128; // interrupt at 15.625 Khz. (refresh rate = 60 Hz).
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
  modes = new MenuModeList(3);
  params = new MenuParamList(7);
  value = new MenuParamValuePercent(1000);
  param = new MenuParam(F("Brightness"), value);
  params->set(0, param);
  value = new MenuParamValuePercent(0);
  param = new MenuParam(F("Saturation"), value);
  params->set(1, param);
  value = new MenuParamValuePercent(0);
  param = new MenuParam(F("Hue       "), value);
  params->set(2, param);
  addFlickerModes(params, FLICKER_BASE_SINGLE);
  value = new MenuParamValuePercent(0);
  mode = new MenuMode(F("Single Colour"), params, &setHSV);
  modes->set(0, mode);
  params = new MenuParamList(6);
  value = new MenuParamValuePercent(1000);
  param = new MenuParam(F("Brightness"), value);
  params->set(0, param);
  value = new MenuParamValuePercent(500);
  param = new MenuParam(F("Speed     "), value);
  params->set(1, param);
  addFlickerModes(params, FLICKER_BASE_FADE);
  mode = new MenuMode(F("Colour Fade  "), params, &colourCycle);
  modes->set(1, mode);
  params = new MenuParamList(4);
  value = new MenuParamValuePercent(1000);
  param = new MenuParam(F("Brightness"), value);
  params->set(0, param);
  value = new MenuParamValuePercent(0);
  param = new MenuParam(F("Smoothing "), value);
  params->set(1, param);
  value = new MenuParamValuePercent(0);
  param = new MenuParam(F("Peakiness "), value);
  params->set(2, param);
  value = new MenuParamValueOption(0, 6);
  value->setOption(0, F("rgb  "));
  value->setOption(1, F("rbg  "));
  value->setOption(2, F("grb  "));
  value->setOption(3, F("gbr  "));
  value->setOption(4, F("brg  "));
  value->setOption(5, F("bgr  "));
  param = new MenuParam(F("Mapping   "), value);
  params->set(3, param);
  mode = new MenuMode(F("Colour Organ "), params, &colourOrgan);
  modes->set(2, mode);
  // initialize the LCD library with the numbers of the pins
  lcd = new LiquidCrystal(7, 8, 9, 10, 11, 12);
  lcd->begin(16, 2);
  // set up rotary encoder.
  rot = new Rotary(2, 3);
  // initialise the menu system.
  /*lcd->setCursor(0, 0);
  lcd->print("top");
  lcd->setCursor(0, 1);
  lcd->print("bottom");*/
  menu = new LcdMenu(lcd, rot, A0, A1, 13, modes);
  menu->setMode(1);
  menu->print();
  attachInterrupt(digitalPinToInterrupt(2), encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderInterrupt, CHANGE);
}

void loop() {
  static char doPrint = 0; // print menus when this is zero.
  static char chan = 0;
//  static int heartbeat = 0;
  while (!doLoop); // should break at every refresh.
  //PORTB |= 1 << DIAG_PIN - 8;
  doLoop = false;
  if (chan < 3) {
    chanVals[chan] = analogRead(chan + CHAN_BASE);
  } else {
  // read audio channels quickly
    menu->process(); // process encoder steps and set menu state.
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
/*  heartbeat++;
  heartbeat %= 1024;
  lcd->setCursor(15,0);
  if ((heartbeat == 0))  {
    lcd->write(0x6d);
  }
  if (heartbeat == 511) {
    lcd->write(0x20);
  }*/
//   PORTB &= ~(1 << DIAG_PIN - 8);
}


