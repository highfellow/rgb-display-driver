#include "LcdMenu.h"

void MenuParamValue::set(unsigned int valueArg) {
  value = valueArg;
}

unsigned int MenuParamValue::get() {
  return value;
}

MenuParamValuePercent::MenuParamValuePercent(unsigned int valueArg)  {
  value = valueArg;
}

void MenuParamValuePercent::step(char steps, unsigned int delay) {
  int accelSteps;
  unsigned int rate;
  char sign;
  if (steps >= 0) {
    accelSteps = steps;
    sign = 1;
  } else {
    accelSteps = -steps;
    sign = -1;
  }
  rate = delay / accelSteps;
  if (rate < 0x80) {
    accelSteps <<= ((0x80 - rate) >> 4);
  }
  accelSteps *= sign;
  if ((int)value + accelSteps < 0) {
    value = 0;
  } else if ((int)value + accelSteps > 1000) {
    value = 1000;
  } else {
    value += accelSteps;
  }
}

void MenuParamValuePercent::print(LiquidCrystal *lcd) {
  if (value < 1000) lcd->print(" ");
  if (value < 100) lcd->print(" ");
  lcd->print(value / 10);
  lcd->print(".");
  lcd->print(value % 10);
}

MenuParamValueOption::MenuParamValueOption(unsigned int valueArg, char lengthArg) {
  value = valueArg;
  length = lengthArg;
  options.reserve(length);
}

void MenuParamValueOption::step(char steps, unsigned int delay) {
  if ((int)value + steps < 0) {
    value = 0;
  } else if ((int)value + steps > length - 1) {
    value = length - 1;
  } else {
    value += steps;
  }
}

void MenuParamValueOption::print(LiquidCrystal *lcd) {
  lcd->print(options[value]);
}

void MenuParamValueOption::setOption(char index, const char *string) {
  options[index] = string;
}

MenuParam::MenuParam(const char *nameArg, MenuParamValue *valueArg) {
  name = nameArg;
  value = valueArg;
}

void MenuParam::print(LiquidCrystal * lcd) {
  lcd->print(name);
}

MenuParamValue *MenuParam::getValue() {
  return value;
}

MenuParamList::MenuParamList(unsigned char lengthArg) {
  length = lengthArg;
  items.reserve(lengthArg);
  current = 0;
}

void MenuParamList::set(unsigned char index, MenuParam *param) {
  items[index] = param;
}

MenuParam *MenuParamList::get(unsigned char index) {
  return items[index];
}

void MenuParamList::step(char steps) {
  if ((current + steps < length) && (current + steps >= 0)) current += steps;
}

MenuParam *MenuParamList::getCurrent() {
  return items[current];
}

MenuMode::MenuMode(const char* nameArg, MenuParamList *paramsArg, void (*cbArg)(MenuMode *)) {
  name = nameArg;
  params = paramsArg;
  callback = cbArg;
}

void MenuMode::runMode() {
  (*callback)(this);
}

void MenuMode::print(LiquidCrystal * lcd) {
  lcd->print(name);
}

MenuParamList * MenuMode::getParams() {
  return params;
}

MenuModeList::MenuModeList(unsigned char lengthArg) {
  length = lengthArg;
  items.reserve(lengthArg);
  current = 0;
}

void MenuModeList::set(unsigned char index, MenuMode *mode) {
  items[index] = mode;
}

void MenuModeList::setCurrent(unsigned char mode) {
  current = mode;
}

void MenuModeList::step(char steps) {
  if ((current + steps < length) && (current + steps >= 0)) current += steps;
}

MenuMode *MenuModeList::getCurrent() {
  return items[current];
}

LcdMenu::LcdMenu(LiquidCrystal *lcdArg, Rotary *rotArg, unsigned char modePinArg, unsigned char paramPinArg, unsigned char backlightPinArg, MenuModeList *modesArg) {
  lcd = lcdArg;
  rot = rotArg;
  modePin = modePinArg;
  paramPin = paramPinArg;
  backlightPin = backlightPinArg;
  modes = modesArg;
  visible = true;
  steps = 0;
  oldTime = 0;
  time = 0;
  pinMode(modePin, INPUT_PULLUP);
  pinMode(paramPin, INPUT_PULLUP);
  pinMode(backlightPin, OUTPUT);
}

void LcdMenu::readEncoder() {
  if (!visible) return;
  switch (rot->process()) {
    case DIR_CW:
      steps++;
      break;
    case DIR_CCW:
      steps--;
      break;
  }
}

void LcdMenu::process() {
  unsigned int delay;
  if (!visible) return;
  time = millis();
  if ((unsigned long)(time - oldTime) >= 0x7fff) {
    oldTime = (unsigned long)(time - 0x7fff); // don't count very long delays.
  }
  delay = (unsigned int)(time - oldTime);
  if (steps != 0) {
    if (! digitalRead(modePin)) {
      modes->step(steps);
    } else if (! digitalRead(paramPin)) {
      modes->getCurrent()->getParams()->step(steps);
    } else {
      modes->getCurrent()->getParams()->getCurrent()->getValue()->step(steps, (unsigned int)delay);
    }
  steps = 0;
  oldTime = time;
  }
}

void LcdMenu::print() {
  if (!visible) return;
  lcd->setCursor(0, 0);
  modes->getCurrent()->print(lcd);
  lcd->setCursor(0, 1);
  modes->getCurrent()->getParams()->getCurrent()->print(lcd);
  lcd->setCursor(11, 1);
  modes->getCurrent()->getParams()->getCurrent()->getValue()->print(lcd);
}


void LcdMenu::runMode() {
  modes->getCurrent()->runMode();
}

void LcdMenu::setMode(unsigned char mode) {
  modes->setCurrent(mode);
}

void LcdMenu::setVisible(bool visArg) {
  if (visArg) {
    visible = true;
    digitalWrite(backlightPin, LOW);
    print();
  } else {
    visible = false;
    digitalWrite(backlightPin, HIGH);
    lcd->clear();
  }
}
