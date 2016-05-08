
#ifndef LcdMenu_h
#define LcdMenu_h

#include "Arduino.h"
#include <LiquidCrystal.h>
#include <Rotary.h>
#include <iterator>
#include <vector>

class MenuParamValue
{
  public:
    virtual void step(char, unsigned int) {}; // process encoder steps.
    virtual void print(LiquidCrystal *) {}; // print to display.
    virtual void setOption(char, const __FlashStringHelper*) {}; // set text of option value.
    void set(unsigned int); // set current value.
    unsigned int get(); // read current value.
 protected:
    unsigned int value; // current value.
};

class MenuParamValuePercent : public MenuParamValue {
  public:
    MenuParamValuePercent(unsigned int); // constructor.
    void step(char, unsigned int); // process encoder steps.
    void print(LiquidCrystal *); // print to display.
/*  private:
    unsigned int value; // current value.*/
};

class MenuParamValueOption : public MenuParamValue {
  public:
    MenuParamValueOption(unsigned int, char); // constructor.
    void step(char, unsigned int); // process encoder steps.
    void print(LiquidCrystal *); // print to display.
    void setOption(char, const __FlashStringHelper*); // set text of option value.
  private:
/*    unsigned int value; // current option value.*/
    std::vector<const __FlashStringHelper *> options; // vector of option strings.
    unsigned char length; // length of option list.
};

class MenuParam {
  public:
    MenuParam(const __FlashStringHelper*, MenuParamValue *); // constructor
    void print(LiquidCrystal *); // print to lcd
    MenuParamValue *getValue(); // get parameter value.
  private:
    MenuParamValue *value; // value of parameter.
    const __FlashStringHelper *name; // name of parameter
};

class MenuParamList {
  public:
    MenuParamList(unsigned char); // constructor.
    void set(unsigned char, MenuParam *); // set param at index.
    void step(char); // process encoder steps.
    MenuParam *getCurrent(); // get current parameter.
    MenuParam *get(unsigned char); // get parameter by index.
  private:
    std::vector<MenuParam *> items; // items in list. start empty then fill at run time.
    unsigned char current; // index of current item.
    unsigned char length; // length of list.
};

class MenuMode
{
  public:
    MenuMode(const __FlashStringHelper *, MenuParamList *, void (*)(MenuMode *)); // constructor.
    void print(LiquidCrystal *); // print to lcd
    void runMode(); // run the mode callback.
    MenuParamList *getParams(); // get parameter list.
  private:
    const __FlashStringHelper *name; // name of mode
    MenuParamList *params; // list of parameters.
    void (*callback)(MenuMode *); // function callback to run mode.
};

class MenuModeList
{
  public:
    MenuModeList(unsigned char); // constructor.
    void set(unsigned char, MenuMode *); // set mode at index.
    void step(char); // process encoder steps.
    void setCurrent(unsigned char mode); // set current mode.
    MenuMode *getCurrent(); // get current mode.
  private:
    std::vector<MenuMode *> items; // items in list. start empty then fill at run time.
    unsigned char current; // index of current item.
    unsigned char length; // length of list.
};

class LcdMenu
{
  public:
    LcdMenu(LiquidCrystal *, Rotary *, unsigned char, unsigned char, unsigned char, MenuModeList *); // constructor.
    void readEncoder(); // count encoder steps (fast).
    void process(); // process changes to menus.
    void print(); // print to display.
    void runMode(); // run current mode callback.
    void setMode(unsigned char mode); // set current mode.
    void setVisible(bool); // show or hide the lcd display.
  private:
    LiquidCrystal *lcd ; // lcd object
    Rotary *rot; // rotary encoder object
    MenuModeList *modes; // list of menu modes.
    unsigned char modePin; // pin for mode button
    unsigned char paramPin; // pin for parameter button
    unsigned char backlightPin; // pin for backlight control (LOW = ON).
    char steps; // number of encoder steps since last called.
    unsigned long oldTime; // time since process() was last called.
    unsigned long time; // time of current call to process().
    bool visible; // is display visible or hidden?
};


/*
class MenuParamValueOptions
*/

#endif
