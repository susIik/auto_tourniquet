/**
 * File:        MyButton.h
 * Author:      Evo Annus
 * Created:     28.11.2025
 *
 * Description: Header for MyButton
 */

#ifndef MYBUTTON_H_
#define MYBUTTON_H_

#include <Arduino.h>

class MyButton {
    public:
        MyButton(int pin);
        int checkButton();
    private:
        int _pin;
        boolean _buttonVal = HIGH;   // value read from button
        boolean _buttonLast = HIGH;  // buffered value of the button's previous state
        boolean _DCwaiting = false;  // whether we're waiting for a double click (down)
        boolean _DConUp = false;     // whether to register a double click on next release, or whether to wait and click
        boolean _singleOK = true;    // whether it's OK to do a single click
        long _downTime = -1;         // time the button was pressed down
        long _upTime = -1;           // time the button was released
        boolean _ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
        boolean _waitForUp = false;        // when held, whether to wait for the up event
        boolean _holdEventPast = false;    // whether or not the hold event happened already
        boolean _longHoldEventPast = false;// whether or not the long hold event happened already
        constexpr static int debounce = 20;          // ms debounce period to prevent flickering when pressing or releasing the button
        constexpr static int DCgap = 250;            // max ms between clicks for a double click event
        constexpr static int holdTime = 1000;        // ms hold period: how long to wait for press+hold event
        constexpr static int longHoldTime = 3000;    // ms long hold period: how long to wait for press+hold event
};

#endif