/**
 * File:        MyButton.cpp
 * Author:      Evo Annus
 * Created:     28.11.2025
 *
 * Description: MyButton class
 */

#include <MyButton.hpp>

MyButton::MyButton(int pin) {
    _pin = pin;
}


int MyButton::checkButton() {
    int event = 0;
    _buttonVal = digitalRead(_pin);
    // Button pressed down
    if (_buttonVal == LOW && _buttonLast == HIGH && (millis() - _upTime) > debounce)
    {
        _downTime = millis();
        _ignoreUp = false;
        _waitForUp = false;
        _singleOK = true;
        _holdEventPast = false;
        _longHoldEventPast = false;
        if ((millis()-_upTime) < DCgap && _DConUp == false && _DCwaiting == true)  _DConUp = true;
        else  _DConUp = false;
        _DCwaiting = false;
    }
    // Button released
    else if (_buttonVal == HIGH && _buttonLast == LOW && (millis() - _downTime) > debounce)
    {        
        if (not _ignoreUp)
        {
            _upTime = millis();
            if (_DConUp == false) _DCwaiting = true;
            else
            {
                event = 2;
                _DConUp = false;
                _DCwaiting = false;
                _singleOK = false;
            }
        }
    }
    // Test for normal click event: DCgap expired
    if ( _buttonVal == HIGH && (millis()-_upTime) >= DCgap && _DCwaiting == true && _DConUp == false && _singleOK == true && event != 2)
    {
        event = 1;
        _DCwaiting = false;
    }
    // Test for hold
    if (_buttonVal == LOW && (millis() - _downTime) >= holdTime) {
        // Trigger "normal" hold
        if (not _holdEventPast)
        {
            event = 3;
            _waitForUp = true;
            _ignoreUp = true;
            _DConUp = false;
            _DCwaiting = false;
            //_downTime = millis();
            _holdEventPast = true;
        } else { // Continius hold
            event = 4;
        }
        
    }
    _buttonLast = _buttonVal;
    return event;
}