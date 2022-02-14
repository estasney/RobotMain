#include "Arduino.h"
#include "RoboMotor.h"

RoboMotor::RoboMotor(uint8_t pinA, uint8_t pinB, uint8_t pinPulse, bool leftSide, int decayX, int decayY) {
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinPulse, OUTPUT);
    _pinA = pinA;
    _pinB = pinB;
    _pinPulse = pinPulse;
    _pinAState = LOW;
    _pinBState = LOW;
    _pulseState = 0;
    _vecX = 0;
    _vecY = 0;
    _leftSide = leftSide;
    _decayX = decayX;
    _decayY = decayY;
}

void RoboMotor::_writeLogicPins() const {
    digitalWrite(_pinA, _pinAState);
    digitalWrite(_pinB, _pinBState);
}

void RoboMotor::_writeThrottle() {
    analogWrite(_pinPulse, abs(_pulseState));
}

void RoboMotor::_setThrottle() {
    
    int16_t vec = _vecY;
    if (_leftSide) {
        vec += _vecX;
    } else {
        vec -= _vecX;
    }

    _pulseState = constrain(vec, -255, 255);

    if (_pulseState < 0) {
        reverse();
    } else if (_pulseState == 0) {
//        brake();
        coast();
    } else {
        forward();
    }
    _writeThrottle();
}

/**
 * @brief Pass throttleVector to update throttle. Effect is cumulative
 * @param deltaX
 *  Represents Left/Right
 * @param deltaY
 *  Represents Forward/Back
 */
void RoboMotor::updateThrottle(int8_t deltaX, int8_t deltaY) {

    /*
     Skid Steering

     For each of deltaX, deltaY, we construct a vector:

     deltaX
     ------
     [Left, Right] if _leftSide else [Right, Left]

     deltaY
     ------
     [Forward, Back] regardless of _leftSide
    */
    int16_t tempVectorX = _vecX + deltaX;
    int16_t tempVectorY = _vecY + deltaY;
    _vecX = constrain(tempVectorX, -255, 255);
    _vecY = constrain(tempVectorY, -255, 255);
    _setThrottle();
}

void RoboMotor::begin() {
    pinMode(_pinA, OUTPUT);
    pinMode(_pinB, OUTPUT);
    pinMode(_pinPulse, OUTPUT);
    _writeLogicPins();
    analogWrite(_pinPulse, _pulseState);
}

void RoboMotor::forward() {
    _pinAState = LOW;
    _pinBState = HIGH;
    _writeLogicPins();
}

void RoboMotor::reverse() {
    _pinAState = HIGH;
    _pinBState = LOW;
    _writeLogicPins();
}

void RoboMotor::coast() {
    _pinAState = HIGH;
    _pinBState = HIGH;
    _writeLogicPins();
}

void RoboMotor::brake() {
    _pinAState = LOW;
    _pinBState = LOW;
    _writeLogicPins();
}

int RoboMotor::getSpeed() {
    return _pulseState;
}

void RoboMotor::tick(bool tickX, bool tickY, uint8_t dx, uint8_t dy) {

    if (!tickX && !tickY) {
        return;
    }

    if (tickX) {
        if (_vecX > dx) {
            _vecX -= dx;
        } else if (_vecX < (0 - dx)) {
            _vecX += dx;
        } else {
            _vecX = 0;
        }
    }
    if (tickY) {
        if (_vecY > dy) {
            _vecY -= dy;
        } else if (_vecY < (0 - dy)) {
            _vecY += dy;
        } else {
            _vecY = 0;
        }
    }
    _setThrottle();

}

