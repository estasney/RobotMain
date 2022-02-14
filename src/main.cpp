#include "RoboMotor.h"
#include <Streaming.h>
#include <RF24.h>
#include <AsyncDelay.h>

#define VERBOSE_SPEED


/*
   FRONT LEFT (FL)     FRONT RIGHT (FR)
   Motor 1             Motor 2
   Board 1             Board 1

   REAR LEFT  (RL)     REAR RIGHT (RR)
   Motor 1             Motor 2
   Board 2             Board 2
*/

AsyncDelay tickTimer;
AsyncDelay receiveTimer;

#define DECAY_X 2
#define DECAY_Y 2
#define CE_PIN 36
#define CSN_PIN 37
#define RADIO_LEVEL RF24_PA_MAX
#define XY_THRESHOLD 30

const uint8_t dX = DECAY_X;
const uint8_t dY = DECAY_Y;

RoboMotor motorFL (24, 25, 5, true, dX, dY);
RoboMotor motorFR (28, 29, 6, false, dX, dY);
RoboMotor motorRL (44, 45, 2, true, dX, dY);
RoboMotor motorRR (48, 49, 3, false, dX, dY);

RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = {"R", "C"};
uint8_t radioNumber = 0;

const int XY_threshold = XY_THRESHOLD;

int16_t XY[2];
int16_t XYBuf[2];

bool dataLock = false;
bool newData = false;
bool decayX = false;
bool decayY = false;


void setup()
{
    Serial.begin(9600);
    while (!radio.begin()) {
        delay(1);
    } // hold in infinite loop
    memset(XY, 0, 2);
    memset(XYBuf, 0, 2);
    radio.openReadingPipe(1, address[!radioNumber]);
    radio.setPALevel(RADIO_LEVEL);
    radio.setPayloadSize(sizeof(XYBuf));
    radio.startListening();
    motorFL.begin();
    motorFR.begin();
    motorRL.begin();
    motorRR.begin();
#ifdef TESTING
    Serial << "Setup Complete" << endl;
#endif
    tickTimer.start(10, AsyncDelay::MILLIS);
    receiveTimer.start(1, AsyncDelay::MILLIS);
}
void receiveOneCommand() {
    if (radio.available()) {
        dataLock = true;
        radio.read(&XYBuf, sizeof(XYBuf));
        XY[1] = XYBuf[0];
        XY[0] = XYBuf[1];
        dataLock = false;
        newData = true;
#ifdef VERBOSE_SIGNAL
        Serial << "X: " << XY[0] << ", Y: " << XY[1] << endl;
#endif
    }
}

void handleXY() {
    if (!newData || dataLock) {
        return;
    }

    newData = false;
    int16_t X = XY[0];
    int16_t Y = XY[1];

    // Check if values meet threshold
    if (abs(X) < XY_threshold) {
        X = 0;
        decayX = true;
    }

    if (abs(Y) < XY_threshold) {
        Y = 0;
        decayY = true;
    }


    auto deltaX = (int8_t)map(X, -255, 255, -3, 3);
    auto deltaY = (int8_t)map(Y, -255, 255, -3, 3);
#ifdef VERBOSE_DELTA
    Serial << "deltaX: " << deltaX << ", deltaY: " << deltaY << endl;
#endif

    motorFL.updateThrottle(deltaX, deltaY);
    motorRL.updateThrottle(deltaX, deltaY);

    motorFR.updateThrottle(deltaX, deltaY);
    motorRR.updateThrottle(deltaX, deltaY);
#ifdef VERBOSE_SPEED
    Serial << "FL: " << motorFL.getSpeed() << ", FR: " << motorFR.getSpeed() << ", RL: " << motorRL.getSpeed() << ", RR: " << motorRR.getSpeed() << endl;
#endif

}


void loop() {
    if (receiveTimer.isExpired()) {
        receiveOneCommand();
        handleXY();
        receiveTimer.restart();
    }

    if (tickTimer.isExpired()) {
        motorFL.tick(decayX, decayY, DECAY_X, DECAY_Y);
        motorRL.tick(decayX, decayY, DECAY_X, DECAY_Y);
        motorFR.tick(decayX, decayY, DECAY_X, DECAY_Y);
        motorRR.tick(decayX, decayY, DECAY_X, DECAY_Y);
        decayX = false;
        decayY = false;
        tickTimer.restart();
    }


}