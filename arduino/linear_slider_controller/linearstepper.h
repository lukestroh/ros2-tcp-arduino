/*
 * Header file for LimitSwitch class
 * Author: Luke Strohbehn
 * Date: 4/12/23
 */

#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H

#include <stdint.h>
#include <Arduino.h>
#include <AccelStepper.h>

class LinearStepper: public AccelStepper {
    // private:

    public:
        void calibrate_stepper();
};

#endif