#pragma once

#include <AP_Common/AP_Common.h>
#include "AngleServoController.h"
#include "A1335.h"

class AngleServoFunction
{
public:
};

class AngleServo
{
public:
    AngleServo();

    /* Do not allow copies */
    AngleServo(const AngleServo&) = delete;
    AngleServo &operator=(const AngleServo&) = delete;

    // initialise the servos
    void init();
    // run all the control loops for the servos. Should be called as fast as possible from the main loop, at least 20Hz
    void update();
    // set the output value for a servo
    bool rcout_srv(uint8_t actuator_id, const float command_value);
    static AngleServo *get_singleton()
    {
        return _singleton;
    }
    AngleServoController controllers[4];
    static const struct AP_Param::GroupInfo var_info[];
private:
    static AngleServo *_singleton;

    uint32_t _since_last_debug;
};

namespace AP
{
AngleServo *angleservo();
}