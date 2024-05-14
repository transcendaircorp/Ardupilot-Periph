#pragma once

#include <AP_Common/AP_Common.h>
class AngleSensor
{
public:
    virtual bool init() = 0;
    virtual uint8_t get_sensor_id() = 0;
    virtual float get_angle() = 0;
};
