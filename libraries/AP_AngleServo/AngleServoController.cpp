#include "AngleServoController.h"
#include "A1335.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AngleServoController::var_info[] = {
    // @Param: _SENS_ADDR
    // @DisplayName: Sensor address
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_SENS", 0, AngleServoController, _sensor_address, 0),
    // @Param: _OFFSET
    // @DisplayName: Zero offset
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("_OFFSET", 1, AngleServoController, _angle_offset, 0),
    // @Param: _REVERSE
    // @DisplayName: Reverse direction of angle
    // @User: Standard
    AP_GROUPINFO("_REV", 2, AngleServoController, _reverse, 0),
    // @Param: _PROP
    // @DisplayName: Proportional gain
    // @Range: -100 100
    // @User: Standard
    AP_GROUPINFO("_PROP", 3, AngleServoController, _prop_gain, 5),
    // @Param: _INT
    // @DisplayName: Integral gain
    // @Range: -100 100
    // @User: Standard
    AP_GROUPINFO("_INT", 4, AngleServoController, _int_gain, 5),
    // @Param: _DIFF
    // @DisplayName: Differential gain
    // @Range: -100 100
    // @User: Standard
    AP_GROUPINFO("_DIFF", 5, AngleServoController, _diff_gain, 1),
    // @Param: _MAX_ANG
    // @DisplayName: Maximum angle
    // @Range: 0 90
    // @User: Standard
    AP_GROUPINFO("_MAXANG", 6, AngleServoController, _max_angle, 45),
    AP_GROUPEND
};


AngleServoController::AngleServoController()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AngleServoController::init()
{
    _sensor = new A1335::Sensor(_sensor_address);
    if (!_sensor->init()) {
        hal.console->printf("Failed to init sensor\n");
    }
    _last_update = AP_HAL::native_micros64();
}


const float iMax = 100;
const float iMin = -100;
void AngleServoController::update()
{
    uint64_t now = AP_HAL::native_micros64();
    float dt = (float)(now - _last_update)/1000000.0;
    _last_update = now;
    // basic PID controller
    float error = get_target_angle() - get_angle();
    // proportional
    float p = _prop_gain * error;
    // integral, clamp to prevent windup
    _integral += error * dt;
    _integral = _integral > iMax ? iMax :
                _integral < iMin ? iMin :
                _integral;
    float i = _int_gain * _integral;
    // differential
    float d =  _diff_gain * (error - _last_error) / dt;
    _last_error = error;

    // update output value, scale down by 1000 to bring numbers into more manageable range
    _output_value -= (p + i + d)/1000.0f;
    _output_value = _output_value > 1.0 ? 1.0 :
                    _output_value < -1.0 ? -1.0 :
                    _output_value;
    // if output is NaN, set to 0
    if (_output_value != _output_value) {
        _output_value = 0;
    }
}

float AngleServoController::get_angle()
{
    return _sensor->get_angle() - _angle_offset;
}

float AngleServoController::get_target_angle()
{
    return _command_value * _max_angle;
}