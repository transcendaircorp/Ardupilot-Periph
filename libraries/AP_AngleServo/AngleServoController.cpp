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
    // @Param: _ACT
    // @DisplayName: Actuator ID
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ACT", 3, AngleServoController, _actuator_id, 0),
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

bool AngleServoController::is_valid() {
    return _sensor->get_sensor_id() != 0 && _actuator_id != 0;
}

void AngleServoController::update()
{
    _output_value = _command_value;
}

float AngleServoController::get_angle()
{
    return (1 - _reverse*2) * _sensor->get_angle() - _angle_offset;
}

float AngleServoController::get_target_angle()
{
    return _command_value * _max_angle;
}

uint8_t AngleServoController::get_actuator_id(){
    return _actuator_id;
}