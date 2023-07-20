#include "AP_AngleServo.h"
#include <AP_HAL/AP_HAL.h>
#include "SRV_Channel/SRV_Channel.h"

#ifdef HAL_BUILD_AP_PERIPH
extern "C" {
    void can_printf(const char *fmt, ...);
}
#define Debug(fmt, args...) can_printf(fmt, ##args)
#else
#define Debug(fmt, args...) do {} while(0)
#endif

extern const AP_HAL::HAL &hal;

// entries for _sensor_address[16]
const AP_Param::GroupInfo AngleServo::var_info[] = {
    // @Group: 1
    // @Path: AngleServoController.cpp
    AP_SUBGROUPINFO(_angle_servos[0], "1", 1, AngleServo, AngleServoController),
    // @Group: 2
    // @Path: AngleServoController.cpp
    AP_SUBGROUPINFO(_angle_servos[1], "2", 2, AngleServo, AngleServoController),
    // @Group: 3
    // @Path: AngleServoController.cpp
    AP_SUBGROUPINFO(_angle_servos[2], "3", 3, AngleServo, AngleServoController),
    // @Group: 4
    // @Path: AngleServoController.cpp
    AP_SUBGROUPINFO(_angle_servos[3], "4", 4, AngleServo, AngleServoController),

    AP_GROUPEND
};

AngleServo::AngleServo()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Airspeed must be singleton");
    }
    _singleton = this;
    for (int i = 0; i < sizeof(_angle_servos)/sizeof(_angle_servos[0]); i++) {
        _angle_servos[i]._actuator_id = i;
        _angle_servos[i]._actuator_func = SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + i);
    }
}

void AngleServo::init()
{
    for (auto &angleServo : _angle_servos) {
        angleServo.init();
    }
}

void AngleServo::update()
{
    bool debug = false;
    uint32_t now = AP_HAL::native_millis();
    if (!_since_last_debug) {
        _since_last_debug = now;
        debug = true;
    } else if ((now - _since_last_debug) >= 250) {
        _since_last_debug += 250;
        debug = true;
    }
    for (auto &angleServo : _angle_servos) {
        angleServo.update();
        if (angleServo._sensor->get_sensor_id() == 0) {
            continue;
        }
        if (debug) {
            Debug("sensor id: %d, angle: %f, target angle: %f, output value: %f\n",
                  angleServo._sensor->get_sensor_id(),
                  angleServo.get_angle(),
                  angleServo.get_target_angle(),
                  angleServo._output_value);
        }
    }
    // set the output values for the servos
    for (auto &angleServo : _angle_servos) {
        SRV_Channels::set_output_norm(angleServo._actuator_func, angleServo._output_value);
    }
    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
}

void AngleServo::rcout_srv(uint8_t actuator_id, const float command_value)
{
    _angle_servos[actuator_id]._command_value = command_value;
}

// singleton instance
AngleServo *AngleServo::_singleton;

namespace AP
{
AngleServo *angleservo()
{
    return AngleServo::get_singleton();
}
} // namespace AP