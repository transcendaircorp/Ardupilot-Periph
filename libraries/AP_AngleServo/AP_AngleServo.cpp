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
    AP_SUBGROUPINFO(controllers[0], "1", 1, AngleServo, AngleServoController),
    // @Group: 2
    // @Path: AngleServoController.cpp
    AP_SUBGROUPINFO(controllers[1], "2", 2, AngleServo, AngleServoController),
    // @Group: 3
    // @Path: AngleServoController.cpp
    AP_SUBGROUPINFO(controllers[2], "3", 3, AngleServo, AngleServoController),
    // @Group: 4
    // @Path: AngleServoController.cpp
    AP_SUBGROUPINFO(controllers[3], "4", 4, AngleServo, AngleServoController),

    AP_GROUPEND
};

AngleServo::AngleServo()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_AngleServo must be singleton");
    }
    _singleton = this;
}

void AngleServo::init()
{
    for (auto &angleServo : controllers) {
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
    for (auto &controller : controllers) {
        if (controller.is_valid())
            controller.update();
        if (debug && controller._debug) {
            Debug("sensor id: %d, angle: %f, target angle: %f, output value: %f\n",
                  controller._sensor->get_sensor_id(),
                  controller.get_angle(),
                  controller.get_target_angle(),
                  controller._output_value);
        }
    }
    // set the output values for the servos
    for (auto &controller : controllers) {
        SRV_Channels::set_output_norm(
            SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + controller._actuator_id - 1),
            controller._output_value);
    }
    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
}

bool AngleServo::rcout_srv(uint8_t actuator_id, const float command_value)
{
    for (auto &controller : controllers) {
        if (controller._actuator_id == actuator_id) {
            controller._command_value = command_value;
            return true;
        }
    }
    return false;
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