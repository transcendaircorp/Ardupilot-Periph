#include "AP_HBridge.h"
#include <GCS_MAVLink/GCS.h>

#ifdef HAL_BUILD_AP_PERIPH
    extern "C" {
      void can_printf(const char *fmt, ...);
    }
    #define Debug(fmt, args...) can_printf(fmt, ##args)
#else
    #define Debug(fmt, args...) do {} while(0)
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_HBridge::var_info[] = {
    // @Param: _ENABLED
    // @DisplayName: H-Bridge Enabled
    // @Description: Allows the H-Bridge to be enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_ENABLED", 0, AP_HBridge, _enabled, 0),

    // @Param: _REVERSE
    // @DisplayName: H-Bridge Reverse
    // @Description: Allows the H-Bridge to be reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_REVERSE", 1, AP_HBridge, _reverse, 0),

    // @Param: _DEBUG
    // @DisplayName: H-Bridge Debug
    // @Description: Allows the H-Bridge to be debugged
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_DEBUG", 2, AP_HBridge, _debug, 0),

    // @Param: _DBG_RATE
    // @DisplayName: H-Bridge Debug Rate
    // @Description: Allows the H-Bridge to be debugged at a rate
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_DBG_RATE", 3, AP_HBridge, _debug_rate, 1),

    // @Param: _TIMEOUT
    // @DisplayName: H-Bridge Calibration Timeout
    // @Description: The time in seconds to wait for the H-Bridge to calibrate
    // @Range: 0 30
    // @Units: s
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_TIMEOUT", 4, AP_HBridge, _calibration_timeout, 15),

    AP_GROUPEND
};


AP_HBridge::AP_HBridge()
{
    AP_Param::setup_object_defaults(this, var_info);

    if(_singleton) {
        AP_HAL::panic("AP_HBridge must be singleton");
    }
    _singleton = this;
}

void AP_HBridge::init()
{
    _since_last_debug = AP_HAL::native_millis();
    _calibrated = Starting;
}

void AP_HBridge::update()
{
    uint32_t now = AP_HAL::native_millis();
    if(_debug && (now - _since_last_debug) >= 1000/_debug_rate) {
        _since_last_debug += 1000/_debug_rate;
        Debug("H-Bridge: %s, %s, %u, %u, %u\n",
              _calibrated == Starting ? "Starting" :
              _calibrated == WaitingForAngle ? "WaitingForAngle" :
              _calibrated == MovingToH ? "MovingToH" :
              _calibrated == MovingToV ? "MovingToV" :
              _calibrated == Calibrating ? "Calibrating" :
              _calibrated == Failed ? "Failed" :
              _calibrated == Success ? "Success" : "Unknown",
              _move_in_progress ? "Moving" : "Stopped",
              (unsigned)(_current_angle*10), (unsigned)(_prev_angle*10), (unsigned)(_target_angle*10));
        Debug("Calibration: v - %u, h - %u", (unsigned)_vertical_angle, (unsigned)_horizontal_angle);
    }
    if(!_enabled) {
        return;
    }
    switch(_calibrated){
        case None:
            break;
        case Starting: {
            _since_previous_state = now;
            _current_angle = NAN;
            _vertical_angle = NAN;
            _horizontal_angle = NAN;
            _calibrated = WaitingForAngle;
            _move_in_progress = false;
            hal.gpio->write(1, 0);
            hal.gpio->write(2, 0);
            break;
        } case WaitingForAngle: {
            if((now - _since_previous_state) >= 1000*_calibration_timeout) { // timeout for getting angle
                _calibrated = Failed;
                break;
            }
            if(!isnan(_current_angle)) { //we got an angle, so proceed with calibration
                hal.gpio->write(1, 1*_reverse);
                hal.gpio->write(2, 1*!_reverse);
                _move_in_progress = true;
                _prev_angle = 0;
                _since_previous_state = now;
                _since_angle_changed = now;
                _calibrated = MovingToV;
            }
            break;
        } case MovingToV: {
            if((now - _since_previous_state) >= 1000*_calibration_timeout || (now - _since_angle_changed) >= 1000) { //if timeout reached
                _vertical_angle = _current_angle;
                hal.gpio->write(1, 1*!_reverse);
                hal.gpio->write(2, 1*_reverse);
                _move_in_progress = true;
                _since_previous_state = now;
                _since_angle_changed = now;
                _calibrated = MovingToH;
                break;
            }
            if(abs(_current_angle - _prev_angle) > 0.1) { //if wing has actually moved
                _prev_angle = _current_angle;
                _since_angle_changed = now;
            }
            break;
        } case MovingToH: {
            if((now - _since_previous_state) >= 1000*_calibration_timeout || (now - _since_angle_changed) >= 1000) { //if timeout reached
                _horizontal_angle = _current_angle;
                _calibrated = Calibrating;
                break;
            }
            if(abs(_current_angle - _prev_angle) > 0.1) { //if wing has actually moved
                _prev_angle = _current_angle;
                _since_angle_changed = now;
            }
            break;
        } case Calibrating: {
            bool vertGThorz = _vertical_angle < _horizontal_angle;
            float angle_diff = abs(_vertical_angle - _horizontal_angle);
            if(abs(angle_diff - 90) < 10 ) { //if the difference is about 90
                _direction = vertGThorz;
            } else if(abs(angle_diff - 270) < 10 ) { //if the difference is about 270, it has wrapped around, so we need to invert the vertGThorz
                _direction = !vertGThorz;
            } else { // if the angle is not near 90 or 270, calibration failed
                _calibrated = Failed;
                break;
            }
            _calibrated = Success;
            break;
        } case Failed:
        case Success: {
            control_hbridge();
            break;
        }
    }
}

void AP_HBridge::set_target_angle(float angle)
{
    _target_angle = angle;
}

void AP_HBridge::set_current_angle(float angle)
{
    if(_calibrated != Success) {
        _current_angle = angle;
        return;
    }
    _current_angle = fmod((_direction ? _horizontal_angle - angle : angle - _horizontal_angle) + 540, 360) - 180;
}

void AP_HBridge::control_hbridge()
{
    if(!_enabled || _calibrated != Success) {
        _move_in_progress = false;
        hal.gpio->write(1, 0);
        hal.gpio->write(2, 0);
        return;
    }

    float angle_diff = _target_angle - _current_angle;

    if(abs(angle_diff) < 0.5f){
        _move_in_progress = false;
        hal.gpio->write(1, 0);
        hal.gpio->write(2, 0);
        return;
    } else {
        _move_in_progress = true;
    }
    if((angle_diff > 0) ^ !_reverse) {
        // move forward
        hal.gpio->write(1, 1);
        hal.gpio->write(2, 0);
    }
    else {
        // move backward
        hal.gpio->write(1, 0);
        hal.gpio->write(2, 1);
    }
}

AP_HBridge *AP_HBridge::_singleton;
AP_HBridge *AP_HBridge::get_singleton()
{
    return _singleton;
}

namespace AP {

AP_HBridge *hbridge()
{
    return AP_HBridge::get_singleton();
}

};
