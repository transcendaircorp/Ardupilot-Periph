/// @file   AP_HBridge.h
/// @brief  HBridge control library
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <SRV_Channel/SRV_Channel.h>

/// @class  AP_HBridge
/// @brief  Class managing the PWM to H-Bridge
class AP_HBridge {

public:
    AP_HBridge();
    enum CalibrationState {
        None,
        Starting,
        WaitingForAngle,
        MovingToH,
        MovingToV,
        Calibrating,
        Failed,
        Success
    };
    /* Do not allow copies */
    AP_HBridge(const AP_HBridge  &other) = delete;
    AP_HBridge &operator=(const AP_HBridge&) = delete;

    /// enabled - returns if the H-Bridge is enabled
    bool enabled() const { return _enabled; }

    /// move_in_progress - returns true if a the hbridge is moving
    bool move_in_progress() const { return _move_in_progress; }

    /// init - initialise the HBridge library
    void init();

    /// update - updates the H-Bridge with current and target angles
    void update();

    /// control_hbridge - control the H-Bridge
    void control_hbridge();

    /// set_target_angle - set the target angle for the H-Bridge
    void set_target_angle(float angle);

    /// set_current_angle - set the current angle of the H-Bridge
    void set_current_angle(float angle);

    /// get_state - returns the current state of the H-Bridge
    CalibrationState get_state() const { return _calibrated; }

    /// returns true if pre arm checks have passed
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const;

    // get singleton instance
    static AP_HBridge *get_singleton();

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_HBridge *_singleton;
    // Parameters
    AP_Int8     _enabled;             // 1 if H-bridge is enabled
    AP_Int8     _reverse;             // 1 if H-bridge is reversed
    AP_Int8     _calibration_timeout; // timeout for calibration
    AP_Int8     _debug;               // 1 if debug is enabled
    AP_Int8     _debug_rate;          // debug rate in Hz

    bool _move_in_progress;

    uint32_t _since_last_debug;

    float _horizontal_angle;
    float _vertical_angle;
    bool _direction;

    //calibration state machine
    CalibrationState _calibrated;
    float _prev_angle;
    uint32_t _since_angle_changed;
    uint32_t _since_previous_state;

    float _target_angle;
    float _current_angle;

};

namespace AP {
    AP_HBridge *hbridge();
};
