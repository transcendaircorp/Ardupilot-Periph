#include "AngleSensor.h"
#include "SRV_Channel/SRV_Channel.h"

class AngleServoController
{
public:
    friend class AngleServo;
    AngleServoController();
    void init();
    void update();
    bool is_valid();
    float get_angle();
    float get_target_angle();
    uint8_t get_actuator_id();
    static const struct AP_Param::GroupInfo var_info[];
private:
    // Params
    AP_Int8 _sensor_address;
    AP_Int8 _actuator_id;
    AP_Float _angle_offset;
    AP_Int8 _reverse;
    AP_Int8 _debug;
    AP_Float _prop_gain;
    AP_Float _int_gain;
    AP_Float _diff_gain;
    AP_Float _max_angle;

    // Runtime variables
    uint64_t _last_update = 0;
    float _last_error = 0;
    float _integral = 0;

    AngleSensor* _sensor;
    float _command_value;
    float _output_value = 0;
};