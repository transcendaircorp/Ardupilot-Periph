#include "AngleSensor.h"
#include "SRV_Channel/SRV_Channel.h"

class AngleServoController
{
public:
    friend class AngleServo;
    AngleServoController();
    void init();
    void update();
    float get_angle();
    float get_target_angle();
    static const struct AP_Param::GroupInfo var_info[];
private:
    // Params
    AP_Int8 _sensor_address;
    AP_Float _angle_offset;
    AP_Int8 _reverse;
    AP_Float _prop_gain;
    AP_Float _int_gain;
    AP_Float _diff_gain;
    AP_Float _max_angle;

    // Runtime variables
    uint64_t _last_update = 0;
    float _last_error = 0;
    float _integral = 0;

    AngleSensor* _sensor;
    SRV_Channel::Aux_servo_function_t _actuator_func;
    uint8_t _actuator_id;
    float _command_value;
    float _output_value = 0;
};