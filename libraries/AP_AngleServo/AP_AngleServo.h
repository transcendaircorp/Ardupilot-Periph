#pragma once

#include <AP_HAL/I2CDevice.h>

class AP_AngleServo {

public:
    // run all the control loops for the servos. Should be called as fast as possible from the main loop, at least 20Hz
    void update(void);
    static AP_AngleServo *get_singleton(void) { return _singleton; }
}

};


namespace AP {
    AP_AngleServo *angleservo();
}