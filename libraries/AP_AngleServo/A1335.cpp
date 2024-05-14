#include "A1335.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

namespace A1335
{
bool Sensor::init()
{
    _dev = hal.i2c_mgr->get_device(0, _address);
    if (!_dev) {
        return false;
    }
    setup();
    return true;
}

void Sensor::setup()
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->set_retries(2);
    _dev->register_periodic_callback(1000000UL/50U, FUNCTOR_BIND_MEMBER(&Sensor::timer, void));
}

void Sensor::timer()
{
    auto angle = RawAngleRegister{ .Raw = _normalRead(NormalRegisters::ANG) }.Fields.Angle/4096.0f*360.0f;
    WITH_SEMAPHORE(sem);
    _last_angle = angle;
}

float Sensor::get_angle()
{
    WITH_SEMAPHORE(sem);
    return _last_angle;
}

void Sensor::_normalWrite(NormalRegisters reg, uint16_t data) const
{
    if (!_dev) {
        return;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->transfer((uint8_t*)&reg, sizeof(reg), NULL, 0);
    _dev->transfer((uint8_t*)&data, sizeof(data), NULL, 0);
}

uint16_t Sensor::_normalRead(NormalRegisters reg) const
{
    if (!_dev) {
        return 0;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());
    uint16_t data;
    _dev->read_registers(reg, (uint8_t*)&data, sizeof(data));
    // reverse byte order
    data = ((data & 0xFF00) >> 8) | ((data & 0x00FF) << 8);
    return data;
}
void Sensor::_extendedWrite(ExtendedRegisters reg, uint32_t data) const
{
    WITH_SEMAPHORE(_dev->get_semaphore());
}
uint32_t Sensor::_extendedRead(ExtendedRegisters reg) const
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    return 0;
}
} // namespace A1335