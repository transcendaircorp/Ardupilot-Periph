#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include "AngleSensor.h"

namespace A1335
{
enum NormalRegisters : uint8_t {
    EWA = 0x02,    // Extended Write Address
    EWD = 0x04,    // Extended Write Data
    EWCS = 0x08,   // Extended Write Control and Status
    ERA = 0x0A,    // Extended Read Address
    ERCS = 0x0C,   // Extended Read Control and Status
    ERD = 0x0E,    // Extended Read Data
    CTRL = 0x1E,   // Device control
    ANG = 0x20,    // Current angle and related data
    STA = 0x22,    // Device status
    ERR = 0x24,    // Device error status
    XERR = 0x26,   // Extended error status
    TSEN = 0x28,   // Temperature sensor data
    FIELD = 0x2A,  // Magnetic field strength
    ERM = 0x34,    // Device error status masking
    XERM = 0x36    // Extended error status masking
};

enum ExtendedRegisters : uint16_t {
    // SRAM Registers
    SRAM_CMDSTATUS = 0x0000,               // Command status
    SRAM_MAXMINANGLE = 0x0001,             // [31:16] MaxAngle   [15:0] MinAngle
    SRAM_CLAMP = 0x0002,                   // [31:16] ClampHi    [15:0] ClampLo
    SRAM_GAINOFFSET_GAIN = 0x0003,         // [31:16] GainOffset [15:0] Gain
    SRAM_BAD_TSEN_ANGLE = 0x0004,          // [31:16] BadTsen    [15:0] BadAngle
    SRAM_MISCSTATUS_BADMAGSENSE = 0x0005,  // [31:16] MiscStatus [15:0] BadMagSense
    SRAM_ALGENB_ZEROOFFSET = 0x0006,       // [31:16] AlgEnb     [15:0] ZeroOffset
    SRAM_FILTERROR_FILTNUM1 = 0x0007,      // [31:16] FiltError  [15:0] FiltNum1
    SRAM_FILTEXP_FILTNUM2 = 0x0008,        // [23:16] FiltExp    [15:0] FiltNum2
    SRAM_HLINMAX_FILTNUM3 = 0x0009,        // [23:16] HLinMax    [15:0] FiltNum3
    SRAM_MAGSENSEHI_FILTDEN2 = 0x000A,     // [31:16] MagSenseHi [15:0] FiltDen2
    SRAM_MAGSENSELO_FILTDEN3 = 0x000B,     // [31:16] MagSenseLo [15:0] FiltDen3
    SRAM_LIN1 = 0x000C,                    // Dependent on linear vs harmonic mode
    SRAM_LIN2 = 0x000D,                    // Dependent on linear vs harmonic mode
    SRAM_LIN3 = 0x000E,                    // Dependent on linear vs harmonic mode
    SRAM_LIN4 = 0x000F,                    // Dependent on linear vs harmonic mode
    SRAM_LIN5 = 0x0010,                    // Dependent on linear vs harmonic mode
    SRAM_LIN6 = 0x0011,                    // Dependent on linear vs harmonic mode
    SRAM_LIN7 = 0x0012,                    // Dependent on linear vs harmonic mode
    SRAM_LIN8 = 0x0013,                    // Dependent on linear vs harmonic mode
    SRAM_LIN9 = 0x0014,                    // Dependent on linear vs harmonic mode
    SRAM_LIN10 = 0x0015,                   // Dependent on linear vs harmonic mode
    SRAM_LIN11 = 0x0016,                   // Dependent on linear vs harmonic mode
    SRAM_LIN12 = 0x0017,                   // Dependent on linear vs harmonic mode
    SRAM_LIN13 = 0x0018,                   // Dependent on linear vs harmonic mode
    SRAM_LIN14 = 0x0019,                   // Dependent on linear vs harmonic mode
    SRAM_LIN15 = 0x001A,                   // Dependent on linear vs harmonic mode

    // EEPROM Registers Map
    x306 = 0x0306,
    x307 = 0x0307,
    x308 = 0x0308,
    x309 = 0x0309,
    x30A = 0x030A,
    x30B = 0x030B,
    x30C = 0x030C,
    x30D = 0x030D,
    x30E = 0x030E,
    x30F = 0x030F,
    x310 = 0x0310,
    x311 = 0x0311,
    x312 = 0x0312,
    x313 = 0x0313,
    x314 = 0x0314,
    x315 = 0x0315,
    x316 = 0x0316,
    x317 = 0x0317,  // [23:12] Customer
    x318 = 0x0318,
    x319 = 0x0319,

    // Other Registers
    ORATE = 0xFFD0,   // Output rate
    UNLOCK = 0xFFFE,  // Unlock EEPROM
};

// Normal register bit definitions
union RawControlRegister {
    uint16_t Raw;
    struct ControlRegisterFields {
        uint16_t Keycode : 8;              // Keycode for writing to the control register
        uint16_t ClearErrors : 1;          // Commands reset of all previously read error status flags. Keycode must be 0x46
        uint16_t ClearExtendedErrors : 1;  // Commands reset of all previously read extended address error status flags.
        // Keycode must be 0x46
        uint16_t ClearStatusFlags : 1;     // Commands reset of the reset status flags. Keycode must be 0x46
        uint16_t : 1;                      // Unused
        uint16_t SoftReset : 1;            // Commands a soft reset. Keycode must be 0xB9
        uint16_t HardReset : 1;            // Commands a hard reset. Keycode must be 0xB9
        uint16_t ProcessorMode : 2;        // Processor mode. Keycode must be 0x46
    } Fields;
};

union RawErrorRegister {
    uint16_t Raw;
    struct ErrorRegisterFields {
        uint16_t ML : 1;          // Magnetic sense low error
        uint16_t MH : 1;          // Magnetic sense high error
        uint16_t UV : 1;          // Undervoltage error
        uint16_t OV : 1;          // Overvoltage error
        uint16_t AL : 1;          // Angle low error
        uint16_t AH : 1;          // Angle high error
        uint16_t AT : 1;          // Angle processing error
        uint16_t NR : 1;          // Processor is not actively processing angle data
        uint16_t CRC : 1;         // Incoming SPI CRC error. Packet was discarded.
        uint16_t IER : 1;         // Invalid EEPROM read error
        uint16_t XOV : 1;         // EEPROM write error
        uint16_t XER : 1;         // EEPROM read error
        uint16_t RegisterID : 4;  // Always 0
    } Fields;
};

union RawAngleRegister {
    uint16_t Raw;
    struct AngleRegisterFields {
        uint16_t Angle : 12;      // Encoded angle reading (n * 360/4096 = angle in deg.)
        uint16_t Parity : 1;      // Odd parity bit for the whole register
        uint16_t NewFlag : 1;     // A new angle is in the angle register
        uint16_t ErrorFlag : 1;   // At least one error in register 0x24
        uint16_t RegisterID : 1;  // Always 0
    } Fields;
};

union RawProcessorStatusRegister {
    uint16_t Raw;
    struct ProcessorStatusRegisterFields {
        uint16_t Phase : 4;         // 0000 Idle; 0001 Processing angles; Only in Self-test mode
        uint16_t Status : 4;        // 0000 Booting; 0001 Idle or Processing angles; 1110 Self-testmode
        uint16_t ErrorFlag : 1;     // At least one error in register 0x24
        uint16_t NewFlag : 1;       // A new angle is in the angle register
        uint16_t SoftReset : 1;     // There was a soft reset since last field reset
        uint16_t PowerOnReset : 1;  // There was a power-on reset since last field reset
        uint16_t RegisterID : 4;    // Always 1000
    } Fields;
};

union RawTemperatureRegister {
    uint16_t Raw;
    struct TemperatureRegisterFields {
        uint16_t Temperature : 12;  // Encoded temperature reading (n / 8 = temperature in K)
        uint16_t RegisterID : 4;    // Always 1111
    } Fields;
};

union RawFieldStrengthRegister {
    uint16_t Raw;
    struct FieldStrengthRegisterFields {
        uint16_t FieldStrength : 12;  // Encoded field strength reading (n = field strenght in Gauss)
        uint16_t RegisterID : 4;      // Always 1110
    } Fields;
};

// Extended register bit definitions
union Raw0x319Register {
    uint32_t Raw;
    struct RegisterFields {
        uint32_t DR : 1;
        uint32_t DAW : 1;
        uint32_t I2C_Slave_Address : 5;  // 5 MSB of the I2C slave address
        uint32_t XS : 1;
        uint32_t FA : 1;
        uint32_t DA : 1;
        uint32_t ZS : 1;
        uint32_t NS : 1;
        uint32_t SCN_MODE : 3;
        uint32_t CIS : 1;
        uint32_t DATA_MODE : 3;
        uint32_t PO : 1;
        uint32_t SDRV : 3;
        uint32_t IS : 1;
    } Fields;
};

union RawAlgEnbZeroOffsetRegister {
    uint32_t Raw;
    struct RegisterFields {
        uint32_t ZeroOffset : 16;
        uint32_t ALGENB : 16;
    } Fields;
};
class Sensor : public virtual AngleSensor
{
public:
    Sensor(uint8_t address) : _address(address) {}
    bool init() override;
    float get_angle() override;
    uint8_t get_sensor_id() override
    {
        return _address;
    }
    void timer();
    HAL_Semaphore sem;
private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t _address;

    float  _last_angle = 0;

    void setup();

    void _normalWrite(NormalRegisters reg, uint16_t data) const;
    uint16_t _normalRead(NormalRegisters reg) const;
    void _extendedWrite(ExtendedRegisters reg, uint32_t data) const;
    uint32_t _extendedRead(ExtendedRegisters reg) const;
};
} // namespace A1335
