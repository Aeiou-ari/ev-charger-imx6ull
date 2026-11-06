#ifndef MODBUS_RTU_58REGS_H
#define MODBUS_RTU_58REGS_H

#include <stdint.h>
#include <stdbool.h>

// Modbus寄存器结构体定义
typedef struct {
    // 参数设置区
    uint16_t paramPassword;
    uint8_t displayMode;
    uint8_t wiringMode;
    uint16_t ptRatio;
    uint16_t ctRatio;
    uint8_t commAddress;
    uint8_t baudRate;
    uint16_t dataFormat;
    uint16_t backlightTime;

    // 电压电流数据区
    uint16_t voltageA, voltageB, voltageC;
    uint16_t voltageAB, voltageBC, voltageCA;
    uint16_t currentA, currentB, currentC;
    uint16_t powerDirection;

    // 功率数据区
    uint16_t activePowerA, activePowerB, activePowerC;
    uint16_t totalActivePower;
    uint16_t reactivePowerA, reactivePowerB, reactivePowerC;
    uint16_t totalReactivePower;
    uint16_t apparentPowerA, apparentPowerB, apparentPowerC;
    uint16_t totalApparentPower;
    uint16_t powerFactorA, powerFactorB, powerFactorC;
    uint16_t totalPowerFactor;
    uint16_t frequency;

    // 电能数据区（32位）
    uint32_t forwardActiveEnergyInt;
    uint16_t forwardActiveEnergyDec;
    uint32_t reverseActiveEnergyInt;
    uint16_t reverseActiveEnergyDec;
    uint32_t inductiveReactiveEnergyInt;
    uint16_t inductiveReactiveEnergyDec;
    uint32_t capacitiveReactiveEnergyInt;
    uint16_t capacitiveReactiveEnergyDec;
} ModbusRegisters;

// 函数声明
bool parseModbusRTU_58Regs(const uint8_t *frame, ModbusRegisters *regs);
uint16_t calculateCRC16(const uint8_t *data, uint16_t length);

#endif // MODBUS_RTU_58REGS_H
