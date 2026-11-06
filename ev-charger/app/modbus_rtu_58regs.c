#include "modbus_rtu_58regs.h"
#include <stdio.h>

// CRC16计算函数（Modbus RTU标准）
uint16_t calculateCRC16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Modbus RTU 58寄存器解析函数
bool parseModbusRTU_58Regs(const unsigned char *frame, ModbusRegisters *regs) 
{
    // 基本帧检查（1地址+1功能码+1字节数+116数据+2CRC=121字节）
    if (frame == NULL || regs == NULL) {
        fprintf(stderr, "无效输入参数\n");
        return false;
    }

    // CRC校验
    uint16_t receivedCRC = (frame[186] << 8) | frame[185];
    uint16_t calculatedCRC = calculateCRC16(frame, 185);
    if (receivedCRC != calculatedCRC) {
        fprintf(stderr, "CRC校验失败：接收值=0x%04X，计算值=0x%04X\n", receivedCRC, calculatedCRC);
        return false;
    }

    // 检查功能码（0x03为读取保持寄存器）
    if (frame[1] != 0x03) {
        fprintf(stderr, "无效功能码：0x%02X\n", frame[1]);
        return false;
    }

    // 检查数据字节数
    uint8_t byteCount = frame[2];
    if (byteCount != 182) {
        fprintf(stderr, "数据长度错误：%d字节，应为116字节\n", byteCount);
        return false;
    }

    // 解析所有寄存器数据（大端序）
    uint16_t dataPos = 3; // 数据起始位置
    for (uint8_t regAddr = 0; regAddr < 59; regAddr++) {
        uint16_t value = (frame[dataPos] << 8) | frame[dataPos + 1];
        dataPos += 2;

        // 根据寄存器地址映射到结构体成员
        switch (regAddr) {
            // 参数设置区
            case 0: regs->paramPassword = value; break;
            case 1:
                regs->displayMode = value >> 8;
                regs->wiringMode = value & 0xFF;
                break;
            case 2: regs->ptRatio = value; break;
            case 3: regs->ctRatio = value; break;
            case 4:
                regs->commAddress = value >> 8;
                regs->baudRate = value & 0xFF;
                break;
            case 5: regs->dataFormat = value; break;
            case 6: regs->backlightTime = value; break;

            // 电压电流数据区
            case 20: regs->voltageA = value; break;
            case 21: regs->voltageB = value; break;
            case 22: regs->voltageC = value; break;
            case 23: regs->voltageAB = value; break;
            case 24: regs->voltageBC = value; break;
            case 25: regs->voltageCA = value; break;
            case 26: regs->currentA = value; break;
            case 27: regs->currentB = value; break;
            case 28: regs->currentC = value; break;
            case 29: regs->powerDirection = value; break;

            // 功率数据区
            case 30: regs->activePowerA = value; break;
            case 31: regs->activePowerB = value; break;
            case 32: regs->activePowerC = value; break;
            case 33: regs->totalActivePower = value; break;
            case 34: regs->reactivePowerA = value; break;
            case 35: regs->reactivePowerB = value; break;
            case 36: regs->reactivePowerC = value; break;
            case 37: regs->totalReactivePower = value; break;
            case 38: regs->apparentPowerA = value; break;
            case 39: regs->apparentPowerB = value; break;
            case 40: regs->apparentPowerC = value; break;
            case 41: regs->totalApparentPower = value; break;
            case 42: regs->powerFactorA = value; break;
            case 43: regs->powerFactorB = value; break;
            case 44: regs->powerFactorC = value; break;
            case 45: regs->totalPowerFactor = value; break;
            case 46: regs->frequency = value; break;

            // 电能数据区（特殊处理32位数据）
            case 47: regs->forwardActiveEnergyInt = (uint32_t)value << 16; break;
            case 48: regs->forwardActiveEnergyInt |= value; break;
            case 49: regs->forwardActiveEnergyDec = value; break;
            case 50: regs->reverseActiveEnergyInt = (uint32_t)value << 16; break;
            case 51: regs->reverseActiveEnergyInt |= value; break;
            case 52: regs->reverseActiveEnergyDec = value; break;
            case 53: regs->inductiveReactiveEnergyInt = (uint32_t)value << 16; break;
            case 54: regs->inductiveReactiveEnergyInt |= value; break;
            case 55: regs->inductiveReactiveEnergyDec = value; break;
            case 56: regs->capacitiveReactiveEnergyInt = (uint32_t)value << 16; break;
            case 57: regs->capacitiveReactiveEnergyInt |= value; break;
            case 58: regs->capacitiveReactiveEnergyDec = value; break;
        }
    }

    return true;
}
