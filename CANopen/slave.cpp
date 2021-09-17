#include <QDebug>
#include <cmath>
#include <iostream>

#include "sdvp_qtcommon/CANopen/slave.h"

// Speed that will be sent with TPDO
void MySlave::commandSpeedReceived(double speed) {
    (*this)[0x2000][1] = (int8_t)std::round(speed*3.6); // [km/h]
    (*this)[0x2000][4] = (int16_t)std::round(speed/0.01); // [m/s] with two decimals
}

// Steering that will be sent with TPDO
void MySlave::commandSteeringReceived(double steering) {
    (*this)[0x2000][2] = (int16_t)std::round(1/steering); // Radius [m]
    (*this)[0x2000][5] = (int32_t)std::round(steering/0.0001); // Curvature [1/m] with four decimals
}

// Status to be sent with TPDO
void MySlave::statusReceived(quint8 status) {
    (*this)[0x2000][3] = (uint8_t)status;
}

// Attributes to be sent with TPDO
void MySlave::commandAttributesReceived(quint32 attributes) {
    // ATTR_HYDRAULIC_MASK 0b00000000000000000000000000111000
    attributes &= 0b00000000000000000000000000111000;
    (*this)[0x2000][6] = (uint8_t)attributes;
}

// Read the value just written to object 200X:0X by RPDO
void MySlave::OnWrite(uint16_t idx, uint8_t subidx) noexcept {
    if (idx == 0x2001 && subidx == 1) {
      emit sendStatus((quint8)(*this)[0x2001][1]);
    }
    if (idx == 0x2001 && subidx == 2) {
        qint8 b = (*this)[0x2001][2];
        double a = (double)b;
        emit sendActualSpeed(a); // [km/h]
    }
    if (idx == 0x2001 && subidx == 3) {
        qint16 b = (*this)[0x2001][3];
        double a = (double)b;
        emit sendActualSteering(a); // Radius [m]

    }
    if (idx == 0x2001 && subidx == 4) {
        quint8 b = (*this)[0x2001][4];
        double a = (double)b;
        emit sendBatterySOC(a); // [%]
    }
    if (idx == 0x2001 && subidx == 5) {
        quint16 b = (*this)[0x2001][5];
        double a = (double)b;
        emit sendBatteryVoltage(a); // [deciV]
    }
    if (idx == 0x2001 && subidx == 6) {
        qint16 b = (*this)[0x2001][6];
        double a = (double)(b*0.01);
        emit sendActualSpeed(a); // [m/s] with two decimals
    }
    if (idx == 0x2001 && subidx == 7) {
        qint32 b = (*this)[0x2001][7];
        double a = (double)(b*0.0001);
        emit sendActualSteering(a); // Curvature [1/m] with four decimals
    }
  }
