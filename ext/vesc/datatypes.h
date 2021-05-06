/*
    Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se

    This file is part of VESC Tool.

    VESC Tool is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VESC Tool is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef DATATYPES_H
#define DATATYPES_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>
#include <stdint.h>

namespace VESC {

typedef struct {
    QString name;
    QString systemPath;
    bool isVesc;
} VSerialInfo_t;

typedef enum {
    CFG_T_UNDEFINED = 0,
    CFG_T_DOUBLE,
    CFG_T_INT,
    CFG_T_QSTRING,
    CFG_T_ENUM,
    CFG_T_BOOL
} CFG_T;

typedef enum {
    VESC_TX_UNDEFINED = 0,
    VESC_TX_UINT8,
    VESC_TX_INT8,
    VESC_TX_UINT16,
    VESC_TX_INT16,
    VESC_TX_UINT32,
    VESC_TX_INT32,
    VESC_TX_DOUBLE16,
    VESC_TX_DOUBLE32,
    VESC_TX_DOUBLE32_AUTO
} VESC_TX_T;

// General purpose drive output mode
typedef enum {
    GPD_OUTPUT_MODE_NONE = 0,
    GPD_OUTPUT_MODE_MODULATION,
    GPD_OUTPUT_MODE_VOLTAGE,
    GPD_OUTPUT_MODE_CURRENT
} gpd_output_mode;

typedef enum {
    HW_TYPE_VESC = 0,
    HW_TYPE_VESC_BMS,
    HW_TYPE_CUSTOM_MODULE
} HW_TYPE;

typedef enum {
    FAULT_CODE_NONE = 0,
    FAULT_CODE_OVER_VOLTAGE,
    FAULT_CODE_UNDER_VOLTAGE,
    FAULT_CODE_DRV,
    FAULT_CODE_ABS_OVER_CURRENT,
    FAULT_CODE_OVER_TEMP_FET,
    FAULT_CODE_OVER_TEMP_MOTOR,
    FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
    FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
    FAULT_CODE_MCU_UNDER_VOLTAGE,
    FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
    FAULT_CODE_ENCODER_SPI,
    FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
    FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
    FAULT_CODE_FLASH_CORRUPTION,
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
    FAULT_CODE_UNBALANCED_CURRENTS,
    FAULT_CODE_RESOLVER_LOT,
    FAULT_CODE_RESOLVER_DOS,
    FAULT_CODE_RESOLVER_LOS,
    FAULT_CODE_FLASH_CORRUPTION_APP_CFG,
    FAULT_CODE_FLASH_CORRUPTION_MC_CFG,
    FAULT_CODE_ENCODER_NO_MAGNET
} mc_fault_code;

typedef enum {
    DISP_POS_MODE_NONE = 0,
    DISP_POS_MODE_INDUCTANCE,
    DISP_POS_MODE_OBSERVER,
    DISP_POS_MODE_ENCODER,
    DISP_POS_MODE_PID_POS,
    DISP_POS_MODE_PID_POS_ERROR,
    DISP_POS_MODE_ENCODER_OBSERVER_ERROR
} disp_pos_mode;

// ADC control types. Remember to add new types here when adding them to the firmware.
typedef enum {
    ADC_CTRL_TYPE_NONE = 0,
    ADC_CTRL_TYPE_CURRENT,
    ADC_CTRL_TYPE_CURRENT_REV_CENTER,
    ADC_CTRL_TYPE_CURRENT_REV_BUTTON,
    ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC,
    ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER,
    ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER,
    ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON,
    ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC,
    ADC_CTRL_TYPE_DUTY,
    ADC_CTRL_TYPE_DUTY_REV_CENTER,
    ADC_CTRL_TYPE_DUTY_REV_BUTTON,
    ADC_CTRL_TYPE_PID,
    ADC_CTRL_TYPE_PID_REV_CENTER,
    ADC_CTRL_TYPE_PID_REV_BUTTON
} adc_control_type;

struct MC_VALUES {
    Q_GADGET

    Q_PROPERTY(double v_in MEMBER v_in)
    Q_PROPERTY(double temp_mos MEMBER temp_mos)
    Q_PROPERTY(double temp_mos_1 MEMBER temp_mos_1)
    Q_PROPERTY(double temp_mos_2 MEMBER temp_mos_2)
    Q_PROPERTY(double temp_mos_3 MEMBER temp_mos_3)
    Q_PROPERTY(double temp_motor MEMBER temp_motor)
    Q_PROPERTY(double current_motor MEMBER current_motor)
    Q_PROPERTY(double current_in MEMBER current_in)
    Q_PROPERTY(double id MEMBER id)
    Q_PROPERTY(double iq MEMBER iq)
    Q_PROPERTY(double rpm MEMBER rpm)
    Q_PROPERTY(double duty_now MEMBER duty_now)
    Q_PROPERTY(double amp_hours MEMBER amp_hours)
    Q_PROPERTY(double amp_hours_charged MEMBER amp_hours_charged)
    Q_PROPERTY(double watt_hours MEMBER watt_hours)
    Q_PROPERTY(double watt_hours_charged MEMBER watt_hours_charged)
    Q_PROPERTY(int tachometer MEMBER tachometer)
    Q_PROPERTY(int tachometer_abs MEMBER tachometer_abs)
    Q_PROPERTY(double position MEMBER position)
    Q_PROPERTY(mc_fault_code fault_code MEMBER fault_code)
    Q_PROPERTY(int vesc_id MEMBER vesc_id)
    Q_PROPERTY(QString fault_str MEMBER fault_str)
    Q_PROPERTY(double vd MEMBER vd)
    Q_PROPERTY(double vq MEMBER vq)

public:
    MC_VALUES() {
        v_in = 0.0;
        temp_mos = 0.0;
        temp_mos_1 = 0.0;
        temp_mos_2 = 0.0;
        temp_mos_3 = 0.0;
        temp_motor = 0.0;
        current_motor = 0.0;
        current_in = 0.0;
        id = 0.0;
        iq = 0.0;
        rpm = 0.0;
        duty_now = 0.0;
        amp_hours = 0.0;
        amp_hours_charged = 0.0;
        watt_hours = 0.0;
        watt_hours_charged = 0.0;
        tachometer = 0;
        tachometer_abs = 0;
        position = 0.0;
        fault_code = FAULT_CODE_NONE;
        vesc_id = 0;
        vd = 0.0;
        vq = 0.0;
    }

    bool operator==(const MC_VALUES &other) const {
        (void)other;
        // compare members
        return true;
    }

    bool operator!=(MC_VALUES const &other) const {
        return !(*this == other);
    }

    double v_in;
    double temp_mos;
    double temp_mos_1;
    double temp_mos_2;
    double temp_mos_3;
    double temp_motor;
    double current_motor;
    double current_in;
    double id;
    double iq;
    double rpm;
    double duty_now;
    double amp_hours;
    double amp_hours_charged;
    double watt_hours;
    double watt_hours_charged;
    int tachometer;
    int tachometer_abs;
    double position;
    mc_fault_code fault_code;
    int vesc_id;
    QString fault_str;
    double vd;
    double vq;
};

//Q_DECLARE_METATYPE(MC_VALUES)

struct SETUP_VALUES {
    Q_GADGET

    Q_PROPERTY(double temp_mos MEMBER temp_mos)
    Q_PROPERTY(double temp_motor MEMBER temp_motor)
    Q_PROPERTY(double current_motor MEMBER current_motor)
    Q_PROPERTY(double current_in MEMBER current_in)
    Q_PROPERTY(double duty_now MEMBER duty_now)
    Q_PROPERTY(double rpm MEMBER rpm)
    Q_PROPERTY(double speed MEMBER speed)
    Q_PROPERTY(double v_in MEMBER v_in)
    Q_PROPERTY(double battery_level MEMBER battery_level)
    Q_PROPERTY(double amp_hours MEMBER amp_hours)
    Q_PROPERTY(double amp_hours_charged MEMBER amp_hours_charged)
    Q_PROPERTY(double watt_hours MEMBER watt_hours)
    Q_PROPERTY(double watt_hours_charged MEMBER watt_hours_charged)
    Q_PROPERTY(double tachometer MEMBER tachometer)
    Q_PROPERTY(double tachometer_abs MEMBER tachometer_abs)
    Q_PROPERTY(double position MEMBER position)
    Q_PROPERTY(mc_fault_code fault_code MEMBER fault_code)
    Q_PROPERTY(int vesc_id MEMBER vesc_id)
    Q_PROPERTY(int num_vescs MEMBER num_vescs)
    Q_PROPERTY(double battery_wh MEMBER battery_wh)
    Q_PROPERTY(QString fault_str MEMBER fault_str)
    Q_PROPERTY(unsigned odometer MEMBER odometer)

public:
    SETUP_VALUES() {
        temp_mos = 0.0;
        temp_motor = 0.0;
        current_motor = 0.0;
        current_in = 0.0;
        duty_now = 0.0;
        rpm = 0.0;
        speed = 0.0;
        v_in = 0.0;
        battery_level = 0.0;
        amp_hours = 0.0;
        amp_hours_charged = 0.0;
        watt_hours = 0.0;
        watt_hours_charged = 0.0;
        tachometer = 0.0;
        tachometer_abs = 0.0;
        position = 0.0;
        fault_code = FAULT_CODE_NONE;
        vesc_id = 0;
        num_vescs = 0;
        battery_wh = 0.0;
        odometer = 0;
    }

    bool operator==(const SETUP_VALUES &other) const {
        (void)other;
        // compare members
        return true;
    }

    bool operator!=(SETUP_VALUES const &other) const {
        return !(*this == other);
    }

    double temp_mos;
    double temp_motor;
    double current_motor;
    double current_in;
    double duty_now;
    double rpm;
    double speed;
    double v_in;
    double battery_level;
    double amp_hours;
    double amp_hours_charged;
    double watt_hours;
    double watt_hours_charged;
    double tachometer;
    double tachometer_abs;
    double position;
    mc_fault_code fault_code;
    int vesc_id;
    int num_vescs;
    double battery_wh;
    QString fault_str;
    unsigned odometer;
};

//Q_DECLARE_METATYPE(SETUP_VALUES)

struct IMU_VALUES {
    Q_GADGET

    Q_PROPERTY(double roll MEMBER roll)
    Q_PROPERTY(double pitch MEMBER pitch)
    Q_PROPERTY(double yaw MEMBER yaw)

    Q_PROPERTY(double accX MEMBER accX)
    Q_PROPERTY(double accY MEMBER accY)
    Q_PROPERTY(double accZ MEMBER accZ)

    Q_PROPERTY(double gyroX MEMBER gyroX)
    Q_PROPERTY(double gyroY MEMBER gyroY)
    Q_PROPERTY(double gyroZ MEMBER gyroZ)

    Q_PROPERTY(double magX MEMBER magX)
    Q_PROPERTY(double magY MEMBER magY)
    Q_PROPERTY(double magZ MEMBER magZ)

    Q_PROPERTY(double q0 MEMBER q0)
    Q_PROPERTY(double q1 MEMBER q1)
    Q_PROPERTY(double q2 MEMBER q2)
    Q_PROPERTY(double q3 MEMBER q3)

public:
    IMU_VALUES() {
        roll = 0; pitch = 0; yaw = 0;
        accX = 0; accY = 0; accZ = 0;
        gyroX = 0; gyroY = 0; gyroZ = 0;
        magX = 0; magY = 0; magZ = 0;
        q0 = 1; q1 = 0; q2 = 0; q3 = 0;
    }

    bool operator==(const IMU_VALUES &other) const {
        (void)other;
        // compare members
        return true;
    }

    bool operator!=(IMU_VALUES const &other) const {
        return !(*this == other);
    }

    double roll;
    double pitch;
    double yaw;

    double accX;
    double accY;
    double accZ;

    double gyroX;
    double gyroY;
    double gyroZ;

    double magX;
    double magY;
    double magZ;

    double q0;
    double q1;
    double q2;
    double q3;
};

//Q_DECLARE_METATYPE(IMU_VALUES)

struct LOG_DATA {
    Q_GADGET

    Q_PROPERTY(MC_VALUES values MEMBER values)
    Q_PROPERTY(SETUP_VALUES setupValues MEMBER setupValues)
    Q_PROPERTY(IMU_VALUES imuValues MEMBER imuValues)
    Q_PROPERTY(int valTime MEMBER valTime)
    Q_PROPERTY(int posTime MEMBER posTime)
    Q_PROPERTY(double lat MEMBER lat)
    Q_PROPERTY(double lon MEMBER lon)
    Q_PROPERTY(double alt MEMBER alt)
    Q_PROPERTY(double gVel MEMBER gVel)
    Q_PROPERTY(double vVel MEMBER vVel)
    Q_PROPERTY(double hAcc MEMBER hAcc)
    Q_PROPERTY(double vAcc MEMBER vAcc)

public:
    LOG_DATA() {
        posTime = -1;
        setupValTime = -1;
        imuValTime = -1;
        lat = 0.0;
        lon = 0.0;
        alt = 0.0;
        gVel = 0.0;
        vVel = 0.0;
        hAcc = 0.0;
        vAcc = 0.0;
    }

    MC_VALUES values;
    SETUP_VALUES setupValues;
    IMU_VALUES imuValues;
    int valTime;
    int setupValTime;
    int imuValTime;
    int posTime;
    double lat;
    double lon;
    double alt;
    double gVel;
    double vVel;
    double hAcc;
    double vAcc;
};

//Q_DECLARE_METATYPE(LOG_DATA)

struct MCCONF_TEMP {
    Q_GADGET

    Q_PROPERTY(double current_min_scale MEMBER current_min_scale)
    Q_PROPERTY(double current_max_scale MEMBER current_max_scale)
    Q_PROPERTY(double erpm_or_speed_min MEMBER erpm_or_speed_min)
    Q_PROPERTY(double erpm_or_speed_max MEMBER erpm_or_speed_max)
    Q_PROPERTY(double duty_min MEMBER duty_min)
    Q_PROPERTY(double duty_max MEMBER duty_max)
    Q_PROPERTY(double watt_min MEMBER watt_min)
    Q_PROPERTY(double watt_max MEMBER watt_max)
    Q_PROPERTY(QString name MEMBER name)

public:
    double current_min_scale;
    double current_max_scale;
    double erpm_or_speed_min;
    double erpm_or_speed_max;
    double duty_min;
    double duty_max;
    double watt_min;
    double watt_max;
    QString name;
};

//Q_DECLARE_METATYPE(MCCONF_TEMP)

struct CONFIG_BACKUP {
    Q_GADGET

    Q_PROPERTY(QString name MEMBER name)
    Q_PROPERTY(QString vesc_uuid MEMBER vesc_uuid)
    Q_PROPERTY(QString mcconf_xml_compressed MEMBER mcconf_xml_compressed)
    Q_PROPERTY(QString appconf_xml_compressed MEMBER appconf_xml_compressed)

public:
    QString name;
    QString vesc_uuid;
    QString mcconf_xml_compressed;
    QString appconf_xml_compressed;
};

//Q_DECLARE_METATYPE(CONFIG_BACKUP)

struct FW_RX_PARAMS {
    Q_GADGET

    Q_PROPERTY(int major MEMBER major)
    Q_PROPERTY(int minor MEMBER minor)
    Q_PROPERTY(QString hw MEMBER hw)
    Q_PROPERTY(QByteArray uuid MEMBER uuid)
    Q_PROPERTY(bool isPaired MEMBER isPaired)
    Q_PROPERTY(int isTestFw MEMBER isTestFw)
    Q_PROPERTY(HW_TYPE hwType MEMBER hwType)
    Q_PROPERTY(int customConfigNum MEMBER customConfigNum)

public:
    FW_RX_PARAMS() {
        major = -1;
        minor = -1;
        isPaired = false;
        isTestFw = false;
        hwType = HW_TYPE_VESC;
        customConfigNum = 0;
    }

    Q_INVOKABLE QString hwTypeStr() {
        QString res = "Unknown Hardware";
        switch (hwType) {
        case HW_TYPE_VESC:
            res = "VESC";
            break;

        case HW_TYPE_VESC_BMS:
            res = "VESC BMS";
            break;

        case HW_TYPE_CUSTOM_MODULE:
            res = "Custom Module";
            break;
        }

        return res;
    }

    int major;
    int minor;
    QString hw;
    QByteArray uuid;
    bool isPaired;
    int isTestFw;
    HW_TYPE hwType;
    int customConfigNum;
};

//Q_DECLARE_METATYPE(FW_RX_PARAMS)

struct BMS_VALUES {
    Q_GADGET

    Q_PROPERTY(double v_tot MEMBER v_tot)
    Q_PROPERTY(double v_charge MEMBER v_charge)
    Q_PROPERTY(double i_in MEMBER i_in)
    Q_PROPERTY(double i_in_ic MEMBER i_in_ic)
    Q_PROPERTY(double ah_cnt MEMBER ah_cnt)
    Q_PROPERTY(double wh_cnt MEMBER wh_cnt)
    Q_PROPERTY(QVector<qreal> v_cells MEMBER v_cells)
    Q_PROPERTY(QVector<qreal> temps MEMBER temps)
    Q_PROPERTY(QVector<bool> is_balancing MEMBER is_balancing)
    Q_PROPERTY(double temp_ic MEMBER temp_ic)
    Q_PROPERTY(double humidity MEMBER humidity)
    Q_PROPERTY(double temp_hum_sensor MEMBER temp_hum_sensor)
    Q_PROPERTY(double temp_cells_highest MEMBER temp_cells_highest)
    Q_PROPERTY(double soc MEMBER soc)
    Q_PROPERTY(double soh MEMBER soh)

public:
    BMS_VALUES() {
        v_tot = 0.0;
        v_charge = 0.0;
        i_in = 0.0;
        i_in_ic = 0.0;
        ah_cnt = 0.0;
        wh_cnt = 0.0;
        humidity = 0.0;
        temp_hum_sensor = 0.0;
        temp_cells_highest = 0.0;
        soc = 0.0;
        soh = 0.0;
    }

    double v_tot;
    double v_charge;
    double i_in;
    double i_in_ic;
    double ah_cnt;
    double wh_cnt;
    QVector<qreal> v_cells;
    QVector<qreal> temps;
    QVector<bool> is_balancing;
    double temp_ic;
    double humidity;
    double temp_hum_sensor;
    double temp_cells_highest;
    double soc;
    double soh;

};

//Q_DECLARE_METATYPE(BMS_VALUES)

typedef enum {
    DEBUG_SAMPLING_OFF = 0,
    DEBUG_SAMPLING_NOW,
    DEBUG_SAMPLING_START,
    DEBUG_SAMPLING_TRIGGER_START,
    DEBUG_SAMPLING_TRIGGER_FAULT,
    DEBUG_SAMPLING_TRIGGER_START_NOSEND,
    DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND,
    DEBUG_SAMPLING_SEND_LAST_SAMPLES
} debug_sampling_mode;

typedef enum {
    COMM_FW_VERSION = 0,
    COMM_JUMP_TO_BOOTLOADER,
    COMM_ERASE_NEW_APP,
    COMM_WRITE_NEW_APP_DATA,
    COMM_GET_VALUES,
    COMM_SET_DUTY,
    COMM_SET_CURRENT,
    COMM_SET_CURRENT_BRAKE,
    COMM_SET_RPM,
    COMM_SET_POS,
    COMM_SET_HANDBRAKE,
    COMM_SET_DETECT,
    COMM_SET_SERVO_POS,
    COMM_SET_MCCONF,
    COMM_GET_MCCONF,
    COMM_GET_MCCONF_DEFAULT,
    COMM_SET_APPCONF,
    COMM_GET_APPCONF,
    COMM_GET_APPCONF_DEFAULT,
    COMM_SAMPLE_PRINT,
    COMM_TERMINAL_CMD,
    COMM_PRINT,
    COMM_ROTOR_POSITION,
    COMM_EXPERIMENT_SAMPLE,
    COMM_DETECT_MOTOR_PARAM,
    COMM_DETECT_MOTOR_R_L,
    COMM_DETECT_MOTOR_FLUX_LINKAGE,
    COMM_DETECT_ENCODER,
    COMM_DETECT_HALL_FOC,
    COMM_REBOOT,
    COMM_ALIVE,
    COMM_GET_DECODED_PPM,
    COMM_GET_DECODED_ADC,
    COMM_GET_DECODED_CHUK,
    COMM_FORWARD_CAN,
    COMM_SET_CHUCK_DATA,
    COMM_CUSTOM_APP_DATA,
    COMM_NRF_START_PAIRING,
    COMM_GPD_SET_FSW,
    COMM_GPD_BUFFER_NOTIFY,
    COMM_GPD_BUFFER_SIZE_LEFT,
    COMM_GPD_FILL_BUFFER,
    COMM_GPD_OUTPUT_SAMPLE,
    COMM_GPD_SET_MODE,
    COMM_GPD_FILL_BUFFER_INT8,
    COMM_GPD_FILL_BUFFER_INT16,
    COMM_GPD_SET_BUFFER_INT_SCALE,
    COMM_GET_VALUES_SETUP,
    COMM_SET_MCCONF_TEMP,
    COMM_SET_MCCONF_TEMP_SETUP,
    COMM_GET_VALUES_SELECTIVE,
    COMM_GET_VALUES_SETUP_SELECTIVE,
    COMM_EXT_NRF_PRESENT,
    COMM_EXT_NRF_ESB_SET_CH_ADDR,
    COMM_EXT_NRF_ESB_SEND_DATA,
    COMM_EXT_NRF_ESB_RX_DATA,
    COMM_EXT_NRF_SET_ENABLED,
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
    COMM_DETECT_APPLY_ALL_FOC,
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
    COMM_ERASE_NEW_APP_ALL_CAN,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN,
    COMM_PING_CAN,
    COMM_APP_DISABLE_OUTPUT,
    COMM_TERMINAL_CMD_SYNC,
    COMM_GET_IMU_DATA,
    COMM_BM_CONNECT,
    COMM_BM_ERASE_FLASH_ALL,
    COMM_BM_WRITE_FLASH,
    COMM_BM_REBOOT,
    COMM_BM_DISCONNECT,
    COMM_BM_MAP_PINS_DEFAULT,
    COMM_BM_MAP_PINS_NRF5X,
    COMM_ERASE_BOOTLOADER,
    COMM_ERASE_BOOTLOADER_ALL_CAN,
    COMM_PLOT_INIT,
    COMM_PLOT_DATA,
    COMM_PLOT_ADD_GRAPH,
    COMM_PLOT_SET_GRAPH,
    COMM_GET_DECODED_BALANCE,
    COMM_BM_MEM_READ,
    COMM_WRITE_NEW_APP_DATA_LZO,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO,
    COMM_BM_WRITE_FLASH_LZO,
    COMM_SET_CURRENT_REL,
    COMM_CAN_FWD_FRAME,
    COMM_SET_BATTERY_CUT,
    COMM_SET_BLE_NAME,
    COMM_SET_BLE_PIN,
    COMM_SET_CAN_MODE,
    COMM_GET_IMU_CALIBRATION,
    COMM_GET_MCCONF_TEMP,

    // Custom configuration for hardware
    COMM_GET_CUSTOM_CONFIG_XML,
    COMM_GET_CUSTOM_CONFIG,
    COMM_GET_CUSTOM_CONFIG_DEFAULT,
    COMM_SET_CUSTOM_CONFIG,

    // BMS commands
    COMM_BMS_GET_VALUES,
    COMM_BMS_SET_CHARGE_ALLOWED,
    COMM_BMS_SET_BALANCE_OVERRIDE,
    COMM_BMS_RESET_COUNTERS,
    COMM_BMS_FORCE_BALANCE,
    COMM_BMS_ZERO_CURRENT_OFFSET,

    // FW updates commands for different HW types
    COMM_JUMP_TO_BOOTLOADER_HW,
    COMM_ERASE_NEW_APP_HW,
    COMM_WRITE_NEW_APP_DATA_HW,
    COMM_ERASE_BOOTLOADER_HW,
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW,
    COMM_ERASE_NEW_APP_ALL_CAN_HW,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW,
    COMM_ERASE_BOOTLOADER_ALL_CAN_HW,

    COMM_SET_ODOMETER,
} COMM_PACKET_ID;

// CAN commands
typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_FILL_RX_BUFFER,
    CAN_PACKET_FILL_RX_BUFFER_LONG,
    CAN_PACKET_PROCESS_RX_BUFFER,
    CAN_PACKET_PROCESS_SHORT_BUFFER,
    CAN_PACKET_STATUS,
    CAN_PACKET_SET_CURRENT_REL,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_STATUS_2,
    CAN_PACKET_STATUS_3,
    CAN_PACKET_STATUS_4,
    CAN_PACKET_PING,
    CAN_PACKET_PONG,
    CAN_PACKET_DETECT_APPLY_ALL_FOC,
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
    CAN_PACKET_CONF_CURRENT_LIMITS,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
    CAN_PACKET_CONF_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_FOC_ERPMS,
    CAN_PACKET_CONF_STORE_FOC_ERPMS,
    CAN_PACKET_STATUS_5,
    CAN_PACKET_POLL_TS5700N8501_STATUS,
    CAN_PACKET_CONF_BATTERY_CUT,
    CAN_PACKET_CONF_STORE_BATTERY_CUT,
    CAN_PACKET_SHUTDOWN
} CAN_PACKET_ID;

typedef struct {
    int js_x;
    int js_y;
    int acc_x;
    int acc_y;
    int acc_z;
    bool bt_c;
    bool bt_z;
} chuck_data;

struct bldc_detect {
    Q_GADGET

    Q_PROPERTY(double cycle_int_limit MEMBER cycle_int_limit)
    Q_PROPERTY(double bemf_coupling_k MEMBER bemf_coupling_k)
    Q_PROPERTY(QVector<int> hall_table MEMBER hall_table)
    Q_PROPERTY(int hall_res MEMBER hall_res)

public:
    double cycle_int_limit;
    double bemf_coupling_k;
    QVector<int> hall_table;
    int hall_res;
};

//Q_DECLARE_METATYPE(bldc_detect)

typedef enum {
    NRF_PAIR_STARTED = 0,
    NRF_PAIR_OK,
    NRF_PAIR_FAIL
} NRF_PAIR_RES;

struct BALANCE_VALUES {
    Q_GADGET

    Q_PROPERTY(double pid_output MEMBER pid_output)
    Q_PROPERTY(double pitch_angle MEMBER pitch_angle)
    Q_PROPERTY(double roll_angle MEMBER roll_angle)
    Q_PROPERTY(int diff_time MEMBER diff_time)
    Q_PROPERTY(double motor_current MEMBER motor_current)
    Q_PROPERTY(double motor_position MEMBER motor_position)
    Q_PROPERTY(int state MEMBER state)
    Q_PROPERTY(int switch_value MEMBER switch_value)
    Q_PROPERTY(double adc1 MEMBER adc1)
    Q_PROPERTY(double adc2 MEMBER adc2)

public:
    BALANCE_VALUES() {
        pid_output = 0;
        pitch_angle = 0;
        roll_angle = 0;
        diff_time = 0;
        motor_current = 0;
        motor_position = 0;
        state = 0;
        switch_value = 0;
        adc1 = 0;
        adc2 = 0;
    }

    double pid_output;
    double pitch_angle;
    double roll_angle;
    int diff_time;
    double motor_current;
    double motor_position;
    int state;
    int switch_value;
    double adc1;
    double adc2;
};

//Q_DECLARE_METATYPE(BALANCE_VALUES)

}

#endif // DATATYPES_H
