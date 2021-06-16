/*
    Copyright 2016-2017 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C++" {
#include <QObject>
}
#endif

// Sizes
#define LOG_NAME_MAX_LEN			20

// Packet IDs
#define ID_ALL						255
#define ID_CAR_CLIENT				254 // Packet for car client only
#define ID_MOTE						254
#define ID_RTCM						211 // Same as RTCM3PREAMB

// External log mode
typedef enum {
    LOG_EXT_OFF = 0,
    LOG_EXT_UART,
    LOG_EXT_UART_POLLED,
    LOG_EXT_ETHERNET
} LOG_EXT_MODE;

// Orientation data
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float integralFBx;
    float integralFBy;
    float integralFBz;
    float accMagP;
    int initialUpdateDone;
} ATTITUDE_INFO;

typedef enum {
    FAULT_CODE_NONE = 0,
    FAULT_CODE_OVER_VOLTAGE,
    FAULT_CODE_UNDER_VOLTAGE,
    FAULT_CODE_DRV8302,
    FAULT_CODE_ABS_OVER_CURRENT,
    FAULT_CODE_OVER_TEMP_FET,
    FAULT_CODE_OVER_TEMP_MOTOR
} mc_fault_code;

#ifdef __cplusplus
extern "C++" {
struct CAR_STATE {
    Q_GADGET

    Q_PROPERTY(uint8_t fw_major MEMBER fw_major)
    Q_PROPERTY(uint8_t fw_minor MEMBER fw_minor)
    Q_PROPERTY(double roll MEMBER roll)
    Q_PROPERTY(double pitch MEMBER pitch)
    Q_PROPERTY(double yaw MEMBER yaw)
    Q_PROPERTY(QList<double> accel READ accelList)
    Q_PROPERTY(QList<double> gyro READ gyroList)
    Q_PROPERTY(QList<double> mag READ magList)
    Q_PROPERTY(double px MEMBER px)
    Q_PROPERTY(double py MEMBER py)
    Q_PROPERTY(double speed MEMBER speed)
    Q_PROPERTY(double vin MEMBER vin)
    Q_PROPERTY(double temp_fet MEMBER temp_fet)
    Q_PROPERTY(mc_fault_code mc_fault MEMBER mc_fault)
    Q_PROPERTY(double px_gps MEMBER px_gps)
    Q_PROPERTY(double py_gps MEMBER py_gps)
    Q_PROPERTY(double ap_goal_px MEMBER ap_goal_px)
    Q_PROPERTY(double ap_goal_py MEMBER ap_goal_py)
    Q_PROPERTY(double ap_rad MEMBER ap_rad)
    Q_PROPERTY(int32_t ms_today MEMBER ms_today)
    Q_PROPERTY(int16_t ap_route_left MEMBER ap_route_left)
    Q_PROPERTY(double px_uwb MEMBER px_uwb)
    Q_PROPERTY(double py_uwb MEMBER py_uwb)

public:
    uint8_t fw_major;
    uint8_t fw_minor;
    double roll;
    double pitch;
    double yaw;
    double accel[3];
    double gyro[3];
    double mag[3];
    double px;
    double py;
    double speed;
    double vin;
    double temp_fet;
    mc_fault_code mc_fault;
    double px_gps;
    double py_gps;
    double ap_goal_px;
    double ap_goal_py;
    double ap_rad;
    int32_t ms_today;
    int16_t ap_route_left;
    double px_uwb;
    double py_uwb;

    QList<double> accelList() {
        QList<double> a;
        a.append(accel[0]);
        a.append(accel[1]);
        a.append(accel[2]);
        return a;
    }

    QList<double> gyroList() {
        QList<double> a;
        a.append(gyro[0]);
        a.append(gyro[1]);
        a.append(gyro[2]);
        return a;
    }

    QList<double> magList() {
        QList<double> a;
        a.append(mag[0]);
        a.append(mag[1]);
        a.append(mag[2]);
        return a;
    }
};
Q_DECLARE_METATYPE(CAR_STATE)
}
#endif

typedef struct {
    uint8_t fw_major;
    uint8_t fw_minor;
    double roll;
    double pitch;
    double yaw;
    double accel[3];
    double gyro[3];
    double mag[3];
    double px;
    double py;
    double pz;
    double speed;
    double vin;
    double px_gps;
    double py_gps;
    double ap_goal_px;
    double ap_goal_py;
    int32_t ms_today;
} MULTIROTOR_STATE;

typedef enum {
    MOTE_PACKET_FILL_RX_BUFFER = 0,
    MOTE_PACKET_FILL_RX_BUFFER_LONG,
    MOTE_PACKET_PROCESS_RX_BUFFER,
    MOTE_PACKET_PROCESS_SHORT_BUFFER,
} MOTE_PACKET;

typedef struct {
    bool yaw_use_odometry; // Use odometry data for yaw angle correction.
    float yaw_imu_gain; // Gain for yaw angle from IMU (vs odometry)
    bool disable_motor; // Disable motor drive commands to make sure that the motor does not move.
    bool simulate_motor; // Simulate motor movement without motor controller feedback
    bool clamp_imu_yaw_stationary; // Clamp IMU yaw when car is stationary
    bool use_uwb_pos; // Use UWB positioning instead of RTK positioning

    float gear_ratio;
    float wheel_diam;
    float motor_poles;
    float steering_max_angle_rad; // = arctan(axist_distance / turn_radius_at_maximum_steering_angle)
    float steering_center;
    float steering_range;
    float steering_ramp_time; // Ramp time constant for the steering servo in seconds
    float axis_distance;
} MAIN_CONFIG_CAR;

typedef struct {
    // Dead reckoning
    float vel_decay_e;
    float vel_decay_l;
    float vel_max;
    float map_min_x;
    float map_max_x;
    float map_min_y;
    float map_max_y;

    // State correction for dead reckoning
    float vel_gain_p;
    float vel_gain_i;
    float vel_gain_d;

    float tilt_gain_p;
    float tilt_gain_i;
    float tilt_gain_d;

    float max_corr_error;
    float max_tilt_error;

    // Attitude controller
    float ctrl_gain_roll_p;
    float ctrl_gain_roll_i;
    float ctrl_gain_roll_dp;
    float ctrl_gain_roll_de;

    float ctrl_gain_pitch_p;
    float ctrl_gain_pitch_i;
    float ctrl_gain_pitch_dp;
    float ctrl_gain_pitch_de;

    float ctrl_gain_yaw_p;
    float ctrl_gain_yaw_i;
    float ctrl_gain_yaw_dp;
    float ctrl_gain_yaw_de;

    // Position controller
    float ctrl_gain_pos_p;
    float ctrl_gain_pos_i;
    float ctrl_gain_pos_d;

    // Altitude controller
    float ctrl_gain_alt_p;
    float ctrl_gain_alt_i;
    float ctrl_gain_alt_d;

    // Joystick gain
    float js_gain_tilt;
    float js_gain_yaw;
    bool js_mode_rate;

    // Motor mapping and configuration
    int8_t motor_fl_f; // x: Front Left  +: Front
    int8_t motor_bl_l; // x: Back Left   +: Left
    int8_t motor_fr_r; // x: Front Right +: Right
    int8_t motor_br_b; // x: Back Right  +: Back
    bool motors_x; // Use x motor configuration (use + if false)
    bool motors_cw; // Front left (or front in + mode) runs in the clockwise direction (ccw if false)
    uint16_t motor_pwm_min_us; // Minimum servo pulse length for motor in microseconds
    uint16_t motor_pwm_max_us; // Maximum servo pulse length for motor in microseconds
} MAIN_CONFIG_MULTIROTOR;

// Car configuration
typedef struct {
    // Common vehicle settings
    bool mag_use; // Use the magnetometer
    bool mag_comp; // Should be 0 when capturing samples for the calibration
    float yaw_mag_gain; // Gain for yaw angle from magnetomer (vs gyro)

    // Magnetometer calibration
    float mag_cal_cx;
    float mag_cal_cy;
    float mag_cal_cz;
    float mag_cal_xx;
    float mag_cal_xy;
    float mag_cal_xz;
    float mag_cal_yx;
    float mag_cal_yy;
    float mag_cal_yz;
    float mag_cal_zx;
    float mag_cal_zy;
    float mag_cal_zz;

    // GPS parameters
    float gps_ant_x; // Antenna offset from vehicle center in X
    float gps_ant_y; // Antenna offset from vehicle center in Y
    bool gps_comp; // Use GPS position correction
    bool gps_req_rtk; // Require RTK solution
    bool gps_use_rtcm_base_as_enu_ref; // Use RTCM base station position as ENU reference
    float gps_corr_gain_stat; // Static GPS correction gain
    float gps_corr_gain_dyn; // Dynamic GPS correction gain
    float gps_corr_gain_yaw; // Gain for yaw correction
    bool gps_send_nmea; // Send NMEA data for logging and debugging
    bool gps_use_ubx_info; // Use info about the ublox solution
    float gps_ubx_max_acc; // Maximum ublox accuracy to use solution (m, higher = worse)

    // UWB
    float uwb_max_corr; // Maximum distance to move UWB position in one sample

    // Autopilot parameters
    bool ap_repeat_routes; // Repeat the same route when the end is reached
    float ap_base_rad; // Smallest allowed radius around car
    float ap_rad_time_ahead; // Radius ahead time
    int ap_mode_time; // Drive to route points based on time (1 = abs time, 2 = rel since start)
    float ap_max_speed; // Maximum allowed speed for autopilot
    int32_t ap_time_add_repeat_ms; // Time to add to each point for each repetition of the route

    // Logging
    int log_rate_hz;
    bool log_en;
    char log_name[LOG_NAME_MAX_LEN + 1];
    LOG_EXT_MODE log_mode_ext;
    int log_uart_baud;

    MAIN_CONFIG_CAR car;
    MAIN_CONFIG_MULTIROTOR mr;
} MAIN_CONFIG;

// Commands
typedef enum {
    // General commands
    CMD_PRINTF = 0,
    CMD_TERMINAL_CMD,

    // Common vehicle commands
    CMD_VESC_FWD = 50,
    CMD_SET_POS,
    CMD_SET_POS_ACK,
    CMD_SET_ENU_REF,
    CMD_GET_ENU_REF,
    CMD_AP_ADD_POINTS,
    CMD_AP_REMOVE_LAST_POINT,
    CMD_AP_CLEAR_POINTS,
    CMD_AP_GET_ROUTE_PART,
    CMD_AP_SET_ACTIVE,
    CMD_AP_REPLACE_ROUTE,
    CMD_AP_SYNC_POINT,
    CMD_SEND_RTCM_USB,
    CMD_SEND_NMEA_RADIO,
    CMD_SET_YAW_OFFSET,
    CMD_SET_YAW_OFFSET_ACK,
    CMD_LOG_LINE_USB,
    CMD_PLOT_INIT,
    CMD_PLOT_DATA,
    CMD_PLOT_ADD_GRAPH,
    CMD_PLOT_SET_GRAPH,
    CMD_SET_MS_TODAY,
    CMD_SET_SYSTEM_TIME,
    CMD_SET_SYSTEM_TIME_ACK,
    CMD_REBOOT_SYSTEM,
    CMD_REBOOT_SYSTEM_ACK,
    CMD_EMERGENCY_STOP,
    CMD_SET_MAIN_CONFIG,
    CMD_GET_MAIN_CONFIG,
    CMD_GET_MAIN_CONFIG_DEFAULT,
    CMD_ADD_UWB_ANCHOR,
    CMD_CLEAR_UWB_ANCHORS,
    CMD_LOG_ETHERNET,
    CMD_CAMERA_IMAGE,
    CMD_CAMERA_STREAM_START,
    CMD_CAMERA_FRAME_ACK,
    CMD_IO_BOARD_SET_PWM_DUTY,
    CMD_IO_BOARD_SET_VALVE,
    CMD_HYDRAULIC_MOVE,
    CMD_HEARTBEAT,
    CMD_SET_AP_MODE,

    // Car commands
    CMD_GET_STATE = 120,
    CMD_RC_CONTROL,
    CMD_SET_SERVO_DIRECT,

    // Multirotor commands
    CMD_MR_GET_STATE = 160,
    CMD_MR_RC_CONTROL,
    CMD_MR_OVERRIDE_POWER,

    // Mote commands
    CMD_MOTE_UBX_START_BASE = 200,
    CMD_MOTE_UBX_START_BASE_ACK,
    CMD_MOTE_UBX_BASE_STATUS
} CMD_PACKET;

// RC control modes
typedef enum {
    RC_MODE_CURRENT = 0,
    RC_MODE_DUTY,
    RC_MODE_PID,
    RC_MODE_CURRENT_BRAKE
} RC_MODE;

// Autopilot mode
typedef enum {
    AP_MODE_FOLLOW_ROUTE = 0,
    AP_MODE_FOLLOW_ME
} AP_MODE;

typedef enum {
    HYDRAULIC_POS_FRONT = 0,
    HYDRAULIC_POS_REAR,
    HYDRAULIC_POS_EXTRA
} HYDRAULIC_POS;

typedef enum {
    HYDRAULIC_MOVE_STOP = 0,
    HYDRAULIC_MOVE_UP,
    HYDRAULIC_MOVE_DOWN,
    HYDRAULIC_MOVE_OUT = HYDRAULIC_MOVE_UP,
    HYDRAULIC_MOVE_IN = HYDRAULIC_MOVE_DOWN,
    HYDRAULIC_MOVE_UNDEFINED
} HYDRAULIC_MOVE;

typedef enum {
    JS_TYPE_HK = 0,
    JS_TYPE_PS4,
    JS_TYPE_PS3,
    JS_TYPE_MICRONAV_ONE
} JS_TYPE;

typedef struct {
    int id;
    float px;
    float py;
    float height;
    float dist_last;
} UWB_ANCHOR;

// RtRange

typedef struct {
    int posMode;
    double lat;
    double lon;
    double alt;
    double velN;
    double velE;
    double velD;
    double yaw;
    double mapX;
    double mapY;
    double mapYawRad;
} ncom_data;

#endif /* DATATYPES_H_ */
