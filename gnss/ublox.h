/*
    Copyright 2017 - 2018 Benjamin Vedder	benjamin@vedder.se

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

#ifndef UBLOX_H
#define UBLOX_H

#include <QObject>
#include <QVector>
#include <QSerialPort>
#include <QTimer>
#include <cstdint>
#include <cmath>
#include "rtcm3_simple.h"

// Datatypes
typedef struct {
    uint16_t ref_station_id;
    uint32_t i_tow; // GPS time of week of the navigation epoch
    float pos_n; // Position north in meters
    float pos_e; // Position east in meters
    float pos_d; // Position down in meters
    float pos_length; // Length of the relative position vector in meters
    float pos_heading; // Heading of the relative position vector in degree
    float acc_n; // Accuracy north in meters
    float acc_e; // Accuracy east in meters
    float acc_d; // Accuracy down in meters
    float acc_length; // Accuracy length in meters
    float acc_heading; // Accuracy heading in degree
    bool fix_ok; // A valid fix
    bool diff_soln; // Differential corrections are applied
    bool rel_pos_valid; // Relative position components and accuracies valid
    int carr_soln; // fix_type 0: no fix, 1: float, 2: fix
    bool is_moving; // Operating in moving base mode
    bool ref_pos_miss; // Extrapolated reference position was used to compute moving base solution
    bool ref_obs_miss; // Extrapolated reference observations were used to compute moving base solution
    bool rel_pos_heading_valid; // Heading is valid
    bool rel_pos_normalized; // Position values are normalized
} ubx_nav_relposned;

typedef struct {
    uint32_t i_tow; // GPS time of week of the navigation epoch
    uint32_t dur; // Passed survey-in observation time (s)
    double meanX; // Current survey-in mean position ECEF X coordinate
    double meanY; // Current survey-in mean position ECEF Y coordinate
    double meanZ; // Current survey-in mean position ECEF Z coordinate
    float meanAcc; // Current survey-in mean position accuracy
    uint32_t obs; // Number of position observations used during survey-in
    bool valid; // Survey-in position validity flag, 1 = valid, otherwise 0
    bool active; // Survey-in in progress flag, 1 = in-progress, otherwise 0
} ubx_nav_svin;

typedef struct {
    uint32_t i_tow; // GPS time of week of the navigation epoch

    /*
     * Fractional part of i_tow. (range +/-500000)
     * The precise GPS time of week in seconds is:
     * (i_tow * 1e-3) + (f_tow * 1e-9)
     */
    int32_t f_tow;

    int16_t weel; // GPS week number of the navigation epoch

    /*
     * GPSfix Type, range 0..5
     * 0x00 = No Fix
     * 0x01 = Dead Reckoning only
     * 0x02 = 2D-Fix
     * 0x03 = 3D-Fix
     * 0x04 = GPS + dead reckoning combined
     * 0x05 = Time only fix
     * 0x06..0xff: reserved
     */
    uint8_t gps_fix;

    bool gpsfixok; // Fix within limits (e.g. DOP & accuracy)
    bool diffsoln; // DGPS used
    bool wknset; // Valid GPS week number
    bool towset; // Valid GPS time of week
    double ecef_x; // ECEF X coordinate
    double ecef_y; // ECEF Y coordinate
    double ecef_z; // ECEF Z coordinate
    float p_acc; // 3D Position Accuracy Estimate
    float ecef_vx; // ECEF X velocity
    float ecef_vy; // ECEF Y velocity
    float ecef_vz; // ECEF Z velocity
    float s_acc; // Speed Accuracy Estimate
    float p_dop; // Position DOP
    uint8_t num_sv; // Number of SVs used in Nav Solution
} ubx_nav_sol;

typedef struct {
    uint32_t i_tow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t second;

    bool valid_date;
    bool valid_time;
    bool fully_resolved;
    bool valid_mag;

    uint32_t t_acc;
    int32_t nano;
    uint8_t fix_type;

    bool gnss_fix_ok;
    bool diffsoln;
    uint8_t psm_state;
    bool head_veh_valid;
    uint8_t carr_soln;

    bool confirmed_avai;
    bool confirmed_date;
    bool confirmed_time;

    uint8_t num_sv;
    double lon;
    double lat;
    double height;
    double hmsl;
    double h_acc;
    double v_acc;
    double vel_n;
    double vel_e;
    double vel_d;
    double g_speed;
    double head_mot;
    double s_acc;
    double head_acc;
    double p_dop;

    bool invalid_llh;

    double head_veh;
    double mag_dec;
    double mag_acc;
} ubx_nav_pvt;

typedef struct {
    uint8_t gnss_id; // 0: GPS, 1: SBAS, 2: GAL, 3: BDS, 5: QZSS, 6: GLO
    uint8_t sv_id;
    uint8_t cno; // Carrier to noise ratio (signal strength)
    int8_t elev; // Elevation (range: +/-90), unknown if out of range
    int16_t azim; // Azimuth (range 0-360), unknown if elevation is out of range
    float pr_res; // Pseudorange residual
    uint8_t quality; // 0: no signal, 1: searching, 2: aquired, 3: unusable, 4: locked, 5-7: carrier, code and time locked
    bool used;
    uint8_t health; // 0: unknown, 1: healthy, 2: unhealthy
    bool diffcorr; // Differential correction available
} ubx_nav_sat_info;

typedef struct {
    uint32_t i_tow_ms; // GPS time of week of the navigation epoch.
    uint8_t num_sv; // Number of satellites.
    ubx_nav_sat_info sats[128];
} ubx_nav_sat;

typedef struct {
    double pr_mes;
    double cp_mes;
    float do_mes;
    uint8_t gnss_id;
    uint8_t sv_id;
    uint8_t freq_id;
    uint16_t locktime;
    uint8_t cno;
    uint8_t pr_stdev;
    uint8_t cp_stdev;
    uint8_t do_stdev;
    bool pr_valid;
    bool cp_valid;
    bool half_cyc_valid;
    bool half_cyc_sub;
} ubx_rxm_rawx_obs;

typedef struct {
    double rcv_tow;
    uint16_t week;
    int8_t leaps;
    uint8_t num_meas;
    bool leap_sec;
    bool clk_reset;
    ubx_rxm_rawx_obs obs[64];
} ubx_rxm_rawx;

#define MAX_ESF_NUM_MEAS 16
enum ubx_esf_datatype_enum {GYRO_Z=5, GYRO_Y=13, GYRO_X=14, ACC_X=16, ACC_Y=17, ACC_Z=18};

inline float esfMeas2Float(unsigned meas, uint8_t exp) {
    return (((meas & 0b100000000000000000000000) ?
                 (-1.0f*(((~(unsigned)((meas-1U) & 0b011111111111111111111111)) & 0b11111111111111111111111)))
               :
                 meas)
            * pow(2,-exp));
}

typedef struct {
    uint32_t time_tag;
    uint8_t time_mark_sent;
    bool time_mark_edge;
    bool calib_t_tag_valid;
    uint8_t num_meas;
    uint16_t id;
    uint32_t data_field[MAX_ESF_NUM_MEAS];
    ubx_esf_datatype_enum data_type[MAX_ESF_NUM_MEAS];
    uint32_t calib_t_tag;
} ubx_esf_meas;

typedef struct {
    uint8_t type;
    bool used;
    bool ready;

    uint8_t calib_status;
    uint8_t time_status;

    uint8_t freq;

    bool bad_meas;
    bool bad_t_tag;
    bool missing_meas;
    bool noisy_meas;
} ubx_esf_status_sensors;

typedef struct {
    uint32_t i_tow;
    uint8_t version;
    uint8_t fusion_mode;
    uint8_t num_sens;
    ubx_esf_status_sensors sensors[255]; // ToDo: set correct nr of sensors
} ubx_esf_status;

typedef struct {
    uint32_t i_tow;
    uint8_t version;
    bool autoMntAlgOn;
    uint8_t status;
    bool tiltAlgError;
    bool yawAlgError;
    bool angleError;
    double yaw;
    double pitch;
    double roll;
} ubx_esf_alg;

typedef struct {
    uint32_t baudrate;
    bool in_rtcm3;
    bool in_rtcm2;
    bool in_nmea;
    bool in_ubx;
    bool out_rtcm3;
    bool out_nmea;
    bool out_ubx;
} ubx_cfg_prt_uart;

typedef struct {
    bool lla; // Use lla instead of ecef
    int mode; // Mode. 0 = Disabled, 1 = Survey in, 2 = Fixed
    double ecefx_lat;
    double ecefy_lon;
    double ecefz_alt;
    float fixed_pos_acc; // Fixed position accuracy
    uint32_t svin_min_dur; // SVIN minimum duration (s)
    float svin_acc_limit; // SVIN accuracy limit
} ubx_cfg_tmode3;

typedef struct {
    bool clear_io_port;
    bool clear_msg_conf;
    bool clear_inf_msg;
    bool clear_nav_conf;
    bool clear_rxm_conf;
    bool clear_sen_conf;
    bool clear_rinv_conf;
    bool clear_ant_conf;
    bool clear_log_conf;
    bool clear_fts_conf;

    bool save_io_port;
    bool save_msg_conf;
    bool save_inf_msg;
    bool save_nav_conf;
    bool save_rxm_conf;
    bool save_sen_conf;
    bool save_rinv_conf;
    bool save_ant_conf;
    bool save_log_conf;
    bool save_fts_conf;

    bool load_io_port;
    bool load_msg_conf;
    bool load_inf_msg;
    bool load_nav_conf;
    bool load_rxm_conf;
    bool load_sen_conf;
    bool load_rinv_conf;
    bool load_ant_conf;
    bool load_log_conf;
    bool load_fts_conf;

    bool dev_bbr; // Battery backed RAM
    bool dev_flash; // Flash
    bool dev_eeprom; // EEPROM
    bool dev_spi_flash; // SPI flash
} ubx_cfg_cfg;

typedef struct {
    bool apply_dyn; // Apply dynamic model settings
    bool apply_min_el; // Apply minimum elevation settings
    bool apply_pos_fix_mode; // Apply fix mode settings
    bool apply_pos_mask; // Apply position mask settings
    bool apply_time_mask; // Apply time mask settings
    bool apply_static_hold_mask; // Apply static hold settings
    bool apply_dgps; // Apply DGPS settings.
    bool apply_cno; // Apply CNO threshold settings (cnoThresh, cnoThreshNumSVs).
    bool apply_utc; // Apply UTC settings

    /*
     * Dynamic platform model:
     * 0: portable
     * 2: stationary
     * 3: pedestrian
     * 4: automotive
     * 5: sea
     * 6: airborne with <1g acceleration
     * 7: airborne with <2g acceleration
     * 8: airborne with <4g acceleration
     * 9: wrist worn watch
     */
    uint8_t dyn_model;

    /*
     * Position Fixing Mode:
     * 1: 2D only
     * 2: 3D only
     * 3: auto 2D/3D
     */
    uint8_t fix_mode;

    double fixed_alt; // Fixed altitude (mean sea level) for 2D fix mode. (m)
    double fixed_alt_var; // Fixed altitude variance for 2D mode. (m^2)
    int8_t min_elev; // Minimum Elevation for a GNSS satellite to be used in NAV (deg)
    float p_dop; // Position DOP Mask to use
    float t_dop; // Time DOP Mask to use
    uint16_t p_acc; // Position Accuracy Mask (m)
    uint16_t t_acc; // Time Accuracy Mask (m)
    uint8_t static_hold_thres; // Static hold threshold (cm/s)
    uint8_t dgnss_timeout; // DGNSS (RTK) timeout (s)
    uint8_t cno_tres_num_sat; // Number of satellites required to have C/N0 above cnoThresh for a fix to be attempted
    uint8_t cno_tres; // C/N0 threshold for deciding whether to attempt a fix (dBHz)
    uint16_t static_hold_max_dist; // Static hold distance threshold (before quitting static hold) (m)

    /*
     * UTC standard to be used:
     * 0: Automatic; receiver selects based on GNSS configuration (see GNSS time bases).
     * 3: UTC as operated by the U.S. Naval Observatory (USNO); derived from GPS time
     * 6: UTC as operated by the former Soviet Union; derived from GLONASS time
     * 7: UTC as operated by the National Time Service Center, China; derived from BeiDou time
     */
    uint8_t utc_standard;
} ubx_cfg_nav5;

typedef struct {
    uint8_t tp_idx; // Timepulse selection. 0=TP1, 1=TP2
    int16_t ant_cable_delay; // Antenna cable delay in ns
    int16_t rf_group_delay; // RF group delay in ns
    uint32_t freq_period; // Frequency or time period, Hz or us
    uint32_t freq_period_lock; // Frequency or time period when locked to GNSS time, Hz or us
    uint32_t pulse_len_ratio; // Pulse length or duty cycle, us or 2^-32
    uint32_t pulse_len_ratio_lock; // Pulse length or duty cycle when locked to GNSS time, us or 2^-32
    int32_t user_config_delay; // User configurable time pulse delay, ns

    /*
     * If set enable time pulse; if pin assigned to another function, other function takes
     * precedence. Must be set for FTS variant.
     */
    bool active;

    /*
     * If set synchronize time pulse to GNSS as soon as GNSS time is valid. If not
     * set, or before GNSS time is valid use local clock. This flag is ignored by
     * the FTS product variant; in this case the receiver always locks to the best
     * available time/frequency reference (which is not necessarily GNSS).
     */
    bool lockGnssFreq;

    /*
     * If set the receiver switches between the timepulse settings given by 'freqPeriodLocked' &
     * 'pulseLenLocked' and those given by 'freqPeriod' & 'pulseLen'. The 'Locked' settings are
     * used where the receiver has an accurate sense of time. For non-FTS products, this occurs
     * when GNSS solution with a reliable time is available, but for FTS products the setting syncMode
     * field governs behavior. In all cases, the receiver only uses 'freqPeriod' & 'pulseLen' when
     * the flag is unset.
     */
    bool lockedOtherSet;

    /*
     * If set 'freqPeriodLock' and 'freqPeriod' are interpreted as frequency,
     * otherwise interpreted as period.
     */
    bool isFreq;

    /*
     * If set 'pulseLenRatioLock' and 'pulseLenRatio' interpreted as pulse
     * length, otherwise interpreted as duty cycle.
     */
    bool isLength;

    /*
     * Align pulse to top of second (period time must be integer fraction of 1s).
     * Also set 'lockGnssFreq' to use this feature.
     * This flag is ignored by the FTS product variant; it is assumed to be always set
     * (as is lockGnssFreq). Set maxSlewRate and maxPhaseCorrRate fields of CFG-SMGR to
     * 0 to disable alignment.
     */
    bool alignToTow;

    /*
     * Pulse polarity:
     * 0: falling edge at top of second
     * 1: rising edge at top of second
     */
    bool polarity;

    /*
     * Timegrid to use:
     * 0: UTC
     * 1: GPS
     * 2: GLONASS
     * 3: BeiDou
     * 4: Galileo (not supported in protocol versions less than 18)
     * This flag is only relevant if 'lockGnssFreq' and 'alignToTow' are set.
     * Note that configured GNSS time is estimated by the receiver if locked to
     * any GNSS system. If the receiver has a valid GNSS fix it will attempt to
     * steer the TP to the specified time grid even if the specified time is not
     * based on information from the constellation's satellites. To ensure timing
     * based purely on a given GNSS, restrict the supported constellations in CFG-GNSS.
     */
    uint8_t gridUtcGnss;

    /*
     * Sync Manager lock mode to use:
     *
     * 0: switch to 'freqPeriodLock' and 'pulseLenRatioLock' as soon as Sync
     * Manager has an accurate time, never switch back to 'freqPeriod' and 'pulseLenRatio'
     *
     * 1: switch to 'freqPeriodLock' and 'pulseLenRatioLock' as soon as Sync Manager has
     * an accurate time, and switch back to 'freqPeriod' and 'pulseLenRatio' as soon as
     * time gets inaccurate.
     *
     * This field is only relevant for the FTS product variant.
     * This field is only relevant if the flag 'lockedOtherSet' is set.
     */
    uint8_t syncMode;
} ubx_cfg_tp5;

typedef struct {
    uint8_t gnss_id;
    uint8_t minTrkCh;
    uint8_t maxTrkCh;
    bool en;
    uint32_t flags;
} ubx_cfg_gnss_block;

typedef struct {
    uint8_t num_ch_hw;
    uint8_t num_ch_use;
    ubx_cfg_gnss_block blocks[10];
    int num_blocks;
} ubx_cfg_gnss;

typedef struct {
    // Filter
    bool posFilt; // Enable position output for failed or invalid fixes
    bool mskPosFilt; // Enable position output for invalid fixes
    bool timeFilt; // Enable time output for invalid times
    bool dateFilt; // Enable date output for invalid dates
    bool gpsOnlyFilt; // Restrict output to GPS satellites only
    bool trackFilt; // Enable COG output even if COG is frozen

    // Flags
    // enable compatibility mode. This might be needed for certain
    // applications when customer's NMEA parser expects a fixed number of digits in position coordinates
    bool compat;
    bool consider; // enable considering mode.
    bool limit82; // enable strict limit to 82 characters maximum.
    bool highPrec; // enable high precision mode.

    // Disable GNSS
    bool disableGps; // Disable reporting of GPS satellites
    bool disableSbas; // Disable reporting of SBAS satellites
    bool disableQzss; // Disable reporting of QZSS satellites
    bool disableGlonass; // Disable reporting of GLONASS satellites
    bool disableBeidou; // Disable reporting of BeiDou satellites

    uint8_t nmeaVersion; // 0x41: 4.10, 0x40: 4.0, 0x23: 2.3, 0x21: 2.1

    // Maximum Number of SVs to report per
    // TalkerId.
    // 0: unlimited
    // 8: 8 SVs
    // 12: 12 SVs
    // 16: 16 SVs
    uint8_t numSv;

    // Configures the display of satellites that do
    // not have an NMEA-defined value.
    // 0: Strict - Satellites are not output
    // 1: Extended - Use proprietary numbering
    uint8_t svNumbering;

    // This field enables the main Talker ID to be overridden.
    // 0: Main Talker ID is not overridden
    // 1: Set main Talker ID to 'GP'
    // 2: Set main Talker ID to 'GL'
    // 3: Set main Talker ID to 'GN'
    // 4: Set main Talker ID to 'GA'
    // 5: Set main Talker ID to 'GB'
    uint8_t mainTalkerId;

    // By default the Talker ID for GSV messages
    // is GNSS specific (as defined by NMEA).
    // This field enables the GSV Talker ID to be
    // overridden.
    // 0: Use GNSS specific Talker ID (as defined by NMEA)
    // 1: Use the main Talker ID
    uint8_t gsvTalkerId;

    // Sets the two characters that should be
    // used for the BeiDou Talker ID
    // If these are set to zero, the default BeiDou
    // TalkerId will be used
    char bdsTalkerId[2];
} ubx_cfg_nmea;

class Ublox : public QObject
{
    Q_OBJECT
public:
    const QVector<unsigned> supportedBaudrates = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    const unsigned defaultBaudrateUart = 38400; // F9P

    explicit Ublox(QObject *parent = 0);
    bool connectSerial(const QSerialPortInfo& serialPortInfo, unsigned baudrate = 921600);
    void disconnectSerial();
    bool isSerialConnected();
    void writeRaw(QByteArray data);

    void ubxPoll(uint8_t msg_class, uint8_t id);
    bool ubxCfgPrtUart(ubx_cfg_prt_uart *cfg);
    bool ubxCfgTmode3(ubx_cfg_tmode3 *cfg);
    bool ubxCfgMsg(uint8_t msg_class, uint8_t id, uint8_t rate);
    bool ubxCfgRate(uint16_t meas_rate_ms, uint16_t nav_rate_ms, uint16_t time_ref);
    bool ubloxCfgCfg(ubx_cfg_cfg *cfg);
    bool ubxCfgNav5(ubx_cfg_nav5 *cfg);
    bool ubloxCfgTp5(ubx_cfg_tp5 *cfg);
    bool ubloxCfgGnss(ubx_cfg_gnss *gnss);
    bool ubloxCfgNmea(ubx_cfg_nmea *nmea);
    bool ubloxCfgValset(unsigned char *values, int len,
                        bool ram, bool bbr, bool flash);

    void ubloxCfgAppendEnableGps(unsigned char *buffer, int *ind,
                                 bool en, bool en_l1c, bool en_l2c);
    void ubloxCfgAppendEnableGal(unsigned char *buffer, int *ind,
                                 bool en, bool en_e1, bool en_e5b);
    void ubloxCfgAppendEnableBds(unsigned char *buffer, int *ind,
                                 bool en, bool en_b1, bool en_b2);
    void ubloxCfgAppendEnableGlo(unsigned char *buffer, int *ind,
                                 bool en, bool en_l1, bool en_l2);
    void ubloxCfgAppendUart1Baud(unsigned char *buffer, int *ind, uint32_t baudrate);
    void ubloxCfgAppendUart1InProt(unsigned char *buffer, int *ind, bool ubx, bool nmea, bool rtcm3x);
    void ubloxCfgAppendUart1OutProt(unsigned char *buffer, int *ind, bool ubx, bool nmea, bool rtcm3x);

signals:
    void rxNavSol(const ubx_nav_sol &sol);
    void rxNavPvt(const ubx_nav_pvt &pvt);
    void rxRelPosNed(const ubx_nav_relposned &pos);
    void rxSvin(const ubx_nav_svin &svin);
    void rxAck(const uint8_t &cls_id, const uint8_t &msg_id);
    void rxNak(const uint8_t &cls_id, const uint8_t &msg_id);
    void rxRawx(const ubx_rxm_rawx &rawx);
    void rxNavSat(const ubx_nav_sat &sat);
    void rxCfgGnss(const ubx_cfg_gnss &gnss);
    void rxEsfMeas(const ubx_esf_meas &meas);
    void rxEsfStatus(const ubx_esf_status &status);
    void rxEsfAlg(const ubx_esf_alg &alg);
    void rxMonVer(const QString &sw, const QString &hw, const QStringList &extensions);
    void ubxRx(const QByteArray &data);
    void rtcmRx(const QByteArray &data, const int &type);

public slots:

private slots:
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);

private:
    typedef struct {
        uint8_t line[256];
        uint8_t ubx[4096];
        unsigned line_pos;
        unsigned ubx_pos;
        uint8_t ubx_class;
        uint8_t ubx_id;
        uint8_t ubx_ck_a;
        uint8_t ubx_ck_b;
        unsigned ubx_len;
    } decoder_state;

    QSerialPort *mSerialPort;
    decoder_state mDecoderState;
    rtcm3_state mRtcmState;
    bool mWaitingAck;

    void ubx_send(QByteArray data);
    bool ubx_encode_send(uint8_t msg_class, uint8_t id, uint8_t *msg, int len, int timeoutMs = -1);
    QByteArray ubx_encode(uint8_t msg_class, uint8_t id, const QByteArray &data);

    void ubx_decode(uint8_t msg_class, uint8_t id, uint8_t *msg, int len);
    void ubx_decode_nav_sol(uint8_t *msg, int len);
    void ubx_decode_nav_pvt(uint8_t *msg, int len);
    void ubx_decode_relposned(uint8_t *msg, int len);
    void ubx_decode_svin(uint8_t *msg, int len);
    void ubx_decode_ack(uint8_t *msg, int len);
    void ubx_decode_nak(uint8_t *msg, int len);
    void ubx_decode_rawx(uint8_t *msg, int len);
    void ubx_decode_nav_sat(uint8_t *msg, int len);
    void ubx_decode_cfg_gnss(uint8_t *msg, int len);
    void ubx_decode_mon_ver(uint8_t *msg, int len);
    void ubx_decode_esf_meas(uint8_t *msg, int len);
    void ubx_decode_esf_status(uint8_t *msg, int len);
    void ubx_decode_esf_alg(uint8_t *msg, int len);
};

// Message classes
#define UBX_CLASS_NAV					0x01
#define UBX_CLASS_RXM					0x02
#define UBX_CLASS_INF					0x04
#define UBX_CLASS_ACK					0x05
#define UBX_CLASS_CFG					0x06
#define UBX_CLASS_UPD					0x09
#define UBX_CLASS_MON					0x0A
#define UBX_CLASS_AID					0x0B
#define UBX_CLASS_TIM					0x0D
#define UBX_CLASS_ESF					0x10
#define UBX_CLASS_MGA					0x13
#define UBX_CLASS_LOG					0x21
#define UBX_CLASS_SEC					0x27
#define UBX_CLASS_HNR					0x28
#define UBX_CLASS_NMEA					0xF0
#define UBX_CLASS_RTCM3					0xF5

// Navigation (NAV) messages
#define UBX_NAV_SOL						0x06
#define UBX_NAV_PVT     				0x07
#define UBX_NAV_RELPOSNED				0x3C
#define UBX_NAV_SVIN					0x3B
#define UBX_NAV_SAT 					0x35

// Receiver Manager (RXM) messages
#define UBX_RXM_RAWX					0x15
#define UBX_RXM_SFRBX					0x13

// Ack messages
#define UBX_ACK_ACK						0x01
#define UBX_ACK_NAK						0x00

// External sensor fusion (ESF) messages
#define UBX_ESF_MEAS                    0x02
#define UBX_ESF_STATUS                  0x10
#define UBX_ESF_ALG                     0x14

// Configuration messages
#define UBX_CFG_PRT						0x00
#define UBX_CFG_MSG						0x01
#define UBX_CFG_RATE					0x08
#define UBX_CFG_CFG						0x09
#define UBX_CFG_NAV5					0x24
#define UBX_CFG_TP5						0x31
#define UBX_CFG_TMODE3					0x71
#define UBX_CFG_GNSS                    0x3E
#define UBX_CFG_NMEA                    0x17
#define UBX_CFG_VALSET                  0x8A
#define UBX_CFG_VALGET                  0x8B
#define UBX_CFG_VALDEL                  0x8C

// Monitoring messages
#define UBX_MON_VER                     0x04

// Configuration Keys
#define CFG_SIGNAL_GPS_ENA              0x1031001F // GPS enable
#define CFG_SIGNAL_GPS_L1C_ENA          0x10310001 // GPS L1C/A
#define CFG_SIGNAL_GPS_L2C_ENA          0x10310003 // GPS L2C (only on u-blox F9)
#define CFG_SIGNAL_GAL_ENA              0x10310021 // Galileo enable
#define CFG_SIGNAL_GAL_E1_ENA           0x10310007 // Galileo E1
#define CFG_SIGNAL_GAL_E5B_ENA          0x1031000A // Galileo E5b (only on u-blox F9)
#define CFG_SIGNAL_BDS_ENA              0x10310022 // BeiDou Enable
#define CFG_SIGNAL_BDS_B1_ENA           0x1031000D // BeiDou B1I
#define CFG_SIGNAL_BDS_B2_ENA           0x1031000E // BeiDou B2I (only on u-blox F9)
#define CFG_SIGNAL_GLO_ENA              0x10310025 // GLONASS Enable
#define CFG_SIGNAL_GLO_L1_ENA           0x10310018 // GLONASS L1
#define CFG_SIGNAL_GLO_L2_ENA           0x1031001A // GLONASS L2 (only on u-blox F9)

#define CFG_UART1_BAUDRATE              0x40520001
#define CFG_UART1INPROT_UBX             0x10730001
#define CFG_UART1INPROT_NMEA            0x10730002
#define CFG_UART1INPROT_RTCM3X          0x10730004
#define CFG_UART1OUTPROT_UBX            0x10740001
#define CFG_UART1OUTPROT_NMEA           0x10740002
#define CFG_UART1OUTPROT_RTCM3X         0x10740004

// RTCM3 messages
#define UBX_RTCM3_1005					0x05 // Stationary RTK reference station ARP
#define UBX_RTCM3_1074					0x4A // GPS MSM4
#define UBX_RTCM3_1077					0x4D // GPS MSM7
#define UBX_RTCM3_1084					0x54 // GLONASS MSM4
#define UBX_RTCM3_1087					0x57 // GLONASS MSM7
#define UBX_RTCM3_1094					0x5E // Galileo MSM4
#define UBX_RTCM3_1097					0x61 // Galileo MSM7
#define UBX_RTCM3_1124					0x7C // BeiDou MSM4
#define UBX_RTCM3_1127					0x7F // BeiDou MSM7
#define UBX_RTCM3_1230					0xE6 // GLONASS code-phase biases
#define UBX_RTCM3_4072_0				0xFE // Reference station PVT (u-blox proprietary RTCM Message)
#define UBX_RTCM3_4072_1				0xFD // Additional reference station information (u-blox proprietary RTCM Message)

// GNSS IDs
#define UBX_GNSS_ID_GPS                 0
#define UBX_GNSS_ID_SBAS                1
#define UBX_GNSS_ID_GALILEO             2
#define UBX_GNSS_ID_BEIDOU              3
#define UBX_GNSS_ID_IMES                4
#define UBX_GNSS_ID_QZSS                5
#define UBX_GNSS_ID_GLONASS             6
#define UBX_GNSS_ID_IRNSS               7

// CFG_GNSS flags
#define UBX_CFG_GNSS_GPS_L1C            0x01
#define UBX_CFG_GNSS_GPS_L2C            0x10
#define UBX_CFG_GNSS_SBAS_L1C           0x01
#define UBX_CFG_GNSS_GAL_E1             0x01
#define UBX_CFG_GNSS_GAL_E5B            0x20
#define UBX_CFG_GNSS_BDS_B1L            0x01
#define UBX_CFG_GNSS_BDS_B2L            0x10
#define UBX_CFG_GNSS_IMES_L1            0x01
#define UBX_CFG_GNSS_QZSS_L1C           0x01
#define UBX_CFG_GNSS_QZSS_L1S           0x04
#define UBX_CFG_GNSS_QZSS_L2C           0x10
#define UBX_CFG_GNSS_GLO_L1             0x01
#define UBX_CFG_GNSS_GLO_L2             0x10
#define UBX_CFG_GNSS_IRNSS_L5A          0x01

// NMEA messages
#define UBX_NMEA_GGA                    0x00

#endif // UBLOX_H
