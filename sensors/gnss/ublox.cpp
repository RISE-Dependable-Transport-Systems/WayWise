/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "ublox.h"
#include <QEventLoop>
#include <cmath>
#include <QDebug>
#include <QDateTime>

namespace {
static uint8_t ubx_get_U1(uint8_t *msg, int *ind) {
    return msg[(*ind)++];
}

static int8_t ubx_get_I1(uint8_t *msg, int *ind) {
    return (int8_t)msg[(*ind)++];
}

static uint8_t ubx_get_X1(uint8_t *msg, int *ind) {
    return msg[(*ind)++];
}

static uint16_t ubx_get_U2(uint8_t *msg, int *ind) {
    uint16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
                    ((uint16_t) msg[*ind]);
    *ind += 2;
    return res;
}

static int16_t ubx_get_I2(uint8_t *msg, int *ind) {
    int16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
                    ((uint16_t) msg[*ind]);
    *ind += 2;
    return res;
}

static uint16_t ubx_get_X2(uint8_t *msg, int *ind) {
    uint16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
                    ((uint16_t) msg[*ind]);
    *ind += 2;
    return res;
}

static uint32_t ubx_get_U4(uint8_t *msg, int *ind) {
    uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind]);
    *ind += 4;
    return res;
}

static int32_t ubx_get_I4(uint8_t *msg, int *ind) {
    int32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind]);
    *ind += 4;
    return res;
}

static uint32_t ubx_get_X4(uint8_t *msg, int *ind) {
    uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind]);
    *ind += 4;
    return res;
}

static float ubx_get_R4(uint8_t *msg, int *ind) {
    uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind]);
    *ind += 4;

    union asd {
        float f;
        uint32_t i;
    } x;

    x.i = res;

    return x.f;
}

static double ubx_get_R8(uint8_t *msg, int *ind) {
    uint64_t res =	((uint64_t) msg[*ind + 7]) << 56 |
                    ((uint64_t) msg[*ind + 6]) << 48 |
                    ((uint64_t) msg[*ind + 5]) << 40 |
                    ((uint64_t) msg[*ind + 4]) << 32 |
                    ((uint64_t) msg[*ind + 3]) << 24 |
                    ((uint64_t) msg[*ind + 2]) << 16 |
                    ((uint64_t) msg[*ind + 1]) << 8 |
                    ((uint64_t) msg[*ind]);
    *ind += 8;

    union asd {
        double f;
        uint64_t i;
    } x;

    x.i = res;

    return x.f;
}

static void ubx_put_U1(uint8_t *msg, int *ind, uint8_t data) {
    msg[(*ind)++] = data;
}

static void ubx_put_I1(uint8_t *msg, int *ind, int8_t data) {
    msg[(*ind)++] = data;
}

static void ubx_put_X1(uint8_t *msg, int *ind, uint8_t data) {
    msg[(*ind)++] = data;
}

static void ubx_put_U2(uint8_t *msg, int *ind, uint16_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
}

static void ubx_put_I2(uint8_t *msg, int *ind, int16_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
}

static void ubx_put_X2(uint8_t *msg, int *ind, uint16_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
}

static void ubx_put_U4(uint8_t *msg, int *ind, uint32_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
    msg[(*ind)++] = data >> 16;
    msg[(*ind)++] = data >> 24;
}

static void ubx_put_I4(uint8_t *msg, int *ind, int32_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
    msg[(*ind)++] = data >> 16;
    msg[(*ind)++] = data >> 24;
}

static void ubx_put_X4(uint8_t *msg, int *ind, uint32_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
    msg[(*ind)++] = data >> 16;
    msg[(*ind)++] = data >> 24;
}

static void ubx_put_R4(uint8_t *msg, int *ind, float data) {
    union asd {
        float f;
        uint64_t i;
    } x;

    x.f = data;

    msg[(*ind)++] = x.i;
    msg[(*ind)++] = x.i >> 8;
    msg[(*ind)++] = x.i >> 16;
    msg[(*ind)++] = x.i >> 24;
}

static void ubx_put_R8(uint8_t *msg, int *ind, double data) {
    union asd {
        double f;
        uint64_t i;
    } x;

    x.f = data;

    msg[(*ind)++] = x.i;
    msg[(*ind)++] = x.i >> 8;
    msg[(*ind)++] = x.i >> 16;
    msg[(*ind)++] = x.i >> 24;
    msg[(*ind)++] = x.i >> 32;
    msg[(*ind)++] = x.i >> 40;
    msg[(*ind)++] = x.i >> 48;
    msg[(*ind)++] = x.i >> 56;
}
}

Ublox::Ublox(QObject *parent) : QObject(parent)
{
    memset(&mDecoderState, 0, sizeof(decoder_state));
    rtcm3_init_state(&mRtcmState);

    mSerialPort = new QSerialPort(this);

    connect(mSerialPort, &QSerialPort::readyRead, this, &Ublox::serialDataAvailable);
    connect(mSerialPort, QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::error), this, &Ublox::serialPortError);

    // Prevent unused warnings
    (void)ubx_get_U1;
    (void)ubx_get_I1;
    (void)ubx_get_X1;
    (void)ubx_get_U2;
    (void)ubx_get_I2;
    (void)ubx_get_X2;
    (void)ubx_get_U4;
    (void)ubx_get_I4;
    (void)ubx_get_X4;
    (void)ubx_get_R4;
    (void)ubx_get_R8;

    (void)ubx_put_U1;
    (void)ubx_put_I1;
    (void)ubx_put_X1;
    (void)ubx_put_U2;
    (void)ubx_put_I2;
    (void)ubx_put_X2;
    (void)ubx_put_U4;
    (void)ubx_put_I4;
    (void)ubx_put_X4;
    (void)ubx_put_R4;
    (void)ubx_put_R8;
}

bool Ublox::connectSerial(const QSerialPortInfo &serialPortInfo, unsigned baudrate)
{
    if(mSerialPort->isOpen()) {
        mSerialPort->close();
    }

    mSerialPort->setPort(serialPortInfo);
    mSerialPort->open(QIODevice::ReadWrite);
    mWaitingAck = false;

    if(!mSerialPort->isOpen()) {
        return false;
    }

    if (supportedBaudrates.contains(baudrate))
        mSerialPort->setBaudRate(baudrate);
    else
        return false;

    mSerialPort->setDataBits(QSerialPort::Data8);
    mSerialPort->setParity(QSerialPort::NoParity);
    mSerialPort->setStopBits(QSerialPort::OneStop);
    mSerialPort->setFlowControl(QSerialPort::NoFlowControl);

    return true;
}

void Ublox::disconnectSerial()
{
    mSerialPort->close();
}

bool Ublox::isSerialConnected()
{
    return mSerialPort->isOpen();
}

void Ublox::writeRaw(QByteArray data)
{
    ubx_send(data);
}

void Ublox::ubxPoll(uint8_t msg_class, uint8_t id)
{
    ubx_encode_send(msg_class, id, 0, 0);
}

/**
 * Set the uart1 port configuration.
 *
 * @param cfg
 * The configuration. Notice that always 8N1 configuration
 * and no tx ready function is used.
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgPrtUart(ubx_cfg_prt_uart *cfg)
{
    uint8_t buffer[20];
    int ind = 0;

    ubx_put_U1(buffer, &ind, 1); // ID for UART1
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_X2(buffer, &ind, 0); // Always disable txready function

    uint32_t mode = 0;
    mode |= 3 << 6; // Always use 8 bits
    mode |= 4 << 9; // No parity
    mode |= 0 << 12; // 1 stop bit

    ubx_put_X4(buffer, &ind, mode);
    ubx_put_U4(buffer, &ind, cfg->baudrate);

    uint16_t in_proto = 0;
    in_proto |= (cfg->in_ubx ? 1 : 0) << 0;
    in_proto |= (cfg->in_nmea ? 1 : 0) << 1;
    in_proto |= (cfg->in_rtcm2 ? 1 : 0) << 2;
    in_proto |= (cfg->in_rtcm3 ? 1 : 0) << 5;

    ubx_put_X2(buffer, &ind, in_proto);

    uint16_t out_proto = 0;
    out_proto |= (cfg->out_ubx ? 1 : 0) << 0;
    out_proto |= (cfg->out_nmea ? 1 : 0) << 1;
    out_proto |= (cfg->out_rtcm3 ? 1 : 0) << 5;

    ubx_put_X2(buffer, &ind, out_proto);
    ubx_put_X2(buffer, &ind, 0); // No extended timeout
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_PRT, buffer, ind, 10);
}

/**
 * Set the tmode3 configuration.
 *
 * @param cfg
 * The configuration.
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgTmode3(ubx_cfg_tmode3 *cfg)
{
    uint8_t buffer[40];
    int ind = 0;

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    uint16_t flags = ((cfg->lla ? 1 : 0) << 8) | cfg->mode;
    ubx_put_X2(buffer, &ind, flags);

    int32_t x_lat = 0;
    int32_t y_lon = 0;
    int32_t z_alt = 0;
    int8_t x_lat_hp = 0;
    int8_t y_lon_hp = 0;
    int8_t z_alt_hp = 0;
    if (cfg->lla) {
        x_lat = round(cfg->ecefx_lat * D(1e7));
        y_lon = round(cfg->ecefy_lon * D(1e7));
        z_alt = round(cfg->ecefz_alt * D(1e2));
        x_lat_hp = ((cfg->ecefx_lat - ((double)x_lat * D(1e-7))) * D(1e9));
        y_lon_hp = ((cfg->ecefy_lon - ((double)y_lon * D(1e-7))) * D(1e9));
        z_alt_hp = ((cfg->ecefz_alt - ((double)z_alt * D(1e-2))) * D(1e4));
    } else {
        x_lat = cfg->ecefx_lat * D(1e2);
        y_lon = cfg->ecefy_lon * D(1e2);
        z_alt = cfg->ecefz_alt * D(1e2);
        x_lat_hp = ((cfg->ecefx_lat - ((double)x_lat * D(1e-2))) * D(1e4));
        y_lon_hp = ((cfg->ecefy_lon - ((double)y_lon * D(1e-2))) * D(1e4));
        z_alt_hp = ((cfg->ecefz_alt - ((double)z_alt * D(1e-2))) * D(1e4));
    }

    ubx_put_I4(buffer, &ind, x_lat);
    ubx_put_I4(buffer, &ind, y_lon);
    ubx_put_I4(buffer, &ind, z_alt);
    ubx_put_I1(buffer, &ind, x_lat_hp);
    ubx_put_I1(buffer, &ind, y_lon_hp);
    ubx_put_I1(buffer, &ind, z_alt_hp);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U4(buffer, &ind, cfg->fixed_pos_acc * 1e4);
    ubx_put_U4(buffer, &ind, cfg->svin_min_dur);
    ubx_put_U4(buffer, &ind, cfg->svin_acc_limit * 1e4);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_TMODE3, buffer, ind, 10);
}

/**
 * Set the msg output configuration.
 *
 * @param msg_class
 * The message class.
 *
 * @param id
 * The message id
 *
 * @param
 * The message rate. 0 = disbaled
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgMsg(uint8_t msg_class, uint8_t id, uint8_t rate)
{
    uint8_t buffer[8];
    int ind = 0;

    ubx_put_U1(buffer, &ind, msg_class);
    ubx_put_U1(buffer, &ind, id);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_MSG, buffer, ind, 10);
}

/**
 * Set the measurement rate, navigation rate and time reference.
 *
 * @param meas_rate_ms
 * The elapsed time between GNSS measurements, which defines the rate,
 * e.g. 100ms => 10Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
 *
 * @param nav_rate_ms
 * The ratio between the number of measurements and the number of navigation
 * solutions, e.g. 5 means five measurements for every navigation solution.
 * Max. value is 127. (This parameter is ignored and the navRate is fixed to 1
 * in protocol versions less than 18)
 *
 * @param time_ref
 * The time system to which measurements are aligned:
 * 0: UTC time
 * 1: GPS time
 * 2: GLONASS time (not supported in protocol versions less than 18)
 * 3: BeiDou time (not supported in protocol versions less than 18)
 * 4: Galileo time (not supported in protocol versions less than 18)
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgRate(uint16_t meas_rate_ms, uint16_t nav_rate_ms, uint16_t time_ref)
{
    uint8_t buffer[6];
    int ind = 0;

    ubx_put_U2(buffer, &ind, meas_rate_ms);
    ubx_put_U2(buffer, &ind, nav_rate_ms);
    ubx_put_U2(buffer, &ind, time_ref);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_RATE, buffer, ind, 10);
}

/**
 * Reset receiver / Clear backup data structures
 *
 * @param navBbrMask
 * BBR sections to clear. The following special sets apply:
 * 0x0000 Hot start
 * 0x0001 Warm start
 * 0xFFFF Cold start
 *
 * @param resetMode
 * Reset Type:
 * 0x00 = Hardware reset (watchdog) immediately
 * 0x01 = Controlled software reset
 * 0x02 = Controlled software reset (GNSS only)
 * 0x04 = Hardware reset (watchdog) after shutdown
 * 0x08 = Controlled GNSS stop
 * 0x09 = Controlled GNSS start
 *
 * @return
 * true: ack or nak received
 * false: timeout
 * Do not expect this message to be acknowledged by the receiver
 * Newer FW version will not acknowledge this message at all.
 */
bool Ublox::ubxCfgRst(uint16_t navBbrMask, uint8_t resetMode)
{
    uint8_t buffer[4];
    int ind = 0;

    ubx_put_X2(buffer, &ind, navBbrMask);
    ubx_put_U1(buffer, &ind, resetMode);
    ubx_put_U1(buffer, &ind, 0); // Reserved

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_CFG, buffer, ind, 10);
}

bool Ublox::ubloxCfgCfg(ubx_cfg_cfg *cfg)
{
    uint8_t buffer[13];
    int ind = 0;

    uint32_t clear = 0;
    clear |= (cfg->clear_io_port ? 1 : 0) << 0;
    clear |= (cfg->clear_msg_conf ? 1 : 0) << 1;
    clear |= (cfg->clear_inf_msg ? 1 : 0) << 2;
    clear |= (cfg->clear_nav_conf ? 1 : 0) << 3;
    clear |= (cfg->clear_rxm_conf ? 1 : 0) << 4;
    clear |= (cfg->clear_sen_conf ? 1 : 0) << 8;
    clear |= (cfg->clear_rinv_conf ? 1 : 0) << 9;
    clear |= (cfg->clear_ant_conf ? 1 : 0) << 10;
    clear |= (cfg->clear_log_conf ? 1 : 0) << 11;
    clear |= (cfg->clear_fts_conf ? 1 : 0) << 12;

    uint32_t save = 0;
    save |= (cfg->save_io_port ? 1 : 0) << 0;
    save |= (cfg->save_msg_conf ? 1 : 0) << 1;
    save |= (cfg->save_inf_msg ? 1 : 0) << 2;
    save |= (cfg->save_nav_conf ? 1 : 0) << 3;
    save |= (cfg->save_rxm_conf ? 1 : 0) << 4;
    save |= (cfg->save_sen_conf ? 1 : 0) << 8;
    save |= (cfg->save_rinv_conf ? 1 : 0) << 9;
    save |= (cfg->save_ant_conf ? 1 : 0) << 10;
    save |= (cfg->save_log_conf ? 1 : 0) << 11;
    save |= (cfg->save_fts_conf ? 1 : 0) << 12;

    uint32_t load = 0;
    load |= (cfg->load_io_port ? 1 : 0) << 0;
    load |= (cfg->load_msg_conf ? 1 : 0) << 1;
    load |= (cfg->load_inf_msg ? 1 : 0) << 2;
    load |= (cfg->load_nav_conf ? 1 : 0) << 3;
    load |= (cfg->load_rxm_conf ? 1 : 0) << 4;
    load |= (cfg->load_sen_conf ? 1 : 0) << 8;
    load |= (cfg->load_rinv_conf ? 1 : 0) << 9;
    load |= (cfg->load_ant_conf ? 1 : 0) << 10;
    load |= (cfg->load_log_conf ? 1 : 0) << 11;
    load |= (cfg->load_fts_conf ? 1 : 0) << 12;

    uint8_t device = 0;
    device |= (cfg->dev_bbr ? 1 : 0) << 0;
    device |= (cfg->dev_flash ? 1 : 0) << 1;
    device |= (cfg->dev_eeprom ? 1 : 0) << 2;
    device |= (cfg->dev_spi_flash ? 1 : 0) << 4;

    ubx_put_X4(buffer, &ind, clear);
    ubx_put_X4(buffer, &ind, save);
    ubx_put_X4(buffer, &ind, load);
    ubx_put_X1(buffer, &ind, device);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_CFG, buffer, ind, 10);
}

/**
 * Set the nav5 configuration.
 *
 * @param cfg
 * The configuration.
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgNav5(ubx_cfg_nav5 *cfg)
{
    uint8_t buffer[36];
    int ind = 0;

    uint16_t mask = 0;
    mask |= (cfg->apply_dyn ? 1 : 0) << 0;
    mask |= (cfg->apply_min_el ? 1 : 0) << 1;
    mask |= (cfg->apply_pos_fix_mode ? 1 : 0) << 2;
    mask |= (cfg->apply_pos_mask ? 1 : 0) << 4;
    mask |= (cfg->apply_time_mask ? 1 : 0) << 5;
    mask |= (cfg->apply_static_hold_mask ? 1 : 0) << 6;
    mask |= (cfg->apply_dgps ? 1 : 0) << 7;
    mask |= (cfg->apply_cno ? 1 : 0) << 8;
    mask |= (cfg->apply_utc ? 1 : 0) << 10;

    ubx_put_X2(buffer, &ind, mask);
    ubx_put_U1(buffer, &ind, cfg->dyn_model);
    ubx_put_U1(buffer, &ind, cfg->fix_mode);
    ubx_put_I4(buffer, &ind, cfg->fixed_alt * D(100.0));
    ubx_put_U4(buffer, &ind, cfg->fixed_alt_var * D(10000.0));
    ubx_put_I1(buffer, &ind, cfg->min_elev);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U2(buffer, &ind, cfg->p_dop * 10.0);
    ubx_put_U2(buffer, &ind, cfg->t_dop * 10.0);
    ubx_put_U2(buffer, &ind, cfg->p_acc);
    ubx_put_U2(buffer, &ind, cfg->t_acc);
    ubx_put_U1(buffer, &ind, cfg->static_hold_thres);
    ubx_put_U1(buffer, &ind, cfg->dgnss_timeout);
    ubx_put_U1(buffer, &ind, cfg->cno_tres_num_sat);
    ubx_put_U1(buffer, &ind, cfg->cno_tres);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U2(buffer, &ind, cfg->static_hold_max_dist);
    ubx_put_U1(buffer, &ind, cfg->utc_standard);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_NAV5, buffer, ind, 10);
}

bool Ublox::ubloxCfgTp5(ubx_cfg_tp5 *cfg)
{
    uint8_t buffer[32];
    int ind = 0;

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 1);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_I2(buffer, &ind, cfg->ant_cable_delay);
    ubx_put_I2(buffer, &ind, cfg->rf_group_delay);
    ubx_put_U4(buffer, &ind, cfg->freq_period);
    ubx_put_U4(buffer, &ind, cfg->freq_period_lock);
    ubx_put_U4(buffer, &ind, cfg->pulse_len_ratio);
    ubx_put_U4(buffer, &ind, cfg->pulse_len_ratio_lock);
    ubx_put_I4(buffer, &ind, cfg->user_config_delay);

    uint32_t mask = 0;
    mask |= (cfg->active ? 1 : 0) << 0;
    mask |= (cfg->lockGnssFreq ? 1 : 0) << 1;
    mask |= (cfg->lockedOtherSet ? 1 : 0) << 2;
    mask |= (cfg->isFreq ? 1 : 0) << 3;
    mask |= (cfg->isLength ? 1 : 0) << 4;
    mask |= (cfg->alignToTow ? 1 : 0) << 5;
    mask |= (cfg->polarity ? 1 : 0) << 6;
    mask |= (cfg->gridUtcGnss & 0b1111) << 7;
    mask |= (cfg->syncMode & 0b111) << 8;

    ubx_put_X4(buffer, &ind, mask);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_TP5, buffer, ind, 10);
}

bool Ublox::ubloxCfgGnss(ubx_cfg_gnss *gnss)
{
    const int max_num_blocks = 10;
    if (gnss->num_blocks > max_num_blocks) {
        return false;
    }

    uint8_t buffer[4 + 8 * max_num_blocks];
    int ind = 0;

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, gnss->num_ch_hw);
    ubx_put_U1(buffer, &ind, gnss->num_ch_use);
    ubx_put_U1(buffer, &ind, gnss->num_blocks);

    for (int i = 0;i < gnss->num_blocks;i++) {
        ubx_put_U1(buffer, &ind, gnss->blocks[i].gnss_id);
        ubx_put_U1(buffer, &ind, gnss->blocks[i].minTrkCh);
        ubx_put_U1(buffer, &ind, gnss->blocks[i].maxTrkCh);
        ubx_put_U1(buffer, &ind, 0);
        uint32_t flags = gnss->blocks[i].en ? 1 : 0;
        flags |= gnss->blocks[i].flags << 16;
        ubx_put_X4(buffer, &ind, flags);
    }

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_GNSS, buffer, ind, 100);
}

bool Ublox::ubloxCfgNmea(ubx_cfg_nmea *nmea)
{
    uint8_t buffer[20];
    int ind = 0;

    uint8_t filter = 0;
    filter |= (nmea->posFilt ? 1 : 0) << 0;
    filter |= (nmea->mskPosFilt ? 1 : 0) << 1;
    filter |= (nmea->timeFilt ? 1 : 0) << 2;
    filter |= (nmea->dateFilt ? 1 : 0) << 3;
    filter |= (nmea->gpsOnlyFilt ? 1 : 0) << 4;
    filter |= (nmea->trackFilt ? 1 : 0) << 5;
    ubx_put_X1(buffer, &ind, filter);

    ubx_put_U1(buffer, &ind, nmea->nmeaVersion);
    ubx_put_U1(buffer, &ind, nmea->numSv);

    uint8_t flags = 0;
    flags |= (nmea->compat ? 1 : 0) << 0;
    flags |= (nmea->consider ? 1 : 0) << 1;
    flags |= (nmea->limit82 ? 1 : 0) << 2;
    flags |= (nmea->highPrec ? 1 : 0) << 3;
    ubx_put_X1(buffer, &ind, flags);

    uint32_t gnss_filter = 0;
    gnss_filter |= (nmea->disableGps ? 1 : 0) << 0;
    gnss_filter |= (nmea->disableSbas ? 1 : 0) << 1;
    gnss_filter |= (nmea->disableQzss ? 1 : 0) << 4;
    gnss_filter |= (nmea->disableGlonass ? 1 : 0) << 5;
    gnss_filter |= (nmea->disableBeidou ? 1 : 0) << 6;
    ubx_put_X4(buffer, &ind, gnss_filter);

    ubx_put_U1(buffer, &ind, nmea->svNumbering);
    ubx_put_U1(buffer, &ind, nmea->mainTalkerId);
    ubx_put_U1(buffer, &ind, nmea->gsvTalkerId);
    ubx_put_U1(buffer, &ind, 1);
    ubx_put_I1(buffer, &ind, nmea->bdsTalkerId[0]);
    ubx_put_I1(buffer, &ind, nmea->bdsTalkerId[1]);

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_NMEA, buffer, ind, 100);
}

bool Ublox::ubloxCfgValset(unsigned char *values, int len,
                           bool ram, bool bbr, bool flash)
{
    const size_t buffersize = 4096;
    assert(((unsigned)len < buffersize) && "Ublox::ubloxCfgValset: message size exceeded");

    uint8_t buffer[buffersize];
    memset(&buffer, 0, buffersize);

    int ind = 0;

    ubx_put_U1(buffer, &ind, 0);

    uint8_t mask = 0;
    mask |= (ram ? 1 : 0) << 0;
    mask |= (bbr ? 1 : 0) << 1;
    mask |= (flash ? 1 : 0) << 2;
    ubx_put_X1(buffer, &ind, mask);

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    memcpy(buffer + ind, values, len);
    ind += len;

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_VALSET, buffer, ind, 500);
}

/**
 *This message is used to get configuration values by providing a list of configuration key IDs, which
 *identify the configuration items to retrieve.
 *
 *This message can specify the configuration layer where the values of the specified configuration items
 *are retrieved from.
 *
 *This message is limited to containing a maximum of 64 key IDs.
 *
 *This message returns a UBX-ACK-NAK:
 *if any key is unknown to the receiver FW
 *if the layer field specifies an invalid layer to get the value from
 *if the keys array specifies more than 64 key IDs
 *
 * @param keys
 * Configuration key IDs of the configuration items to be retrieved
 *
 * @param len
 * Number of key value pairs to retrieve
 *
 * @param layer
 * The layer from which the configuration items should be retrieved:
 * 0: RAM layer
 * 1: BBR layer
 * 2: Flash layer
 * 7: Default layer
 *
 * @param position
 * Skip this many key values before constructing output message
 */
bool Ublox::ubloxCfgValget(unsigned char *keys, int len, uint8_t layer, uint16_t position)
{
    const size_t buffersize = 4096;
    assert(((unsigned)len < buffersize) && "Ublox::ubloxCfgValset: message size exceeded");

    uint8_t buffer[buffersize];
    memset(&buffer, 0, buffersize);

    int ind = 0;

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, layer);
    ubx_put_U2(buffer, &ind, position);

    memcpy(buffer + ind, keys, len);
    ind += len;

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_VALGET, buffer, ind, 100);
}

void Ublox::ubloxCfgAppendKey(unsigned char *buffer, int *ind, uint32_t key)
{
    ubx_put_U4(buffer, ind, key);
}

void Ublox::ubloxCfgAppendEnableGps(unsigned char *buffer, int *ind,
                                    bool en, bool en_l1c, bool en_l2c)
{
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GPS_ENA);
    ubx_put_U1(buffer, ind, en);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GPS_L1C_ENA);
    ubx_put_U1(buffer, ind, en_l1c);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GPS_L2C_ENA);
    ubx_put_U1(buffer, ind, en_l2c);
}

void Ublox::ubloxCfgAppendEnableGal(unsigned char *buffer, int *ind,
                                    bool en, bool en_e1, bool en_e5b)
{
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GAL_ENA);
    ubx_put_U1(buffer, ind, en);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GAL_E1_ENA);
    ubx_put_U1(buffer, ind, en_e1);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GAL_E5B_ENA);
    ubx_put_U1(buffer, ind, en_e5b);
}

void Ublox::ubloxCfgAppendEnableBds(unsigned char *buffer, int *ind,
                                    bool en, bool en_b1, bool en_b2)
{
    ubx_put_X4(buffer, ind, CFG_SIGNAL_BDS_ENA);
    ubx_put_U1(buffer, ind, en);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_BDS_B1_ENA);
    ubx_put_U1(buffer, ind, en_b1);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_BDS_B2_ENA);
    ubx_put_U1(buffer, ind, en_b2);
}

void Ublox::ubloxCfgAppendEnableGlo(unsigned char *buffer, int *ind,
                                    bool en, bool en_l1, bool en_l2)
{
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GLO_ENA);
    ubx_put_U1(buffer, ind, en);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GLO_L1_ENA);
    ubx_put_U1(buffer, ind, en_l1);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GLO_L2_ENA);
    ubx_put_U1(buffer, ind, en_l2);
}

void Ublox::ubloxCfgAppendUart1Baud(unsigned char *buffer, int *ind, uint32_t baudrate)
{
    ubx_put_X4(buffer, ind, CFG_UART1_BAUDRATE);
    ubx_put_U4(buffer, ind, baudrate);
}

void Ublox::ubloxCfgAppendUart1InProt(unsigned char *buffer, int *ind, bool ubx, bool nmea, bool rtcm3x)
{
    ubx_put_X4(buffer, ind, CFG_UART1INPROT_UBX);
    ubx_put_U1(buffer, ind, ubx);
    ubx_put_X4(buffer, ind, CFG_UART1INPROT_NMEA);
    ubx_put_U1(buffer, ind, nmea);
    ubx_put_X4(buffer, ind, CFG_UART1INPROT_RTCM3X);
    ubx_put_U1(buffer, ind, rtcm3x);
}

void Ublox::ubloxCfgAppendUart1OutProt(unsigned char *buffer, int *ind, bool ubx, bool nmea, bool rtcm3x)
{
    ubx_put_X4(buffer, ind, CFG_UART1OUTPROT_UBX);
    ubx_put_U1(buffer, ind, ubx);
    ubx_put_X4(buffer, ind, CFG_UART1OUTPROT_NMEA);
    ubx_put_U1(buffer, ind, nmea);
    ubx_put_X4(buffer, ind, CFG_UART1OUTPROT_RTCM3X);
    ubx_put_U1(buffer, ind, rtcm3x);
}

/**
 * The HPS feature can be enabled and disabled with the CFG-SFCORE-USE_SF key.
 * If HPS is disabled, the receiver outputs a GNSS-only solution.
 * IMU sensor measurements are still available in UBX-ESF-MEAS and UBX-ESF-RAW messages.
 *
 * @param ena
 * Enable automatic IMU-mount alignment
 */
void Ublox::ubloxCfgAppendEnableSf(unsigned char *buffer, int *ind, bool ena)
{
    ubx_put_X4(buffer, ind, CFG_SFCORE_USE_SF);
    ubx_put_U1(buffer, ind, ena);
}

/**
 * The automatic IMU-mount alignment engine automatically estimates the IMU-mount roll, pitch and
 * yaw angles. It requires an initialization phase during which no INS/GNSS fusion can be achieved
 * It is strongly recommended to use the automatic IMU-mount alignment.
 *
 * https://www.u-blox.com/en/ubx-viewer/view/ZED-F9R_Integrationmanual_UBX-20039643?url=https%3A%2F%2Fwww.u-blox.com%2Fsites%2Fdefault%2Ffiles%2FZED-F9R_Integrationmanual_UBX-20039643.pdf#%5B%7B%22num%22%3A370%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C59.527%2C140.168%2Cnull%5D
 * 3.2.3.1.1 Automatic IMU-mount alignment
 * 3.2.3.1.2 User-defined IMU-mount alignment
 *
 * @param automatic
 * Enable automatic IMU-mount alignment
 *
 * @param yaw
 * User-defined IMU-mount yaw angle [0, 360] in deg
 *
 * @param pitch
 * User-defined IMU-mount pitch angle [-90, 90] in deg
 *
 * @param roll
 * User-defined IMU-mount roll angle [-180, 180] in deg
 */
void Ublox::ubloxCfgAppendMntalg(unsigned char *buffer, int *ind, bool automatic, uint32_t yaw, int16_t pitch, int16_t roll)
{
    if (automatic) {
    ubx_put_X4(buffer, ind, CFG_SFIMU_AUTO_MNTALG_ENA);
    ubx_put_U1(buffer, ind, automatic);
    } else {
    ubx_put_X4(buffer, ind, CFG_SFIMU_IMU_MNTALG_YAW);
    ubx_put_I2(buffer, ind, yaw);
    ubx_put_X4(buffer, ind, CFG_SFIMU_IMU_MNTALG_PITCH);
    ubx_put_I1(buffer, ind, pitch);
    ubx_put_X4(buffer, ind, CFG_SFIMU_IMU_MNTALG_ROLL);
    ubx_put_I2(buffer, ind, roll);
    }
}

/**
 * Set the measurement rate, navigation rate, and time reference.
 *
 * @param meas
 * The elapsed time between GNSS measurements, which defines the rate,
 * e.g. 100ms => 10Hz, 1000ms => 1Hz, 10000ms => 0.1Hz. The minimum value is 25.
 *
 * @param nav
 * The ratio between the number of measurements and the number of navigation
 * solutions, e.g. 5 means five measurements for every navigation solution.
 * Max. value is 127. (This parameter is ignored and the navRate is fixed to 1
 * in protocol versions less than 18)
 *
 * @param timeref
 * The time system to which measurements are aligned:
 * 0: UTC time
 * 1: GPS time
 * 2: GLONASS time (not supported in protocol versions less than 18)
 * 3: BeiDou time (not supported in protocol versions less than 18)
 * 4: Galileo time (not supported in protocol versions less than 18)
 *
 * @param prio
 * When not zero, the receiver outputs navigation data as a set of messages with two priority levels: 1)
 * Priority messages:
 * Navigation solution data are computed and output with high rate and low latency; 2)
 * Non-priority messages auxiliary navigation data are computed and output with low rate and higher latency.
 * When zero, the receiver outputs the navigation data as a set of messages with the same priority.
 * The allowed range for the priority navigation mode is 0-30 Hz.
 */
void Ublox::ubloxCfgAppendRate(unsigned char *buffer, int *ind, uint16_t meas, uint16_t nav, uint8_t timeref)
{
    ubx_put_X4(buffer,ind, CFG_RATE_MEAS);
    ubx_put_U2(buffer, ind, meas);
    ubx_put_X4(buffer, ind, CFG_RATE_NAV);
    ubx_put_U2(buffer, ind, nav);
    ubx_put_X4(buffer, ind, CFG_RATE_TIMEREF);
    ubx_put_U1(buffer, ind, timeref);
}

/**
 * Set the measurement rate, navigation rate, time reference and navigation rate.
 *
 * @param meas
 * The elapsed time between GNSS measurements, which defines the rate,
 * e.g. 100ms => 10Hz, 1000ms => 1Hz, 10000ms => 0.1Hz. The minimum value is 25.
 *
 * @param nav
 * The ratio between the number of measurements and the number of navigation
 * solutions, e.g. 5 means five measurements for every navigation solution.
 * Max. value is 127. (This parameter is ignored and the navRate is fixed to 1
 * in protocol versions less than 18)
 *
 * @param timeref
 * The time system to which measurements are aligned:
 * 0: UTC time
 * 1: GPS time
 * 2: GLONASS time (not supported in protocol versions less than 18)
 * 3: BeiDou time (not supported in protocol versions less than 18)
 * 4: Galileo time (not supported in protocol versions less than 18)
 *
 * @param prio
 * When not zero, the receiver outputs navigation data as a set of messages with two priority levels: 1)
 * Priority messages:
 * Navigation solution data are computed and output with high rate and low latency; 2)
 * Non-priority messages auxiliary navigation data are computed and output with low rate and higher latency.
 * When zero, the receiver outputs the navigation data as a set of messages with the same priority.
 * The allowed range for the priority navigation mode is 0-30 Hz.
 */
void Ublox::ubloxCfgAppendRate(unsigned char *buffer, int *ind, uint16_t meas, uint16_t nav, uint8_t timeref, uint8_t prio)
{
    ubx_put_X4(buffer,ind, CFG_RATE_MEAS);
    ubx_put_U2(buffer, ind, meas);
    ubx_put_X4(buffer, ind, CFG_RATE_NAV);
    ubx_put_U2(buffer, ind, nav);
    ubx_put_X4(buffer, ind, CFG_RATE_TIMEREF);
    ubx_put_U1(buffer, ind, timeref);
    ubx_put_X4(buffer, ind, CFG_RATE_NAV_PRIO);
    ubx_put_U1(buffer, ind, prio);
}

/**
 * Reset receiver / Clear backup data structures
 *
 * @param cmd
 * 0 Create backup in flash
 * 1 Clear backup in flash
 * 7 Poll backup restore status
 */
void Ublox::ubloxUpdSos(uint8_t cmd)
{
    if (cmd == 7){
        ubx_encode_send(UBX_CLASS_UPD, UBX_UPD_SOS, 0, 0);
    } else if(cmd == 0 || cmd == 1){
        uint8_t buffer[4];
        int ind = 0;

        ubx_put_U1(buffer, &ind, cmd);
        ubx_put_U1(buffer, &ind, 0); // Reserved
        ubx_put_U1(buffer, &ind, 0); // Reserved
        ubx_put_U1(buffer, &ind, 0); // Reserved

        ubx_encode_send(UBX_CLASS_UPD, UBX_UPD_SOS, buffer, ind, 0);
    }
}

/**
 * Input external wheel tick or speed measurements from a host to ZED-F9R.
 * It is strongly recommended to use the absolute wheel tick sensors to ensure robust
 * measurement processing even after sensor failures or outages
 *
 * @param dataType
 * Data type 10 shall be used for single-tick odometer data and data type 11 shall be used for
 * speed odometer data
 * @param dataField
 * Bits 0-22: unsigned tick value. Bit 23: direction indicator (0=forward, 1=backward)
 * The speed data shall be delivered in meters per second.
 */
void Ublox::ubloxOdometerInput(ubx_esf_datatype_enum dataType, uint32_t dataField)
{
    uint8_t buffer[12];
    int ind = 0;

    uint32_t data = 0;
    data |= dataField << 0;
    data |= dataType << 24;

    uint32_t timeTag = QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()).msecsSinceStartOfDay();

    ubx_put_U4(buffer, &ind, timeTag); // TODO: Time  tag  of  measurement  generated  by  external sensor?
    ubx_put_X2(buffer, &ind, 4096); // Binary: 0001000000000000. Flags. Set all unused bits to zero. We have 1 measurement
    ubx_put_U2(buffer, &ind, 0); // Identification number of data provider?
    ubx_put_X4(buffer, &ind, data);

    ubx_encode_send(UBX_CLASS_ESF, UBX_ESF_MEAS, buffer, ind, 0);
}

void Ublox::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();

        for (int i = 0;i < data.size();i++) {
            uint8_t ch = data.at(i);
            bool ch_used = false;

            // RTCM
            if (!ch_used && mDecoderState.line_pos == 0 && mDecoderState.ubx_pos == 0) {
                int res = rtcm3_input_data(ch, &mRtcmState);
                ch_used = res >= 0;

                if (res >= 1000) {
                    QByteArray rtcmData((const char*)mRtcmState.buffer, mRtcmState.len + 3);
                    emit rtcmRx(rtcmData, res);
                }
            }

            // Ubx
            if (!ch_used && mDecoderState.line_pos == 0) {
                unsigned ubx_pos_last = mDecoderState.ubx_pos;

                if (mDecoderState.ubx_pos == 0) {
                    if (ch == 0xB5) {
                        mDecoderState.ubx_pos++;
                    }
                } else if (mDecoderState.ubx_pos == 1) {
                    if (ch == 0x62) {
                        mDecoderState.ubx_pos++;
                        mDecoderState.ubx_ck_a = 0;
                        mDecoderState.ubx_ck_b = 0;
                    }
                } else if (mDecoderState.ubx_pos == 2) {
                    mDecoderState.ubx_class = ch;
                    mDecoderState.ubx_ck_a += ch;
                    mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                    mDecoderState.ubx_pos++;
                } else if (mDecoderState.ubx_pos == 3) {
                    mDecoderState.ubx_id = ch;
                    mDecoderState.ubx_ck_a += ch;
                    mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                    mDecoderState.ubx_pos++;
                } else if (mDecoderState.ubx_pos == 4) {
                    mDecoderState.ubx_len = ch;
                    mDecoderState.ubx_ck_a += ch;
                    mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                    mDecoderState.ubx_pos++;
                } else if (mDecoderState.ubx_pos == 5) {
                    mDecoderState.ubx_len |= ch << 8;
                    if (mDecoderState.ubx_len >= sizeof(mDecoderState.ubx)) {
                        qDebug() << "Too large UBX packet" << mDecoderState.ubx_len;
                    } else {
                        mDecoderState.ubx_ck_a += ch;
                        mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                        mDecoderState.ubx_pos++;
                    }
                } else if ((mDecoderState.ubx_pos - 6) < mDecoderState.ubx_len) {
                    mDecoderState.ubx[mDecoderState.ubx_pos - 6] = ch;
                    mDecoderState.ubx_ck_a += ch;
                    mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                    mDecoderState.ubx_pos++;
                } else if ((mDecoderState.ubx_pos - 6) == mDecoderState.ubx_len) {
                    if (ch == mDecoderState.ubx_ck_a) {
                        mDecoderState.ubx_pos++;
                    }
                } else if ((mDecoderState.ubx_pos - 6) == (mDecoderState.ubx_len + 1)) {
                    if (ch == mDecoderState.ubx_ck_b) {
                        ubx_decode(mDecoderState.ubx_class, mDecoderState.ubx_id,
                                   mDecoderState.ubx, mDecoderState.ubx_len);
                        mDecoderState.ubx_pos = 0;
                    }
                }

                if (ubx_pos_last != mDecoderState.ubx_pos) {
                    ch_used = true;
                } else {
                    mDecoderState.ubx_pos = 0;
                }
            }

            // NMEA
            if (!ch_used) {
                mDecoderState.line[mDecoderState.line_pos++] = ch;
                if (mDecoderState.line_pos == sizeof (mDecoderState.line)) {
                    mDecoderState.line_pos = 0;
                }

                if (mDecoderState.line_pos > 0 && mDecoderState.line[mDecoderState.line_pos - 1] == '\n') {
                    mDecoderState.line[mDecoderState.line_pos] = '\0';
                    mDecoderState.line_pos = 0;

                    QByteArray line((char*)mDecoderState.line);
                    // Check whether this is NMEA GGA with correct checksum
                    // Example: $GNGGA,120020.115,5743.153,N,01256.431,E,1,12,1.0,0.0,M,0.0,M,,*6E
                    if (line.at(0) == '$' && line.size() > 6)
                        if (line.mid(3, 3) == QString("GGA")) {
                            QList split = line.split('*');
                            if (split.size() == 2) {
                                // Test checksum
                                int checksum = 0;
                                for (int i = 1; i < split.at(0).size(); i++)
                                    checksum ^= split.at(0).at(i);
                                if (checksum == split.at(1).trimmed().toInt(nullptr, 16))
                                    emit rxNmeaGga(line);
                            }
                        }
                }
            }
        }
    }
}

void Ublox::serialPortError(QSerialPort::SerialPortError error)
{
    QString message;
    switch (error) {
    case QSerialPort::NoError:
        break;

    default:
        message = "Serial port error: " + mSerialPort->errorString();
        break;
    }

    if(!message.isEmpty()) {
        qDebug() << message;

        if(mSerialPort->isOpen()) {
            mSerialPort->close();
        }
    }
}

void Ublox::ubx_send(QByteArray data)
{
    if (mSerialPort->isOpen()) {
        mSerialPort->write(data);
    }
}

bool Ublox::ubx_encode_send(uint8_t msg_class, uint8_t id, uint8_t *msg, int len, int timeoutMs)
{
    auto ubx = ubx_encode(msg_class, id, QByteArray((const char*)msg, len));

    bool retVal = false;
    if (timeoutMs > 0) {
        if (mWaitingAck) {
            qDebug() << "Already waiting for ack";
        } else {
            mWaitingAck = true;

            QEventLoop loop;
            QTimer timeoutTimer;
            timeoutTimer.setSingleShot(true);
            timeoutTimer.start(timeoutMs);
            auto conn = connect(this, &Ublox::rxAck,
                                [&loop, &retVal](uint8_t, uint8_t){retVal = true; loop.quit();});
            connect(this, &Ublox::rxNak, &loop, &QEventLoop::quit);
            connect(&timeoutTimer, &QTimer::timeout, &loop, &QEventLoop::quit);

            ubx_send(ubx);
            loop.exec();

            disconnect(conn);

            mWaitingAck = false;
        }
    } else {
        ubx_send(ubx);
        retVal = true;
    }

    return retVal;
}

QByteArray Ublox::ubx_encode(uint8_t msg_class, uint8_t id, const QByteArray &data)
{
    QByteArray ubx;

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    ubx.append(char(0xB5));
    ubx.append(char(0x62));

    ubx.append(msg_class);
    ck_a += ubx.at(ubx.size() - 1);
    ck_b += ck_a;

    ubx.append(id);
    ck_a += ubx.at(ubx.size() - 1);
    ck_b += ck_a;

    ubx.append(data.size() & 0xFF);
    ck_a += ubx.at(ubx.size() - 1);
    ck_b += ck_a;

    ubx.append((data.size() >> 8) & 0xFF);
    ck_a += ubx.at(ubx.size() - 1);
    ck_b += ck_a;

    for (int i = 0;i < data.size();i++) {
        ubx.append(data.at(i));
        ck_a += data.at(i);
        ck_b += ck_a;
    }

    ubx.append(ck_a);
    ubx.append(ck_b);

    return ubx;
}

void Ublox::ubx_decode(uint8_t msg_class, uint8_t id, uint8_t *msg, int len)
{
    emit ubxRx(ubx_encode(msg_class, id, QByteArray((const char*)msg, len)));

    switch (msg_class) {
    case UBX_CLASS_NAV: {
        switch (id) {
        case UBX_NAV_RELPOSNED:
            ubx_decode_relposned(msg, len);
            break;
        case UBX_NAV_SVIN:
            ubx_decode_svin(msg, len);
            break;
        case UBX_NAV_SOL: // TODO: dropped on F9P, implement UBX-NAV-PVT or UBX-NAV-HPPOSLLH (see: https://cdn.sparkfun.com/assets/learn_tutorials/8/5/6/ZED-F9P_FW_1.00_HPG_1.00_release_notes.pdf)
            ubx_decode_nav_sol(msg, len);
            break;
        case UBX_NAV_PVT:
            ubx_decode_nav_pvt(msg, len);
            break;
        case UBX_NAV_SAT:
            ubx_decode_nav_sat(msg, len);
            break;
        default:
            break;
        }
    } break;

    case UBX_CLASS_ACK: {
        switch (id) {
        case UBX_ACK_ACK:
            ubx_decode_ack(msg, len);
            break;
        case UBX_ACK_NAK:
            ubx_decode_nak(msg, len);
            break;
        default:
            break;
        }
    } break;

    case UBX_CLASS_ESF: {
        switch (id) {
        case UBX_ESF_MEAS:
            ubx_decode_esf_meas(msg, len);
            break;
         case UBX_ESF_STATUS:
            ubx_decode_esf_status(msg, len);
            break;
        case UBX_ESF_ALG:
           ubx_decode_esf_alg(msg, len);
           break;
         default:
            break;
        }
    } break;

    case UBX_CLASS_RXM: {
        switch (id) {
        case UBX_RXM_RAWX:
            ubx_decode_rawx(msg, len);
            break;
        default:
            break;
        }
    } break;

    case UBX_CLASS_CFG: {
        switch (id) {
        case UBX_CFG_GNSS:
            ubx_decode_cfg_gnss(msg, len);
            break;
        case UBX_CFG_VALGET:
            ubx_decode_cfg_valget(msg, len);
            break;
        default:
            break;
        }
    } break;

    case UBX_CLASS_MON: {
        switch (id) {
        case UBX_MON_VER:
            ubx_decode_mon_ver(msg, len);
            break;
        default:
            break;
        }
    } break;

    case UBX_CLASS_UPD: {
        switch (id) {
        case UBX_UPD_SOS:
            ubx_decode_upd_sos(msg, len);
            break;
        default:
            break;
        }
    } break;

    default:
        break;
    }
}

void Ublox::ubx_decode_nav_sol(uint8_t *msg, int len)
{
    (void)len;

    static ubx_nav_sol sol;
    int ind = 0;
    uint8_t flags;

    sol.i_tow = ubx_get_U4(msg, &ind); // 0
    sol.f_tow = ubx_get_I4(msg, &ind); // 4
    sol.weel = ubx_get_I2(msg, &ind); // 8
    sol.gps_fix = ubx_get_U1(msg, &ind); // 10
    flags = ubx_get_X1(msg, &ind); // 11
    sol.gpsfixok = flags & 0x01;
    sol.diffsoln = flags & 0x02;
    sol.wknset = flags & 0x04;
    sol.towset = flags & 0x08;
    sol.ecef_x = (double)ubx_get_I4(msg, &ind) / D(100.0); // 12
    sol.ecef_y = (double)ubx_get_I4(msg, &ind) / D(100.0); // 16
    sol.ecef_z = (double)ubx_get_I4(msg, &ind) / D(100.0); // 20
    sol.p_acc = (float)ubx_get_U4(msg, &ind) / 100.0; // 24
    sol.ecef_vx = (float)ubx_get_I4(msg, &ind) / 100.0; // 28
    sol.ecef_vy = (float)ubx_get_I4(msg, &ind) / 100.0; // 32
    sol.ecef_vz = (float)ubx_get_I4(msg, &ind) / 100.0; // 36
    sol.s_acc = (float)ubx_get_U4(msg, &ind) / 100.0; // 40
    sol.p_dop = (float)ubx_get_U2(msg, &ind) * 0.01; // 44
    ind += 1; // 46
    sol.num_sv = ubx_get_U1(msg, &ind); // 47

    emit rxNavSol(sol);
}

void Ublox::ubx_decode_nav_pvt(uint8_t *msg, int len)
{
    (void)len;

    static ubx_nav_pvt pvt;
    int ind = 0;
    uint8_t flags;

    pvt.i_tow  = ubx_get_U4(msg, &ind); // 0
    pvt.year   = ubx_get_U2(msg, &ind); // 4
    pvt.month  = ubx_get_U1(msg, &ind); // 6
    pvt.day    = ubx_get_U1(msg, &ind); // 7
    pvt.hour   = ubx_get_U1(msg, &ind); // 8
    pvt.min    = ubx_get_U1(msg, &ind); // 9
    pvt.second = ubx_get_U1(msg, &ind); // 10

    flags              = ubx_get_X1(msg, &ind); // 11
    pvt.valid_date     = (flags >> 0) & 1;
    pvt.valid_time     = (flags >> 1) & 1;
    pvt.fully_resolved = (flags >> 2) & 1;
    pvt.valid_mag      = (flags >> 3) & 1;

    pvt.t_acc    = ubx_get_U4(msg, &ind); // 12
    pvt.nano     = ubx_get_I4(msg, &ind); // 16
    pvt.fix_type = ubx_get_U1(msg, &ind); // 20

    flags              = ubx_get_X1(msg, &ind); // 21
    pvt.gnss_fix_ok    = (flags >> 0) & 1;
    pvt.diffsoln       = (flags >> 1) & 1;
    pvt.psm_state      = (flags >> 2) & 6;
    pvt.head_veh_valid = (flags >> 5) & 1;
    pvt.carr_soln      = (flags >> 6) & 3;

    flags              = ubx_get_X1(msg, &ind); // 22
    pvt.confirmed_avai = (flags >> 5) & 1;
    pvt.confirmed_date = (flags >> 6) & 1;
    pvt.confirmed_time = (flags >> 7) & 1;

    pvt.num_sv   = ubx_get_U1(msg, &ind);                  // 23
    pvt.lon      = ((double)ubx_get_I4(msg, &ind))*1.0e-7; // 24
    pvt.lat      = ((double)ubx_get_I4(msg, &ind))*1.0e-7; // 28
    pvt.height   = ((double)ubx_get_I4(msg, &ind))*1.0e-3; // 32
    pvt.hmsl     = ((double)ubx_get_I4(msg, &ind))*1.0e-3; // 36
    pvt.h_acc    = ((double)ubx_get_U4(msg, &ind))*1.0e-3; // 40
    pvt.v_acc    = ((double)ubx_get_U4(msg, &ind))*1.0e-3; // 44
    pvt.vel_n    = ((double)ubx_get_I4(msg, &ind))*1.0e-3; // 48
    pvt.vel_e    = ((double)ubx_get_I4(msg, &ind))*1.0e-3; // 52
    pvt.vel_d    = ((double)ubx_get_I4(msg, &ind))*1.0e-3; // 56
    pvt.g_speed  = ((double)ubx_get_I4(msg, &ind))*1.0e-3; // 60
    pvt.head_mot = ((double)ubx_get_I4(msg, &ind))*1.0e-5; // 64
    pvt.s_acc    = ((double)ubx_get_U4(msg, &ind))*1.0e-3; // 68
    pvt.head_acc = ((double)ubx_get_U4(msg, &ind))*1.0e-5; // 72
    pvt.p_dop    = ((double)ubx_get_U2(msg, &ind))*1.0e-2; // 76

    flags           = ubx_get_X1(msg, &ind); // 78
    pvt.invalid_llh = (flags >> 0) & 1;

    ind += 5;  //  79-83 reserved

    pvt.head_veh = ((double)ubx_get_I4(msg, &ind))*1.0e-5; // 84
    pvt.mag_dec   = ((double)ubx_get_I2(msg, &ind))*1.0e-2; // 88
    pvt.mag_acc   = ((double)ubx_get_U2(msg, &ind))*1.0e-2; // 92

    emit rxNavPvt(pvt);
}

void Ublox::ubx_decode_relposned(uint8_t *msg, int len)
{
    (void)len;

    ubx_nav_relposned pos;
    int ind = 0;
    uint32_t flags;

    int version = ubx_get_U1(msg, &ind);
    ubx_get_U1(msg, &ind);

    pos.ref_station_id = ubx_get_U2(msg, &ind);
    pos.i_tow = ubx_get_U4(msg, &ind);
    pos.pos_n = (float)ubx_get_I4(msg, &ind) / 100.0;
    pos.pos_e = (float)ubx_get_I4(msg, &ind) / 100.0;
    pos.pos_d = (float)ubx_get_I4(msg, &ind) / 100.0;
    if (version == 1) {
        pos.pos_length = (float)ubx_get_I4(msg, &ind) / 100.0;
        pos.pos_heading = (float)ubx_get_I4(msg, &ind) / 100000.0;
        ind += 4;
    }

    pos.pos_n += (float)ubx_get_I1(msg, &ind) / 10000.0;
    pos.pos_e += (float)ubx_get_I1(msg, &ind) / 10000.0;
    pos.pos_d += (float)ubx_get_I1(msg, &ind) / 10000.0;
    if (version == 1)
        pos.pos_length += (float)ubx_get_I1(msg, &ind) / 10000.0;
    else
        ind += 1;
    pos.acc_n = (float)ubx_get_U4(msg, &ind) / 10000.0;
    pos.acc_e = (float)ubx_get_U4(msg, &ind) / 10000.0;
    pos.acc_d = (float)ubx_get_U4(msg, &ind) / 10000.0;
    if (version == 1) {
        pos.acc_length = (float)ubx_get_I4(msg, &ind) / 10000.0;
        pos.acc_heading = (float)ubx_get_I4(msg, &ind) / 100000.0;
        ind += 4;
    }

    flags = ubx_get_X4(msg, &ind);
    pos.fix_ok =                (flags >> 0) & 1;
    pos.diff_soln =             (flags >> 1) & 1;
    pos.rel_pos_valid =         (flags >> 2) & 1;
    pos.carr_soln =             (flags >> 3) & 3;
    pos.is_moving =             (flags >> 5) & 1;
    pos.ref_pos_miss =          (flags >> 6) & 1;
    pos.ref_obs_miss =          (flags >> 7) & 1;
    pos.rel_pos_heading_valid = (flags >> 8) & 1;
    pos.rel_pos_normalized =    (flags >> 9) & 1;

    emit rxRelPosNed(pos);
}

void Ublox::ubx_decode_svin(uint8_t *msg, int len)
{
    (void)len;

    ubx_nav_svin svin;
    int ind = 4;

    svin.i_tow = ubx_get_U4(msg, &ind);
    svin.dur = ubx_get_U4(msg, &ind);
    svin.meanX = (double)ubx_get_I4(msg, &ind) / D(100.0);
    svin.meanY = (double)ubx_get_I4(msg, &ind) / D(100.0);
    svin.meanZ = (double)ubx_get_I4(msg, &ind) / D(100.0);
    svin.meanX += (double)ubx_get_I1(msg, &ind) / D(10000.0);
    svin.meanY += (double)ubx_get_I1(msg, &ind) / D(10000.0);
    svin.meanZ += (double)ubx_get_I1(msg, &ind) / D(10000.0);
    ind += 1;
    svin.meanAcc = (float)ubx_get_U4(msg, &ind) / 10000.0;
    svin.obs = ubx_get_U4(msg, &ind);
    svin.valid = ubx_get_U1(msg, &ind);
    svin.active = ubx_get_U1(msg, &ind);

    emit rxSvin(svin);
}

void Ublox::ubx_decode_ack(uint8_t *msg, int len)
{
    (void)len;

    int ind = 0;

    uint8_t cls_id = ubx_get_I1(msg, &ind);
    uint8_t msg_id = ubx_get_I1(msg, &ind);

    emit rxAck(cls_id, msg_id);
}

void Ublox::ubx_decode_nak(uint8_t *msg, int len)
{
    (void)len;

    int ind = 0;

    uint8_t cls_id = ubx_get_I1(msg, &ind);
    uint8_t msg_id = ubx_get_I1(msg, &ind);

    emit rxNak(cls_id, msg_id);
}

void Ublox::ubx_decode_rawx(uint8_t *msg, int len)
{
    (void)len;

    ubx_rxm_rawx raw;
    int ind = 0;
    uint32_t flags;

    raw.rcv_tow = ubx_get_R8(msg, &ind);
    raw.week = ubx_get_U2(msg, &ind);
    raw.leaps = ubx_get_I1(msg, &ind);
    raw.num_meas = ubx_get_U1(msg, &ind);
    flags = ubx_get_X1(msg, &ind);
    raw.leap_sec = flags & 0x01;
    raw.clk_reset = flags & 0x02;

    ind = 16;

    for (int i = 0;i < raw.num_meas;i++) {
        raw.obs[i].pr_mes = ubx_get_R8(msg, &ind);
        raw.obs[i].cp_mes = ubx_get_R8(msg, &ind);
        raw.obs[i].do_mes = ubx_get_R4(msg, &ind);
        raw.obs[i].gnss_id = ubx_get_U1(msg, &ind);
        raw.obs[i].sv_id = ubx_get_U1(msg, &ind);
        ind += 1;
        raw.obs[i].freq_id = ubx_get_U1(msg, &ind);
        raw.obs[i].locktime = ubx_get_U2(msg, &ind);
        raw.obs[i].cno = ubx_get_U1(msg, &ind);
        raw.obs[i].pr_stdev = ubx_get_X1(msg, &ind) & 0x0F;
        raw.obs[i].cp_stdev = ubx_get_X1(msg, &ind) & 0x0F;
        raw.obs[i].do_stdev = ubx_get_X1(msg, &ind) & 0x0F;
        flags = ubx_get_X1(msg, &ind);
        raw.obs[i].pr_valid = flags & 0x01;
        raw.obs[i].cp_valid = flags & 0x02;
        raw.obs[i].half_cyc_valid = flags & 0x04;
        raw.obs[i].half_cyc_sub = flags & 0x08;
        ind += 1;
    }

    emit rxRawx(raw);
}

void Ublox::ubx_decode_nav_sat(uint8_t *msg, int len)
{
    (void)len;

    ubx_nav_sat sat;
    int ind = 0;

    sat.i_tow_ms = ubx_get_U4(msg, &ind);
    ubx_get_U1(msg, &ind);
    sat.num_sv = ubx_get_U1(msg, &ind);
    ubx_get_U1(msg, &ind);
    ubx_get_U1(msg, &ind);

    if (sat.num_sv > 128) {
        sat.num_sv = 128;
    }

    for (int i = 0;i < sat.num_sv;i++) {
        sat.sats[i].gnss_id = ubx_get_U1(msg, &ind);
        sat.sats[i].sv_id = ubx_get_U1(msg, &ind);
        sat.sats[i].cno = ubx_get_U1(msg, &ind);
        sat.sats[i].elev = ubx_get_I1(msg, &ind);
        sat.sats[i].azim = ubx_get_I2(msg, &ind);
        sat.sats[i].pr_res = (float)ubx_get_I2(msg, &ind) * 0.1;
        uint32_t flags = ubx_get_X4(msg, &ind);
        sat.sats[i].quality = (flags >> 0) & 0x07;
        sat.sats[i].used = (flags >> 3) & 0x01;
        sat.sats[i].health = (flags >> 4) & 0x03;
        sat.sats[i].diffcorr = (flags >> 6) & 0x01;
    }

    emit rxNavSat(sat);
}

void Ublox::ubx_decode_cfg_gnss(uint8_t *msg, int len)
{
    (void)len;

    ubx_cfg_gnss cfg;
    int ind = 0;

    ubx_get_U1(msg, &ind);
    cfg.num_ch_hw = ubx_get_U1(msg, &ind);
    cfg.num_ch_use = ubx_get_U1(msg, &ind);
    cfg.num_blocks = ubx_get_U1(msg, &ind);

    if (cfg.num_blocks > 10) {
        cfg.num_blocks = 10;
    }

    for (int i = 0;i < cfg.num_blocks;i++) {
        cfg.blocks[i].gnss_id = ubx_get_U1(msg, &ind);
        cfg.blocks[i].minTrkCh = ubx_get_U1(msg, &ind);
        cfg.blocks[i].maxTrkCh = ubx_get_U1(msg, &ind);
        ubx_get_U1(msg, &ind);
        uint32_t flags = ubx_get_X4(msg, &ind);
        cfg.blocks[i].en = flags & 1;
        cfg.blocks[i].flags = flags >> 16 & 0xFF;
    }

    emit rxCfgGnss(cfg);
}

void Ublox::ubx_decode_mon_ver(uint8_t *msg, int len)
{
    QString sw, hw;
    QStringList extensions;

    int ind = 0;
    sw = QString::fromLocal8Bit((const char*)msg + ind);
    ind += 30;
    hw = QString::fromLocal8Bit((const char*)msg + ind);
    ind += 10;

    while (ind < len) {
        extensions.append(QString::fromLocal8Bit((const char*)msg + ind));
        ind += 30;
    }

    emit rxMonVer(sw, hw, extensions);
}

void Ublox::ubx_decode_esf_meas(uint8_t *msg, int len)
{
    (void)len;

    ubx_esf_meas meas;
    int ind = 0;

    meas.time_tag = ubx_get_U4(msg, &ind);
    uint16_t flags = ubx_get_X2(msg, &ind);
    meas.time_mark_sent =       (flags & 0b0000000000000011);
    meas.time_mark_edge =       (flags & 0b0000000000000100);
    meas.calib_t_tag_valid =    (flags & 0b0000000000001000);
    meas.num_meas =             len - 8 - meas.calib_t_tag_valid * 4; //flags & 0b1111100000000000;
    meas.id = ubx_get_U2(msg, &ind);

    for (int i = 0; i < meas.num_meas && i < MAX_ESF_NUM_MEAS; i++) {
        uint32_t data = ubx_get_X4(msg, &ind);
        meas.data_field[i] = (data & 0b00000000111111111111111111111111);
        meas.data_type[i] =  (ubx_esf_datatype_enum)((data & 0b00111111000000000000000000000000) >> 24);
    }
    if (meas.num_meas > MAX_ESF_NUM_MEAS)
        qDebug() << "Warning: dropped" << MAX_ESF_NUM_MEAS-meas.num_meas << "measurements while decoding UBX-EAF-MEAS.";

    if (meas.calib_t_tag_valid)
        meas.calib_t_tag = ubx_get_U4(msg, &ind);

    emit rxEsfMeas(meas);
}

void Ublox::ubx_decode_esf_status(uint8_t *msg, int len)
{
    (void)len;

    static ubx_esf_status status;
    int ind = 0;
    uint8_t flags;

    status.i_tow       = ubx_get_U4(msg, &ind); // 0
    status.version     = ubx_get_U1(msg, &ind); // 4
    ind                += 7;                    // 5-11 reserved
    status.fusion_mode = ubx_get_U1(msg, &ind); // 12
    ind                += 2;                    // 13-14
    status.num_sens    = ubx_get_U1(msg, &ind); // 15

    for (int i = 0;i < status.num_sens;i++) {
        flags                   = ubx_get_X1(msg, &ind); // 16 + (num_sens*4)
        status.sensors[i].type  = (flags >> 0) & 63;
        status.sensors[i].used  = (flags >> 6) & 1;
        status.sensors[i].ready = (flags >> 7) & 1;

        flags                          = ubx_get_X1(msg, &ind); // 17 + (num_sens*4)
        status.sensors[i].calib_status = (flags >> 0) & 3;
        status.sensors[i].time_status  = (flags >> 2) & 3;

        status.sensors[i].freq = ubx_get_U1(msg, &ind); // 18

        flags                          = ubx_get_X1(msg, &ind); // 19 + (num_sens*4)
        status.sensors[i].bad_meas     = (flags >> 0) & 1;
        status.sensors[i].bad_t_tag    = (flags >> 1) & 1;
        status.sensors[i].missing_meas = (flags >> 2) & 1;
        status.sensors[i].noisy_meas   = (flags >> 3) & 1;
    }

    emit rxEsfStatus(status);
}

void Ublox::ubx_decode_esf_alg(uint8_t *msg, int len)
{
    (void)len;

    static ubx_esf_alg alg;
    int ind = 0;
    uint8_t flags;

    alg.i_tow       = ubx_get_U4(msg, &ind); // 0
    alg.version     = ubx_get_U1(msg, &ind); // 4

    flags           = ubx_get_U1(msg, &ind); // 5
    alg.autoMntAlgOn= (flags >> 0) & 1;
    alg.status      = (flags >> 1) & 7;

    flags           = ubx_get_U1(msg, &ind); // 6
    alg.tiltAlgError= (flags >> 0) & 1;
    alg.yawAlgError = (flags >> 1) & 1;
    alg.angleError  = (flags >> 2) & 1;

    ind++; // 7 reserved
    alg.yaw         = ubx_get_U4(msg, &ind) * 1e-2; // 8
    alg.pitch       = ubx_get_I2(msg, &ind) * 1e-2; // 12
    alg.roll        = ubx_get_I2(msg, &ind) * 1e-2; // 14

    emit rxEsfAlg(alg);
}

void Ublox::ubx_decode_upd_sos(uint8_t *msg, int len)
{
    (void)len;

    static ubx_upd_sos sos;
    int ind = 0;

    sos.cmd = ubx_get_U1(msg, &ind);
    ind += 3; // Reserved 1-3
    sos.response = ubx_get_U1(msg, &ind);
    ind += 3; // Reserved 5-7

    emit rxUpdSos(sos);
}

void Ublox::ubx_decode_cfg_valget(uint8_t *msg, int len)
{
    static ubx_cfg_valget valget;
    int ind = 0;

    valget.version      = ubx_get_U1(msg, &ind);
    valget.layer        = ubx_get_U1(msg, &ind);
    valget.position     = ubx_get_U2(msg, &ind);

    for (int i=0; i<len; i++) {
        valget.key[i]          = ubx_get_X4(msg, &ind);
        valget.cfgData[i]      = ubx_get_U1(msg, &ind);
    }

    emit rxCfgValget(valget);
}
