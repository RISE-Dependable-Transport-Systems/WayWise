#include "ublox_basestation.h"
#include <QDebug>

const UbloxBasestation::BasestationConfig UbloxBasestation::defaultConfig;

UbloxBasestation::UbloxBasestation(QObject *parent) : QObject(parent)
{
    connect(&mUblox, &Ublox::rxNavPvt, [this](const ubx_nav_pvt& pvt) {
        emit currentPosition({pvt.lat, pvt.lon, pvt.height});
    });

    connect(&mUblox, &Ublox::rxNavSat, [this](const ubx_nav_sat& sat) {
        emit rxNavSat(sat);
    });

    connect(&mUblox, &Ublox::rxSvin, [this](ubx_nav_svin svin){
        emit rxSvin(svin);
    });

    connect(&mUblox, &Ublox::rxCfgGnss, [this](const ubx_cfg_gnss &gnss){
        emit rxCfgGnss(gnss);
    });

    connect(&mUblox, &Ublox::rxMonVer, [this](const QString &sw, const QString &hw, const QStringList &extensions){
        emit rxMonVer(sw, hw, extensions);
    });

    connect(&mUblox, &Ublox::rtcmRx, this, &UbloxBasestation::rtcmRx);
}

bool UbloxBasestation::connectSerial(const QSerialPortInfo& serialPortInfo, const BasestationConfig basestationConfig)
{
    if (mUblox.connectSerial(serialPortInfo))
        if (configureUblox(basestationConfig))
            return true;
        else {
            mUblox.disconnectSerial();
            return false;
        }
    else
        return false;
}

bool UbloxBasestation::disconnectSerial()
{
    mUblox.disconnectSerial();
    return true;
}


bool UbloxBasestation::configureUblox(const BasestationConfig& basestationConfig)
{
    if (!mUblox.isSerialConnected()) {
        qDebug() << "configureUblox: not possible, because serial is not connected";
        return false;
    }

    ubx_cfg_prt_uart uart;
    uart.baudrate = basestationConfig.baudrate;
    uart.in_ubx = true;
    uart.in_nmea = true;
    uart.in_rtcm2 = false;
    uart.in_rtcm3 = true;
    uart.out_ubx = true;
    uart.out_nmea = true;
    uart.out_rtcm3 = true;
/*    qDebug() << "CfgPrtUart:" << */mUblox.ubxCfgPrtUart(&uart);

    mUblox.ubxCfgRate(basestationConfig.measurementRate, basestationConfig.navSolutionRate, 0);

    // Configure messages sent from ublox
    mUblox.ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_RAWX, 0);
    mUblox.ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_SFRBX, 0);
    mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SOL, 0);
    mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SAT, 1);
    mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_PVT, 1);

    switch (basestationConfig.mode) {
    case BasestationMode::MovingBase:
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, 0);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1074, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, 0);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1084, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, 0);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1094, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1097, 0);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1124, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1127, 0);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1230, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_0, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_1, 0);

        mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SVIN, 0);
        break;
    case BasestationMode::Fixed:
    case BasestationMode::SurveyIn:
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1074, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1084, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1094, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1097, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1124, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1127, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1230, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_0, 1);
        mUblox.ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_1, 0);

        mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SVIN, 1);
        break;
    }

    // Enable GNSS systems
    unsigned char buffer[512];
    int ind = 0;
    mUblox.ubloxCfgAppendEnableGps(buffer, &ind, true, true, true);
    mUblox.ubloxCfgAppendEnableGal(buffer, &ind, true, true, true);
    mUblox.ubloxCfgAppendEnableBds(buffer, &ind, true, true, true);
    mUblox.ubloxCfgAppendEnableGlo(buffer, &ind, true, true, true);
    mUblox.ubloxCfgValset(buffer, ind, true, true, true);

    // Set BasestationMode
    ubx_cfg_tmode3 cfg_mode;
    memset(&cfg_mode, 0, sizeof(cfg_mode));
    switch (basestationConfig.mode) {
    case BasestationMode::MovingBase:
        cfg_mode.mode = 0;
        break;
    case BasestationMode::Fixed:
        cfg_mode.mode = 2;
        cfg_mode.lla = true;
        cfg_mode.ecefx_lat = basestationConfig.fixedRefLat;
        cfg_mode.ecefy_lon = basestationConfig.fixedRefLon;
        cfg_mode.ecefz_alt = basestationConfig.fixedRefHeight;
        break;
    case BasestationMode::SurveyIn:
        cfg_mode.mode = 1;
        cfg_mode.fixed_pos_acc = 5.0;
        cfg_mode.svin_min_dur = basestationConfig.surveyInMinDuration;
        cfg_mode.svin_acc_limit = basestationConfig.surveyInMinAcc;
        break;
    }
    mUblox.ubxCfgTmode3(&cfg_mode);

    // Stationary dynamic model
    ubx_cfg_nav5 nav5;
    memset(&nav5, 0, sizeof(ubx_cfg_nav5));
    nav5.apply_dyn = true;
    nav5.apply_dyn = true;
    nav5.dyn_model = 2;
    mUblox.ubxCfgNav5(&nav5);

    // Time pulse configuration
    ubx_cfg_tp5 tp5;
    memset(&tp5, 0, sizeof(ubx_cfg_tp5));
    tp5.active = true;
    tp5.polarity = true;
    tp5.alignToTow = true;
    tp5.lockGnssFreq = true;
    tp5.lockedOtherSet = true;
    tp5.syncMode = false;
    tp5.isFreq = false;
    tp5.isLength = true;
    tp5.freq_period = 1000000;
    tp5.pulse_len_ratio = 0;
    tp5.freq_period_lock = 1000000;
    tp5.pulse_len_ratio_lock = 100000;
    tp5.gridUtcGnss = 0;
    tp5.user_config_delay = 0;
    tp5.rf_group_delay = 0;
    tp5.ant_cable_delay = 50;
    mUblox.ubloxCfgTp5(&tp5);

    // Save everything
    ubx_cfg_cfg cfg;
    memset(&cfg, 0, sizeof(ubx_cfg_cfg));
    cfg.save_io_port = true;
    cfg.save_msg_conf = true;
    cfg.save_inf_msg = true;
    cfg.save_nav_conf = true;
    cfg.save_rxm_conf = true;
    cfg.save_sen_conf = true;
    cfg.save_rinv_conf = true;
    cfg.save_ant_conf = true;
    cfg.save_log_conf = true;
    cfg.save_fts_conf = true;
    cfg.dev_bbr = true;
    cfg.dev_flash = true;
    mUblox.ubloxCfgCfg(&cfg);

    return true;
}

void UbloxBasestation::rtcmRx(const QByteArray& data, const int &type)
{
    // Send base station position every sendRtcmRefDelayMultiplier cycles to save bandwidth.
    static int basePosCnt = 0;
    if (type == 1006 || type == 1005) {
        basePosCnt++;
        if (basePosCnt < sendRtcmRefDelayMultiplier)
            return;
        basePosCnt = 0;
    }

    emit rtcmData(data, type);
}

void UbloxBasestation::pollMonVer()
{
    mUblox.ubxPoll(UBX_CLASS_MON, UBX_MON_VER);
}

void UbloxBasestation::pollCfgGNSS()
{
    mUblox.ubxPoll(UBX_CLASS_CFG, UBX_CFG_GNSS);
}
