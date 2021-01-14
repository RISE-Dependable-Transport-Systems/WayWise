#include "ublox_basestation.h"
#include "coordinate_transforms.h"
#include <QDebug>

const UbloxBasestation::BasestationConfig UbloxBasestation::defaultConfig;

UbloxBasestation::UbloxBasestation(QObject *parent) : QObject(parent)
{
    connect(&mUblox, &Ublox::rxNavPvt, this, &UbloxBasestation::rxNavPvt);
    connect(&mUblox, &Ublox::rxNavSat, this, &UbloxBasestation::rxNavSat);
    connect(&mUblox, &Ublox::rxSvin, this, &UbloxBasestation::rxSvin);
    connect(&mUblox, &Ublox::rtcmRx, this, &UbloxBasestation::rtcmRx);

//    connect(&mUblox, &Ublox::rxMonVer, [this]
//            (QString sw, QString hw, QStringList extensions) {
//        QString txt = "SW: " + sw + "\nHW: " +
//                hw + "\nExtensions:\n";

//        for (QString s: extensions) {
//            txt += s + "\n";
//        }

//        QMessageBox::information(this, "Ublox Version",
//                                 txt.mid(0, txt.size() - 1));
//    });

//    connect(&mUblox, &Ublox::rxCfgGnss, [this](ubx_cfg_gnss cfg) {
//        QString str = QString("TrkChHw   : %1\n"
//                              "TrkChUse  : %2\n"
//                              "Blocks    : %3\n\n").
//                arg(cfg.num_ch_hw).arg(cfg.num_ch_use).arg(cfg.num_blocks);

//        for (int i = 0;i < cfg.num_blocks;i++) {
//            str += QString("GNSS ID: %1, Enabled: %2\n"
//                           "MinTrkCh  : %3\n"
//                           "MaxTrkCh  : %4\n"
//                           "Flags     : %5").
//                    arg(cfg.blocks[i].gnss_id).
//                    arg(cfg.blocks[i].en).
//                    arg(cfg.blocks[i].minTrkCh).
//                    arg(cfg.blocks[i].maxTrkCh).
//                    arg(cfg.blocks[i].flags);

//            if (i != cfg.num_blocks - 1) {
//                str += "\n\n";
//            }
//        }

//        QMessageBox::information(this, "Cfg GNSS", str);
    //    });
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


bool UbloxBasestation::configureUblox(const BasestationConfig& basestationConfig)
{
    if (!mUblox.isSerialConnected()) {
        qDebug() << "configureUblox: not possible, because serial is not connected";
        return false;
    }

    // Serial port baudrate (buffering will create problems if too low)
    // TODO: still necessary?
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

void UbloxBasestation::rxNavPvt(ubx_nav_pvt pvt)
{
    // TODO: use pvt information (GNSS signal type, etc.)
    emit currentPosition(pvt.lat, pvt.lon, pvt.height);
}

void UbloxBasestation::rxNavSat(ubx_nav_sat sat)
{
    int satsGps = 0;
    int satsGlo = 0;
    int satsGal = 0;
    int satsBds = 0;

    int visibleGps = 0;
    int visibleGlo = 0;
    int visibleGal = 0;
    int visibleBds = 0;

    for (int i = 0;i < sat.num_sv;i++) {
        ubx_nav_sat_info s = sat.sats[i];

        if (s.gnss_id == 0) {
            visibleGps++;
        } else if (s.gnss_id == 2) {
            visibleGal++;
        } else if (s.gnss_id == 3) {
            visibleBds++;
        } else if (s.gnss_id == 6) {
            visibleGlo++;
        }

        if (s.used && s.quality >= 4) {
            if (s.gnss_id == 0) {
                satsGps++;
            } else if (s.gnss_id == 2) {
                satsGal++;
            } else if (s.gnss_id == 3) {
                satsBds++;
            } else if (s.gnss_id == 6) {
                satsGlo++;
            }
        }
    }

    QString rtcmMsgs;
    QMapIterator<int, int> i(mRtcmUbx);
    while (i.hasNext()) {
        i.next();
        if (!rtcmMsgs.isEmpty()) {
            rtcmMsgs += ", ";
        }

        rtcmMsgs += QString("%1:%2").
                arg(i.key()).arg(i.value());
    }

    QString txt = QString("         Visible   Used\n"
                          "GPS:     %1        %5\n"
                          "GLONASS: %2        %6\n"
                          "Galileo: %3        %7\n"
                          "BeiDou:  %4        %8\n"
                          "Total:   %9        %10\n\n"
                          "RTCM Sent:\n"
                          + rtcmMsgs).
            arg(visibleGps, -2).arg(visibleGlo, -2).
            arg(visibleGal, -2).arg(visibleBds, -2).
            arg(satsGps, -2).arg(satsGlo, -2).
            arg(satsGal, -2).arg(satsBds, -2).
            arg(visibleGps + visibleGlo + visibleGal + visibleBds, -2).
            arg(satsGps + satsGlo + satsGal + satsBds, -2);
    //qDebug() << txt;
}

void UbloxBasestation::rxSvin(ubx_nav_svin svin)
{
    double llh[3];
    coordinateTransforms::xyzToLlh(svin.meanX, svin.meanY, svin.meanZ,
                      &llh[0], &llh[1], &llh[2]);

    QString txt = QString(
                "Lat:          %1\n"
                "Lon:          %2\n"
                "Height:       %3\n"
                "Observarions: %4\n"
                "P ACC:        %5 m\n"
                "Duration:     %6 s\n"
                "Valid:        %7\n"
                "Active:       %8").
            arg(llh[0], 0, 'f', 8).
            arg(llh[1], 0, 'f', 8).
            arg(llh[2]).
            arg(svin.obs).
            arg(svin.meanAcc).
            arg(svin.dur).
            arg(svin.valid).
            arg(svin.active);

    //qDebug() << txt;
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

    mRtcmUbx[type]++;
    emit rtcmData(data, type);
}
