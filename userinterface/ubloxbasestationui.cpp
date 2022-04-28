#include "ubloxbasestationui.h"
#include "ui_ubloxbasestationui.h"
#include <QDebug>
#include <QMessageBox>

UbloxBasestationUI::UbloxBasestationUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::UbloxBasestationUI)
{
    ui->setupUi(this);
    mUbloxBasestation = QSharedPointer<UbloxBasestation>::create();

    connect(mUbloxBasestation.get(), &UbloxBasestation::rxNavSat, [this](const ubx_nav_sat &sat) {
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
        QMapIterator<int, int> i(mRtcmSentMap);
        while (i.hasNext()) {
            i.next();
            if (!rtcmMsgs.isEmpty()) {
                rtcmMsgs += ", ";
            }

            rtcmMsgs += QString("%1:%2").
                    arg(i.key()).arg(i.value());
        }

        QString txt = QString("\tVisible\tUsed\n"
                              "GPS:\t%1\t%5\n"
                              "GLONASS:\t%2\t%6\n"
                              "Galileo:\t%3\t%7\n"
                              "BeiDou:\t%4\t%8\n"
                              "Total:\t%9\t%10\n\n"
                              "RTCM Sent:\n"
                              + rtcmMsgs).
                arg(visibleGps, 2).arg(visibleGlo, 2).
                arg(visibleGal, 2).arg(visibleBds, 2).
                arg(satsGps, 2).arg(satsGlo, 2).
                arg(satsGal, 2).arg(satsBds, 2).
                arg(visibleGps + visibleGlo + visibleGal + visibleBds, 2).
                arg(satsGps + satsGlo + satsGal + satsBds, 2);
        QTextDocument txtDoc(txt);
        ui->sattelitesStatusTextEdit->setPlainText(txt);
    });

    connect(mUbloxBasestation.get(), &UbloxBasestation::rxSvin, [this](const ubx_nav_svin &svin) {
        llh_t llh = coordinateTransforms::xyzToLlh({svin.meanX, svin.meanY, svin.meanZ});
        QString txt = QString(
                    "Lat:\t%1\n"
                    "Lon:\t%2\n"
                    "Height:\t%3\n\n"
                    "Observarions: %4\n"
                    "P ACC:\t%5 m\n"
                    "Duration:\t%6 s\n"
                    "Valid:\t%7\n"
                    "Active:\t%8").
                arg(llh.latitude, 0, 'f', 8).
                arg(llh.longitude, 0, 'f', 8).
                arg(llh.height).
                arg(svin.obs).
                arg(svin.meanAcc).
                arg(svin.dur).
                arg(svin.valid).
                arg(svin.active);
        ui->surveyInStatusTextEdit->setPlainText(txt);
    });

    connect(mUbloxBasestation.get(), &UbloxBasestation::rtcmData, [this](const QByteArray& data, const int& type) {
        mRtcmSentMap[type]++;
        emit rtcmData(data, type);
    });

    connect(mUbloxBasestation.get(), &UbloxBasestation::currentPosition, [this](const llh_t &llh) {
       emit currentPosition(llh);
    });

    connect(mUbloxBasestation.get(), &UbloxBasestation::rxMonVer, [this]
            (QString sw, QString hw, QStringList extensions) {
        QString txt = "SW: " + sw + "\nHW: " +
                hw + "\nExtensions:\n";

        for (QString s: extensions) {
            txt += s + "\n";
        }

        QMessageBox::information(this, "u-blox Version",
                                 txt.mid(0, txt.size() - 1));
    });

    connect(mUbloxBasestation.get(), &UbloxBasestation::rxCfgGnss, [this](ubx_cfg_gnss cfg) {
        QString str = QString("TrkChHw\t: %1\n"
                              "TrkChUse: %2\n"
                              "Blocks\t: %3\n\n").
                arg(cfg.num_ch_hw).arg(cfg.num_ch_use).arg(cfg.num_blocks);

        for (int i = 0;i < cfg.num_blocks;i++) {
            str += QString("GNSS ID: %1, Enabled: %2\n"
                           "MinTrkCh\t: %3\n"
                           "MaxTrkCh\t: %4\n"
                           "Flags\t\t: %5").
                    arg(cfg.blocks[i].gnss_id).
                    arg(cfg.blocks[i].en).
                    arg(cfg.blocks[i].minTrkCh).
                    arg(cfg.blocks[i].maxTrkCh).
                    arg(cfg.blocks[i].flags);

            if (i != cfg.num_blocks - 1) {
                str += "\n\n";
            }
        }

        QMessageBox::information(this, "u-blox GNSS Configuration", str);
    });
}

bool UbloxBasestationUI::isBasestationRunning()
{
    return mUbloxBasestation->isSerialConnected();
}

UbloxBasestationUI::~UbloxBasestationUI()
{
    delete ui;
}

void UbloxBasestationUI::updateBasestationConfigFromUI()
{
    if (ui->fixedPositionRadioButton->isChecked())
        mUbloxBasestationConfig.mode = UbloxBasestation::BasestationMode::Fixed;
    else if (ui->surveyInRadioButton->isChecked())
        mUbloxBasestationConfig.mode = UbloxBasestation::BasestationMode::SurveyIn;
    else if (ui->movingBaseRadioButton->isChecked())
        mUbloxBasestationConfig.mode = UbloxBasestation::BasestationMode::MovingBase;

    mUbloxBasestationConfig.fixedRefLon = ui->fixedPosLonSpinBox->value();
    mUbloxBasestationConfig.fixedRefLat = ui->fixedPosLatSpinBox->value();
    mUbloxBasestationConfig.fixedRefHeight = ui->fixedPosHeightSpinBox->value();

    mUbloxBasestationConfig.surveyInMinAcc = ui->surveyInMinAccSpinBox->value();
    mUbloxBasestationConfig.surveyInMinDuration = ui->surveyInMinDurSpinBox->value();

    mUbloxBasestationConfig.measurementRate = 1000/ui->measurementRateSpinBox->value();
    mUbloxBasestationConfig.navSolutionRate = ui->measurementsPerSolutionSpinBox->value();
}

void UbloxBasestationUI::on_startBasestationButton_clicked()
{
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &portInfo, ports) {
        if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
            updateBasestationConfigFromUI();
            mUbloxBasestation->connectSerial(portInfo, mUbloxBasestationConfig);
            qDebug() << "UbloxBasestation connected to:" << portInfo.systemLocation();
        }
    }

    if (mUbloxBasestation->isSerialConnected()) {
//        // Base position = ENU reference
//        connect(&mUbloxBasestation, &UbloxBasestation::currentPosition, ui->mapWidget, &MapWidget::setEnuRef);
//        connect(&mUbloxBasestation, &UbloxBasestation::rtcmData, mMavsdkStation.get(), &MavsdkStation::forwardRtcmData); // TODO: not fully implemented
    } else {
        QMessageBox::warning(this, "u-blox Basestation", "Unable to connect to u-blox serial interface.");
    }
}

void UbloxBasestationUI::on_stopBasestationButton_clicked()
{
    mUbloxBasestation->disconnectSerial();
}

void UbloxBasestationUI::on_gnssInfoButton_clicked()
{
    mUbloxBasestation->pollCfgGNSS();
}

void UbloxBasestationUI::on_ubloxVersionButton_clicked()
{
    mUbloxBasestation->pollMonVer();
}

void UbloxBasestationUI::on_surveyInRadioButton_toggled(bool checked)
{
    ui->surveyInMinAccSpinBox->setEnabled(checked);
    ui->surveyInMinDurSpinBox->setEnabled(checked);
}

void UbloxBasestationUI::on_fixedPositionRadioButton_toggled(bool checked)
{
    ui->fixedPosLonSpinBox->setEnabled(checked);
    ui->fixedPosLatSpinBox->setEnabled(checked);
    ui->fixedPosHeightSpinBox->setEnabled(checked);
}
