#ifndef UBLOXBASESTATIONUI_H
#define UBLOXBASESTATIONUI_H

#include <QWidget>
#include <QSharedPointer>
#include "sensors/gnss/ublox_basestation.h"

namespace Ui {
class UbloxBasestationUI;
}

class UbloxBasestationUI : public QWidget
{
    Q_OBJECT

signals:
    void rtcmData(const QByteArray& data, const int& type);
    void currentPosition(const llh_t &llh);

public:
    explicit UbloxBasestationUI(QWidget *parent = nullptr);
    bool isBasestationRunning();
    ~UbloxBasestationUI();

private slots:
    void on_startBasestationButton_clicked();

    void on_stopBasestationButton_clicked();

    void on_gnssInfoButton_clicked();

    void on_ubloxVersionButton_clicked();

    void on_surveyInRadioButton_toggled(bool checked);

    void on_fixedPositionRadioButton_toggled(bool checked);

private:
    void updateBasestationConfigFromUI();

    Ui::UbloxBasestationUI *ui;
    QSharedPointer<UbloxBasestation> mUbloxBasestation;
    UbloxBasestation::BasestationConfig mUbloxBasestationConfig;
    QMap<int, int> mRtcmSentMap;
};

#endif // UBLOXBASESTATIONUI_H
