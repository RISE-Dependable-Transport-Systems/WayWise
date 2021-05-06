/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

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

#ifndef COMMANDS_H
#define COMMANDS_H

#include <QObject>
#include <QTimer>
#include "../../vbytearray.h"
#include "datatypes.h"
#include "packet.h"
#include "configparams.h"

namespace VESC {

class Commands : public QObject
{
    Q_OBJECT
public:
    explicit Commands(QObject *parent = nullptr);

    void setLimitedMode(bool is_limited);
    Q_INVOKABLE bool isLimitedMode();
    Q_INVOKABLE bool setSendCan(bool sendCan, int id = -1);
    Q_INVOKABLE bool getSendCan();
    Q_INVOKABLE void setCanSendId(unsigned int id);
    Q_INVOKABLE int getCanSendId();
    void setMcConfig(ConfigParams *mcConfig);
    void setAppConfig(ConfigParams *appConfig);
    void checkMcConfig();
    Q_INVOKABLE void emitEmptyValues();
    Q_INVOKABLE void emitEmptySetupValues();

    Q_INVOKABLE bool getLimitedSupportsFwdAllCan() const;
    void setLimitedSupportsFwdAllCan(bool limitedSupportsFwdAllCan);

    Q_INVOKABLE bool getLimitedSupportsEraseBootloader() const;
    void setLimitedSupportsEraseBootloader(bool limitedSupportsEraseBootloader);

    Q_INVOKABLE QVector<int> getLimitedCompatibilityCommands() const;
    void setLimitedCompatibilityCommands(QVector<int> compatibilityCommands);

    Q_INVOKABLE static QString faultToStr(mc_fault_code fault);

    Q_INVOKABLE QByteArray bmReadMemWait(uint32_t addr, quint16 size, int timeoutMs = 3000);
    Q_INVOKABLE int bmWriteMemWait(uint32_t addr, QByteArray data, int timeoutMs = 3000);

    Q_INVOKABLE void setOdometer(unsigned odometer_meters);

signals:
    void dataToSend(QByteArray &data);

    void fwVersionReceived(FW_RX_PARAMS params);
    void eraseNewAppResReceived(bool ok);
    void eraseBootloaderResReceived(bool ok);
    void writeNewAppDataResReceived(bool ok, bool hasOffset, quint32 offset);
    void ackReceived(QString ackType);
    void valuesReceived(MC_VALUES values, unsigned int mask);
    void printReceived(QString str);
    void samplesReceived(QByteArray bytes);
    void rotorPosReceived(double pos);
    void experimentSamplesReceived(QVector<double> samples);
    void bldcDetectReceived(bldc_detect param);
    void decodedPpmReceived(double value, double last_len);
    void decodedAdcReceived(double value, double voltage, double value2, double voltage2);
    void decodedChukReceived(double value);
    void decodedBalanceReceived(BALANCE_VALUES values);
    void motorRLReceived(double r, double l);
    void motorLinkageReceived(double flux_linkage);
    void encoderParamReceived(double offset, double ratio, bool inverted);
    void customAppDataReceived(QByteArray data);
    void focHallTableReceived(QVector<int> hall_table, int res);
    void nrfPairingRes(int res);
    void mcConfigCheckResult(QStringList paramsNotSet);
    void gpdBufferNotifyReceived();
    void gpdBufferSizeLeftReceived(int sizeLeft);
    void valuesSetupReceived(SETUP_VALUES values, unsigned int mask);
    void detectAllFocReceived(int result);
    void pingCanRx(QVector<int> devs, bool isTimeout);
    void valuesImuReceived(IMU_VALUES values, unsigned int mask);
    void imuCalibrationReceived(QVector<double> cal);
    void bmConnRes(int res);
    void bmEraseFlashAllRes(int res);
    void bmWriteFlashRes(int res);
    void bmRebootRes(int res);
    void bmMapPinsDefaultRes(bool ok);
    void bmMapPinsNrf5xRes(bool ok);
    void plotInitReceived(QString xLabel, QString yLabel);
    void plotDataReceived(double x, double y);
    void plotAddGraphReceived(QString name);
    void plotSetGraphReceived(int graph);
    void bmReadMemRes(int res, QByteArray data);
    void deserializeConfigFailed(bool isMc, bool isApp);
    void canFrameRx(QByteArray data, quint32 id, bool isExtended);
    void bmsValuesRx(BMS_VALUES val);
    void customConfigChunkRx(int confInd, int lenConf, int ofsConf, QByteArray data);
    void customConfigRx(int confInd, QByteArray data);

public slots:
    void processPacket(QByteArray data);

    void getFwVersion();
    void eraseNewApp(bool fwdCan, quint32 fwSize, HW_TYPE hwType, QString hwName);
    void eraseBootloader(bool fwdCan, HW_TYPE hwType, QString hwName);
    void writeNewAppData(QByteArray data, quint32 offset, bool fwdCan, HW_TYPE hwType, QString hwName);
    void writeNewAppDataLzo(QByteArray data, quint32 offset, quint16 decompressedLen, bool fwdCan);
    void jumpToBootloader(bool fwdCan, HW_TYPE hwType, QString hwName);
    void getValues();
    void sendTerminalCmd(QString cmd);
    void sendTerminalCmdSync(QString cmd);
    void setDutyCycle(double dutyCycle);
    void setCurrent(double current);
    void setCurrentBrake(double current);
    void setRpm(int rpm);
    void setPos(double pos);
    void setHandbrake(double current);
    void setDetect(disp_pos_mode mode);
    void samplePrint(debug_sampling_mode mode, int sample_len, int dec);
    void getMcconf();
    void getMcconfDefault();
    void setMcconf(bool check = true);
    void getAppConf();
    void getAppConfDefault();
    void setAppConf();
    void detectMotorParam(double current, double min_rpm, double low_duty);
    void reboot();
    void sendAlive();
    void getDecodedPpm();
    void getDecodedAdc();
    void getDecodedChuk();
    void getDecodedBalance();
    void setServoPos(double pos);
    void measureRL();
    void measureLinkage(double current, double min_rpm, double low_duty, double resistance);
    void measureEncoder(double current);
    void measureHallFoc(double current);
    void sendCustomAppData(QByteArray data);
    void sendCustomAppData(unsigned char *data, unsigned int len);
    void setChukData(chuck_data &data);
    void pairNrf(int ms);
    void gpdSetFsw(float fsw);
    void getGpdBufferSizeLeft();
    void gpdFillBuffer(QVector<float> samples);
    void gpdOutputSample(float sample);
    void gpdSetMode(gpd_output_mode mode);
    void gpdFillBufferInt8(QVector<qint8> samples);
    void gpdFillBufferInt16(QVector<qint16> samples);
    void gpdSetBufferIntScale(float scale);
    void getValuesSetup();
    void setMcconfTemp(const MCCONF_TEMP &conf, bool is_setup, bool store,
                       bool forward_can, bool divide_by_controllers, bool ack);
    void getValuesSelective(unsigned int mask);
    void getValuesSetupSelective(unsigned int mask);
    void measureLinkageOpenloop(double current, double erpm_per_sec, double low_duty,
                                double resistance, double inductanec);
    void detectAllFoc(bool detect_can, double max_power_loss, double min_current_in,
                      double max_current_in, double openloop_rpm, double sl_erpm);
    void pingCan();
    void disableAppOutput(int time_ms, bool fwdCan);
    void getImuData(unsigned int mask);
    void getImuCalibration(double yaw);
    void bmConnect();
    void bmEraseFlashAll();
    void bmWriteFlash(uint32_t addr, QByteArray data);
    void bmWriteFlashLzo(uint32_t addr, quint16 decompressedLen, QByteArray data);
    void bmReboot();
    void bmDisconnect();
    void bmMapPinsDefault();
    void bmMapPinsNrf5x();
    void bmReadMem(uint32_t addr, quint16 size);
    void setCurrentRel(double current);
    void forwardCanFrame(QByteArray data, quint32 id, bool isExtended);
    void setBatteryCut(double start, double end, bool store, bool fwdCan);

    void bmsGetValues();
    void bmsSetChargeAllowed(bool allowed);
    void bmsSetBalanceOverride(uint8_t cell, uint8_t override);
    void bmsResetCounters(bool ah, bool wh);
    void bmsForceBalance(bool bal_en);
    void bmsZeroCurrentOffset();

    void customConfigGetChunk(int confInd, int len, int offset);
    void customConfigGet(int confInd, bool isDefault);
    void customConfigSet(int confInd, QByteArray confData);

private slots:
    void timerSlot();

private:
    void emitData(QByteArray data);

    QTimer *mTimer;
    bool mSendCan;
    int mCanId;
    bool mIsLimitedMode;
    bool mLimitedSupportsFwdAllCan;
    bool mLimitedSupportsEraseBootloader;
    QVector<int> mCompatibilityCommands; // int to be QML-compatible

    ConfigParams *mMcConfig;
    ConfigParams *mAppConfig;
    ConfigParams mMcConfigLast;
    bool mCheckNextMcConfig;

    int mTimeoutCount;
    int mTimeoutFwVer;
    int mTimeoutMcconf;
    int mTimeoutAppconf;
    int mTimeoutValues;
    int mTimeoutValuesSetup;
    int mTimeoutImuData;
    int mTimeoutDecPpm;
    int mTimeoutDecAdc;
    int mTimeoutDecChuk;
    int mTimeoutDecBalance;
    int mTimeoutPingCan;
    int mTimeoutCustomConf;
    int mTimeoutBmsVal;

};

}

#endif // COMMANDS_H
