#ifndef DRIVEUI_H
#define DRIVEUI_H

#include <QWidget>
#include <QKeyEvent>
#include <QTimer>
#include "communication/vehicleconnections/vehicleconnection.h"

namespace Ui {
class DriveUI;
}

class DriveUI : public QWidget
{
    Q_OBJECT

public:
    explicit DriveUI(QWidget *parent = nullptr);
    ~DriveUI();

    void setCurrentVehicleConnection(const QSharedPointer<VehicleConnection> &currentVehicleConnection);
    void gotRouteForAutopilot(const QList<PosPoint>& route);

private slots:
    void on_apRestartButton_clicked();

    void on_apStartButton_clicked();

    void on_apPauseButton_clicked();

    void on_apStopButton_clicked();

    void on_spinBox_valueChanged(int arg1);

private:
    Ui::DriveUI *ui;

    QSharedPointer<VehicleConnection> mCurrentVehicleConnection;
    struct {bool upPressed, downPressed, leftPressed, rightPressed;} mArrowKeyStates;
    struct {double throttle, steering;} mKeyControlState;
    QTimer mKeyControlTimer;

    double getMaxSignedStepFromValueTowardsGoal(double value, double goal, double maxStepSize);

    // QWidget interface
protected:
    virtual void keyPressEvent(QKeyEvent *event) override;
    virtual void keyReleaseEvent(QKeyEvent *event) override;
};

#endif // DRIVEUI_H
