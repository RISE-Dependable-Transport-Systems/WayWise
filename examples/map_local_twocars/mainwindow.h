#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "vehicles/carstate.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    const int mUpdateVehicleStatePeriod_ms = 25;
    QTimer mUpdateVehicleStateTimer;
    QSharedPointer<CarState> car1;
    QSharedPointer<CarState> car2;
};
#endif // MAINWINDOW_H
