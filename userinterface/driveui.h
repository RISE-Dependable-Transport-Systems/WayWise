#ifndef DRIVEUI_H
#define DRIVEUI_H

#include <QWidget>

namespace Ui {
class DriveUI;
}

class DriveUI : public QWidget
{
    Q_OBJECT

public:
    explicit DriveUI(QWidget *parent = nullptr);
    ~DriveUI();

private:
    Ui::DriveUI *ui;
};

#endif // DRIVEUI_H
