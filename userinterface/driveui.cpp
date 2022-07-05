#include "driveui.h"
#include "ui_driveui.h"

DriveUI::DriveUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DriveUI)
{
    ui->setupUi(this);
}

DriveUI::~DriveUI()
{
    delete ui;
}
