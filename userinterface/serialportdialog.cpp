/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "serialportdialog.h"
#include "ui_serialportdialog.h"
#include <QDebug>

Q_DECLARE_METATYPE(QSerialPortInfo)

SerialPortDialog::SerialPortDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SerialPortDialog)
{
    ui->setupUi(this);
    ui->addSerialConnectionButton->setIcon(QApplication::style()->standardIcon(QStyle::SP_DialogOkButton));
    ui->cancelButton->setIcon(QApplication::style()->standardIcon(QStyle::SP_DialogCancelButton));

    for (const qint32 &rate : QSerialPortInfo::standardBaudRates())
        ui->baudrateCombo->addItem(QString::number(rate));
    ui->baudrateCombo->setCurrentText("57600");
}

SerialPortDialog::~SerialPortDialog()
{
    delete ui;
}

void SerialPortDialog::on_addSerialConnectionButton_clicked()
{
    emit selectedSerialPort(ui->serialPortList->currentItem()->data(Qt::UserRole).value<QSerialPortInfo>(), ui->baudrateCombo->currentText().toInt());
    hide();
}

void SerialPortDialog::on_cancelButton_clicked()
{
    hide();
}


void SerialPortDialog::showEvent(QShowEvent *event)
{
    Q_UNUSED(event)

    ui->serialPortList->clear();
    if (!QSerialPortInfo::availablePorts().isEmpty()) {
        for (const auto &serialPortInfo : QSerialPortInfo::availablePorts()) {
            QString listItemString = serialPortInfo.systemLocation() + " (" + serialPortInfo.manufacturer() + " " + serialPortInfo.description() + ")";
            QVariant listItemData = QVariant::fromValue<QSerialPortInfo>(serialPortInfo);
            QListWidgetItem *listItem = new QListWidgetItem(listItemString, ui->serialPortList);
            listItem->setData(Qt::UserRole, listItemData);
        }
        ui->addSerialConnectionButton->setEnabled(true);
    } else {
        new QListWidgetItem(tr("No serialports found."), ui->serialPortList);
        ui->addSerialConnectionButton->setEnabled(false);
    }
}
