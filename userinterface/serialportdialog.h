/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#ifndef SERIALPORTDIALOG_H
#define SERIALPORTDIALOG_H

#include <QDialog>
#include <QSerialPort>
#include <QSerialPortInfo>

namespace Ui {
class SerialPortDialog;
}

class SerialPortDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SerialPortDialog(QWidget *parent = nullptr);
    ~SerialPortDialog();

signals:
    void selectedSerialPort(QSerialPortInfo serialPortInfo, qint32 baudrate);

private slots:
    void on_cancelButton_clicked();
    void on_addSerialConnectionButton_clicked();

private:
    Ui::SerialPortDialog *ui;

    // QWidget interface
protected:
    virtual void showEvent(QShowEvent *event) override;
};

#endif // SERIALPORTDIALOG_H
