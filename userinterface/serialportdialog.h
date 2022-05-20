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
