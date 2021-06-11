#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <QObject>

class ServoController : public QObject
{
    Q_OBJECT
public:
    virtual void requestSteering(float steering) = 0;

    bool getInvertOutput() const;
    void setInvertOutput(bool getInvertOutput);

signals:

private:
    bool mInvertOutput = false;

};

#endif // SERVOCONTROLLER_H
