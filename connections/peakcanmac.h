#ifndef PEAKCANMAC_H
#define PEAKCANMAC_H

#include <QSerialPort>
#include <QCanBusDevice>
#include <QThread>
#include <QTimer>
#include <QTcpSocket>

/*************/
#include <QDateTime>
/*************/

#include "canframemodel.h"
#include "canconnection.h"
#include "canconmanager.h"



class PeakCANMAC : public CANConnection
{
    Q_OBJECT

public:
    PeakCANMAC(QString portName, bool useTcp);
    virtual ~PeakCANMAC();

protected:

    virtual void piStarted();
    virtual void piStop();
    virtual void piSetBusSettings(int pBusIdx, CANBus pBus);
    virtual bool piGetBusSettings(int pBusIdx, CANBus& pBus);
    virtual void piSuspend(bool pSuspend);
    virtual bool piSendFrame(const CANFrame&) ;

    void disconnectDevice();

public slots:
    void debugInput(QByteArray bytes);

private slots:
    void connectDevice();
    void connectionTimeout();
    void readSerialData();
    void serialError(QSerialPort::SerialPortError err);
    void tcpConnected();
    void handleTick();

private:
    void readSettings();
    void procRXChar(unsigned char);
    void sendCommValidation();
    void rebuildLocalTimeBasis();
    void sendToSerial(const QByteArray &bytes);

protected:
    QTimer             mTimer;
    QThread            mThread;

    bool doValidation;
    bool gotValidated;
    bool isAutoRestart;
    bool continuousTimeSync;
    bool useTcp;
    QSerialPort *serial;
    QTcpSocket *tcpClient;
    int framesRapid;
    uint32_t rx_step;
    CANFrame buildFrame;
    int can0Baud, can1Baud, swcanBaud, lin1Baud, lin2Baud;
    bool can0Enabled, can1Enabled, swcanEnabled, lin1Enabled, lin2Enabled;
    bool can0ListenOnly, can1ListenOnly, swcanListenOnly;
    int deviceBuildNum;
    int deviceSingleWireMode;
    uint32_t buildTimeBasis;
    int32_t timeBasis;
    uint64_t lastSystemTimeBasis;
    uint64_t timeAtGVRETSync;
};

#endif // PEAKCANMAC_H
