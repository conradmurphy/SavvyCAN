#include <QObject>
#include <QDebug>
#include <QCanBusFrame>
#include <QSerialPortInfo>
#include <QSettings>
#include <QStringBuilder>
#include <QtNetwork>

#include "peakcanmac.h"
#include "PCBUSB.h"

#define PCAN_CHANNEL PCAN_USBBUS1
#define PCAN_BAUDRATE PCAN_BAUD_250K

PeakCANMAC::PeakCANMAC(QString portName, bool useTcp) :
    CANConnection(portName, CANCon::PEAKCAN_MAC, 1, 4000, true),
    // useTcp(useTcp),
    mTimer(this) /*NB: set this as parent of timer to manage it from working thread */
{
    qDebug() << "PeakCANMAC()";
    debugOutput("PeakCANMAC()");
    QTextStream(stdout) << "PeakCANMAC() Constructor" << endl;

    timeBasis = 0;
    lastSystemTimeBasis = 0;

    readSettings();
}


PeakCANMAC::~PeakCANMAC()
{
    stop();
    qDebug() << "~PeakCANMAC()";
    debugOutput("~PeakCANMAC()");
}

void PeakCANMAC::sendToSerial(const QByteArray &bytes)
{
    if (serial == NULL && tcpClient == NULL)
    {
        debugOutput("Attempt to write to serial port when it has not been initialized!");
        return;
    }

    if (serial && !serial->isOpen())
    {
        debugOutput("Attempt to write to serial port when it is not open!");
        return;
    }

    if (tcpClient && !tcpClient->isOpen())
    {
        debugOutput("Attempt to write to TCP/IP port when it is not open!");
        return;
    }

    QString buildDebug;
    buildDebug = "Write to serial -> ";
    foreach (int byt, bytes) {
        byt = (unsigned char)byt;
        buildDebug = buildDebug % QString::number(byt, 16) % " ";
    }
    debugOutput(buildDebug);

    if (serial) serial->write(bytes);
    if (tcpClient) tcpClient->write(bytes);
}

void PeakCANMAC::piStarted()
{    
    connectDevice();
}


void PeakCANMAC::piSuspend(bool pSuspend)
{
    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}


void PeakCANMAC::piStop()
{
    mTimer.stop();
    disconnectDevice();
}


bool PeakCANMAC::piGetBusSettings(int pBusIdx, CANBus& pBus)
{
    return getBusConfig(pBusIdx, pBus);
}


void PeakCANMAC::piSetBusSettings(int pBusIdx, CANBus bus)
{
    /* sanity checks */
    if( (pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    /* copy bus config */
    setBusConfig(pBusIdx, bus);

    qDebug() << "About to update bus " << pBusIdx << " on GVRET";

}


bool PeakCANMAC::piSendFrame(const CANFrame& frame)
{
    QByteArray buffer;
    unsigned int c;
    int ID;

    //qDebug() << "Sending out GVRET frame with id " << frame.ID << " on bus " << frame.bus;

    framesRapid++;

    if (serial == NULL) return false;
    if (!serial->isOpen()) return false;
    //if (!isConnected) return false;

    // Doesn't make sense to send an error frame
    // to an adapter
    if (frame.ID & 0x20000000) {
        return true;
    }
    ID = frame.ID;
    if (frame.extended) ID |= 1 << 31;

    buffer[0] = (unsigned char)0xF1; //start of a command over serial
    buffer[1] = 0; //command ID for sending a CANBUS frame
    buffer[2] = (unsigned char)(ID & 0xFF); //four bytes of ID LSB first
    buffer[3] = (unsigned char)(ID >> 8);
    buffer[4] = (unsigned char)(ID >> 16);
    buffer[5] = (unsigned char)(ID >> 24);
    buffer[6] = (unsigned char)((frame.bus) & 3);
    buffer[7] = (unsigned char)frame.len;
    for (c = 0; c < frame.len; c++)
    {
        buffer[8 + c] = frame.data[c];
    }
    buffer[8 + frame.len] = 0;

    sendToSerial(buffer);

    return true;
}



/****************************************************************/

void PeakCANMAC::readSettings()
{
    QSettings settings;

    if (settings.value("Main/ValidateComm", true).toBool())
    {
        doValidation = true;
    }
    else doValidation = false;
}


void PeakCANMAC::connectDevice()
{
    QTextStream(stdout) << "PeakCANMAC::connectDevice()" << endl;
    QSettings settings;
    //TPCANMsg message;
    TPCANStatus status;
    int fd;
    // fd_set fds;

    /* disconnect device */
    CAN_Uninitialize(PCAN_NONEBUS);

    /* open new device */
    status = CAN_Initialize(PCAN_CHANNEL, PCAN_BAUDRATE, 0, 0, 0);
    printf("Initialize CAN: 0x%lx\n", status);
    // if(status != PCAN_ERROR_OK) goto leave;

    status = CAN_GetValue(PCAN_CHANNEL, PCAN_RECEIVE_EVENT, &fd, sizeof(int));
    // if(status != PCAN_ERROR_OK) goto leave;

    QSocketNotifier *sn;
    sn = new QSocketNotifier( fd, QSocketNotifier::Read, this);
    QObject::connect( sn, SIGNAL(activated(int)),
        this, SLOT(readSerialData()) );

}

void PeakCANMAC::tcpConnected()
{
    
}

void PeakCANMAC::disconnectDevice() {
    CAN_Uninitialize(PCAN_NONEBUS);
    setStatus(CANCon::NOT_CONNECTED);
    CANConStatus stats;
    stats.conStatus = getStatus();
    stats.numHardwareBuses = mNumBuses;
    emit status(stats);
}

void PeakCANMAC::serialError(QSerialPort::SerialPortError err)
{
    QString errMessage;
    bool killConnection = false;
    switch (err)
    {
    case QSerialPort::NoError:
        return;
    case QSerialPort::DeviceNotFoundError:
        errMessage = "Device not found error on serial";
        killConnection = true;
        break;
    case QSerialPort::PermissionError:
        errMessage =  "Permission error on serial port";
        killConnection = true;
        break;
    case QSerialPort::OpenError:
        errMessage =  "Open error on serial port";
        killConnection = true;
        break;
    case QSerialPort::ParityError:
        errMessage = "Parity error on serial port";
        break;
    case QSerialPort::FramingError:
        errMessage = "Framing error on serial port";
        break;
    case QSerialPort::BreakConditionError:
        errMessage = "Break error on serial port";
        break;
    case QSerialPort::WriteError:
        errMessage = "Write error on serial port";
        break;
    case QSerialPort::ReadError:
        errMessage = "Read error on serial port";
        break;
    case QSerialPort::ResourceError:
        errMessage = "Serial port seems to have disappeared.";
        killConnection = true;
        break;
    case QSerialPort::UnsupportedOperationError:
        errMessage = "Unsupported operation on serial port";
        killConnection = true;
        break;
    case QSerialPort::UnknownError:
        errMessage = "Beats me what happened to the serial port.";
        killConnection = true;
        break;
    case QSerialPort::TimeoutError:
        errMessage = "Timeout error on serial port";
        killConnection = true;
        break;
    case QSerialPort::NotOpenError:
        errMessage = "The serial port isn't open dummy";
        killConnection = true;
        break;
    }
    serial->clearError();
    serial->flush();
    serial->close();
    if (errMessage.length() > 1)
    {
        qDebug() << errMessage;
        debugOutput(errMessage);
    }
    if (killConnection)
    {
        qDebug() << "Shooting the serial object in the head. It deserves it.";
        disconnectDevice();
    }
}


void PeakCANMAC::connectionTimeout()
{
    //one second after trying to connect are we actually connected?
    //if (CANCon::NOT_CONNECTED==getStatus()) //no?
    if (!gotValidated)
    {
        //then emit the the failure signal and see if anyone cares
        qDebug() << "Failed to connect to GVRET at that com port";

        disconnectDevice();
    }
}


void PeakCANMAC::readSerialData()
{
    QByteArray data;
    unsigned char c;
    QString debugBuild;

    TPCANStatus status;
    TPCANMsg message;

    QTextStream(stdout) << "PeakCANMAC::readSerialData()" << endl;
    status = CAN_Read(PCAN_CHANNEL, &message, NULL);
    if (status != PCAN_ERROR_OK) {
        printf("Error 0x%lx\n", status);
    }
    printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
            (int) message.ID, (int) message.LEN, (int) message.DATA[0],
            (int) message.DATA[1], (int) message.DATA[2],
            (int) message.DATA[3], (int) message.DATA[4],
            (int) message.DATA[5], (int) message.DATA[6],
            (int) message.DATA[7]);

    CANFrame* frame_p = getQueue().get();

    frame_p->ID = message.ID;
    // buildFrame.bus = this.mBus;
    frame_p->extended = message.MSGTYPE == PCAN_MESSAGE_EXTENDED;
    frame_p->isReceived = true;
    frame_p->len = message.LEN;
    memcpy(frame_p->data, message.DATA, 8);
    frame_p->timestamp = QDateTime::currentMSecsSinceEpoch() * 1000l;

    checkTargettedFrame(*frame_p);
    
    /* enqueue frame */
    getQueue().queue();

    /* enqueue frame */
    // getQueue().queue();

    /*
    if (serial) data = serial->readAll();
    if (tcpClient) data = tcpClient->readAll();

    debugOutput("Got data from serial. Len = " % QString::number(data.length()));
    //qDebug() << (tr("Got data from serial. Len = %0").arg(data.length()));
    for (int i = 0; i < data.length(); i++)
    {
        c = data.at(i);
        //qDebug() << c << "    " << QString::number(c, 16) << "     " << QString(c);
        debugBuild = debugBuild % QString::number(c, 16) % " ";
        procRXChar(c);
    }
    debugOutput(debugBuild);*/
}

//Debugging data sent from connection window. Inject it into Comm traffic.
void PeakCANMAC::debugInput(QByteArray bytes) {
   sendToSerial(bytes);
}

void PeakCANMAC::procRXChar(unsigned char c)
{
    
}

void PeakCANMAC::rebuildLocalTimeBasis()
{
    qDebug() << "Rebuilding GVRET time base. GVRET local base = " << buildTimeBasis;

    /*
      our time basis is the value we have to modulate the main system basis by in order
      to sync the GVRET timestamps to the rest of the system.
      The rest of the system uses CANConManager::getInstance()->getTimeBasis as the basis.
      GVRET returns to us the current time since boot up in microseconds.
      timeAtGVRETSync stores the "system" timestamp when the GVRET timestamp was retrieved.
    */
    lastSystemTimeBasis = CANConManager::getInstance()->getTimeBasis();
    int64_t systemDelta = timeAtGVRETSync - lastSystemTimeBasis;
    int32_t localDelta = buildTimeBasis - systemDelta;
    timeBasis = -localDelta;
}

// checkTargettedFrame
void PeakCANMAC::handleTick()
{
    if (lastSystemTimeBasis != CANConManager::getInstance()->getTimeBasis()) rebuildLocalTimeBasis();
    //qDebug() << "Tick!";

    if( CANCon::CONNECTED == getStatus() )
    {
        if (!gotValidated && doValidation)
        {
            if (serial == NULL && tcpClient == NULL) return;
            if ( (serial && serial->isOpen()) || (tcpClient && tcpClient->isOpen())) //if it's still false we have a problem...
            {
                qDebug() << "Comm validation failed. ";

                setStatus(CANCon::NOT_CONNECTED);
                //emit status(getStatus());

                disconnectDevice(); //start by stopping everything.
                //Then wait 500ms and restart the connection automatically
                //QTimer::singleShot(500, this, SLOT(connectDevice()));
                return;
            }
        }
        else if (doValidation); //qDebug()  << "Comm connection validated";
    }
    if (doValidation && serial && serial->isOpen()) sendCommValidation();
    if (doValidation && tcpClient && tcpClient->isOpen()) sendCommValidation();
}


void PeakCANMAC::sendCommValidation()
{
    QByteArray output;

    gotValidated = false;
    output.append((unsigned char)0xF1); //another command to the GVRET
    output.append((unsigned char)0x09); //request a reply to get validation

    sendToSerial(output);
}



