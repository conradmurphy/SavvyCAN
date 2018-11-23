#include "serialbusconnection.h"

#include "canconmanager.h"

#include <QCanBus>
#include <QCanBusFrame>
#include <QDateTime>
#include <QDebug>

/***********************************/
/****    class definition       ****/
/***********************************/

SerialBusConnection::SerialBusConnection(QString portName, CANCon::type pType) :
    CANConnection(portName, pType, 1, 4000, true),
    mTimer(this) /*NB: set connection as parent of timer to manage it from working thread */
{
}


SerialBusConnection::~SerialBusConnection()
{
    stop();
}


void SerialBusConnection::piStarted()
{
    connect(&mTimer, SIGNAL(timeout()), this, SLOT(testConnection()));
    mTimer.setInterval(1000);
    mTimer.setSingleShot(false); //keep ticking
    mTimer.start();
    mBusData[0].mBus.setEnabled(true);
    mBusData[0].mConfigured = true;
}


void SerialBusConnection::piSuspend(bool pSuspend)
{
    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}


void SerialBusConnection::piStop() {
    mTimer.stop();
    disconnectDevice();
}


bool SerialBusConnection::piGetBusSettings(int pBusIdx, CANBus& pBus)
{
    return getBusConfig(pBusIdx, pBus);
}


void SerialBusConnection::piSetBusSettings(int pBusIdx, CANBus bus)
{
    CANConStatus stats;
    /* sanity checks */
    if(0 != pBusIdx)
        return;

    /* disconnect device if we have one connected */
    disconnectDevice();

    /* copy bus config */
    setBusConfig(0, bus);

    /* if bus is not active we are done */
    if(!bus.active)
        return;

    /* create device */
    QString errorString;
    switch(getType()) {
        case CANCon::SOCKETCAN:
             mDev_p = QCanBus::instance()->createDevice("socketcan", getPort(), &errorString);
        case CANCon::KVASER:
             mDev_p = QCanBus::instance()->createDevice("peakcan", getPort(), &errorString);
        default: { mDev_p = NULL;}
    }

    if (!mDev_p) {
        disconnectDevice();
        qDebug() << "Error: createDevice(" << getType() << getPort() << "):" << errorString;
        return;
    }

    /* connect slots */
    connect(mDev_p, &QCanBusDevice::errorOccurred, this, &SerialBusConnection::errorReceived);
    connect(mDev_p, &QCanBusDevice::framesWritten, this, &SerialBusConnection::framesWritten);
    connect(mDev_p, &QCanBusDevice::framesReceived, this, &SerialBusConnection::framesReceived);

    /* set configuration */
    /*if (p.useConfigurationEnabled) {
     foreach (const SettingsDialog::ConfigurationItem &item, p.configurations)
         mDev->setConfigurationParameter(item.first, item.second);
    }*/

    //You cannot set the speed of a socketcan interface, it has to be set with console commands.
    //mDev_p->setConfigurationParameter(QCanBusDevice::BitRateKey, bus.speed);

    /* connect device */
    if (!mDev_p->connectDevice()) {
        disconnectDevice();
        qDebug() << "can't connect device";
    }
}


bool SerialBusConnection::piSendFrame(const CANFrame& pFrame)
{
    /* sanity checks */
    if(0 != pFrame.bus || pFrame.len>8)
        return false;
    if (!mDev_p) return false;

    /* fill frame */
    QCanBusFrame frame;
    frame.setFrameId(pFrame.ID);
    frame.setExtendedFrameFormat(pFrame.extended);
    if (pFrame.remote) {
        frame.setFrameType(QCanBusFrame::FrameType::RemoteRequestFrame);
    } else {
        frame.setFrameType(QCanBusFrame::FrameType::DataFrame);
    }
    frame.setPayload(QByteArray(reinterpret_cast<const char *>(pFrame.data),
                                static_cast<int>(pFrame.len)));
    return mDev_p->writeFrame(frame);
}


/***********************************/
/****   private methods         ****/
/***********************************/


/* disconnect device */
void SerialBusConnection::disconnectDevice() {
    if(mDev_p) {
        mDev_p->disconnectDevice();
        delete mDev_p;
        mDev_p = nullptr;
    }
}


void SerialBusConnection::errorReceived(QCanBusDevice::CanBusError error) const
{
    switch (error) {
        case QCanBusDevice::ReadError:
        case QCanBusDevice::WriteError:
        case QCanBusDevice::ConnectionError:
        case QCanBusDevice::ConfigurationError:
        case QCanBusDevice::UnknownError:
        qWarning() << mDev_p->errorString();
        break;
    default:
        break;
    }
}

void SerialBusConnection::framesWritten(qint64 count)
{
    Q_UNUSED(count);
    //qDebug() << "Number of frames written:" << count;
}

void SerialBusConnection::framesReceived()
{
    uint64_t timeBasis = CANConManager::getInstance()->getTimeBasis();

    /* sanity checks */
    if(!mDev_p)
        return;

    /* read frame */
    while(true)
    {
        const QCanBusFrame recFrame = mDev_p->readFrame();

        /* exit case */
        if(!recFrame.isValid())
            break;

        /* drop frame if capture is suspended */
        if(isCapSuspended())
            continue;

        /* check frame */
        if (recFrame.payload().length() <= 8) {
            CANFrame* frame_p = getQueue().get();
            if(frame_p) {
                frame_p->len           = static_cast<uint32_t>(recFrame.payload().length());
                frame_p->bus           = 0;

                if (recFrame.frameType() == QCanBusFrame::ErrorFrame) {
                    // Constants defined in include/uapi/linux/can/error.h
                    switch (recFrame.error()) {
                        case QCanBusFrame::TransmissionTimeoutError:
                            frame_p->ID = 0x20000001;
                            break;
                        case QCanBusFrame::LostArbitrationError:
                            frame_p->ID = 0x20000002;
                            break;
                        case QCanBusFrame::ControllerError:
                            frame_p->ID = 0x20000004;
                            break;
                        case QCanBusFrame::ProtocolViolationError:
                            frame_p->ID = 0x20000008;
                            break;
                        case QCanBusFrame::TransceiverError:
                            frame_p->ID = 0x20000010;
                            break;
                        case QCanBusFrame::MissingAcknowledgmentError:
                            frame_p->ID = 0x20000020;
                            break;
                        case QCanBusFrame::BusOffError:
                            frame_p->ID = 0x20000040;
                            break;
                        case QCanBusFrame::BusError:
                            frame_p->ID = 0x20000080;
                            break;
                        case QCanBusFrame::ControllerRestartError:
                            frame_p->ID = 0x20000100;
                            break;
                        default:
                            break;
                    }
                    frame_p->extended = true;
                } else {
                    frame_p->extended      = recFrame.hasExtendedFrameFormat();
                    frame_p->ID            = recFrame.frameId();
                    frame_p->remote        = (recFrame.frameType() == recFrame.RemoteRequestFrame);
                    memcpy(frame_p->data, recFrame.payload().data(), frame_p->len);
                    frame_p->isReceived = true;
                }
              
                frame_p->isReceived    = true;
                if (useSystemTime) {
                    frame_p->timestamp = QDateTime::currentMSecsSinceEpoch() * 1000ul;
                }
                else frame_p->timestamp     = (recFrame.timeStamp().seconds() * 1000000ul + recFrame.timeStamp().microSeconds()) - timeBasis;

                checkTargettedFrame(*frame_p);

                /* enqueue frame */
                getQueue().queue();
            }
#if 0
            else
                qDebug() << "can't get a frame, ERROR";
#endif
        }
    }
}


void SerialBusConnection::testConnection() {
    QCanBusDevice*  dev_p = QCanBus::instance()->createDevice("socketcan", getPort());
    CANConStatus stats;

    switch(getStatus())
    {
        case CANCon::CONNECTED:
            if (!dev_p || !dev_p->connectDevice()) {
                /* we have lost connectivity */
                disconnectDevice();

                setStatus(CANCon::NOT_CONNECTED);
                stats.conStatus = getStatus();
                stats.numHardwareBuses = mNumBuses;
                emit status(stats);
            }
            break;
        case CANCon::NOT_CONNECTED:
            if (dev_p && dev_p->connectDevice()) {
                if(!mDev_p) {
                    /* try to reconnect */
                    CANBus bus;
                    if(getBusConfig(0, bus))
                    {
                        bus.setEnabled(true);
                        setBusSettings(0, bus);
                    }
                }
                /* disconnect test instance */
                dev_p->disconnectDevice();

                setStatus(CANCon::CONNECTED);
                stats.conStatus = getStatus();
                stats.numHardwareBuses = mNumBuses;
                emit status(stats);
            }
            break;
        default: {}
    }

    if(dev_p)
        delete dev_p;
}
