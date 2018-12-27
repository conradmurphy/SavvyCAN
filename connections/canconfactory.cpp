#include <QString>
#include "canconfactory.h"
#include "serialbusconnection.h"
#include "gvretserial.h"
#include "peakcanmac.h"

using namespace CANCon;

CANConnection* CanConFactory::create(type pType, QString pPortName)
{
    switch(pType) {
    case SOCKETCAN:
        return new SerialBusConnection(pPortName, CANCon::SOCKETCAN);
    case KVASER:
        return new SerialBusConnection(pPortName, CANCon::KVASER);
    case GVRET_SERIAL:
        return new GVRetSerial(pPortName, false);
    case REMOTE:
        return new GVRetSerial(pPortName, true);  //it's a special case of GVRET connected over TCP/IP so it uses the same class
    case PEAKCAN_MAC:
        return new PeakCANMAC(pPortName, true);
    default: {}
    }

    return NULL;
}
