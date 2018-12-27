#include <QCanBus>
#include <QThread>

#include "connectionwindow.h"
#include "mainwindow.h"
#include "helpwindow.h"
#include "ui_connectionwindow.h"
#include "connections/canconfactory.h"
#include "connections/canconmanager.h"
#include "canbus.h"

ConnectionWindow::ConnectionWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConnectionWindow)
{
    ui->setupUi(this);

    QSettings settings;

    qRegisterMetaType<CANBus>("CANBus");
    qRegisterMetaType<const CANFrame *>("const CANFrame *");
    qRegisterMetaType<const QList<CANFrame> *>("const QList<CANFrame> *");

    connModel = new CANConnectionModel(this);
    ui->tableConnections->setModel(connModel);
    ui->tableConnections->setColumnWidth(0, 40);
    ui->tableConnections->setColumnWidth(1, 70);
    ui->tableConnections->setColumnWidth(2, 70);
    ui->tableConnections->setColumnWidth(3, 70);
    ui->tableConnections->setColumnWidth(4, 70);
    ui->tableConnections->setColumnWidth(5, 70);
    ui->tableConnections->setColumnWidth(6, 70);
    ui->tableConnections->setColumnWidth(7, 90);
    QHeaderView *HorzHdr = ui->tableConnections->horizontalHeader();
    HorzHdr->setStretchLastSection(true); //causes the data column to automatically fill the tableview

    ui->textConsole->setEnabled(false);
    ui->btnClearDebug->setEnabled(false);
    ui->btnSendHex->setEnabled(false);
    ui->btnSendText->setEnabled(false);
    ui->lineSend->setEnabled(false);

    /* load connection configuration */
    loadConnections();

    ui->rbSocketCAN->setEnabled(isSocketCanAvailable());
// #ifdef Q_OS_WIN
    ui->rbKvaser->setEnabled(true);
// #endif

    connect(ui->btnOK, &QAbstractButton::clicked, this, &ConnectionWindow::handleOKButton);
    connect(ui->rbGVRET, &QAbstractButton::clicked, this, &ConnectionWindow::handleConnTypeChanged);
    connect(ui->rbKvaser, &QAbstractButton::clicked, this, &ConnectionWindow::handleConnTypeChanged);
    connect(ui->rbSocketCAN, &QAbstractButton::clicked, this, &ConnectionWindow::handleConnTypeChanged);
    connect(ui->rbRemote, &QAbstractButton::clicked, this, &ConnectionWindow::handleConnTypeChanged);
    connect(ui->rbPeakCANOSX, &QAbstractButton::clicked, this, &ConnectionWindow::handleConnTypeChanged);
    connect(ui->tableConnections->selectionModel(), &QItemSelectionModel::currentRowChanged, this, &ConnectionWindow::currentRowChanged);
    connect(ui->btnActivateAll, &QPushButton::clicked, this, &ConnectionWindow::handleEnableAll);
    connect(ui->btnDeactivateAll, &QPushButton::clicked, this, &ConnectionWindow::handleDisableAll);
    connect(ui->btnReconnect, &QPushButton::clicked, this, &ConnectionWindow::handleReconnect);
    connect(ui->btnRemoveBus, &QPushButton::clicked, this, &ConnectionWindow::handleRemoveConn);
    connect(ui->btnClearDebug, &QPushButton::clicked, this, &ConnectionWindow::handleClearDebugText);
    connect(ui->btnSendHex, &QPushButton::clicked, this, &ConnectionWindow::handleSendHex);
    connect(ui->btnSendText, &QPushButton::clicked, this, &ConnectionWindow::handleSendText);
    connect(ui->ckEnableConsole, &QCheckBox::toggled, this, &ConnectionWindow::consoleEnableChanged);
}

ConnectionWindow::~ConnectionWindow()
{
    QList<CANConnection*>& conns = CANConManager::getInstance()->getConnections();
    CANConnection* conn_p;

    /* save configuration */
    saveConnections();

    /* delete connections */
    while(!conns.isEmpty())
    {
        conn_p = conns.takeFirst();
        conn_p->stop();
        delete conn_p;
    }

    delete ui;
}


void ConnectionWindow::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);
    qDebug() << "Show connectionwindow";
    installEventFilter(this);
    readSettings();
    ui->tableConnections->selectRow(0);
    currentRowChanged(ui->tableConnections->currentIndex(), ui->tableConnections->currentIndex());
    handleConnTypeChanged();
}

void ConnectionWindow::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event);
    removeEventFilter(this);
    writeSettings();
}

bool ConnectionWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyRelease) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        switch (keyEvent->key())
        {
        case Qt::Key_F1:
            HelpWindow::getRef()->showHelp("connectionwindow.html");
            break;
        }
        return true;
    } else {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
    return false;
}

void ConnectionWindow::readSettings()
{
    QSettings settings;
    if (settings.value("Main/SaveRestorePositions", false).toBool())
    {
        resize(settings.value("ConnWindow/WindowSize", QSize(956, 665)).toSize());
        move(settings.value("ConnWindow/WindowPos", QPoint(100, 100)).toPoint());
    }
}

void ConnectionWindow::writeSettings()
{
    QSettings settings;

    if (settings.value("Main/SaveRestorePositions", false).toBool())
    {
        settings.setValue("ConnWindow/WindowSize", size());
        settings.setValue("ConnWindow/WindowPos", pos());
    }
}

void ConnectionWindow::setSuspendAll(bool pSuspend)
{
    QList<CANConnection*>& conns = CANConManager::getInstance()->getConnections();

    foreach(CANConnection* conn_p, conns)
        conn_p->suspend(pSuspend);

    connModel->refresh();
}


void ConnectionWindow::setActiveAll(bool pActive)
{
    QTextStream(stdout) << "ConnectionWindow::setActiveAll called" << endl;
    CANBus bus;
    QList<CANConnection*>& conns = CANConManager::getInstance()->getConnections();

    foreach(CANConnection* conn_p, conns)
    {
        for(int i=0 ; i<conn_p->getNumBuses() ; i++) {
            QTextStream(stdout) << "Found Bus: " <<  conn_p->getPort() <<endl;
            if( conn_p->getBusSettings(i, bus) ) {
                bus.active = pActive;
                QTextStream(stdout) << "Setting Bus Setting for " <<  conn_p->getPort() <<endl;
                conn_p->setBusSettings(i, bus);
            }
        }
    }

    connModel->refresh();
}

void ConnectionWindow::handleReconnect()
{
    int selIdx = ui->tableConnections->selectionModel()->currentIndex().row();
    if (selIdx <0) return;

    int busId;
    CANConnection* conn_p = connModel->getAtIdx(selIdx, busId);
    if(!conn_p) return;

    conn_p->stop();
    conn_p->start();
}

void ConnectionWindow::consoleEnableChanged(bool checked) {
    ui->textConsole->setEnabled(checked);
    ui->btnClearDebug->setEnabled(checked);
    ui->btnSendHex->setEnabled(checked);
    ui->btnSendText->setEnabled(checked);
    ui->lineSend->setEnabled(checked);

    QList<CANConnection*>& conns = CANConManager::getInstance()->getConnections();

    foreach(CANConnection* conn_p, conns)
    {
        if (checked) { //enable console
            connect(conn_p, SIGNAL(debugOutput(QString)), this, SLOT(getDebugText(QString)));
            connect(this, SIGNAL(sendDebugData(QByteArray)), conn_p, SLOT(debugInput(QByteArray)));
        }
        else { //turn it off
            disconnect(conn_p, SIGNAL(debugOutput(QString)), 0, 0);
            disconnect(this, SIGNAL(sendDebugData(QByteArray)), conn_p, SLOT(debugInput(QByteArray)));
        }
    }
}

void ConnectionWindow::handleNewConn()
{
    ui->tableConnections->setCurrentIndex(QModelIndex());
    currentRowChanged(ui->tableConnections->currentIndex(), ui->tableConnections->currentIndex());
}


void ConnectionWindow::handleEnableAll()
{
    setActiveAll(true);
}

void ConnectionWindow::handleDisableAll()
{
    setActiveAll(false);
}

void ConnectionWindow::handleConnTypeChanged()
{
    if (ui->rbGVRET->isChecked()) selectSerial();
    if (ui->rbKvaser->isChecked()) selectKvaser();
    if (ui->rbSocketCAN->isChecked()) selectSocketCan();
    if (ui->rbRemote->isChecked()) selectRemote();
    if (ui->rbPeakCANOSX->isChecked()) selectPeakCANOSX();
}


/* status */
void ConnectionWindow::connectionStatus(CANConStatus pStatus)
{
    Q_UNUSED(pStatus);

    qDebug() << "Connectionstatus changed";
    connModel->refresh();
}


void ConnectionWindow::handleOKButton()
{
    CANConnection* conn_p = NULL;
    QTextStream(stdout) << "handleOKButton" << endl;

    if( ! CANConManager::getInstance()->getByName(getPortName()) )
    {
        /* create connection */
        QTextStream(stdout) << "Creating Connection" << endl;
        conn_p = create(getConnectionType(), getPortName());
        if(!conn_p)
            return;
        /* add connection to model */
        QTextStream(stdout) << "Adding to Model" << endl;
        connModel->add(conn_p);
        consoleEnableChanged(ui->ckEnableConsole->isChecked());
    }
}

void ConnectionWindow::currentRowChanged(const QModelIndex &current, const QModelIndex &previous)
{
    Q_UNUSED(previous);

    int selIdx = current.row();

    int busId;

    disconnect(connModel->getAtIdx(previous.row(), busId), SIGNAL(debugOutput(QString)), 0, 0);
    disconnect(this, SIGNAL(sendDebugData(QByteArray)), connModel->getAtIdx(previous.row(), busId), SLOT(debugInput(QByteArray)));
return;

    /* enable / diable connection type */
    ui->stPort->setEnabled(selIdx==-1);
    ui->gbType->setEnabled(selIdx==-1);
    ui->lPort->setEnabled(selIdx==-1);

    /* set parameters */
    if (selIdx == -1)
    {
        ui->btnOK->setText(tr("Create New Connection"));
        ui->rbGVRET->setChecked(true);
        setSpeed(0);
        setPortName(CANCon::GVRET_SERIAL, "");
    }
    else
    {
        bool ret;
        CANBus bus;
        CANConnection* conn_p = connModel->getAtIdx(selIdx, busId);
        if(!conn_p) return;

        if (ui->ckEnableConsole->isChecked()) { //only connect if console is actually enabled
            connect(conn_p, SIGNAL(debugOutput(QString)), this, SLOT(getDebugText(QString)));
            connect(this, SIGNAL(sendDebugData(QByteArray)), conn_p, SLOT(debugInput(QByteArray)));
        }

        ret = conn_p->getBusSettings(busId, bus);
        if(!ret) return;

        ui->btnOK->setText(tr("Update Connection Settings"));
        setSpeed(bus.getSpeed());
        setPortName(conn_p->getType(), conn_p->getPort());
    }
}

void ConnectionWindow::getDebugText(QString debugText) {
    ui->textConsole->append(debugText);
}

void ConnectionWindow::handleClearDebugText() {
    ui->textConsole->clear();
}

void ConnectionWindow::handleSendHex() {
    QByteArray bytes;
    QStringList tokens = ui->lineSend->text().split(' ');
    foreach (QString token, tokens) {
        bytes.append(token.toInt(nullptr, 16));
    }
    emit sendDebugData(bytes);
}

void ConnectionWindow::handleSendText() {
    QByteArray bytes;
    bytes = ui->lineSend->text().toLatin1();
    bytes.append('\r'); //add carriage return for line ending
    emit sendDebugData(bytes);
}

void ConnectionWindow::selectSerial()
{
    ui->lPort->setText("Port:");
    /* set combobox page visible */
    ui->stPort->setCurrentWidget(ui->cbPage);

    ui->cbPort->clear();
    ports = QSerialPortInfo::availablePorts();

    for (int i = 0; i < ports.count(); i++)
        ui->cbPort->addItem(ports[i].portName());
}

void ConnectionWindow::selectKvaser()
{
    ui->lPort->setText("CAN Port:");
    /* set combobox page visible */
    ui->stPort->setCurrentWidget(ui->cbPage);

    ui->cbPort->clear();
    QString errorString;
    const QList<QCanBusDeviceInfo> devices = QCanBus::instance()->availableDevices(QStringLiteral("peakcan"), &errorString);
    if(QCanBus::instance()->plugins().contains(QStringLiteral("peakcan")))
        QTextStream(stdout) << "PeakCAN Installed" << endl;
    else{
        QTextStream(stdout) << "PeakCAN Not Installed" << endl;
    }
    QTextStream(stdout) << "CAN Devices:" << endl;
    for (int i = 0; i < devices.count(); i++){
        ui->cbPort->addItem(devices[i].name());
        QTextStream(stdout) << devices[i].name() << endl;
    }
}

void ConnectionWindow::selectPeakCANOSX()
{
    ui->lPort->setText("CAN Port:");
    /* set combobox page visible */
    ui->stPort->setCurrentWidget(ui->cbPage);

    ui->cbPort->clear();
    QString errorString;
    const QList<QCanBusDeviceInfo> devices = QCanBus::instance()->availableDevices(QStringLiteral("peakcan"), &errorString);
    if(QCanBus::instance()->plugins().contains(QStringLiteral("peakcan")))
        QTextStream(stdout) << "PeakCAN Installed" << endl;
    else{
        QTextStream(stdout) << "PeakCAN Not Installed" << endl;
    }
    QTextStream(stdout) << "CAN Devices:" << endl;
    for (int i = 0; i < devices.count(); i++){
        ui->cbPort->addItem(devices[i].name());
        QTextStream(stdout) << devices[i].name() << endl;
    }
}

void ConnectionWindow::selectSocketCan()
{
    ui->lPort->setText("Port:");
    /* set edit text page visible */
    ui->stPort->setCurrentWidget(ui->etPage);
}

void ConnectionWindow::selectRemote()
{
    ui->lPort->setText("IP Address:");
    ui->stPort->setCurrentWidget(ui->etPage);
}

void ConnectionWindow::setSpeed(int speed0)
{
}

void ConnectionWindow::setPortName(CANCon::type pType, QString pPortName)
{
    switch(pType)
    {
        case CANCon::GVRET_SERIAL:
            ui->rbGVRET->setChecked(true);
            break;
        case CANCon::KVASER:
            ui->rbKvaser->setChecked(true);
            break;
        case CANCon::SOCKETCAN:
            ui->rbSocketCAN->setChecked(true);
            //you can't configure any of the below three with socketcan so dim them out
            break;
        default: {}
    }

    /* refresh names whenever needed */
    handleConnTypeChanged();

    switch(pType)
    {
        case CANCon::GVRET_SERIAL:
        {
            int idx = ui->cbPort->findText(pPortName);
            if( idx<0 ) idx=0;
            ui->cbPort->setCurrentIndex(idx);
            break;
        }
        case CANCon::SOCKETCAN:
        case CANCon::REMOTE:
        {
            ui->lePort->setText(pPortName);
            break;
        }
        default: {}
    }
}


//-1 means leave it at whatever it booted up to. 0 means disable. Otherwise the actual rate we want.
int ConnectionWindow::getSpeed()
{
    return -1;
}

QString ConnectionWindow::getPortName()
{
    switch( getConnectionType() ) {
    case CANCon::GVRET_SERIAL:
    case CANCon::KVASER:
    case CANCon::PEAKCAN_MAC:
        QTextStream(stdout)  << "getPortName: " << ui->cbPort->currentText() << endl;
        return ui->cbPort->currentText();
    case CANCon::SOCKETCAN:
    case CANCon::REMOTE:
        return ui->lePort->text();
    default:
        qDebug() << "getPortName: can't get port";
    }

    return "";
}

CANCon::type ConnectionWindow::getConnectionType()
{
    if (ui->rbGVRET->isChecked()) return CANCon::GVRET_SERIAL;
    if (ui->rbKvaser->isChecked()) return CANCon::KVASER;
    if (ui->rbSocketCAN->isChecked()) return CANCon::SOCKETCAN;
    if (ui->rbRemote->isChecked()) return CANCon::REMOTE;
    if (ui->rbPeakCANOSX->isChecked()) return CANCon::PEAKCAN_MAC;
    qDebug() << "getConnectionType: error";
    return CANCon::NONE;
}


void ConnectionWindow::setSWMode(bool mode)
{
}

bool ConnectionWindow::getSWMode()
{
    return false;
}

void ConnectionWindow::handleRemoveConn()
{
    int selIdx = ui->tableConnections->selectionModel()->currentIndex().row();
    if (selIdx <0) return;

    qDebug() << "remove connection at index: " << selIdx;

    int busId;
    CANConnection* conn_p = connModel->getAtIdx(selIdx, busId);
    if(!conn_p) return;

    /* remove connection from model & manager */
    connModel->remove(conn_p);

    /* stop and delete connection */
    conn_p->stop();
    delete conn_p;

    /* select first connection in list */
    ui->tableConnections->selectRow(0);
}

void ConnectionWindow::handleRevert()
{

}


bool ConnectionWindow::isSocketCanAvailable()
{
#ifdef Q_OS_LINUX
    if (QCanBus::instance()->plugins().contains(QStringLiteral("socketcan"))) return true;
#endif
    return false;
}


CANConnection* ConnectionWindow::create(CANCon::type pTye, QString pPortName)
{
    CANConnection* conn_p;

    /* create connection */
    conn_p = CanConFactory::create(pTye, pPortName);
    if(conn_p)
    {
        /* connect signal */
        connect(conn_p, SIGNAL(status(CANConStatus)),
                this, SLOT(connectionStatus(CANConStatus)));

        /*TODO add return value and checks */
        conn_p->start();
    }
    return conn_p;
}


void ConnectionWindow::loadConnections()
{
    qRegisterMetaTypeStreamOperators<CANBus>();
    qRegisterMetaTypeStreamOperators<QList<CANBus>>();

    QSettings settings;

    /* fill connection list */
    QVector<QString> portNames = settings.value("connections/portNames").value<QVector<QString>>();
    QVector<int>    devTypes = settings.value("connections/types").value<QVector<int>>();

    for(int i=0 ; i<portNames.count() ; i++)
    {
        CANConnection* conn_p = create((CANCon::type)devTypes[i], portNames[i]);
        /* add connection to model */
        connModel->add(conn_p);
    }

    if (connModel->rowCount() > 0) {
        ui->tableConnections->selectRow(0);
    }
}

void ConnectionWindow::saveConnections()
{
    QList<CANConnection*>& conns = CANConManager::getInstance()->getConnections();

    QSettings settings;
    QVector<QString> portNames;
    QVector<int> devTypes;

    /* save connections */
    foreach(CANConnection* conn_p, conns)
    {
        portNames.append(conn_p->getPort());
        devTypes.append(conn_p->getType());
    }

    settings.setValue("connections/portNames", QVariant::fromValue(portNames));
    settings.setValue("connections/types", QVariant::fromValue(devTypes));
}
