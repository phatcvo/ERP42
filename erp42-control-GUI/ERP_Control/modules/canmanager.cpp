#include "canmanager.h"
#include <QFile>
#include <QCanBus>
#include <QCanBusFrame>
#include <QCloseEvent>
#include <QDesktopServices>

QCanBusDevice *CanManager::send_device;
QCanBusFrame CanManager::m_busFrame;
PC2ERP PCanManager::m_pc2erp;

CanManager::CanManager(QObject *parent) : QObject(parent),
    m_canDevice(nullptr)
{
    send_device = nullptr;
    m_write_frame.reserve(PACKET_SIZE);
    m_numberFramesWritten = 0;
    pluginName = PLUGNAME;
    deviceInterfaceName = InterFaceNAME;
}

void CanManager::processErrors(QCanBusDevice::CanBusError error) const
{
    switch (error) {
    case QCanBusDevice::ReadError:
    case QCanBusDevice::WriteError:
    case QCanBusDevice::ConnectionError:
    case QCanBusDevice::ConfigurationError:
    case QCanBusDevice::UnknownError:
        break;
    default:
        break;
    }
}

void CanManager::connectDevice()
{
    QString errorString;

    m_canDevice = QCanBus::instance()->createDevice(
        pluginName, deviceInterfaceName, &errorString);
    if (!m_canDevice) {
        qWarning() << tr("Error creating device '%1', reason: '%2'")
                          .arg(pluginName).arg(errorString);
        return;
    }

    m_numberFramesWritten = 0;

    connect(m_canDevice, &QCanBusDevice::errorOccurred, this, &CanManager::processErrors);
    connect(m_canDevice, &QCanBusDevice::framesReceived, this, &CanManager::processReceivedFrames);
    connect(m_canDevice, &QCanBusDevice::framesWritten, this, &CanManager::processFramesWritten);

    if (!m_canDevice->connectDevice()) {
        qWarning() << tr("Connection error: %1").arg(m_canDevice->errorString());

        delete m_canDevice;
        m_canDevice = nullptr;
    } else {
        bitRate = m_canDevice->configurationParameter(QCanBusDevice::BitRateKey);
        if (bitRate.isValid()) {
            qDebug() << tr("Plugin: %1, connected to %2 at %3 kBit/s")
                    .arg(pluginName).arg(deviceInterfaceName)
                    .arg(bitRate.toInt() / 1000);
        } else {
            qDebug() << tr("Plugin: %1, connected to %2")
                    .arg(pluginName).arg(deviceInterfaceName);
        }
    }
}

static QString frameFlags(const QCanBusFrame &frame)
{
    QString result = QLatin1String(" --- ");

    if (frame.hasBitrateSwitch())
        result[1] = QLatin1Char('B');
    if (frame.hasErrorStateIndicator())
        result[2] = QLatin1Char('E');
    if (frame.hasLocalEcho())
        result[3] = QLatin1Char('L');

    return result;
}


void CanManager::processReceivedFrames()
{
    if (!m_canDevice)
        return;

    send_device = m_canDevice;

    while (m_canDevice->framesAvailable()) {
        const QCanBusFrame frame = m_canDevice->readFrame();

        QString view;
        if (frame.frameType() == QCanBusFrame::ErrorFrame)
            view = m_canDevice->interpretErrorFrame(frame);
        else
            view = frame.toString().simplified();

        const QString time = QString::fromLatin1("%1.%2  ")
                .arg(frame.timeStamp().seconds(), 10, 10, QLatin1Char(' '))
                .arg(frame.timeStamp().microSeconds() / 100, 4, 10, QLatin1Char('0'));

        const QString flags = frameFlags(frame);

//        std::cout << send_device << std::endl;

        QStringList list = view.split(" ");
//        qDebug()<<"for %I in" << list;
//        std::cout << list[4].toStdString() << std::endl;

        m_TextArea = view;

//        buttontest();
        writeCanFrame();
//        emit TextAreaChanged();

        QThread::usleep(1000);
    }
}


void CanManager::disconnectDevice()
{
    if (!m_canDevice)
        return;

    m_canDevice->disconnectDevice();
    delete m_canDevice;
    m_canDevice = nullptr;
    qDebug() << tr("Disconnected");
}

void CanManager::processFramesWritten(qint64 count)
{
    m_numberFramesWritten += count;
    qDebug() << tr("%1 frames written").arg(m_numberFramesWritten);
}

void CanManager::sendRawFrame(QCanBusFrame &frame) const
{
    if (!send_device)
        return;

//    std::cout <<send_device <<std::endl;
    send_device->writeFrame(frame);
}

void CanManager::setFrameID(const QString &frameID)
{
    if(m_FrameID == frameID)
        return;
    m_FrameID = frameID;
    emit frameIDChanged(m_FrameID);

}
void CanManager::setFrameData(const QString &frameData)
{
    if(m_FrameData == frameData)
        return;
    m_FrameData = frameData;
    emit frameDataChanged(m_FrameData);
}

std::vector<quint8> CanManager::setCanFrame()
{
    m_write_frame.emplace_back(p_canManager->m_pc2erp.MODE);
    m_write_frame.emplace_back(p_canManager->m_pc2erp.speed.speed[1]);
    m_write_frame.emplace_back(p_canManager->m_pc2erp.speed.speed[0]);
    m_write_frame.emplace_back(p_canManager->m_pc2erp.steer.steer[1]);
    m_write_frame.emplace_back(p_canManager->m_pc2erp.steer.steer[0]);
    m_write_frame.emplace_back(p_canManager->m_pc2erp.brake);
    m_write_frame.emplace_back(p_canManager->m_pc2erp.alive);

//    for (const auto& i : m_write_frame)
//    {
//        cout << hex << i << " ";
//    }
//    cout << endl;
    return m_write_frame;
}

void CanManager::writeCanFrame()
{
    const uint id = WriteframeID;
    std::vector<quint8> data = setCanFrame();

    QByteArray databuf;
    databuf = QByteArray(reinterpret_cast<char*>(data.data()), PACKET_SIZE);
//    databuf = QByteArray(reinterpret_cast<char*>(data), PACKET_SIZE);

    QCanBusFrame CanFrame = QCanBusFrame(id, databuf);

    if (!send_device)
    {
        std::cout << "not connect" << std::endl;
        return;
    }
    else
        sendRawFrame(CanFrame);
}

void CanManager::buttontest()
{
    qDebug()<<m_FrameID<<m_FrameData;

    QString id = m_FrameID;

    const uint frameId = id.toUInt(nullptr, 16);

    QString data = m_FrameData;
    const QByteArray payload = QByteArray::fromHex(data.remove(QLatin1Char(' ')).toLatin1());

    QByteArray ba_as_hex_string = payload.toHex();

    QString str = "";
    str.append(payload);

    m_busFrame = QCanBusFrame(frameId, payload);

//    setCanFrame();

    if (!send_device)
    {
        std::cout << "not connect" << std::endl;
        return;
    }
    else
        sendRawFrame(m_busFrame);
}


