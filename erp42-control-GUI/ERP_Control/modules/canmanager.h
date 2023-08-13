#ifndef CANMANAGER_H
#define CANMANAGER_H

#include <QObject>
#include <QtSerialBus/QCanBus>
#include <QDebug>
#include <QJsonObject>
#include <QJsonDocument>
#include <QTimer>
#include <iostream>
#include <QThread>
#include <vector>
#include "pcanmanager.h"

const int8_t PACKET_SIZE = 8;
const QString PLUGNAME = "peakcan";
const QString InterFaceNAME = "usb0";
const uint WriteframeID = 0x777;

using namespace std;

class CanManager : public QObject
{
    Q_OBJECT

public:
    explicit CanManager(QObject *parent = nullptr);
    virtual ~CanManager() noexcept {}
    CanManager ( const CanManager & ) = delete;
    CanManager ( const CanManager && ) = delete;

    PCanManager *p_canManager;

    Q_PROPERTY(QString frameID   READ getframeID WRITE setFrameID NOTIFY frameIDChanged)
    Q_PROPERTY(QString frameData READ getframeData WRITE setFrameData NOTIFY frameDataChanged)

    Q_INVOKABLE void buttontest();

    QString getTextArea() const { return m_TextArea;}
    QString getframeID() const { return m_FrameID;}
    QString getframeData() const { return m_FrameData;}

    void setFrameID(const QString &frameID);
    void setFrameData(const QString &frameData);

    /* Can connect function */
    void connectDevice();
    void disconnectDevice();
    void processReceivedFrames(); //dbc parsing using GENEVI/CANdb
    void processErrors(QCanBusDevice::CanBusError) const;
    void processFramesWritten(qint64);
    void sendRawFrame(QCanBusFrame &frame) const;
    std::vector<quint8> setCanFrame();
    void writeCanFrame();

    /* static */
    static QString m_TextArea;
    static QCanBusFrame m_busFrame;
    static QCanBusDevice *send_device;

private:
    QCanBusDevice *m_canDevice;

    qint64 m_numberFramesWritten;
    QVariant bitRate;
    QString pluginName;
    QString deviceInterfaceName;
    QString m_FrameID;
    QString m_FrameData;
    const QCanBusFrame m_Frame;

//    quint8* m_write_frame;
    std::vector<quint8> m_write_frame;

//    std::vector<quint8> m_can_packet;
//    std::vector<quint8> get_packet() const {return m_can_packet;}


    //Thread
//    void run();

signals:
    void TextAreaChanged();
    void frameIDChanged(QString frameID);
    void frameDataChanged(QString frameData);

public slots:

};

#endif // CANMANAGER_H

