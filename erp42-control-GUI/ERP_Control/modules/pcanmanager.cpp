#include "pcanmanager.h"
#include "canmanager.h"
#define T true

QString CanManager::m_TextArea;
//extern QCanBusDevice *send_device;
//extern QCanBusFrame m_busFrame;
//QCanBusFrame CanManager::m_busFrame;


PCanManager::PCanManager(QObject *parent):
    m_MorA(0x00),
    m_Estop(0x00),
    m_Gear(N),
    m_SteerAngle(0),
    m_Speed(0),
    m_Brake(0),
    m_Cycle(20),
    m_ActiveEnable(false),
    m_AutoEnable(false),
    m_EstopEnable(false),
    m_SteerEnable(false),
    m_SpeedEnable(false),
    m_BrakeEnable(false),
    m_GearDrive(false),
    m_GearNeutral(false),
    m_GearReverse(false),

    m_str_ID1("0"),
    m_str_ID2("0"),

    m_str_QMorA("0"),
    m_str_ESTOP("0"),
    m_str_GEAR("0"),
    m_str_SPEED("0"),
    m_str_STEER("0"),
    m_str_BRAKE("0"),
    m_str_ALIVE("0"),
    m_modified_str_SteerAngle("0"),
    m_modified_str_Speed("0"),
    m_modified_str_Brake("0"),

    m_FB_str_QMorA("0"),
    m_FB_str_ESTOP("0"),
    m_FB_str_GEAR("0"),
    m_FB_str_SPEED("0"),
    m_FB_str_STEER("0"),
    m_FB_str_BRAKE("0"),
    m_FB_str_ALIVE("0"),
    m_FB_modified_str_SteerAngle("0"),
    m_FB_modified_str_Speed("0"),
    m_FB_modified_str_Brake("0"),

    m_FB_str_Encoder0("0"),
    m_FB_str_Encoder1("0"),
    m_FB_str_Encoder2("0"),
    m_FB_str_Encoder3("0"),
    m_FB_str_BCR("0"),
    m_FB_str_BR("0"),
    m_FB_str_BE("0"),
    m_FB_str_BIM("0"),
    m_FB_modified_str_Encoder0("0"),
    m_FB_modified_str_Encoder1("0"),
    m_FB_modified_str_Encoder2("0"),
    m_FB_modified_str_Encoder3("0"),
    m_FB_modified_str_BCR("0"),
    m_FB_modified_str_BR("0"),
    m_FB_modified_str_BE("0"),
    m_FB_modified_str_BIM("0") {}

void PCanManager::setActive(const bool &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);

    m_ActiveEnable = arg;
    emit ActiveChanged();
}

void PCanManager::setAutoEnable(const bool &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);

    m_AutoEnable = arg;
    m_MorA = (m_AutoEnable) ? 0x01 : 0x00;

    m_str_QMorA = QString::number(m_MorA, 16);

    emit AutoEnableChanged();
    emit get_str_QMorAChanged();
}

void PCanManager::setEstopEnable(const bool &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);

    m_EstopEnable = arg;
    m_Estop = (m_EstopEnable) ? 0x02 : 0x00;
    m_str_ESTOP = QString::number(m_Estop, 16);

    emit EstopEnableChanged();
    emit get_str_ESTOPChanged();
}

void PCanManager::setSteerEnable(const bool &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);
    m_SteerEnable = arg;
    emit SteerEnableChanged();

}

void PCanManager::setSpeedEnable(const bool &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);
    m_SpeedEnable = arg;
    emit SpeedEnableChanged();

}

void PCanManager::setBrakeEnable(const bool &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);
    if(!m_BrakeEnable)
    {
        m_BrakeEnable = arg;
        emit BrakeEnableChanged();
    }
}

void PCanManager::setGearDrive(const bool &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);

    emit GearDriveChanged();

    m_str_GEAR = QString::number(D, 16);
    m_Gear = D;
    emit get_str_GEARChanged();
}

void PCanManager::setGearNeutral(const bool &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);
    if(!m_GearNeutral)
    {
        m_GearNeutral = arg;
        emit GearNeutralChanged();
    }
    m_str_GEAR = QString::number(N, 16);
    m_Gear = N;
    emit get_str_GEARChanged();
}

void PCanManager::setGearReverse(const bool &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);
    if(!m_GearReverse)
    {
        m_GearReverse = arg;
        emit GearReverseChanged();
    }
    m_str_GEAR = QString::number(R, 16);
    m_Gear = R;
    emit get_str_GEARChanged();
}

void PCanManager::setSteerAngle(const qint16 &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);

    m_SteerAngle = arg;
    emit SteerAngleChanged();

    m_str_STEER = QString::number((arg*STEER_FACTOR) & 0xffff, 16);
    m_modified_str_SteerAngle = QString::number((arg*STEER_FACTOR));
    emit get_str_STEERChanged();
}
void PCanManager::setSpeed(const quint16 &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);
    m_Speed = arg;
    emit SpeedChanged();

    m_str_SPEED = QString::number(arg*SPEED_FACTOR,16);
    m_modified_str_Speed = QString::number((arg*SPEED_FACTOR));
    emit get_str_SPEEDChanged();
}

void PCanManager::setBrake(const quint8 &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);
    m_Brake = arg;
    emit BrakeChanged();

    m_str_BRAKE = QString::number(arg,16);
    m_modified_str_Brake = QString::number(arg);
    emit get_str_BRAKEChanged();
}


void PCanManager::setCycle(const quint16 &arg)
{
    if(T)
        qDebug() << tr("%1 > arg : %2").arg(__func__).arg(arg);
    m_Cycle = arg;
}

void PCanManager::getFeedback()
{
    std::cout << " test 1 : " << getData(CanManager::m_TextArea).toStdString() << std::endl;
    QStringList list = CanManager::m_TextArea.split(" ");
    qDebug()<<"for %I in" << list;

    bool ok;
    const uint ERP_CAN_Id =  list[0].toUInt(&ok, 16);

    switch (ERP_CAN_Id)
    {
    case ERP_ID_1:
        cout << "1" << endl;
        m_str_ID1 = QString::number(TEST_ID,16);

        m_erp2pc_1.MorA             = (list[2].toUInt(&ok, 16) & 0x01);
        m_erp2pc_1.ESTOP            = (list[2].toUInt(&ok, 16) & 0x02);
        m_erp2pc_1.GEAR             = (list[2].toUInt(&ok, 16) & 0x0c);
        m_erp2pc_1.speed._speed     = (list[4].toUInt(&ok, 16) & 0xff) << 8 | (list[3].toUInt(&ok, 16) & 0xff);
        m_erp2pc_1.steer._steer     = (list[6].toUInt(&ok, 16) & 0xff) << 8 | (list[5].toUInt(&ok, 16) & 0xff);
        m_erp2pc_1.brake            = list[7].toUInt(&ok, 16);
        m_erp2pc_1.alive            = list[9].toUInt(&ok, 16);

        setFeedback1();
        emit set_str_ID1Changed();
        break;

    case ERP_ID_2:
        cout << "2" << endl;
        m_str_ID2 = QString::number(ERP_ID_2,16);

        m_erp2pc_2.Encoder[0]       = list[2].toUInt(&ok, 16);
        m_erp2pc_2.Encoder[1]       = list[3].toUInt(&ok, 16);
        m_erp2pc_2.Encoder[2]       = list[4].toUInt(&ok, 16);
        m_erp2pc_2.Encoder[3]       = list[5].toUInt(&ok, 16);
        m_erp2pc_2.Brake_Cmd_Raw    = list[6].toUInt(&ok, 16);
//        m_erp2pc_2.Brake_Raw        = list[7].toUInt(&ok, 16);
//        m_erp2pc_2.Brake_Echo       = list[8].toUInt(&ok, 16);
//        m_erp2pc_2.Brake_Init_Max   = list[9].toUInt(&ok, 16);

        setFeedback2();
        emit set_str_ID2Changed();
        break;

    // 2B0
//    case TEST_ID:
//        cout << "test" << endl;
//        break;

    default:
        cout << " None.. " << endl;
        break;
    }

}

void PCanManager::setFeedback1()
{
    m_FB_MorA           = m_erp2pc_1.MorA;
    m_FB_Estop          = m_erp2pc_1.ESTOP;
    m_FB_Gear           = m_erp2pc_1.GEAR;
    m_FB_Speed          = m_erp2pc_1.speed._speed;
    m_FB_SteerAngle     = m_erp2pc_1.steer._steer;
    m_FB_Brake          = m_erp2pc_1.brake;
    m_FB_Cycle          = m_erp2pc_1.alive;

    m_FB_str_QMorA  = QString::number(m_FB_MorA,  16);
    m_FB_str_ESTOP  = QString::number(m_FB_Estop, 16);
    m_FB_str_GEAR   = QString::number(m_FB_Gear,  16);
    m_FB_str_SPEED  = QString::number(static_cast<qint16>(m_FB_Speed / SPEED_FACTOR), 16);
    m_FB_str_STEER  = QString::number(static_cast<qint16>(m_FB_SteerAngle / STEER_FACTOR), 16);
    m_FB_str_BRAKE  = QString::number(m_FB_Brake, 16);
    m_FB_str_ALIVE  = QString::number(m_FB_Cycle, 16);

    m_FB_modified_str_Speed = QString::number(static_cast<qint16>(m_FB_Speed / SPEED_FACTOR));
    m_FB_modified_str_SteerAngle = QString::number(static_cast<qint16>(m_FB_SteerAngle / STEER_FACTOR));
    m_FB_modified_str_Brake = QString::number(m_FB_Brake);

    emit set_str_QMorAChanged();
    emit set_str_ESTOPChanged();
    emit set_str_GEARChanged();
    emit set_str_SPEEDChanged();
    emit set_str_STEERChanged();
    emit set_str_BRAKEChanged();
    emit set_str_ALIVEChanged();
}


void PCanManager::setFeedback2()
{
    m_FB_Encoder0   = m_erp2pc_2.Encoder[0];
    m_FB_Encoder1   = m_erp2pc_2.Encoder[1];
    m_FB_Encoder2   = m_erp2pc_2.Encoder[2];
    m_FB_Encoder3   = m_erp2pc_2.Encoder[3];
    m_FB_BCR        = m_erp2pc_2.Brake_Cmd_Raw;
    m_FB_BR         = m_erp2pc_2.Brake_Raw;
    m_FB_BE         = m_erp2pc_2.Brake_Echo;
    m_FB_BIM        = m_erp2pc_2.Brake_Init_Max;

    m_FB_str_Encoder0   = QString::number(m_FB_Encoder0, 16);
    m_FB_str_Encoder1   = QString::number(m_FB_Encoder1, 16);
    m_FB_str_Encoder2   = QString::number(m_FB_Encoder2, 16);
    m_FB_str_Encoder3   = QString::number(m_FB_Encoder3, 16);
    m_FB_str_BCR        = QString::number(m_FB_BCR, 16);
    m_FB_str_BR         = QString::number(m_FB_BR, 16);
    m_FB_str_BE         = QString::number(m_FB_BE, 16);
    m_FB_str_BIM        = QString::number(m_FB_BIM, 16);

    m_FB_modified_str_Encoder0   = QString::number(m_FB_Encoder0);
    m_FB_modified_str_Encoder1   = QString::number(m_FB_Encoder1);
    m_FB_modified_str_Encoder2   = QString::number(m_FB_Encoder2);
    m_FB_modified_str_Encoder3   = QString::number(m_FB_Encoder3);
    m_FB_modified_str_BCR        = QString::number(m_FB_BCR);
    m_FB_modified_str_BR         = QString::number(m_FB_BR);
    m_FB_modified_str_BE         = QString::number(m_FB_BE);
    m_FB_modified_str_BIM        = QString::number(m_FB_BIM);

    emit set_str_Encoder0Changed();
    emit set_str_Encoder1Changed();
    emit set_str_Encoder2Changed();
    emit set_str_Encoder3Changed();
    emit set_str_BCRChanged();
    emit set_str_BRChanged();
    emit set_str_BEChanged();
    emit set_str_BIMChanged();
}


void PCanManager::run()
{

    qDebug() << "Inside the worker thread!";
    quint8 alive_cnt = 0;
    m_stop = false;

    while(!m_stop)
    {
        alive_cnt++;

        m_pc2erp.MorA = m_MorA;
        m_pc2erp.ESTOP = m_Estop;
        m_pc2erp.GEAR = m_Gear;
        m_pc2erp.MODE = m_MorA + m_Estop + m_Gear;

        m_pc2erp.speed._speed = m_Speed * SPEED_FACTOR;
        m_pc2erp.steer._steer = (m_SteerAngle * STEER_FACTOR);

        m_pc2erp.speed.speed[0] = (m_pc2erp.speed._speed & 0xff);
        m_pc2erp.speed.speed[1] = (m_pc2erp.speed._speed & 0xff00) >> 8;

        m_pc2erp.steer.steer[0] = (m_pc2erp.steer._steer & 0xff);
        m_pc2erp.steer.steer[1] = (m_pc2erp.steer._steer & 0xff00) >> 8;

        m_pc2erp.brake = m_Brake;
        m_pc2erp.alive = alive_cnt;

//        showData(m_pc2erp);
        setData(m_pc2erp.MODE);
        m_str_ALIVE = QString::number(alive_cnt,16);
        emit get_str_ALIVEChanged();

        if (!CanManager::m_TextArea.isNull())
        {
            getFeedback();
        }

        msleep(m_Cycle);
    }
}




