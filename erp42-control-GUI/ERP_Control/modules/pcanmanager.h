#ifndef PCanManager_H
#define PCanManager_H
//#include "canmanager.h"
#include <QTimer>
#include <QThread>
#include <QDebug>
#include <iostream>
#include <string>

const quint8 SPEED_FACTOR=10;
const quint8 STEER_FACTOR=71;

typedef struct _pc_to_erp42
{
    quint8 MODE;
    quint8 MorA;
    quint8 ESTOP;
    quint8 GEAR;
    union speed{ quint8 speed[2]; quint16 _speed;}; union speed speed;
    union steer{ qint8 steer[2]; qint16 _steer;}; union steer steer;
    quint8 brake = 0;
    quint8 alive = 0;

}PC2ERP;

typedef struct _erp42_to_pc_1
{
    quint8 MODE;
    quint8 MorA;
    quint8 ESTOP;
    quint8 GEAR;
    union speed{ quint8 speed[2]; quint16 _speed;}; union speed speed;
    union steer{ qint8 steer[2]; qint16 _steer;}; union steer steer;
    quint8 brake = 0;
    quint8 alive = 0;
}ERP2PC_1;

typedef struct _erp42_to_pc_2
{
    qint32 Encoder[4];
    quint8 Brake_Cmd_Raw;
    quint8 Brake_Raw;
    quint8 Brake_Echo;
    quint8 Brake_Init_Max;
}ERP2PC_2;


class PCanManager : public QThread
{

    Q_OBJECT

public:
    explicit PCanManager(QObject *parent = nullptr);
    virtual ~PCanManager() noexcept
    {
        quit();
        wait();
    }

    enum Mode{Mannaul = 0x00, Auto = 0x01};
    enum Estop{Off = 0x00, On = 0x02};
    enum Gear{D = 0x00, N = 0x04, R = 0x08};
    enum CANID
    {
        ERP_ID_1 = 0x778,
        ERP_ID_2 = 0x779,
        TEST_ID  = 0x2B0
    };

    Q_ENUM(Mode)
    Q_ENUM(Estop)
    Q_ENUM(Gear)

    /* Set UI */
    Q_PROPERTY(bool Active READ getActive WRITE setActive NOTIFY ActiveChanged)
    Q_PROPERTY(bool AutoEnable READ getAutoEnable
               WRITE setAutoEnable NOTIFY AutoEnableChanged)
    Q_PROPERTY(bool EstopEnable READ getEstopEnable
               WRITE setEstopEnable NOTIFY EstopEnableChanged)
    Q_PROPERTY(bool SteerEnable READ getSteerEnable
               WRITE setSteerEnable NOTIFY SteerEnableChanged)
    Q_PROPERTY(bool SpeedEnable READ getSpeedEnable
               WRITE setSpeedEnable NOTIFY SpeedEnableChanged)
    Q_PROPERTY(bool BrakeEnable READ getBrakeEnable
               WRITE setBrakeEnable NOTIFY BrakeEnableChanged)
    Q_PROPERTY(bool GearDrive READ getGearDrive
               WRITE setGearDrive NOTIFY GearDriveChanged)
    Q_PROPERTY(bool GearNeutral READ getGearNeutral
               WRITE setGearNeutral NOTIFY GearNeutralChanged)
    Q_PROPERTY(bool GearReverse READ getGearReverse
               WRITE setGearReverse NOTIFY GearReverseChanged)
    Q_PROPERTY(qint16 SteerAngle READ getSteerAngle
               WRITE setSteerAngle NOTIFY SteerAngleChanged)
    Q_PROPERTY(quint16 Speed READ getSpeed
               WRITE setSpeed NOTIFY SpeedChanged)
    Q_PROPERTY(quint8 Brake READ getBrake
               WRITE setBrake NOTIFY BrakeChanged)
    Q_PROPERTY(quint16 Cycle READ getCycle
               WRITE setCycle NOTIFY CycleChanged)

    /* PC to ERP */
    Q_PROPERTY(QString get_QMorA READ get_str_QMorA NOTIFY get_str_QMorAChanged)
    Q_PROPERTY(QString get_ESTOP READ get_str_ESTOP NOTIFY get_str_ESTOPChanged)
    Q_PROPERTY(QString get_GEAR  READ get_str_GEAR  NOTIFY get_str_GEARChanged)
    Q_PROPERTY(QString get_SPEED READ get_str_SPEED NOTIFY get_str_SPEEDChanged)
    Q_PROPERTY(QString get_STEER READ get_str_STEER NOTIFY get_str_STEERChanged)
    Q_PROPERTY(QString get_BRAKE READ get_str_BRAKE NOTIFY get_str_BRAKEChanged)
    Q_PROPERTY(QString get_ALIVE READ get_str_ALIVE NOTIFY get_str_ALIVEChanged)

    Q_PROPERTY(QString get_modified_SPEED READ get_modified_str_SPEED NOTIFY get_str_SPEEDChanged)
    Q_PROPERTY(QString get_modified_STEER READ get_modified_str_STEER NOTIFY get_str_STEERChanged)
    Q_PROPERTY(QString get_modified_BRAKE READ get_modified_str_BRAKE NOTIFY get_str_BRAKEChanged)

    /* ERP to PC */

    // ID
    Q_PROPERTY(QString set_ID1 READ set_str_ID1 NOTIFY set_str_ID1Changed)
    Q_PROPERTY(QString set_ID2 READ set_str_ID2 NOTIFY set_str_ID2Changed)

    // FeedBack 1
    Q_PROPERTY(QString set_QMorA READ set_str_QMorA NOTIFY set_str_QMorAChanged)
    Q_PROPERTY(QString set_ESTOP READ set_str_ESTOP NOTIFY set_str_ESTOPChanged)
    Q_PROPERTY(QString set_GEAR  READ set_str_GEAR  NOTIFY set_str_GEARChanged)
    Q_PROPERTY(QString set_SPEED READ set_str_SPEED NOTIFY set_str_SPEEDChanged)
    Q_PROPERTY(QString set_STEER READ set_str_STEER NOTIFY set_str_STEERChanged)
    Q_PROPERTY(QString set_BRAKE READ set_str_BRAKE NOTIFY set_str_BRAKEChanged)
    Q_PROPERTY(QString set_ALIVE READ set_str_ALIVE NOTIFY set_str_ALIVEChanged)

    Q_PROPERTY(QString set_modified_SPEED READ set_modified_str_SPEED NOTIFY set_str_SPEEDChanged)
    Q_PROPERTY(QString set_modified_STEER READ set_modified_str_STEER NOTIFY set_str_STEERChanged)
    Q_PROPERTY(QString set_modified_BRAKE READ set_modified_str_BRAKE NOTIFY set_str_BRAKEChanged)

    // FeedBack 2
    Q_PROPERTY(QString set_Encoder0 READ set_str_Encoder0 NOTIFY set_str_Encoder0Changed)
    Q_PROPERTY(QString set_Encoder1 READ set_str_Encoder1 NOTIFY set_str_Encoder1Changed)
    Q_PROPERTY(QString set_Encoder2 READ set_str_Encoder2 NOTIFY set_str_Encoder2Changed)
    Q_PROPERTY(QString set_Encoder3 READ set_str_Encoder3 NOTIFY set_str_Encoder3Changed)
    Q_PROPERTY(QString set_BCR READ set_str_BCR NOTIFY set_str_BCRChanged)
    Q_PROPERTY(QString set_BR READ set_str_BR NOTIFY set_str_BRChanged)
    Q_PROPERTY(QString set_BE READ set_str_BE NOTIFY set_str_BEChanged)
    Q_PROPERTY(QString set_BIM READ set_str_BIM NOTIFY set_str_BIMChanged)

    Q_PROPERTY(QString set_modified_Encoder0 READ set_modified_str_Encoder0 NOTIFY set_str_Encoder0Changed)
    Q_PROPERTY(QString set_modified_Encoder1 READ set_modified_str_Encoder1 NOTIFY set_str_Encoder1Changed)
    Q_PROPERTY(QString set_modified_Encoder2 READ set_modified_str_Encoder2 NOTIFY set_str_Encoder2Changed)
    Q_PROPERTY(QString set_modified_Encoder3 READ set_modified_str_Encoder3 NOTIFY set_str_Encoder3Changed)
    Q_PROPERTY(QString set_modified_BCR      READ set_modified_str_BCR      NOTIFY set_str_BCRChanged)
    Q_PROPERTY(QString set_modified_BR       READ set_modified_str_BR       NOTIFY set_str_BRChanged)
    Q_PROPERTY(QString set_modified_BE       READ set_modified_str_BE       NOTIFY set_str_BEChanged)
    Q_PROPERTY(QString set_modified_BIM      READ set_modified_str_BIM      NOTIFY set_str_BIMChanged)


    /* TEST */
    Q_PROPERTY(QVariant data READ getData WRITE setData NOTIFY dataChanged )

    Q_INVOKABLE void setActive(const bool &);
    Q_INVOKABLE void setAutoEnable(const bool &);
    Q_INVOKABLE void setEstopEnable(const bool &);
    Q_INVOKABLE void setSteerEnable(const bool &);
    Q_INVOKABLE void setSpeedEnable(const bool &);
    Q_INVOKABLE void setBrakeEnable(const bool &);
    Q_INVOKABLE void setGearDrive(const bool &);
    Q_INVOKABLE void setGearNeutral(const bool &);
    Q_INVOKABLE void setGearReverse(const bool &);
    Q_INVOKABLE void setSteerAngle(const qint16 &);
    Q_INVOKABLE void setSpeed(const quint16 &);
    Q_INVOKABLE void setBrake(const quint8 &);
    Q_INVOKABLE void setCycle(const quint16 &);

//    Q_INVOKABLE void setQMorA(const QString &);
    Q_INVOKABLE QVariant getData() const { return data_; }

    bool getActive() const {return m_ActiveEnable;}
    bool getAutoEnable() const {return m_AutoEnable;}
    bool getEstopEnable() const {return m_EstopEnable;}
    bool getSteerEnable() const {return m_SteerEnable;}
    bool getSpeedEnable() const {return m_SpeedEnable;}
    bool getBrakeEnable() const {return m_Brake;}
    bool getGearDrive() const {return m_GearDrive;}
    bool getGearNeutral() const {return m_GearNeutral;}
    bool getGearReverse() const {return m_GearReverse;}

    qint16  getSteerAngle() const {return m_SteerAngle;}
    quint16 getSpeed() const {return m_Speed;}
    quint8  getBrake() const {return m_Brake;}
    quint16 getCycle() const {return m_Cycle;}

    QString get_str_QMorA() const {return m_str_QMorA;}
    QString get_str_ESTOP() const {return m_str_ESTOP;}
    QString get_str_GEAR()  const {return m_str_GEAR;}
    QString get_str_SPEED() const {return m_str_SPEED;}
    QString get_str_STEER() const {return m_str_STEER;}
    QString get_str_BRAKE() const {return m_str_BRAKE;}
    QString get_str_ALIVE() const {return m_str_ALIVE;}

    QString get_modified_str_SPEED() const {return m_modified_str_Speed;}
    QString get_modified_str_STEER() const {return m_modified_str_SteerAngle;}
    QString get_modified_str_BRAKE() const {return m_modified_str_Brake;}


    QString set_str_ID1() const {return m_str_ID1;}
    QString set_str_ID2() const {return m_str_ID2;}

    QString set_str_QMorA() const {return m_FB_str_QMorA;}
    QString set_str_ESTOP() const {return m_FB_str_ESTOP;}
    QString set_str_GEAR()  const {return m_FB_str_GEAR;}
    QString set_str_SPEED() const {return m_FB_str_SPEED;}
    QString set_str_STEER() const {return m_FB_str_STEER;}
    QString set_str_BRAKE() const {return m_FB_str_BRAKE;}
    QString set_str_ALIVE() const {return m_FB_str_ALIVE;}

    QString set_modified_str_SPEED() const {return m_FB_modified_str_Speed;}
    QString set_modified_str_STEER() const {return m_FB_modified_str_SteerAngle;}
    QString set_modified_str_BRAKE() const {return m_FB_modified_str_Brake;}


    QString set_str_Encoder0() const {return m_FB_str_Encoder0;}
    QString set_str_Encoder1() const {return m_FB_str_Encoder1;}
    QString set_str_Encoder2() const {return m_FB_str_Encoder2;}
    QString set_str_Encoder3() const {return m_FB_str_Encoder3;}
    QString set_str_BCR()      const {return m_FB_str_BCR;}
    QString set_str_BR()       const {return m_FB_str_BR;}
    QString set_str_BE()       const {return m_FB_str_BE;}
    QString set_str_BIM()      const {return m_FB_str_BIM;}

    QString set_modified_str_Encoder0() const {return m_FB_modified_str_Encoder0;}
    QString set_modified_str_Encoder1() const {return m_FB_modified_str_Encoder1;}
    QString set_modified_str_Encoder2() const {return m_FB_modified_str_Encoder2;}
    QString set_modified_str_Encoder3() const {return m_FB_modified_str_Encoder3;}
    QString set_modified_str_BCR()      const {return m_FB_modified_str_BCR;}
    QString set_modified_str_BR()       const {return m_FB_modified_str_BR;}
    QString set_modified_str_BE()       const {return m_FB_modified_str_BE;}
    QString set_modified_str_BIM()      const {return m_FB_modified_str_BIM;}

    QString getData(QString m_getData) const {return m_getData;}

    static PC2ERP m_pc2erp;

    quint8 m_MorA;
    quint8 m_Estop;
    quint8 m_Gear;
    qint16 m_SteerAngle;
    quint16 m_Speed;
    quint8 m_Brake;
    quint16 m_Cycle;



    bool m_stop;
    void quit() { m_stop = true; }

private:
    bool m_ActiveEnable;
    bool m_AutoEnable;
    bool m_EstopEnable;
    bool m_SteerEnable;
    bool m_SpeedEnable;
    bool m_BrakeEnable;
    bool m_GearDrive;
    bool m_GearNeutral;
    bool m_GearReverse;

    QString m_str_ID1;
    QString m_str_ID2;

    QString m_str_QMorA;
    QString m_str_ESTOP;
    QString m_str_GEAR;
    QString m_str_SPEED;
    QString m_str_STEER;
    QString m_str_BRAKE;
    QString m_str_ALIVE;

    QString m_modified_str_SteerAngle;
    QString m_modified_str_Speed;
    QString m_modified_str_Brake;

    quint8  m_FB_MorA;
    quint8  m_FB_Estop;
    quint8  m_FB_Gear;
    qint16  m_FB_SteerAngle;
    quint16 m_FB_Speed;
    quint8  m_FB_Brake;
    quint16 m_FB_Cycle;

    QString m_FB_str_QMorA;
    QString m_FB_str_ESTOP;
    QString m_FB_str_GEAR;
    QString m_FB_str_SPEED;
    QString m_FB_str_STEER;
    QString m_FB_str_BRAKE;
    QString m_FB_str_ALIVE;

    QString m_FB_modified_str_SteerAngle;
    QString m_FB_modified_str_Speed;
    QString m_FB_modified_str_Brake;

    qint32 m_FB_Encoder0;
    qint32 m_FB_Encoder1;
    qint32 m_FB_Encoder2;
    qint32 m_FB_Encoder3;
    quint8 m_FB_BCR;
    quint8 m_FB_BR;
    quint8 m_FB_BE;
    quint8 m_FB_BIM;

    QString m_FB_str_Encoder0;
    QString m_FB_str_Encoder1;
    QString m_FB_str_Encoder2;
    QString m_FB_str_Encoder3;
    QString m_FB_str_BCR;
    QString m_FB_str_BR;
    QString m_FB_str_BE;
    QString m_FB_str_BIM;

    QString m_FB_modified_str_Encoder0;
    QString m_FB_modified_str_Encoder1;
    QString m_FB_modified_str_Encoder2;
    QString m_FB_modified_str_Encoder3;
    QString m_FB_modified_str_BCR;
    QString m_FB_modified_str_BR;
    QString m_FB_modified_str_BE;
    QString m_FB_modified_str_BIM;


    const QString m_getData;

//    quint16 m_getSteerData;

    QVariant data_{ "" };
    ERP2PC_1 m_erp2pc_1;
    ERP2PC_2 m_erp2pc_2;

    /* Thread */
    void run();

    //    quint16 getSteerData(quint16 m_getSteerData) const {return m_getSteerData;}

    void showData(PC2ERP m_pc2erp) const
    {
        std::cout <<" MODE: "  << +m_pc2erp.MODE
                  <<" MorA: "  << +m_pc2erp.MorA
                  <<" ESTOP: " << +m_pc2erp.ESTOP
                  <<" GEAR: "  << +m_pc2erp.GEAR
                  <<" speed: " << m_pc2erp.speed._speed
                  <<" steer: " << m_pc2erp.steer._steer
                  <<std::hex<<" steer[0]: " << +m_pc2erp.steer.steer[0]
                  <<std::hex<<" steer[1]: " << +m_pc2erp.steer.steer[1]
                  <<" brake: " << +m_pc2erp.brake
                  <<" alive: " << +m_pc2erp.alive
                  << std::endl;
    }

    void getFeedback();
    void setFeedback1();
    void setFeedback2();


signals:
    void ActiveChanged();
    void AutoEnableChanged();
    void EstopEnableChanged();
    void SteerEnableChanged();
    void SpeedEnableChanged();
    void BrakeEnableChanged();
    void GearDriveChanged();
    void GearNeutralChanged();
    void GearReverseChanged();
    void SteerAngleChanged();
    void SpeedChanged();
    void BrakeChanged();

    void CycleChanged();

    void set_str_ID1Changed();
    void set_str_ID2Changed();

    void get_str_QMorAChanged();
    void get_str_ESTOPChanged();
    void get_str_GEARChanged();
    void get_str_SPEEDChanged();
    void get_str_STEERChanged();
    void get_str_BRAKEChanged();
    void get_str_ALIVEChanged();

    void set_str_QMorAChanged();
    void set_str_ESTOPChanged();
    void set_str_GEARChanged();
    void set_str_SPEEDChanged();
    void set_str_STEERChanged();
    void set_str_BRAKEChanged();
    void set_str_ALIVEChanged();

    void set_str_Encoder0Changed();
    void set_str_Encoder1Changed();
    void set_str_Encoder2Changed();
    void set_str_Encoder3Changed();
    void set_str_BCRChanged();
    void set_str_BRChanged();
    void set_str_BEChanged();
    void set_str_BIMChanged();



    void dataChanged( const QVariant data );

private slots:
    void setData( const QVariant data ) {
        if ( data != data_ ) {
            data_ = data;
            emit dataChanged( data_ );
        }
    }



};
#endif // PCanManager_H
