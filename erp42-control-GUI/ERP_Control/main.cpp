#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include <QStringList>
#include <QLoggingCategory>

#include "modules/comboboxmodel.h"
#include "modules/canmanager.h"
#include "modules/pcanmanager.h"
#include "modules/connect.h"
#include <iostream>
#include <QThread>

int main(int argc, char** argv)
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QLoggingCategory::setFilterRules(QStringLiteral("qt.canbus* = true"));
    QGuiApplication app(argc, argv);

    std::shared_ptr<CanManager> canManager = std::make_shared<CanManager>();
    std::shared_ptr<PCanManager> pcanManager = std::make_shared<PCanManager>();

    QQmlApplicationEngine engine;
    qmlRegisterType<CanManager>("unmansol.erp42.canmanager", 0, 1, "CanManager");
    engine.rootContext()->setContextProperty("canManager", canManager.get());


    qmlRegisterType<PCanManager>("unmansol.erp42.pcanmanager", 0, 1, "PCanManager");
    engine.rootContext()->setContextProperty("pcanManager", pcanManager.get());


    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.load(url);

    canManager->connectDevice();
    pcanManager->start();

    return app.exec();
}



