#ifndef WEBSOCKETSERVER_H
#define WEBSOCKETSERVER_H

#include <QWebSocketServer>
#include <QtDebug>
#include <mutex>

using namespace std;

class PlatformControlP1;
class PlatformControlP2;
class PlatformLocationP1;
class PlatformManipulatorAndIRBumper;
class BodyControlP1;
class ArmControlRight;
class ArmControlLeft;

class WebSocketServer: public QObject
{
public:
    static WebSocketServer *getInstance();
    ~WebSocketServer();

private:
    WebSocketServer();
    static WebSocketServer* pWebSocketServer;       // WebSocketServer's singleton instance
    static bool instanceFlag;

    QWebSocketServer *m_pWebSocketServer;
    QList<QWebSocket *> m_clients;

    void watchDogWorker();
    bool watchDogActivated;
    int watchCnt;
    int watchDogSleep;
    std::mutex watchDog_mutex;

    PlatformControlP1 *platformControlP1;
    PlatformControlP2 *platformControlP2;
    PlatformLocationP1 *platformLocationP1;
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper;
    BodyControlP1 *bodyControlP1;
    ArmControlRight *armControlRight;
    ArmControlLeft *armControlLeft;

    void stopAllModules();

private Q_SLOTS:
    void onNewConnection();
    void processTextMessage(QString message);
//    void processBinaryMessage(QByteArray message);
    void socketDisconnected();
    void onClosed();
};

#endif // WEBSOCKETSERVER_H
