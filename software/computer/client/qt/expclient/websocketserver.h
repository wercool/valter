#ifndef WEBSOCKETSERVER_H
#define WEBSOCKETSERVER_H

#include <QWebSocketServer>
#include <QtDebug>
#include <mutex>

using namespace std;

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
    std::mutex watchDog_mutex;


private Q_SLOTS:
    void onNewConnection();
    void processTextMessage(QString message);
//    void processBinaryMessage(QByteArray message);
    void socketDisconnected();
    void onClosed();
};

#endif // WEBSOCKETSERVER_H
