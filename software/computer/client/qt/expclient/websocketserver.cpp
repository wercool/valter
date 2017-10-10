#include "valter.h"
#include "websocketserver.h"

#include "QtWebSockets/qwebsocketserver.h"
#include "QtWebSockets/qwebsocket.h"

WebSocketServer *WebSocketServer::pWebSocketServer = NULL;
bool WebSocketServer::instanceFlag = false;

WebSocketServer::WebSocketServer()
{
   m_pWebSocketServer = new QWebSocketServer(QStringLiteral("Valter WebSocket Server"), QWebSocketServer::NonSecureMode, this);

   if (m_pWebSocketServer->listen(QHostAddress::Any, quint16(8888)))
   {
       qDebug() << "WebSocketServer listening on port" << m_pWebSocketServer->serverPort();
       connect(m_pWebSocketServer, &QWebSocketServer::newConnection, this, &WebSocketServer::onNewConnection);
       connect(m_pWebSocketServer, &QWebSocketServer::closed, this, &WebSocketServer::onClosed);
   }
}

WebSocketServer *WebSocketServer::getInstance()
{
    if(!instanceFlag)
    {
        pWebSocketServer = new WebSocketServer();
        instanceFlag = true;
        return pWebSocketServer;
    }
    else
    {
        return pWebSocketServer;
    }
}

WebSocketServer::~WebSocketServer()
{
    m_pWebSocketServer->close();
    qDeleteAll(m_clients.begin(), m_clients.end());
}

void WebSocketServer::onNewConnection()
{
    QWebSocket *pSocket = m_pWebSocketServer->nextPendingConnection();

    connect(pSocket, &QWebSocket::textMessageReceived, this, &WebSocketServer::processTextMessage);
//    connect(pSocket, &QWebSocket::binaryMessageReceived, this, &WebSocketServer::processBinaryMessage);
    connect(pSocket, &QWebSocket::disconnected, this, &WebSocketServer::socketDisconnected);

    qDebug() << "socketConnected:" << pSocket;

    m_clients << pSocket;
}

void WebSocketServer::processTextMessage(QString message)
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());

    qDebug() << "Message received:" << message;

    vector<string>message_str_values = Valter::split(message.toStdString() , '#');

    std::string cmdType = message_str_values[0];
    std::string cmdValue = message_str_values[1];
    std::string cmdResponse = "OK";

    switch (Valter::str2int(cmdValue.c_str()))
    {
        case Valter::str2int("CLIENT READY"):
            cmdResponse = "SERVER READY";
        break;
    }

    if (pClient)
    {
        pClient->sendTextMessage(cmdResponse.c_str());
    }
}

//void WebSocketServer::processBinaryMessage(QByteArray message)
//{
//    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());

//    qDebug() << "Binary Message received:" << message;
//    if (pClient)
//    {
//        pClient->sendBinaryMessage(message);
//    }
//}

void WebSocketServer::socketDisconnected()
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());

    qDebug() << "socketDisconnected:" << pClient;
    if (pClient)
    {
        m_clients.removeAll(pClient);
        pClient->deleteLater();
    }
}

void WebSocketServer::onClosed()
{
    qDebug() << "Valter WebSocket Server closed!";
}
