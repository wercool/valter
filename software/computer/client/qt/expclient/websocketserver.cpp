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

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("SRV"))
    {
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

        return;
    }

    //PLATFORM-CONTROL-P1 direct commands **********************************BEGIN**************************************************
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("PCP1"))
    {
        switch (Valter::str2int(cmdValue.c_str()))
        {
            case Valter::str2int("LRF"): //LeftRightForward
                if (platformControlP1->preparePlatformMovement())
                {
                    //left and right forward
                    if (platformControlP1->setLeftMotorDirection(true) && platformControlP1->setRightMotorDirection(true))
                    {
                        platformControlP1->setLeftMotorActivated(true);
                        platformControlP1->setRightMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("RF"): //RightForward
                if (platformControlP1->preparePlatformMovement())
                {
                    //right forward
                    if (platformControlP1->setRightMotorDirection(true))
                    {
                        platformControlP1->setRightMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("LF"): //LeftForward
                if (platformControlP1->preparePlatformMovement())
                {
                    //left forward
                    if (platformControlP1->setLeftMotorDirection(true))
                    {
                        platformControlP1->setLeftMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("LRB"): //LeftRightBackward
                if (platformControlP1->preparePlatformMovement())
                {
                    //left and right backward
                    if (platformControlP1->setLeftMotorDirection(false) && platformControlP1->setRightMotorDirection(false))
                    {
                        platformControlP1->setLeftMotorActivated(true);
                        platformControlP1->setRightMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("RB"): //RightBackward
                if (platformControlP1->preparePlatformMovement())
                {
                    //right backward
                    if (platformControlP1->setRightMotorDirection(false))
                    {
                        platformControlP1->setRightMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("LB"): //LeftBackward
                if (platformControlP1->preparePlatformMovement())
                {
                    //left backward
                    if (platformControlP1->setLeftMotorDirection(false))
                    {
                        platformControlP1->setLeftMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("RL"): //RotateLeft
                if (platformControlP1->preparePlatformMovement())
                {
                    //left backward right forward
                    if (platformControlP1->setLeftMotorDirection(false) && platformControlP1->setRightMotorDirection(true))
                    {
                        platformControlP1->setLeftMotorActivated(true);
                        platformControlP1->setRightMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("RR"): //RotateRight
                if (platformControlP1->preparePlatformMovement())
                {
                    //left forward right backward
                    if (platformControlP1->setLeftMotorDirection(true) && platformControlP1->setRightMotorDirection(false))
                    {
                        platformControlP1->setLeftMotorActivated(true);
                        platformControlP1->setRightMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("LRD"): //LeftRightDeactivate
                platformControlP1->setLeftMotorActivated(false);
                platformControlP1->setRightMotorActivated(false);
            break;
            case Valter::str2int("LD"): //LeftDeactivate
                platformControlP1->setLeftMotorActivated(false);
            break;
            case Valter::str2int("RD"): //RightDeactivate
                platformControlP1->setRightMotorActivated(false);
            break;
            case Valter::str2int("STOP"): //Emergency Stop
                platformControlP1->setPlatformEmergencyStop(true);
            break;
        }
    }
    //PLATFORM-CONTROL-P1 direct commands **********************************END***************************************************

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
