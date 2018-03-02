#include "valter.h"
#include "websocketserver.h"

#include "QtWebSockets/qwebsocketserver.h"
#include "QtWebSockets/qwebsocket.h"

WebSocketServer *WebSocketServer::pWebSocketServer = NULL;
bool WebSocketServer::instanceFlag = false;

WebSocketServer::WebSocketServer()
{
   m_pWebSocketServer = new QWebSocketServer(QStringLiteral("Valter WebSocket Server"), QWebSocketServer::NonSecureMode, this);

   watchDogActivated = false;
   watchCnt = 0;
   watchDogSleep = 1000;
   new std::thread(&WebSocketServer::watchDogWorker, this);
   Valter::log("WebSocketServer watchDogWorker spawned...");

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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
//    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();

    //Stop selected modules
    platformControlP1->stopAll();
    qDebug() << "WebSocketServer::watchDogWorker platformControlP1 STOPPED";
    bodyControlP1->stopAll();
    qDebug() << "WebSocketServer::watchDogWorker bodyControlP1 STOPPED";

    platformControlP2->setBeepDuration(50);
    platformControlP2->alarmBeep();

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
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();

    //Service commands
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("SRV"))
    {
        switch (Valter::str2int(cmdValue.c_str()))
        {
            case Valter::str2int("CLIENT READY"):
                cmdResponse = "SERVER READY";
            break;
        }

        if (cmdValue.find("WDIN") != std::string::npos)
        {
            vector<string>cmdValue_str_values = Valter::split(cmdValue , ':');
            std::string cmdValue = cmdValue_str_values[1];
            cmdResponse = Valter::format_string("SRV#WDOUT:%s", cmdValue.c_str());

            std::lock_guard<std::mutex> guard(watchDog_mutex);
            watchDogActivated = true;
            watchCnt = 3;
        }

        if (pClient)
        {
            pClient->sendTextMessage(cmdResponse.c_str());
        }

        return;
    }

    if (watchCnt < 0)
    {
        platformControlP2->setBeepDuration(25);
        platformControlP2->alarmBeep();
        return;
    }

    //Valter Tasks
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("TASK"))
    {
        TaskManager::getInstance()->processScript(cmdValue.c_str());
        return;
    }

    //Valter Perephiral Controls (WebCams, ...)
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("CTRL"))
    {
        switch (Valter::str2int(cmdValue.c_str()))
        {
            case Valter::str2int("FCAMON"): //Frontal Camera ON
                Valter::getInstance()->executeShellCmdLinuxAndDetach("/home/maska/startFrontalVideo");
            break;
            case Valter::str2int("FCAMOFF"): //Frontal Camera OFF
                Valter::getInstance()->executeShellCmdLinuxAndDetach("sudo killall mjpg_streamer_frontal_camera");
            break;
            case Valter::str2int("RCAMON"): //Rear Camera ON
                Valter::getInstance()->executeShellCmdLinuxAndDetach("/home/maska/startRearVideo");
            break;
            case Valter::str2int("RCAMOFF"): //Rear Camera OFF
                Valter::getInstance()->executeShellCmdLinuxAndDetach("sudo killall mjpg_streamer_rear_camera");
            break;
            case Valter::str2int("STARTFRONTMIC"): //Start front microphone stream
                Valter::getInstance()->executeShellCmdLinuxAndDetach("/home/maska/startFrontMicStream");
            break;
            case Valter::str2int("STOPFRONTMIC"): //Start front microphone stream
                Valter::getInstance()->executeShellCmdLinuxAndDetach("sudo killall avconv");
            break;
        }
    }

    /**********************************PLATFORM-CONTROL-P1***************************************************/
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
                //Stop selected modules
                platformControlP1->stopAll();
                qDebug() << "WebSocketServer::watchDogWorker platformControlP1 STOPPED";
                bodyControlP1->stopAll();
                qDebug() << "WebSocketServer::watchDogWorker bodyControlP1 STOPPED";
            break;
        }

        //RightMotorDuty
        if (cmdValue.find("RMD") != std::string::npos)
        {
            vector<string>cmdValue_str_values = Valter::split(cmdValue , '=');
            std::string cmdValue = cmdValue_str_values[1];
            int value = atoi(cmdValue.c_str());
            platformControlP1->setRightMotorDutyMax(value);
        }
        //LeftMotorDuty
        if (cmdValue.find("LMD") != std::string::npos)
        {
            vector<string>cmdValue_str_values = Valter::split(cmdValue , '=');
            std::string cmdValue = cmdValue_str_values[1];
            int value = atoi(cmdValue.c_str());
            platformControlP1->setLeftMotorDutyMax(value);
        }
        return;
    }

    /*********************************PLATFORM-LOCATION-P1***************************************************/
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("PLP1"))
    {
        switch (Valter::str2int(cmdValue.c_str()))
        {
            case Valter::str2int("SONARLEDSON"): //Sonar leds ON
                platformLocationP1->setAllSonarsLedsState(true);
            break;
            case Valter::str2int("SONARLEDSOFF"): //Sonar leds OFF
                platformLocationP1->setAllSonarsLedsState(false);
            break;
        }
    }

    /*********************************BODY-CONTROL-P1***************************************************/
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("BCP1"))
    {
        switch (Valter::str2int(cmdValue.c_str()))
        {
            case Valter::str2int("HEADLEDON"): //Head led ON
                bodyControlP1->setHeadLedOnOff(true);
            break;
            case Valter::str2int("HEADLEDOFF"): //Head led OFF
                bodyControlP1->setHeadLedOnOff(false);
            break;
            case Valter::str2int("HEAD24VON"): //Head 24V ON
                bodyControlP1->setHead24VOnOff(true);
            break;
            case Valter::str2int("HEAD24VOFF"): //Head 24V OFF
                bodyControlP1->setHead24VOnOff(false);
            break;
            case Valter::str2int("LEFTACCON"): //Left Accumulator connected
                bodyControlP1->setLeftAccumulatorOnOff(true);
            break;
            case Valter::str2int("LEFTACCOFF"): //Left Accumulator disconnected
                bodyControlP1->setLeftAccumulatorOnOff(false);
            break;
            case Valter::str2int("RIGHTACCON"): //Right Accumulator connected
                bodyControlP1->setRightAccumulatorOnOff(true);
            break;
            case Valter::str2int("RIGHTACCOFF"): //Right Accumulator disconnected
                bodyControlP1->setRightAccumulatorOnOff(false);
            break;
            case Valter::str2int("HEADYAWENABLE"): //Head YAW ON
                bodyControlP1->setHeadYawMotorOnOff(true);
            break;
            case Valter::str2int("HEADYAWDISABLE"): //Head YAW OFF
                bodyControlP1->setHeadYawMotorOnOff(false);
            break;
            case Valter::str2int("HEADPITCHENABLE"): //Head PITCH ON
                bodyControlP1->setHeadPitchMotorOnOff(true);
            break;
            case Valter::str2int("HEADPITCHDISABLE"): //Head PITCH OFF
                bodyControlP1->setHeadPitchMotorOnOff(false);
            break;
            case Valter::str2int("HEADMOTORSACTIVATE"): //Head MOTORS ACTIVATED
                bodyControlP1->setHead24VOnOff(true);
                bodyControlP1->setHeadYawMotorOnOff(true);
                bodyControlP1->setHeadPitchMotorOnOff(true);
            break;
            case Valter::str2int("HEADMOTORSDEACTIVATE"): //Head MOTORS DEACTIVATED
                bodyControlP1->setHeadPitchMotorOnOff(false);
                bodyControlP1->setHeadYawMotorOnOff(false);
                bodyControlP1->setHead24VOnOff(false);
            break;
            case Valter::str2int("HEADYAWRIGHT"): //Head YAW RIGHT
                watchDogSleep = 500;

                TaskManager::getInstance()->stopTasksByName("SetHeadYawPositionTask");
                bodyControlP1->setHeadYawDirection(true);
                bodyControlP1->setHeadYawMotorActivated(true);
            break;
            case Valter::str2int("HEADYAWLEFT"): //Head YAW LEFT
                watchDogSleep = 500;

                TaskManager::getInstance()->stopTasksByName("SetHeadYawPositionTask");
                bodyControlP1->setHeadYawDirection(false);
                bodyControlP1->setHeadYawMotorActivated(true);
            break;
            case Valter::str2int("HEADYAWDONE"): //Head YAW DONE
                watchDogSleep = 1000;

                bodyControlP1->setHeadYawMotorActivated(false);
            break;
            case Valter::str2int("HEADPITCHDOWN"): //Head PITCH DOWN
                watchDogSleep = 500;

                TaskManager::getInstance()->stopTasksByName("SetHeadPitchPositionTask");
                bodyControlP1->setHeadPitchDirection(true);
                bodyControlP1->setHeadPitchMotorActivated(true);
            break;
            case Valter::str2int("HEADPITCHUP"): //Head PITCH UP
                watchDogSleep = 500;

                TaskManager::getInstance()->stopTasksByName("SetHeadPitchPositionTask");
                bodyControlP1->setHeadPitchDirection(false);
                bodyControlP1->setHeadPitchMotorActivated(true);
            break;
            case Valter::str2int("HEADPITCHDONE"): //Head PITCH DONE
                watchDogSleep = 1000;

                bodyControlP1->setHeadPitchMotorActivated(false);
            break;
        }
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

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
//    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();

    //Stop selected modules
    platformControlP1->stopAll();
    qDebug() << "WebSocketServer::watchDogWorker platformControlP1 STOPPED";
    bodyControlP1->stopAll();
    qDebug() << "WebSocketServer::watchDogWorker bodyControlP1 STOPPED";

    watchDogActivated = false;

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

void WebSocketServer::watchDogWorker()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();

    while (!platformControlP1->stopAllProcesses)
    {
        if (watchDogActivated)
        {
            if (watchCnt > 3) watchCnt = 3;
            watchCnt--;
            //qDebug() << "WebSocketServer::watchDogWorker ACTIVE " << watchCnt;
            if (watchCnt <= 0)
            {
                watchDogActivated = false;

                qDeleteAll(m_clients.begin(), m_clients.end());

                watchCnt = -1;

                //Stop selected modules
                platformControlP1->stopAll();
                qDebug() << "WebSocketServer::watchDogWorker platformControlP1 STOPPED";
                bodyControlP1->stopAll();
                qDebug() << "WebSocketServer::watchDogWorker bodyControlP1 STOPPED";

                platformControlP2->setBeepDuration(50);
                platformControlP2->alarmBeep();
                this_thread::sleep_for(std::chrono::milliseconds(500));
                platformControlP2->setBeepDuration(50);
                platformControlP2->alarmBeep();
                this_thread::sleep_for(std::chrono::milliseconds(500));
                platformControlP2->setBeepDuration(50);
                platformControlP2->alarmBeep();
            }
        }

        this_thread::sleep_for(std::chrono::milliseconds(watchDogSleep));
    }
    qDebug() << "STOPPED: WebSocketServer::watchDogWorker";
}
