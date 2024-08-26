#include "websocketserver.h"
#include "valter.h"

#include "QtWebSockets/qwebsocketserver.h"
#include "QtWebSockets/qwebsocket.h"

WebSocketServer *WebSocketServer::pWebSocketServer = NULL;
bool WebSocketServer::instanceFlag = false;

WebSocketServer::WebSocketServer()
{
    platformControlP1 = PlatformControlP1::getInstance();
    platformControlP2 = PlatformControlP2::getInstance();
    platformLocationP1 = PlatformLocationP1::getInstance();
    platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    bodyControlP1 = BodyControlP1::getInstance();
    armControlRight = ArmControlRight::getInstance();
    armControlLeft = ArmControlLeft::getInstance();

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
    stopAllModules();

    platformControlP2->setBeepDuration(50);
    platformControlP2->alarmBeep();

    QWebSocket *pSocket = m_pWebSocketServer->nextPendingConnection();

    connect(pSocket, &QWebSocket::textMessageReceived, this, &WebSocketServer::processTextMessage);
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
                stopAllModules();
            break;
            case Valter::str2int("TORSOYAWRIGHT"): //Torso yaw right
                if (platformControlP1->preparePlatformMovement())
                {
                    platformControlP1->setTurretMotorDutyMax(25);
                    //rotate right (cw)
                    if (platformControlP1->setTurretMotorDirection(true))
                    {
                        platformControlP1->setTurretMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("TORSOYAWLEFT"): //Torso yaw left
                if (platformControlP1->preparePlatformMovement())
                {
                    platformControlP1->setTurretMotorDutyMax(25);
                    //rotate left (ccw)
                    if (platformControlP1->setTurretMotorDirection(false))
                    {
                        platformControlP1->setTurretMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("TORSOYAWDONE"): //Torso yaw stop
                platformControlP1->setTurretMotorActivated(false);
            break;
        }

        //RightMotorDuty
        if (cmdValue.find("RMD") != std::string::npos)
        {
            vector<string>cmdValue_str_values = Valter::split(cmdValue , '=');
            std::string cmdValue = cmdValue_str_values[1];
            int value = atoi(cmdValue.c_str());
            if (value <= 0 || value > 40) {
                value = 20;
            }
            platformControlP1->setRightMotorDutyMax(value);
        }
        //LeftMotorDuty
        if (cmdValue.find("LMD") != std::string::npos)
        {
            vector<string>cmdValue_str_values = Valter::split(cmdValue , '=');
            std::string cmdValue = cmdValue_str_values[1];
            int value = atoi(cmdValue.c_str());
            if (value <= 0 || value > 40) {
                value = 20;
            }
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

    /**********************************PLATFORM-MANIPULATOR-AND-IR-BUMPER***************************************************/
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("PMIB"))
    {
        switch (Valter::str2int(cmdValue.c_str()))
        {
            case Valter::str2int("DC24ON"): //24V DC power ON
                platformManipulatorAndIRBumper->setPower24VOnOff(true);
            break;
            case Valter::str2int("DC24OFF"): //24V DC power OFF
                platformManipulatorAndIRBumper->setPower24VOnOff(false);
            break;
            case Valter::str2int("GRIPCCW"): //Rotate gripper CCW
                platformManipulatorAndIRBumper->manGripperRotateCCW();
            break;
            case Valter::str2int("GRIPCW"): //Rotate gripper CW
                platformManipulatorAndIRBumper->manGripperRotateCW();
            break;
            case Valter::str2int("GRIPSTOP"): //Rotate gripper STOP
                platformManipulatorAndIRBumper->manGripperRotateStop();
            break;
            case Valter::str2int("L2DOWN"): //Link2 descent movement
                if (platformManipulatorAndIRBumper->prepareManLink2Movement())
                {
                    //descent
                    if (platformManipulatorAndIRBumper->setLink2MovementDirection(false))
                    {
                        platformManipulatorAndIRBumper->setLink2MotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("L2UP"): //Link2 descent movement
                if (platformManipulatorAndIRBumper->prepareManLink2Movement())
                {
                    //ascent
                    if (platformManipulatorAndIRBumper->setLink2MovementDirection(true))
                    {
                        platformManipulatorAndIRBumper->setLink2MotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("L2STOP"):
                platformManipulatorAndIRBumper->setLink2MotorActivated(false);
            break;
        }
    }

    /*********************************BODY-CONTROL-P1***************************************************/
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("BCP1"))
    {
        switch (Valter::str2int(cmdValue.c_str()))
        {
            case Valter::str2int("STOPALL"): //Stop all activities on Body Control P1
                bodyControlP1->stopAll();
            break;
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
            case Valter::str2int("LEFTARMROLL12VON"): // Left Arm Roll 12V ON
                bodyControlP1->setLeftArm12VPowerOnOff(true);
            break;
            case Valter::str2int("LEFTARMROLL12VOFF"): // Left Arm Roll 12V OFF
                bodyControlP1->setLeftArm12VPowerOnOff(false);
            break;
            case Valter::str2int("RIGHTARMROLL12VON"): // Right Arm Roll 12V ON
                bodyControlP1->setRightArm12VPowerOnOff(true);
            break;
            case Valter::str2int("RIGHTARMROLL12VOFF"): // Right Arm Roll 12V OFF
                bodyControlP1->setRightArm12VPowerOnOff(false);
            break;
            case Valter::str2int("BODY5_5VON"): //Body 5.5V ON
                bodyControlP1->setPowerSource5VOnOff(true);
            break;
            case Valter::str2int("BODY5_5VOFF"): //Body 5.5V OFF
                bodyControlP1->setPowerSource5VOnOff(false);
            break;
            case Valter::str2int("RESETBODYCAMERA"): //Reset body camera position
                bodyControlP1->setBodyCameraPosition(1460 * 4);
            break;
            case Valter::str2int("BODYPITCHDOWN"): //Pitch body down
                if (bodyControlP1->prepareBodyPitchMovement())
                {
                    bodyControlP1->setBodyPitchMotorDutyMax(90);
                    //down
                    if (bodyControlP1->setBodyPitchMovementDirection(true))
                    {
                        bodyControlP1->setBodyPitchMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("BODYPITCHUP"): //Pitch body up
                if (bodyControlP1->prepareBodyPitchMovement())
                {
                    bodyControlP1->setBodyPitchMotorDutyMax(90);
                    //up
                    if (bodyControlP1->setBodyPitchMovementDirection(false))
                    {
                        bodyControlP1->setBodyPitchMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("BODYPITCHSTOP"): //Pitch body stop
                bodyControlP1->setBodyPitchMotorActivated(false);
            break;
        }
    }

    /**********************************ARM-CONTROL-RIGHT***************************************************/
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("ACR"))
    {
        switch (Valter::str2int(cmdValue.c_str()))
        {
            case Valter::str2int("STOPALL"): // Stop All activities on Right Arm
                armControlRight->stopAll();
            break;
            case Valter::str2int("STARTALLWATCHERS"): // Start Right Arm Watchers
                armControlRight->startAllWatchers();
            break;
            case Valter::str2int("STOPALLWATCHERS"): // Stop Right Arm Watchers
                armControlRight->stopAllWatchers();
            break;
            case Valter::str2int("RIGHTARMROLLMOTORON"): // Right Arm Roll (ON) stepper motor keep position
                armControlRight->setForearmRollMotorOnOff(true);
            break;
            case Valter::str2int("RIGHTARMROLLMOTOROFF"): // Right Arm Roll (OFF) stepper motor release
                armControlRight->setForearmRollMotorOnOff(false);
            break;
            case Valter::str2int("RIGHTFOREARMUP"): // Right Forearm UP
                if (armControlRight->prepareRightForearmMovement())
                {
                    armControlRight->setRightForearmMotorDutyMax(90);
                    //up
                    if (armControlRight->setRightForearmMotorMovementDirection(false))
                    {
                        armControlRight->setRightForearmMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("RIGHTFOREARMDOWN"): // Right Forearm DOWN
                if (armControlRight->prepareRightForearmMovement())
                {
                    armControlRight->setRightForearmMotorDutyMax(90);
                    //down
                    if (armControlRight->setRightForearmMotorMovementDirection(true))
                    {
                        armControlRight->setRightForearmMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("RIGHTFOREARMSTOP"): // Right Forearm STOP
                armControlRight->setRightForearmMotorActivated(false);
            break;
            case Valter::str2int("RIGHTHANDCLOSE"): // Close right hand
                armControlRight->fingersToInitialPositions();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                armControlRight->fingersGrasp();
            break;
            case Valter::str2int("RIGHTHANDOPEN"): // Open right hand
                armControlRight->fingersToInitialPositions();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                armControlRight->releaseAllFingers();
            break;
        }
    }

    /**********************************ARM-CONTROL-LEFT***************************************************/
    if (Valter::str2int(cmdType.c_str()) == Valter::str2int("ACL"))
    {
        switch (Valter::str2int(cmdValue.c_str()))
        {
            case Valter::str2int("STOPALL"): // Stop All activities on Left Arm
                armControlLeft->stopAll();
            break;
            case Valter::str2int("STARTALLWATCHERS"): // Start Left Arm Watchers
                armControlLeft->startAllWatchers();
            break;
            case Valter::str2int("STOPALLWATCHERS"): // Stop Left Arm Watchers
                armControlLeft->stopAllWatchers();
            break;
            case Valter::str2int("LEFTARMROLLMOTORON"): // Left Arm Roll (ON) stepper motor keep position
                armControlLeft->setForearmRollMotorOnOff(true);
            break;
            case Valter::str2int("LEFTARMROLLMOTOROFF"): // Left Arm Roll (OFF) stepper motor release
                armControlLeft->setForearmRollMotorOnOff(false);
            break;
            case Valter::str2int("LEFTFOREARMUP"): // Left Forearm UP
                if (armControlLeft->prepareLeftForearmMovement())
                {
                    armControlLeft->setLeftForearmMotorDutyMax(90);
                    //up
                    if (armControlLeft->setLeftForearmMotorMovementDirection(false))
                    {
                        armControlLeft->setLeftForearmMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("LEFTFOREARMDOWN"): // Left Forearm DOWN
                if (armControlLeft->prepareLeftForearmMovement())
                {
                    armControlLeft->setLeftForearmMotorDutyMax(90);
                    //down
                    if (armControlLeft->setLeftForearmMotorMovementDirection(true))
                    {
                        armControlLeft->setLeftForearmMotorActivated(true);
                    }
                }
            break;
            case Valter::str2int("LEFTFOREARMSTOP"): // Left Forearm STOP
                armControlLeft->setLeftForearmMotorActivated(false);
            break;
            case Valter::str2int("LEFTHANDCLOSE"): // Close left hand
                armControlLeft->fingersToInitialPositions();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                armControlLeft->fingersGrasp();
            break;
            case Valter::str2int("LEFTHANDOPEN"): // Open left hand
                armControlLeft->fingersToInitialPositions();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                armControlLeft->releaseAllFingers();
            break;
        }
    }

    if (pClient)
    {
        pClient->sendTextMessage(cmdResponse.c_str());
    }
}

void WebSocketServer::socketDisconnected()
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());

    stopAllModules();

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

                stopAllModules();

                watchCnt = -1;

                platformControlP2->setBeepDuration(25);
                platformControlP2->alarmBeep();
                this_thread::sleep_for(std::chrono::milliseconds(250));
                platformControlP2->setBeepDuration(25);
                platformControlP2->alarmBeep();
                this_thread::sleep_for(std::chrono::milliseconds(250));
                platformControlP2->setBeepDuration(25);
                platformControlP2->alarmBeep();
            }
        }

        this_thread::sleep_for(std::chrono::milliseconds(watchDogSleep));
    }
    qDebug() << "STOPPED: WebSocketServer::watchDogWorker";
}

void WebSocketServer::stopAllModules()
{

    qDebug() << "WebSocketServer::stopAllModules()";

    Valter::getInstance()->stopAllModules();

//    //Stop selected modules

////    platformControlP1->stopAll();
//    platformControlP1->setPlatformEmergencyStop(true);
//    platformControlP1->setTurretEmergencyStop(true);

//    platformControlP2->stopAll();
//    platformLocationP1->stopAll();
//    platformManipulatorAndIRBumper->stopAll();
//    bodyControlP1->stopAll();
//    armControlRight->stopAll();
//    armControlLeft->stopAll();
}
