#include "valter.h"
#include "cmdveltask.h"

CmdVelTask::CmdVelTask()
{
    qDebugOn = true;
    taskName = "CmdVelTask";
    blocking = false;
    attachable = true;

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    initialLeftMotorMaxDuty = platformControlP1->getLeftMotorDutyMax();
    initialRightMotorMaxDuty = platformControlP1->getRightMotorDutyMax();
}

bool CmdVelTask::checkFeasibility()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    return true;
}

bool CmdVelTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    //T_PCP1_CmdVelTask_0.1_0.0
    float _linearVelocity = atof(((string)taskInitiationParts[1]).c_str());
    float _angularVelocity = atof(((string)taskInitiationParts[2]).c_str());
    setLinearVelocity(_linearVelocity);
    setAngularVelocity(_angularVelocity);

    return true;
}

void CmdVelTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&CmdVelTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s, %s~%s~%s",
                                                                                                  getTaskId(),
                                                                                                  getTaskName().c_str(),
                                                                                                  (blocking) ? "blocking" : "non blocking",
                                                                                                  "attachable",
                                                                                                  ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))),
                                                                                                  getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void CmdVelTask::stopExecution()
{
    stopped = true;
}

void CmdVelTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s",
                                                                                          getTaskId(),
                                                                                          getTaskName().c_str(),
                                                                                          (blocking) ? "blocking" : "non blocking",
                                                                                          ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))),
                                                                                          getTaskScriptLine().c_str()));
}

ITask *CmdVelTask::create()
{
    return (ITask*)new CmdVelTask();
}

float CmdVelTask::getLinearVelocity() const
{
    return linearVelocity;
}

void CmdVelTask::setLinearVelocity(float value)
{
    std::lock_guard<std::mutex> lock(values_mutex);
    linearVelocity = value;
}

float CmdVelTask::getAngularVelocity() const
{
    return angularVelocity;
}

void CmdVelTask::setAngularVelocity(float value)
{
    std::lock_guard<std::mutex> lock(values_mutex);
    angularVelocity = value;
}


int CmdVelTask::getLeftWheelEncoderReading()
{
    //return PlatformControlP2::getInstance()->getLeftEncoder();
    return PlatformControlP1::getInstance()->getLeftWheelEncoder();
}

int CmdVelTask::getRightWheelEncoderReading()
{
    //return PlatformControlP2::getInstance()->getRightEncoder();
    return PlatformControlP1::getInstance()->getRightWheelEncoder();
}

void CmdVelTask::executionWorker()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    bool leftMotorDirection  = true; //forward
    bool rightMotorDirection = true; //forward

    bool prevLeftMotorDirection  = true; //forward
    bool prevRightMotorDirection = true; //forward

    int maxAllowedDuty = 35;
    int minAllowedDuty = 10;

    int leftMotorDuty  = minAllowedDuty;
    int rightMotorDuty = minAllowedDuty;

    //******************************************************************************************experimental timings based
    //robot spins 2*pi at the duty of [maxAllowedDuty = 35] about [st] seconds, so the maxAngVel = (2*pi)/st (rad/sec)
    double maxAngVel = (2 * M_PI) / 10.7; //0.587196262

    //robot linear speed at the duty of [maxAllowedDuty = 35] maxLinVel = x(m/s)
    double maxLinVel = 0.12;


    //******************************************************************************************encoder based
    /*
    int ticksPerMeter = PlatformControlP1::vagueEncoderTicksPerMeter;
    int curVelocityTicksL = 0;
    int curVelocityTicksR = 0;

    int velocityTicksL = 0;
    int velocityTicksR = 0;

    int curLeftEncoder = 0;
    int curRightEncoder = 0;

    int prevLeftEncoder = 0;
    int prevRightEncoder = 0;

    float wheelSeparation = ((double)Valter::wheelBase / 1000) / 2;
    */


    string stopExecutionReason = "";

    bool justStarted = false;
    bool stoppedMsg = false;

    while (!stopped)
    {
        stopped = !checkFeasibility();

        if (!stopped)
        {
            if (!executing)
            {
                if (!platformControlP1->preparePlatformMovement())
                {
                    stopExecutionReason = Valter::format_string("Task#%lu (%s)has been terminated. preparePlatformMovement() returns FALSE.", getTaskId(), getTaskName().c_str());
                    qDebug("%s", stopExecutionReason.c_str());
                    stopExecution();
                    continue;
                }
                else
                {
                    executing = true;
                    justStarted = true;
                }
            }
            else
            {
                double linVel = (double)getLinearVelocity();
                double angVel = (double)getAngularVelocity();

                //******************************************************************************************experimental timings based
                double angVelDuty = (angVel * maxAllowedDuty) / maxAngVel;
                double linVelDuty = (linVel * maxAllowedDuty) / maxLinVel;

                leftMotorDuty  = round(linVelDuty - angVelDuty);
                rightMotorDuty = round(linVelDuty + angVelDuty);

                if (leftMotorDuty != 0 || rightMotorDuty != 0)
                {
                    leftMotorDirection  = (leftMotorDuty > 0)  ? true : false;
                    rightMotorDirection = (rightMotorDuty > 0) ? true : false;

                    if (leftMotorDirection != prevLeftMotorDirection || rightMotorDirection != prevRightMotorDirection || justStarted)
                    {
                        justStarted = false;
                        if (platformControlP1->setLeftMotorDirection(leftMotorDirection) && platformControlP1->setRightMotorDirection(rightMotorDirection))
                        {
                            prevLeftMotorDirection  = leftMotorDirection;
                            prevRightMotorDirection = rightMotorDirection;
                            platformControlP1->setLeftMotorActivated(true);
                            platformControlP1->setRightMotorActivated(true);
                            qDebug("[%s] motors activated", getTaskName().c_str());
                            this_thread::sleep_for(std::chrono::milliseconds(5));
                        }
                        else
                        {
                            platformControlP1->setLeftMotorActivated(false);
                            platformControlP1->setRightMotorActivated(false);
                            this_thread::sleep_for(std::chrono::milliseconds(5));
                            qDebug("[%s] DECELERATION leftMotorDuty=%d[%s], rightMotorDuty=%d[%s]", getTaskName().c_str(), platformControlP1->getLeftMotorDuty(), ((platformControlP1->getLeftMotorDirection() > 0) ? "↑" : "↓"), platformControlP1->getRightMotorDuty(), ((platformControlP1->getRightMotorDirection() > 0) ? "↑" : "↓"));
                            continue;
                        }
                    }
                }

                if (leftMotorDuty == 0)
                {
                    leftMotorDuty = 1;
                    platformControlP1->setLeftMotorActivated(false);
                }
                else if (leftMotorDuty > 1)
                {
                    platformControlP1->setLeftMotorActivated(true);
                }

                if (rightMotorDuty == 0)
                {
                    rightMotorDuty = 1;
                    platformControlP1->setRightMotorActivated(false);
                }
                else if (rightMotorDuty > 1)
                {
                    platformControlP1->setRightMotorActivated(true);
                }

                if (abs(leftMotorDuty) > maxAllowedDuty)
                {
                    leftMotorDuty = maxAllowedDuty;
                }
                if (abs(rightMotorDuty) > maxAllowedDuty)
                {
                    rightMotorDuty = maxAllowedDuty;
                }

                if (platformControlP1->getRightMotorStop() && platformControlP1->getRightMotorStop())
                {
                    leftMotorDuty = rightMotorDuty = 1;
                    platformControlP1->setLeftMotorDutyMax(abs(leftMotorDuty));
                    platformControlP1->setRightMotorDutyMax(abs(rightMotorDuty));
                    this_thread::sleep_for(std::chrono::milliseconds(50));
                }

                platformControlP1->setLeftMotorDutyMax(abs(leftMotorDuty));
                platformControlP1->setRightMotorDutyMax(abs(rightMotorDuty));

                if (!platformControlP1->getLeftMotorStop() || !platformControlP1->getRightMotorStop())
                {
                    qDebug("[%s] linVel=%f, angVel=%f, leftMotorDuty=%d[%s], rightMotorDuty=%d[%s], L[%s], R[%s]", getTaskName().c_str(), linVel, angVel, platformControlP1->getLeftMotorDuty(), ((platformControlP1->getLeftMotorDirection() > 0) ? "↑" : "↓"), platformControlP1->getRightMotorDuty(), ((platformControlP1->getRightMotorDirection() > 0) ? "↑" : "↓"), ((platformControlP1->getLeftMotorStop()) ? "S" : "A"), ((platformControlP1->getRightMotorStop()) ? "S" : "A"));
                    stoppedMsg = false;
                }
                else if (!stoppedMsg)
                {
                    qDebug("[%s] linVel=%f, angVel=%f, leftMotorDuty=%d[%s], rightMotorDuty=%d[%s], L[%s], R[%s] STOPPED", getTaskName().c_str(), linVel, angVel, platformControlP1->getLeftMotorDuty(), ((platformControlP1->getLeftMotorDirection() > 0) ? "↑" : "↓"), platformControlP1->getRightMotorDuty(), ((platformControlP1->getRightMotorDirection() > 0) ? "↑" : "↓"), ((platformControlP1->getLeftMotorStop()) ? "S" : "A"), ((platformControlP1->getRightMotorStop()) ? "S" : "A"));
                    stoppedMsg = true;
                }

                //******************************************************************************************encoder based
                /*
                velocityTicksL = round(linVel * ticksPerMeter - angVel * wheelSeparation * ticksPerMeter);
                velocityTicksR = round(linVel * ticksPerMeter + angVel * wheelSeparation * ticksPerMeter);

                qDebug("tL %d tR %d", velocityTicksL, velocityTicksR);

                curLeftEncoder  = getLeftWheelEncoderReading();
                curRightEncoder = getRightWheelEncoderReading();

                curVelocityTicksL = curLeftEncoder  - prevLeftEncoder;
                curVelocityTicksR = curRightEncoder - prevRightEncoder;

                qDebug("cL %d cR %d", curVelocityTicksL, curVelocityTicksR);

                prevLeftEncoder  = getLeftWheelEncoderReading();
                prevRightEncoder = getRightWheelEncoderReading();
                */
            }
            this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
    platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
    platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);

    if (!getCompleted())
    {
        string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal.%s", getTaskId(), ((stopExecutionReason.length() > 0) ? Valter::format_string(" REASON: %s", stopExecutionReason.c_str()).c_str() : ""));
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        setCompleted();
    }
}

