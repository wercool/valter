#include "valter.h"
#include "setleftforearmpositiontask.h"

SetLeftForearmPositionTask::SetLeftForearmPositionTask()
{
    qDebugOn = true;
    taskName = "SetLeftForearmPositionTask";
    blocking = false;
}

bool SetLeftForearmPositionTask::checkFeasibility()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (!armControlLeft->prepareLeftForearmMovement())
    {
        string msg = Valter::format_string("Task#%lu prepareLeftForearmMovement() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    if (angle < -30 || angle > 60)
    {
        string msg = Valter::format_string("Task#%lu target Left Forearm angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetLeftForearmPositionTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    float angle = atof(((string)taskInitiationParts[1]).c_str());
    setAngle(angle);

/************************************ emulation *********************start***************************/
//    return true;
/************************************ emulation *********************finish**************************/

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        qDebug("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        return false;
    }
    return true;
}

void SetLeftForearmPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetLeftForearmPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetLeftForearmPositionTask::stopExecution()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftForearmMotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetLeftForearmPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetLeftForearmPositionTask::create()
{
    return (ITask*)new SetLeftForearmPositionTask();
}

void SetLeftForearmPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();

/************************************ emulation *********************start***************************/
//    for (int i = 0; i < 15; i++)
//    {
//        armControlLeft->setForearmADCPosition(ArmControlLeft::forearmAngleADCZero);
//    }
//    qDebug("Current armControlLeft->getForearmPosition() = %.2f", armControlLeft->getForearmPosition());
/************************************ emulation *********************finish**************************/

    float sigma = 2.0; //precision in degrees

    //move up (angle increased) - true
    bool direction = (angle > armControlLeft->getForearmPosition()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 0.95) : (angle / 0.95); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(angle - armControlLeft->getForearmPosition()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //move Left Forearm up
            {
                if (armControlLeft->prepareLeftForearmMovement())
                {
                    //move up
                    if (armControlLeft->setLeftForearmMotorMovementDirection(false))
                    {
                        armControlLeft->setLeftForearmMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlLeft->prepareLeftForearmMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    continue;
                }
            }
            else
            {
                if (armControlLeft->prepareLeftForearmMovement())
                {
                    //down
                    if (armControlLeft->setLeftForearmMotorMovementDirection(true))
                    {
                        armControlLeft->setLeftForearmMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlLeft->prepareLeftForearmMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    continue;
                }
            }
        }
        else
        {

            if (qDebugOn)
            {
                qDebug("Task#%lu: getForearmPosition (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), armControlLeft->getForearmPosition(), angle, cutoffAngle, abs(angle - armControlLeft->getForearmPosition()), sigma, (armControlLeft->getLeftForearmMotorMovementDirection() ? "down" : "up"));
            }

            if ((abs(angle - armControlLeft->getForearmPosition()) < sigma))
            {
                armControlLeft->setLeftForearmMotorActivated(false);
                setCompleted();
                break;
            }

/************************************ emulation *********************start***************************/
//            int positionADC = ArmControlLeft::forearmAngleADCZero + round(armControlLeft->getForearmPosition() * ArmControlLeft::forearmDegreesDiv);
//            int noise = rand() % 20;
//            if (noise == 0)
//            {
//                qDebug("NOISE ++++++++++++++++++++++++++++++++++++++++++");
//                positionADC = 1023;
//            }
//            if (noise == 1)
//            {
//                positionADC = 0;
//                qDebug("NOISE ------------------------------------------");
//            }
//            if (noise == 2)
//            {
//                qDebug("NOISE +++++++++++");
//                positionADC += 100;
//            }
//            if (noise == 3)
//            {
//                positionADC -= 100;
//                qDebug("NOISE -----------");
//            }
//            if (direction)
//            {
//                positionADC += 10;
//            }
//            else
//            {
//                positionADC -= 10;
//            }
//            armControlLeft->setForearmADCPosition(positionADC);
/************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!getCompleted())
    {
        string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));


        setCompleted();
    }
}


float SetLeftForearmPositionTask::getAngle() const
{
    return angle;
}

void SetLeftForearmPositionTask::setAngle(float value)
{
    angle = value;
}
