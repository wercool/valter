#include "valter.h"
#include "setrightforearmpositiontask.h"


SetRightForearmPositionTask::SetRightForearmPositionTask()
{
    qDebugOn = true;
    taskName = "SetRightForearmPositionTask";
    blocking = false;
}

bool SetRightForearmPositionTask::checkFeasibility()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (!armControlRight->prepareRightForearmMovement())
    {
        string msg = Valter::format_string("Task#%lu prepareRightForearmMovement() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    if (angle < -30 || angle > 60)
    {
        string msg = Valter::format_string("Task#%lu target Right Forearm angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetRightForearmPositionTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    float angle = atof(((string)taskInitiationParts[1]).c_str());
    setAngle(angle);

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        qDebug("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        return false;
    }
    return true;
}

void SetRightForearmPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetRightForearmPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetRightForearmPositionTask::stopExecution()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightForearmMotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetRightForearmPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetRightForearmPositionTask::create()
{
    return (ITask*)new SetRightForearmPositionTask();
}

void SetRightForearmPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();

    /************************************ emulation *********************start***************************/
//    armControlRight->setForearmADCPosition(327);
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    //move up (angle increased) - true
    bool direction = (angle > armControlRight->getForearmPosition()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 0.98) : (angle / 0.98); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(angle - armControlRight->getForearmPosition()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //move Right Forearm up
            {
                if (armControlRight->prepareRightForearmMovement())
                {
                    //move up
                    if (armControlRight->setRightForearmMotorMovementDirection(false))
                    {
                        armControlRight->setRightForearmMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlRight->prepareRightForearmMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    return;
                }
            }
            else
            {
                if (armControlRight->prepareRightForearmMovement())
                {
                    //down
                    if (armControlRight->setRightForearmMotorMovementDirection(true))
                    {
                        armControlRight->setRightForearmMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlRight->prepareRightForearmMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    return;
                }
            }
        }
        else
        {

            if (qDebugOn)
            {
                qDebug("Task#%lu: getForearmPosition (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), armControlRight->getForearmPosition(), angle, cutoffAngle, abs(angle - armControlRight->getForearmPosition()), sigma, (armControlRight->getRightForearmMotorMovementDirection() ? "down" : "up"));
            }

            if ((abs(angle - armControlRight->getForearmPosition()) < sigma))
            {
                armControlRight->setRightForearmMotorActivated(false);
                setCompleted();
                return;
            }

            /************************************ emulation *********************start***************************/
//            int positionADC = armControlRight->getForearmADCPosition();
//            if (direction)
//            {
//                positionADC -= 5;
//            }
//            else
//            {
//                positionADC += 5;
//            }
//            armControlRight->setForearmADCPosition(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    qDebug("%s", msg.c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));


    setCompleted();
}

float SetRightForearmPositionTask::getAngle() const
{
    return angle;
}

void SetRightForearmPositionTask::setAngle(float value)
{
    angle = value;
}
