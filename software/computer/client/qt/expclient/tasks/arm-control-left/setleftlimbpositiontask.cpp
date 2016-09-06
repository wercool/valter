#include "valter.h"
#include "setleftlimbpositiontask.h"

SetLeftLimbPositionTask::SetLeftLimbPositionTask()
{
    qDebugOn = true;
    taskName = "SetLeftLimbPositionTask";
    blocking = false;
}

bool SetLeftLimbPositionTask::checkFeasibility()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (!armControlLeft->prepareLeftLimbMovement())
    {
        string msg = Valter::format_string("Task#%lu prepareLeftLimbMovement() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    if (angle < -40 || angle > 85)
    {
        string msg = Valter::format_string("Task#%lu target Left Limb angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetLeftLimbPositionTask::initialize()
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

void SetLeftLimbPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetLeftLimbPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetLeftLimbPositionTask::stopExecution()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftLimbMotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetLeftLimbPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetLeftLimbPositionTask::create()
{
    return (ITask*)new SetLeftLimbPositionTask();
}

void SetLeftLimbPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();

    /************************************ emulation *********************start***************************/
//    armControlLeft->setLimbADCPosition(405);
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    //move up (angle increased) - true
    bool direction = (angle > armControlLeft->getLimbPosition()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 0.98) : (angle / 0.98); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(angle - armControlLeft->getLimbPosition()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //move Limb up
            {
                if (armControlLeft->prepareLeftLimbMovement())
                {
                    //move up
                    if (armControlLeft->setLeftLimbMotorMovementDirection(false))
                    {
                        armControlLeft->setLeftLimbMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlLeft->prepareLeftLimbMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    continue;
                }
            }
            else
            {
                if (armControlLeft->prepareLeftLimbMovement())
                {
                    //down
                    if (armControlLeft->setLeftLimbMotorMovementDirection(true))
                    {
                        armControlLeft->setLeftLimbMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlLeft->prepareLeftLimbMovement()", getTaskId());
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
                qDebug("Task#%lu: getLimbPosition (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), armControlLeft->getLimbPosition(), angle, cutoffAngle, abs(angle - armControlLeft->getLimbPosition()), sigma, (armControlLeft->getLeftLimbMotorMovementDirection() ? "down" : "up"));
            }

            if ((abs(angle - armControlLeft->getLimbPosition()) < sigma))
            {
                armControlLeft->setLeftLimbMotorActivated(false);
                setCompleted();
                break;
            }

            /************************************ emulation *********************start***************************/
//            int positionADC = armControlLeft->getLimbADCPosition();
//            if (direction)
//            {
//                positionADC -= 5;
//            }
//            else
//            {
//                positionADC += 5;
//            }
//            armControlLeft->setLimbADCPosition(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!getCompleted())
    {
        string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));


        setCompleted();
    }
}

float SetLeftLimbPositionTask::getAngle() const
{
    return angle;
}

void SetLeftLimbPositionTask::setAngle(float value)
{
    angle = value;
}
