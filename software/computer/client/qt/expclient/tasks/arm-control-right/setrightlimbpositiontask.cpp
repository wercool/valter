#include "valter.h"
#include "setrightlimbpositiontask.h"


SetRightLimbPositionTask::SetRightLimbPositionTask()
{
    qDebugOn = true;
    taskName = "SetRightLimbPositionTask";
    blocking = false;
}

bool SetRightLimbPositionTask::checkFeasibility()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (!armControlRight->prepareRightLimbMovement())
    {
        string msg = Valter::format_string("Task#%lu prepareRightLimbMovement() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    if (angle < -40 || angle > 85)
    {
        string msg = Valter::format_string("Task#%lu target Right Limb angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetRightLimbPositionTask::initialize()
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

void SetRightLimbPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetRightLimbPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetRightLimbPositionTask::stopExecution()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightLimbMotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetRightLimbPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetRightLimbPositionTask::create()
{
    return (ITask*)new SetRightLimbPositionTask();
}

void SetRightLimbPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();

    /************************************ emulation *********************start***************************/
//    armControlRight->setLimbADCPosition(405);
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    //move up (angle increased) - true
    bool direction = (angle > armControlRight->getLimbPosition()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 0.98) : (angle / 0.98); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(angle - armControlRight->getLimbPosition()) < sigma)
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
                if (armControlRight->prepareRightLimbMovement())
                {
                    //move up
                    if (armControlRight->setRightLimbMotorMovementDirection(false))
                    {
                        armControlRight->setRightLimbMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlRight->prepareRightLimbMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    return;
                }
            }
            else
            {
                if (armControlRight->prepareRightLimbMovement())
                {
                    //down
                    if (armControlRight->setRightLimbMotorMovementDirection(true))
                    {
                        armControlRight->setRightLimbMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlRight->prepareRightLimbMovement()", getTaskId());
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
                qDebug("Task#%lu: getLimbPosition (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), armControlRight->getLimbPosition(), angle, cutoffAngle, abs(angle - armControlRight->getLimbPosition()), sigma, (armControlRight->getRightLimbMotorMovementDirection() ? "down" : "up"));
            }

            if ((abs(angle - armControlRight->getLimbPosition()) < sigma))
            {
                armControlRight->setRightLimbMotorActivated(false);
                setCompleted();
                return;
            }

            /************************************ emulation *********************start***************************/
//            int positionADC = armControlRight->getLimbADCPosition();
//            if (direction)
//            {
//                positionADC -= 5;
//            }
//            else
//            {
//                positionADC += 5;
//            }
//            armControlRight->setLimbADCPosition(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    qDebug("%s", msg.c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));


    setCompleted();
}

float SetRightLimbPositionTask::getAngle() const
{
    return angle;
}

void SetRightLimbPositionTask::setAngle(float value)
{
    angle = value;
}
