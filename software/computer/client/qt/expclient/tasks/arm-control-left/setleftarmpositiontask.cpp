#include "valter.h"
#include "setleftarmpositiontask.h"

SetLeftArmPositionTask::SetLeftArmPositionTask()
{
    qDebugOn = true;
    taskName = "SetLeftArmPositionTask";
    blocking = false;
}

bool SetLeftArmPositionTask::checkFeasibility()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (!armControlLeft->prepareLeftArmMovement())
    {
        string msg = Valter::format_string("Task#%lu prepareLeftArmMovement() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    if (angle < 0 || angle > 90)
    {
        string msg = Valter::format_string("Task#%lu target Left Arm angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetLeftArmPositionTask::initialize()
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

void SetLeftArmPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetLeftArmPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetLeftArmPositionTask::stopExecution()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftArmMotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetLeftArmPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetLeftArmPositionTask::create()
{
    return (ITask*)new SetLeftArmPositionTask();
}

void SetLeftArmPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();

    /************************************ emulation *********************start***************************/
//    armControlLeft->setArmADCPosition(750);
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    //move up (angle increased) - true
    bool direction = (angle > armControlLeft->getArmPosition()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 0.98) : (angle / 0.98); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(angle - armControlLeft->getArmPosition()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //move Left Arm up
            {
                if (armControlLeft->prepareLeftArmMovement())
                {
                    //move up
                    if (armControlLeft->setLeftArmMotorMovementDirection(false))
                    {
                        armControlLeft->setLeftArmMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlLeft->prepareLeftArmMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    return;
                }
            }
            else
            {
                if (armControlLeft->prepareLeftArmMovement())
                {
                    //down
                    if (armControlLeft->setLeftArmMotorMovementDirection(true))
                    {
                        armControlLeft->setLeftArmMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlLeft->prepareLeftArmMovement()", getTaskId());
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
                qDebug("Task#%lu: getArmPosition() (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), armControlLeft->getArmPosition(), angle, cutoffAngle, abs(angle - armControlLeft->getArmPosition()), sigma, (armControlLeft->getLeftArmMotorMovementDirection() ? "down" : "up"));
            }

            if ((abs(angle - armControlLeft->getArmPosition()) < sigma))
            {
                armControlLeft->setLeftArmMotorActivated(false);
                setCompleted();
                return;
            }

            /************************************ emulation *********************start***************************/
//            int positionADC = armControlLeft->getArmADCPosition();
//            if (direction)
//            {
//                positionADC -= 5;
//            }
//            else
//            {
//                positionADC += 5;
//            }
//            armControlLeft->setArmADCPosition(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    qDebug("%s", msg.c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));


    setCompleted();
}

float SetLeftArmPositionTask::getAngle() const
{
    return angle;
}

void SetLeftArmPositionTask::setAngle(float value)
{
    angle = value;
}
