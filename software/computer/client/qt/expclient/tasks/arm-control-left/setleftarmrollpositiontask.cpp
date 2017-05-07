#include "valter.h"
#include "setleftarmrollpositiontask.h"

SetLeftArmRollPositionTask::SetLeftArmRollPositionTask()
{
    qDebugOn = true;
    taskName = "SetLeftArmRollPositionTask";
    blocking = false;
}

bool SetLeftArmRollPositionTask::checkFeasibility()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }

    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (!bodyControlP1->getLeftArm12VPowerSourceState() && !bodyControlP1->getLeftArm24VPowerSourceState())
    {
        string msg = Valter::format_string("Task#%lu bodyControlP1->getLeftArm12VPowerSourceState() and bodyControlP1->getLeftArm24VPowerSourceState() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }

    if (angle < 0 || angle > 255)
    {
        string msg = Valter::format_string("Task#%lu target Left Arm Roll angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }

    return true;
}

bool SetLeftArmRollPositionTask::initialize()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();

    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    float angle = atof(((string)taskInitiationParts[1]).c_str());
    setAngle(angle);

///************************************ emulation *********************start***************************/
////    return true;
///************************************ emulation *********************finish**************************/

    armControlLeft->setForearmRollMotorOnOff(true);

    if (armControlLeft->getForearmRollPositionUndefined())
    {
        armControlLeft->setForearmRollResettingStepPosition(true);

        while (armControlLeft->getForearmRollPositionUndefined() && !stopped)
        {
            qDebug("Left Forearm re-setting Roll position....");
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    return true;
}

void SetLeftArmRollPositionTask::execute()
{
    if (checkFeasibility())
    {
        if (initialize())
        {
            new std::thread(&SetLeftArmRollPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetLeftArmRollPositionTask::stopExecution()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setForearmRollMotorOnOff(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetLeftArmRollPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetLeftArmRollPositionTask::create()
{
    return (ITask*)new SetLeftArmRollPositionTask();
}

void SetLeftArmRollPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();

    /************************************ emulation *********************start***************************/
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    //CW - true, CCW - false (in CCW angle increases)
    bool direction = (angle > armControlLeft->getForearmRollPosition()) ? true : false;

    if (abs(angle - armControlLeft->getForearmRollPosition()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            armControlLeft->setForearmRollDirection(direction);
            armControlLeft->setForearmRollMotorActivated(true);

            executing = true;
        }

        if (abs(angle - armControlLeft->getForearmRollPosition()) < sigma)
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
            armControlLeft->setForearmRollMotorActivated(false);
            setCompleted();
            return;
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

float SetLeftArmRollPositionTask::getAngle() const
{
    return angle;
}

void SetLeftArmRollPositionTask::setAngle(float value)
{
    angle = value;
}
