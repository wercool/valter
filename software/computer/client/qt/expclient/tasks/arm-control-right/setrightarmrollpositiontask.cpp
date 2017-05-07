#include "valter.h"
#include "setrightarmrollpositiontask.h"

SetRightArmRollPositionTask::SetRightArmRollPositionTask()
{
    qDebugOn = true;
    taskName = "SetRightArmRollPositionTask";
    blocking = false;
}

bool SetRightArmRollPositionTask::checkFeasibility()
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
    if (!bodyControlP1->getRightArm12VPowerSourceState() && !bodyControlP1->getRightArm24VPowerSourceState())
    {
        string msg = Valter::format_string("Task#%lu bodyControlP1->getRightArm12VPowerSourceState() and bodyControlP1->getRightArm24VPowerSourceState() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }

    if (angle < 0 || angle > 270)
    {
        string msg = Valter::format_string("Task#%lu target Right Arm Roll angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }

    return true;
}

bool SetRightArmRollPositionTask::initialize()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();

    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    float angle = atof(((string)taskInitiationParts[1]).c_str());
    setAngle(angle);

///************************************ emulation *********************start***************************/
////    return true;
///************************************ emulation *********************finish**************************/

    armControlRight->setForearmRollMotorOnOff(true);

    if (armControlRight->getForearmRollPositionUndefined())
    {
        armControlRight->setForearmRollResettingStepPosition(true);

        while (armControlRight->getForearmRollPositionUndefined() && !stopped)
        {
            qDebug("Right Forearm re-setting Roll position....");
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    return true;
}

void SetRightArmRollPositionTask::execute()
{
    if (checkFeasibility())
    {
        if (initialize())
        {
            new std::thread(&SetRightArmRollPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetRightArmRollPositionTask::stopExecution()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setForearmRollMotorOnOff(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetRightArmRollPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetRightArmRollPositionTask::create()
{
    return (ITask*)new SetRightArmRollPositionTask();
}

void SetRightArmRollPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();

    /************************************ emulation *********************start***************************/
//        qDebug("Current armControlRight->getLimbPosition() = %.2f", armControlRight->getLimbPosition());
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    //CW - true, CCW - false (in CCW angle increases)
    bool direction = (angle > armControlRight->getForearmRollPosition()) ? true : false;

    if (abs(angle - armControlRight->getForearmRollPosition()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            armControlRight->setForearmRollDirection(direction);
            armControlRight->setForearmRollMotorActivated(true);

            executing = true;
        }

        if (abs(angle - armControlRight->getForearmRollPosition()) < sigma)
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
            armControlRight->setForearmRollMotorActivated(false);
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

float SetRightArmRollPositionTask::getAngle() const
{
    return angle;
}

void SetRightArmRollPositionTask::setAngle(float value)
{
    angle = value;
}
