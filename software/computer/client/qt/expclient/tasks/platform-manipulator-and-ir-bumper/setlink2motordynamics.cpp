#include "valter.h"
#include "setlink2motordynamics.h"

SetLink2MotorDynamics::SetLink2MotorDynamics()
{
    motorDutyMax = 0;
    motorDeceleration = 0;
    motorAcceleration = 0;

    qDebugOn = true;
    taskName = "SetLink2MotorDynamics";
    blocking = false;
}

bool SetLink2MotorDynamics::checkFeasibility()
{
    return true;
}

bool SetLink2MotorDynamics::initialize()
{
    return true;
}

void SetLink2MotorDynamics::execute()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
    if (motorDutyMax != 0)
    {
        platformManipulatorAndIRBumper->setLink2MotorDutyMax(motorDutyMax);
        qDebug("Task#%lu (%s) motorDutyMax = %u", getTaskId(), getTaskName().c_str(), motorDutyMax);
    }
    if (motorDeceleration != 0)
    {
        platformManipulatorAndIRBumper->setLink2MotorDeceleration(motorDeceleration);
        qDebug("Task#%lu (%s) motorDeceleration = %u", getTaskId(), getTaskName().c_str(), motorDeceleration);
    }
    if (motorAcceleration != 0)
    {
        platformManipulatorAndIRBumper->setLink2MotorAcceleration(motorAcceleration);
        qDebug("Task#%lu (%s) motorAcceleration = %u", getTaskId(), getTaskName().c_str(), motorAcceleration);
    }
    setCompleted();
}

void SetLink2MotorDynamics::stopExecution()
{

}

void SetLink2MotorDynamics::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetLink2MotorDynamics::create()
{
    return (ITask*)new SetLink2MotorDynamics();
}

void SetLink2MotorDynamics::setMotorDutyMax(unsigned int value)
{
    motorDutyMax = value;
}

void SetLink2MotorDynamics::setMotorDeceleration(unsigned int value)
{
    motorDeceleration = value;
}

void SetLink2MotorDynamics::setMotorAcceleration(unsigned int value)
{
    motorAcceleration = value;
}

void SetLink2MotorDynamics::executionWorker()
{

}
