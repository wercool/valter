#include "valter.h"
#include "setlink1motordynamics.h"

SetLink1MotorDynamics::SetLink1MotorDynamics()
{
    motorDutyMax = 0;
    motorDeceleration = 0;
    motorAcceleration = 0;

    qDebugOn = true;
    taskName = "SetLink1MotorDynamics";
    blocking = false;
}

bool SetLink1MotorDynamics::checkFeasibility()
{
    return true;
}

bool SetLink1MotorDynamics::initialize()
{
    return true;
}

void SetLink1MotorDynamics::execute()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
    if (motorDutyMax != 0)
    {
        platformManipulatorAndIRBumper->setLink1MotorDutyMax(motorDutyMax);
        qDebug("Task#%lu (%s) motorDutyMax = %u", getTaskId(), getTaskName().c_str(), motorDutyMax);
    }
    if (motorDeceleration != 0)
    {
        platformManipulatorAndIRBumper->setLink1MotorDeceleration(motorDeceleration);
        qDebug("Task#%lu (%s) motorDeceleration = %u", getTaskId(), getTaskName().c_str(), motorDeceleration);
    }
    if (motorAcceleration != 0)
    {
        platformManipulatorAndIRBumper->setLink1MotorAcceleration(motorAcceleration);
        qDebug("Task#%lu (%s) motorAcceleration = %u", getTaskId(), getTaskName().c_str(), motorAcceleration);
    }
    setCompleted();
}

void SetLink1MotorDynamics::stopExecution()
{

}

void SetLink1MotorDynamics::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetLink1MotorDynamics::create()
{
    return (ITask*)new SetLink1MotorDynamics();
}

void SetLink1MotorDynamics::executionWorker()
{

}

void SetLink1MotorDynamics::setMotorAcceleration(unsigned int value)
{
    motorAcceleration = value;
}

void SetLink1MotorDynamics::setMotorDeceleration(unsigned int value)
{
    motorDeceleration = value;
}

void SetLink1MotorDynamics::setMotorDutyMax(unsigned int value)
{
    motorDutyMax = value;
}
