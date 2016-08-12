#include "valter.h"
#include "setgripperrotationmotordynamics.h"

SetGripperRotationMotorDynamics::SetGripperRotationMotorDynamics()
{
    motorDutyMax = 0;

    qDebugOn = true;
    taskName = "SetGripperRotationMotorDynamics";
    blocking = false;
}

bool SetGripperRotationMotorDynamics::checkFeasibility()
{
    return true;
}

bool SetGripperRotationMotorDynamics::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    unsigned int motorDutyMax = atoi(((string)taskInitiationParts[1]).c_str());
    setMotorDutyMax(motorDutyMax);

    return true;
}

void SetGripperRotationMotorDynamics::execute()
{
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    if (motorDutyMax != 0)
    {
        platformManipulatorAndIRBumper->setManGripperRotationMotorDuty(motorDutyMax);
        qDebug("Task#%lu (%s) motorDutyMax = %u", getTaskId(), getTaskName().c_str(), motorDutyMax);
    }
    setCompleted();
}

void SetGripperRotationMotorDynamics::stopExecution()
{

}

void SetGripperRotationMotorDynamics::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetGripperRotationMotorDynamics::create()
{
    return (ITask*)new SetGripperRotationMotorDynamics();
}

void SetGripperRotationMotorDynamics::setMotorDutyMax(unsigned int value)
{
    motorDutyMax = value;
}

void SetGripperRotationMotorDynamics::executionWorker()
{

}
