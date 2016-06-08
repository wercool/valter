#include "valter.h"
#include "setgripperrotationmotordynamics.h"

SetGripperRotationMotorDynamics::SetGripperRotationMotorDynamics()
{
    qDebugOn = true;
    taskName = "SetGripperRotationMotorDynamics";
    blocking = false;
    checkFeasibility();
}

bool SetGripperRotationMotorDynamics::checkFeasibility()
{
    return true;
}

bool SetGripperRotationMotorDynamics::initialize()
{
    return true;
}

void SetGripperRotationMotorDynamics::execute()
{
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
