#include "valter.h"
#include "setgripperrotationposition.h"

SetGripperRotationPosition::SetGripperRotationPosition()
{
    qDebugOn = true;
    taskName = "SetGripperRotationPosition";
    blocking = false;
}

bool SetGripperRotationPosition::checkFeasibility()
{
    if (angle > 177 || angle < -89)
    {
        qDebug("Task#%lu (%s) target angle %f in unreachable.", getTaskId(), getTaskName().c_str(), angle);
        return false;
    }
    return true;
}

bool SetGripperRotationPosition::initialize()
{
    return true;
}

void SetGripperRotationPosition::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetGripperRotationPosition::executionWorker, this);
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
            stopExecution();
            setCompleted();
        }
    }
}

void SetGripperRotationPosition::stopExecution()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperRotateStop();
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetGripperRotationPosition::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
}

ITask *SetGripperRotationPosition::create()
{
    return (ITask*)new SetGripperRotationPosition();
}

float SetGripperRotationPosition::getAngle() const
{
    return angle;
}

void SetGripperRotationPosition::setAngle(float value)
{
    angle = value;
}

void SetGripperRotationPosition::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    /************************************ emulation *********************start***************************/
    platformManipulatorAndIRBumper->setGripperADCRotation(355); //0 degrees = 355;
    /************************************ emulation *********************finish**************************/

    float sigma = 0.5; //precision in degrees

    //true CW
    bool direction = (angle > platformManipulatorAndIRBumper->getGripperRotation()) ? true : false;

    float cutoffAngle = angle * 0.99; //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(abs(angle) - abs(platformManipulatorAndIRBumper->getGripperRotation())) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //rotate gripper CW
            {
                platformManipulatorAndIRBumper->manGripperRotateCW();
            }
            else
            {
                platformManipulatorAndIRBumper->manGripperRotateCCW();
            }
            executing = true;
        }
        else
        {

            if (qDebugOn)
            {
                qDebug("Task#%lu: GripperRotation (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), platformManipulatorAndIRBumper->getGripperRotation(), angle, cutoffAngle, abs(abs(angle) - abs(platformManipulatorAndIRBumper->getGripperRotation())), sigma, (direction ? "CW" : "CCW"));
            }

            if ((direction && platformManipulatorAndIRBumper->getGripperRotation() >= cutoffAngle) || ((!direction && platformManipulatorAndIRBumper->getGripperRotation() <= cutoffAngle)) || (abs(angle - platformManipulatorAndIRBumper->getGripperRotation()) < sigma))
            {
                platformManipulatorAndIRBumper->manGripperRotateStop();
                setCompleted();
                return;
            }

            /************************************ emulation *********************start***************************/
            int positionADC = platformManipulatorAndIRBumper->getGripperADCRotation();
            if (direction)
            {
                positionADC += 1;
            }
            else
            {
                positionADC -= 1;
            }
            platformManipulatorAndIRBumper->setGripperADCRotation(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    qDebug("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    setCompleted();
}
