#include "valter.h"
#include "setgrippertiltpositiontask.h"

SetGripperTiltPositionTask::SetGripperTiltPositionTask()
{
    qDebugOn = true;
    taskName = "SetGripperTiltPositionTask";
    blocking = false;
}

bool SetGripperTiltPositionTask::checkFeasibility()
{
    if (angle > 45 || angle < -16)
    {
        qDebug("Task#%lu target GripperTiltPosition angle %f in unreachable.", getTaskId(), angle);
        return false;
    }
    return true;
}

bool SetGripperTiltPositionTask::initialize()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        qDebug("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        return false;
    }
    return true;
}

void SetGripperTiltPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetGripperTiltPositionTask::executionWorker, this);
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetGripperTiltPositionTask::stopExecution()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manLink3Stop();
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetGripperTiltPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
}

ITask *SetGripperTiltPositionTask::create()
{
    return (ITask*)new SetGripperTiltPositionTask();
}

float SetGripperTiltPositionTask::getAngle() const
{
    return angle;
}

void SetGripperTiltPositionTask::setAngle(float value)
{
    angle = value;
}

void SetGripperTiltPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    /************************************ emulation *********************start***************************/
    platformManipulatorAndIRBumper->setGripperADCTilt(754); //0 degrees = 754;
    /************************************ emulation *********************finish**************************/

    float sigma = 0.5; //precision in degrees

    bool direction = (angle > platformManipulatorAndIRBumper->getGripperTilt()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 1) : (angle / 1); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(abs(angle) - abs(platformManipulatorAndIRBumper->getGripperTilt())) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //tilt gripper down
            {
                platformManipulatorAndIRBumper->manLink3TiltUp();
            }
            else
            {
                platformManipulatorAndIRBumper->manLink3TiltDown();
            }
            executing = true;
        }
        else
        {

            if (qDebugOn)
            {
                qDebug("Task#%lu: GripperTilt (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), platformManipulatorAndIRBumper->getGripperTilt(), angle, cutoffAngle, abs(abs(angle) - abs(platformManipulatorAndIRBumper->getGripperTilt())), sigma, (direction ? "descent" : "ascent"));
            }

            if ((direction && platformManipulatorAndIRBumper->getGripperTilt() >= cutoffAngle) || ((!direction && platformManipulatorAndIRBumper->getGripperTilt() <= cutoffAngle)) || (abs(abs(angle) - abs(platformManipulatorAndIRBumper->getGripperTilt())) < sigma))
            {
                platformManipulatorAndIRBumper->manLink3Stop();
                setCompleted();
                return;
            }

            /************************************ emulation *********************start***************************/
            int positionADC = platformManipulatorAndIRBumper->getGripperADCTilt();
            if (direction)
            {
                positionADC -= 1;
            }
            else
            {
                positionADC += 1;
            }
            platformManipulatorAndIRBumper->setGripperADCTilt(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    qDebug("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    setCompleted();
}
