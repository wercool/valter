#include "valter.h"
#include "setgrippergrasperposition.h"

SetGripperGrasperPosition::SetGripperGrasperPosition()
{
    qDebugOn = true;
    taskName = "SetGripperGrasperPosition";
    blocking = false;
}

bool SetGripperGrasperPosition::checkFeasibility()
{
    if (position > 105 || position < 0)
    {
        qDebug("Task#%lu (%s) target position %f in unreachable.", getTaskId(), getTaskName().c_str(), position);
        return false;
    }
    return true;
}

bool SetGripperGrasperPosition::initialize()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        qDebug("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        return false;
    }
    return true;
}

void SetGripperGrasperPosition::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetGripperGrasperPosition::executionWorker, this);
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetGripperGrasperPosition::stopExecution()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperStop();
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetGripperGrasperPosition::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
}

ITask *SetGripperGrasperPosition::create()
{
    return (ITask*)new SetGripperGrasperPosition();
}

float SetGripperGrasperPosition::getPosition() const
{
    return position;
}

void SetGripperGrasperPosition::setPosition(float value)
{
    position = value;
}

void SetGripperGrasperPosition::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    /************************************ emulation *********************start***************************/
    platformManipulatorAndIRBumper->setGripperADCPosition(300); //0 (opened) = 300;
    /************************************ emulation *********************finish**************************/

    float sigma = 0.25; //precision in mm

    //opening - true
    bool direction = (position > platformManipulatorAndIRBumper->getGripperPosition()) ? true : false;

    float cutoffPosition = (direction) ? (position * 1) : (position / 1); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(position - platformManipulatorAndIRBumper->getGripperPosition()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //open grasper
            {
                platformManipulatorAndIRBumper->manGripperOpen();
            }
            else
            {
                platformManipulatorAndIRBumper->manGripperClose();
            }
            executing = true;
        }
        else
        {

            if (qDebugOn)
            {
                qDebug("Task#%lu: GripperGrasperPosition (mm) = %f, target (mm) = %f, cutoffPosition = %f, dSigma = %f ? %f, direction: %s", getTaskId(), platformManipulatorAndIRBumper->getGripperPosition(), position, cutoffPosition, abs(position - platformManipulatorAndIRBumper->getGripperPosition()), sigma, (direction ? "openning" : "closing"));
            }

            if ((direction && platformManipulatorAndIRBumper->getGripperPosition() >= cutoffPosition) || ((!direction && platformManipulatorAndIRBumper->getGripperPosition() <= cutoffPosition)) || (abs(position - platformManipulatorAndIRBumper->getGripperPosition()) < sigma))
            {
                platformManipulatorAndIRBumper->manGripperStop();
                setCompleted();
                return;
            }

            /************************************ emulation *********************start***************************/
            int positionADC = platformManipulatorAndIRBumper->getGripperADCPosition();
            if (direction)
            {
                positionADC -= 1;
            }
            else
            {
                positionADC += 1;
            }
            platformManipulatorAndIRBumper->setGripperADCPosition(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    qDebug("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    setCompleted();
}
