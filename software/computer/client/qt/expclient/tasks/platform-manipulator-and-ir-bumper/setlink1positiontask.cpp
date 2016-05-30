#include "valter.h"
#include "setlink1positiontask.h"

float SetLink1PositionTask::prevAngle = 0.0;

SetLink1PositionTask::SetLink1PositionTask()
{
    taskName = "SetLink1PositionTask";
    checkFeasibility();
}

bool SetLink1PositionTask::checkFeasibility()
{
    if (angle < 0 || angle > 64)
    {
        stopExecution();

        qDebug("Task#%lu target Link1 angle in unreachable.", getTaskId());
        return false;
    }
    return true;
}

bool SetLink1PositionTask::initialize()
{
    return true;
}

void SetLink1PositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetLink1PositionTask::executionWorker, this);
        }
    }
}

void SetLink1PositionTask::stopExecution()
{
    stopped = true;
    setCompleted(true);
}

void SetLink1PositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
}

ITask *SetLink1PositionTask::create()
{
    return (ITask*)new SetLink1PositionTask();
}

void SetLink1PositionTask::executionWorker()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    /************************************ emulation *********************start***************************/
    platformManipulatorAndIRBumper->setLink1ADCPosition(0); //32 degrees = 512;
    /************************************ emulation *********************finish**************************/

    float sigma = 0.5; //precision in degrees
    float cutoffAngle = angle * 0.85; //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< dynamic parameter

    while (!stopped)
    {
        if (!executing)
        {
            if (prevAngle > angle) //move Link1 ascending
            {
                if (platformManipulatorAndIRBumper->prepareManLink1Movement())
                {
                    //ascent
                    if (platformManipulatorAndIRBumper->setLink1MovementDirection(true))
                    {
                        platformManipulatorAndIRBumper->setLink1MotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    qDebug("Task#%lu could not be completed; reason platformManipulatorAndIRBumper->prepareManLink1Movement()", getTaskId());
                    stopExecution();
                    return;
                }
            }
            else
            {
                if (platformManipulatorAndIRBumper->prepareManLink1Movement())
                {
                    //descent
                    if (platformManipulatorAndIRBumper->setLink1MovementDirection(false))
                    {
                        platformManipulatorAndIRBumper->setLink1MotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    qDebug("Task#%lu could not be completed; reason platformManipulatorAndIRBumper->prepareManLink1Movement()", getTaskId());
                    stopExecution();
                    return;
                }
            }
        }

        if (qDebugOn)
        {
            qDebug("Task#%lu: link1Position (deg) = %f, target (deg) = %f, dSigma = %f ? %f, cutoff = %f, direction: %s", getTaskId(), platformManipulatorAndIRBumper->getLink1Position(), angle, abs(angle - platformManipulatorAndIRBumper->getLink1Position()), sigma, cutoffAngle, (platformManipulatorAndIRBumper->getLink1MovementDirection() ? "descent" : "ascent"));
        }

        if (abs(angle - platformManipulatorAndIRBumper->getLink1Position()) < sigma)
        {
            setCompleted(true);
            return;
        }
        else
        {
            if (abs(cutoffAngle - platformManipulatorAndIRBumper->getLink1Position()) < sigma)
            {
                if (platformManipulatorAndIRBumper->getLink1MotorActivated())
                {
                    platformManipulatorAndIRBumper->setLink1MotorActivated(false);
                }
            }
        }

        prevAngle = angle;

        /************************************ emulation *********************start***************************/
        int positionADC = platformManipulatorAndIRBumper->getLink1ADCPosition();
        positionADC++;
        positionADC++;
        platformManipulatorAndIRBumper->setLink1ADCPosition(positionADC);
        /************************************ emulation *********************finish**************************/

        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    qDebug("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    setCompleted(true);
}

float SetLink1PositionTask::getAngle() const
{
    return angle;
}

void SetLink1PositionTask::setAngle(float value)
{
    angle = value;
}
