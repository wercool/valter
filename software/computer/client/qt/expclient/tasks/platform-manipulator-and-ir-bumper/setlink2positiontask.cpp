#include "valter.h"
#include "setlink2positiontask.h"

float SetLink2PositionTask::prevAngle = 0.0;

SetLink2PositionTask::SetLink2PositionTask()
{
    taskName = "SetLink2PositionTask";
    checkFeasibility();
}

bool SetLink2PositionTask::checkFeasibility()
{
    if (angle < 0 || angle > 90)
    {
        stopExecution();

        qDebug("Task#%lu target Link2 angle in unreachable.", getTaskId());
        return false;
    }
    return true;
}

bool SetLink2PositionTask::initialize()
{
    return true;
}

void SetLink2PositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetLink2PositionTask::executionWorker, this);
        }
    }
}

void SetLink2PositionTask::stopExecution()
{
    stopped = true;
    setCompleted(true);
}

void SetLink2PositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) completed.", getTaskId(), getTaskName().c_str());
}

ITask *SetLink2PositionTask::create()
{
    return (ITask*)new SetLink2PositionTask();
}

void SetLink2PositionTask::executionWorker()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    /************************************ emulation *********************start***************************/
    platformManipulatorAndIRBumper->setLink2ADCPosition(136); //45 degrees = 542;
    /************************************ emulation *********************finish**************************/

    float sigma = 0.5; //precision in degrees
    float cutoffAngle = angle * 0.85; //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< dynamic parameter

    while (!stopped)
    {
        if (!executing)
        {
            if (prevAngle > angle) //move Link2 descending
            {

                if (platformManipulatorAndIRBumper->prepareManLink2Movement())
                {
                    //descent
                    if (platformManipulatorAndIRBumper->setLink2MovementDirection(false))
                    {
                        platformManipulatorAndIRBumper->setLink2MotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    qDebug("Task#%lu could not be completed; reason platformManipulatorAndIRBumper->prepareManLink2Movement()", getTaskId());
                    stopExecution();
                    return;
                }
            }
            else
            {
                if (platformManipulatorAndIRBumper->prepareManLink2Movement())
                {
                    //ascent
                    if (platformManipulatorAndIRBumper->setLink2MovementDirection(true))
                    {
                        platformManipulatorAndIRBumper->setLink2MotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    qDebug("Task#%lu could not be completed; reason platformManipulatorAndIRBumper->prepareManLink2Movement()", getTaskId());
                    stopExecution();
                    return;
                }
            }
        }

        if (qDebugOn)
        {
            qDebug("Task#%lu: link2Position (deg) = %f, target (deg) = %f, dSigma = %f ? %f, cutoff = %f, direction: %s", getTaskId(), platformManipulatorAndIRBumper->getLink2Position(), angle, abs(angle - platformManipulatorAndIRBumper->getLink2Position()), sigma, cutoffAngle, (platformManipulatorAndIRBumper->getLink2MovementDirection() ? "descent" : "ascent"));
        }

        if (abs(angle - platformManipulatorAndIRBumper->getLink2Position()) < sigma)
        {
            setCompleted(true);
            return;
        }
        else
        {
            if (abs(cutoffAngle - platformManipulatorAndIRBumper->getLink2Position()) < sigma)
            {
                if (platformManipulatorAndIRBumper->getLink2MotorActivated())
                {
                    platformManipulatorAndIRBumper->setLink2MotorActivated(false);
                }
            }
        }

        prevAngle = angle;

        /************************************ emulation *********************start***************************/
        int positionADC = platformManipulatorAndIRBumper->getLink2ADCPosition();
        positionADC++;
        positionADC++;
        platformManipulatorAndIRBumper->setLink2ADCPosition(positionADC);
        /************************************ emulation *********************finish**************************/

        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    qDebug("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    setCompleted(true);
}

float SetLink2PositionTask::getAngle() const
{
    return angle;
}

void SetLink2PositionTask::setAngle(float value)
{
    angle = value;
}
