#include "valter.h"
#include "setlink1positiontask.h"

SetLink1PositionTask::SetLink1PositionTask()
{
    qDebugOn = true;
    taskName = "SetLink1PositionTask";
    blocking = false;
    checkFeasibility();
}

bool SetLink1PositionTask::checkFeasibility()
{
    if (angle < 0 || angle > 71)
    {
        qDebug("Task#%lu target Link1 angle %f in unreachable.", getTaskId(), angle);
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
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
            stopExecution();
        }
    }
}

void SetLink1PositionTask::stopExecution()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink1MotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
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
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    /************************************ emulation *********************start***************************/
//    platformManipulatorAndIRBumper->setLink1ADCPosition(512); //35.5 degrees = 512;
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    //descending (angle increased) - true
    bool direction = (angle > platformManipulatorAndIRBumper->getLink1Position()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 0.98) : (angle / 0.98); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(angle - platformManipulatorAndIRBumper->getLink1Position()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //move Link1 ascending
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
        else
        {

            if (qDebugOn)
            {
                qDebug("Task#%lu: link1Position (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), platformManipulatorAndIRBumper->getLink1Position(), angle, cutoffAngle, abs(angle - platformManipulatorAndIRBumper->getLink1Position()), sigma, (platformManipulatorAndIRBumper->getLink1MovementDirection() ? "ascent" : "descent"));
            }

            if ((direction && platformManipulatorAndIRBumper->getLink1Position() >= cutoffAngle) || ((!direction && platformManipulatorAndIRBumper->getLink1Position() <= cutoffAngle)) || (abs(angle - platformManipulatorAndIRBumper->getLink1Position()) < sigma))
            {
                if (platformManipulatorAndIRBumper->getLink1MotorActivated())
                {
                    platformManipulatorAndIRBumper->setLink1MotorActivated(false);
                    setCompleted();
                    return;
                }
            }

            /************************************ emulation *********************start***************************/
//            int positionADC = platformManipulatorAndIRBumper->getLink1ADCPosition();
//            if (direction)
//            {
//                positionADC -= 5;
//            }
//            else
//            {
//                positionADC += 5;
//            }
//            platformManipulatorAndIRBumper->setLink1ADCPosition(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    qDebug("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    setCompleted();
}

float SetLink1PositionTask::getAngle() const
{
    return angle;
}

void SetLink1PositionTask::setAngle(float value)
{
    angle = value;
}
