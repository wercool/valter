#include "valter.h"
#include "trasnslateplatformlinearlytask.h"

signed int TrasnslatePlatformLinearlyTask::prevDirection = -1;

TrasnslatePlatformLinearlyTask::TrasnslatePlatformLinearlyTask()
{
    direction = -1;

    qDebugOn = true;
    taskName = "TrasnslatePlatformLinearlyTask";
    blocking = false;
}

bool TrasnslatePlatformLinearlyTask::checkFeasibility()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (direction < 0)
    {
        qDebug("Task#%lu (%s) could not be performed. Direction is undefined.", getTaskId(), getTaskName().c_str());
        return false;
    }
    if (direction == prevDirection)
    {
        if (platformControlP1->getRightMotorActivated() || platformControlP1->getLeftMotorActivated())
        {
            qDebug("Task#%lu (%s) could not be performed. Saltatory inversion of movement direction.", getTaskId(), getTaskName().c_str());
            return false;
        }
    }
    return true;
}

bool TrasnslatePlatformLinearlyTask::initialize()
{
    qDebug("Task#%lu (%s) direction = %s, distance = %f", getTaskId(), getTaskName().c_str(), (direction > 0 && direction != -1) ? "forward" : "backward", distance);
    return true;
}

void TrasnslatePlatformLinearlyTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&TrasnslatePlatformLinearlyTask::executionWorker, this);
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
            stopExecution();
            setCompleted();
        }
    }
}

void TrasnslatePlatformLinearlyTask::stopExecution()
{

}

void TrasnslatePlatformLinearlyTask::reportCompletion()
{

}

ITask *TrasnslatePlatformLinearlyTask::create()
{
    return (ITask*)new TrasnslatePlatformLinearlyTask();
}

void TrasnslatePlatformLinearlyTask::executionWorker()
{
    while (!stopped)
    {
        if (!executing)
        {
            //temp
            this_thread::sleep_for(std::chrono::milliseconds(5000));

            setCompleted();
            return;
        }
    }
    qDebug("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    setCompleted();
}

void TrasnslatePlatformLinearlyTask::setDirection(signed int value)
{
    direction = value;
}

void TrasnslatePlatformLinearlyTask::setDistance(float value)
{
    distance = value;
}
