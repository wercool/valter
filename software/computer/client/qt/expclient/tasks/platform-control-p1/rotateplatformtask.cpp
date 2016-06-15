#include "valter.h"
#include "rotateplatformtask.h"

signed int RotatePlatformTask::prevDirection = -1;

RotatePlatformTask::RotatePlatformTask()
{
    qDebugOn = true;
    taskName = "RotatePlatformTask";
    blocking = false;

    direction = -1;
}

bool RotatePlatformTask::checkFeasibility()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (direction < 0)
    {
        qDebug("Task#%lu (%s) could not be performed. Rotation direction is undefined.", getTaskId(), getTaskName().c_str());
        return false;
    }
    if (direction == prevDirection)
    {
        if (platformControlP1->getTurretMotorActivated())
        {
            qDebug("Task#%lu (%s) could not be performed. Saltatory inversion of rotation direction.", getTaskId(), getTaskName().c_str());
            return false;
        }
    }
    return true;
}

bool RotatePlatformTask::initialize()
{

}

void RotatePlatformTask::execute()
{

}

void RotatePlatformTask::stopExecution()
{

}

void RotatePlatformTask::reportCompletion()
{

}

void RotatePlatformTask::executionWorker()
{

}

void RotatePlatformTask::setDirection(int value)
{
    direction = value;
    prevDirection = direction;
}
