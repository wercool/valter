#include "valter.h"
#include "rotatebodytask.h"

signed int RotateBodyTask::prevDirection = -1;

RotateBodyTask::RotateBodyTask()
{
    qDebugOn = true;
    taskName = "RotateBodyTasks";
    blocking = false;

    direction = -1;
    angle = -1;
}

bool RotateBodyTask::checkFeasibility()
{

}

bool RotateBodyTask::initialize()
{

}

void RotateBodyTask::execute()
{

}

void RotateBodyTask::stopExecution()
{

}

void RotateBodyTask::reportCompletion()
{

}

ITask *RotateBodyTask::create()
{
    return (ITask*)new RotateBodyTask();
}

void RotateBodyTask::setDirection(int value)
{
    direction = value;
    prevDirection = direction;
}

void RotateBodyTask::setAngle(float value)
{
    angle = value;
}

void RotateBodyTask::executionWorker()
{

}

