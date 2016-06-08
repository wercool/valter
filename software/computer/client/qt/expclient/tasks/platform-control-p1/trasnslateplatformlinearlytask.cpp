#include "trasnslateplatformlinearlytask.h"

TrasnslatePlatformLinearlyTask::TrasnslatePlatformLinearlyTask()
{
    qDebugOn = true;
    taskName = "TrasnslatePlatformLinearlyTask";
    blocking = false;
    checkFeasibility();
}

bool TrasnslatePlatformLinearlyTask::checkFeasibility()
{
    return true;
}

bool TrasnslatePlatformLinearlyTask::initialize()
{

}

void TrasnslatePlatformLinearlyTask::execute()
{

}

void TrasnslatePlatformLinearlyTask::stopExecution()
{

}

void TrasnslatePlatformLinearlyTask::reportCompletion()
{

}

ITask *TrasnslatePlatformLinearlyTask::create()
{

}

void TrasnslatePlatformLinearlyTask::executionWorker()
{

}

void TrasnslatePlatformLinearlyTask::setDistance(float value)
{
    distance = value;
}
