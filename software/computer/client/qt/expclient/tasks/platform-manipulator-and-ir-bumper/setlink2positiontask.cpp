#include "setlink2positiontask.h"

SetLink2PositionTask::SetLink2PositionTask(float targetAngle)
{
    angle = targetAngle;
}

bool SetLink2PositionTask::checkFeasibility()
{
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

}

void SetLink2PositionTask::executionWorker()
{
    while (!stopped)
    {
        qDebug("Task#%lu", getTaskId());
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
