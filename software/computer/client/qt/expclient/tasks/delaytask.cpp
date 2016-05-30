#include "delaytask.h"

DelayTask::DelayTask(unsigned int delay_ms_value)
{
    delay_ms = delay_ms_value;
    setBlocking(true);
}

bool DelayTask::checkFeasibility()
{

}

bool DelayTask::initialize()
{

}

void DelayTask::execute()
{
    new std::thread(&DelayTask::executionWorker, this);
}

void DelayTask::stopExecution()
{

}

void DelayTask::reportCompletion()
{

}

void DelayTask::executionWorker()
{
    this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    qDebug("DELAY TASK %d ms", delay_ms);
}
