#include "valter.h"
#include "delaytask.h"

DelayTask::DelayTask(unsigned int delay_ms_value)
{
    completed = false;
    taskName = "DELAY";
    delay_ms = delay_ms_value;
    setBlocking(true);
}

bool DelayTask::checkFeasibility()
{
    return true;
}

bool DelayTask::initialize()
{
    return true;
}

void DelayTask::execute()
{
    executing = true;
    new std::thread(&DelayTask::executionWorker, this);
}

void DelayTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void DelayTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %dms completed.", getTaskId(), getTaskName().c_str(), delay_ms);
    Valter::getInstance()->getTaskManager()->sendMessageToCentralHostTaskManager(Valter::format_string("RTMM~%lu~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", (completed) ? "completed" : ((executing) ? "executing" : "queued")));
}

void DelayTask::executionWorker()
{
    qDebug("Task#%lu (%s) sleeping for %dms...", getTaskId(), getTaskName().c_str(), delay_ms);
    unsigned int t = 0;
    while ((t < delay_ms) && (!stopped))
    {
        this_thread::sleep_for(std::chrono::milliseconds(1));
        t++;
    }
    setCompleted();
}
