#include "valter.h"
#include "saytask.h"

SayTask::SayTask()
{
    completed = false;
    taskName = "SAY";
    blocking = false;
}

bool SayTask::checkFeasibility()
{
    return true;
}

bool SayTask::initialize()
{
    return true;
}

void SayTask::execute()
{
    executing = true;
    new std::thread(&SayTask::executionWorker, this);
}

void SayTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SayTask::reportCompletion()
{
    qDebug("Task#%lu (%s) is completed.", getTaskId(), getTaskName().c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void SayTask::executionWorker()
{
    std::string cmd = "/home/maska/speech-ru " + getTaskScriptLine();
    Valter::getInstance()->executeShellCmdLinux(cmd);
    setCompleted();
}

