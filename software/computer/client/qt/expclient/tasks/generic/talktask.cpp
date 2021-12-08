#include "valter.h"
#include "talktask.h"

TalkTask::TalkTask()
{
    completed = false;
    taskName = "TALK";
    blocking = false;
}

bool TalkTask::checkFeasibility()
{
    return true;
}

bool TalkTask::initialize()
{
    return true;
}

void TalkTask::execute()
{
    executing = true;
    new std::thread(&TalkTask::executionWorker, this);
}

void TalkTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void TalkTask::reportCompletion()
{
    qDebug("Task#%lu (%s) is completed.", getTaskId(), getTaskName().c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void TalkTask::executionWorker()
{
    std::string cmd = "/home/maska/say.sh " + getTaskScriptLine();
    Valter::getInstance()->executeShellCmdLinux(cmd);
    setCompleted();
}

