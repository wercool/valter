#include "valter.h"
#include "stoptalktask.h"

StopTalkTask::StopTalkTask()
{
    completed = false;
    taskName = "STOPTALK";
    blocking = false;
}

bool StopTalkTask::checkFeasibility()
{
    return true;
}

bool StopTalkTask::initialize()
{
    return true;
}

void StopTalkTask::execute()
{
    executing = true;
    new std::thread(&StopTalkTask::executionWorker, this);
}

void StopTalkTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void StopTalkTask::reportCompletion()
{
    qDebug("Task#%lu (%s) is completed.", getTaskId(), getTaskName().c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void StopTalkTask::executionWorker()
{
    std::string festival_kill_cmd = "killall festival";
    Valter::getInstance()->executeShellCmdLinux(festival_kill_cmd);

    std::string aplay_kill_cmd = "killall aplay";
    Valter::getInstance()->executeShellCmdLinux(aplay_kill_cmd);

    setCompleted();
}

