#include "valter.h"
#include "wikitalktask.h"

WikiTalkTask::WikiTalkTask()
{
    completed = false;
    taskName = "WIKITALK";
    blocking = false;
}

bool WikiTalkTask::checkFeasibility()
{
    return true;
}

bool WikiTalkTask::initialize()
{
    return true;
}

void WikiTalkTask::execute()
{
    executing = true;
    new std::thread(&WikiTalkTask::executionWorker, this);
}

void WikiTalkTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void WikiTalkTask::reportCompletion()
{
    qDebug("Task#%lu (%s) is completed.", getTaskId(), getTaskName().c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void WikiTalkTask::executionWorker()
{
    std::string cmd = "/home/maska/git/valter/software/raspberrypi3/ValterTalks/wiki-question-talk.py " + getTaskScriptLine();
    Valter::getInstance()->executeShellCmdLinux(cmd);
    setCompleted();
}

