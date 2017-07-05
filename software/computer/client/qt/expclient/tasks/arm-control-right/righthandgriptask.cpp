#include "valter.h"
#include "righthandgriptask.h"

RightHandgripTask::RightHandgripTask()
{
    qDebugOn = true;
    taskName = "RightHandgripTask";
    blocking = false;
    mode = -1;
}

bool RightHandgripTask::checkFeasibility()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (!bodyControlP1->getPowerSource5V5State())
    {
        string msg = Valter::format_string("Task#%lu bodyControlP1->getPowerSource5V5State() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool RightHandgripTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    int mode = atoi(((string)taskInitiationParts[1]).c_str());
    setMode(mode);

    return true;
}

void RightHandgripTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&RightHandgripTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void RightHandgripTask::stopExecution()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->fingersToInitialPositions();
    this_thread::sleep_for(std::chrono::milliseconds(500));
    armControlRight->releaseAllFingers();
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void RightHandgripTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *RightHandgripTask::create()
{
    return (ITask*)new RightHandgripTask();
}

void RightHandgripTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();


    if (mode == 0)
    {
        armControlRight->releaseAllFingers();
    }

    if (mode == 1)
    {
        armControlRight->fingersToInitialPositions();
        this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (mode == 2)
    {
        armControlRight->fingersGrasp();
        this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (mode == 3)
    {
        armControlRight->fingersToInitialPositions();
        this_thread::sleep_for(std::chrono::milliseconds(500));
        armControlRight->fingersGrasp();
    }

    if (mode == 4)
    {
        armControlRight->fingersToInitialPositions();
        this_thread::sleep_for(std::chrono::milliseconds(500));
        armControlRight->releaseAllFingers();
    }

    setCompleted();
}

int RightHandgripTask::getMode() const
{
    return mode;
}

void RightHandgripTask::setMode(int value)
{
    mode = value;
}
