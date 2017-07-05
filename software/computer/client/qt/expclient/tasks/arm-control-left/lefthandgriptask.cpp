#include "valter.h"
#include "lefthandgriptask.h"

LeftHandgripTask::LeftHandgripTask()
{
    qDebugOn = true;
    taskName = "LeftHandgripTask";
    blocking = false;
    mode = -1;
}

bool LeftHandgripTask::checkFeasibility()
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

bool LeftHandgripTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    int mode = atoi(((string)taskInitiationParts[1]).c_str());
    setMode(mode);

    return true;
}

void LeftHandgripTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&LeftHandgripTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void LeftHandgripTask::stopExecution()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->fingersToInitialPositions();
    this_thread::sleep_for(std::chrono::milliseconds(500));
    armControlLeft->releaseAllFingers();
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void LeftHandgripTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *LeftHandgripTask::create()
{
    return (ITask*)new LeftHandgripTask();
}

void LeftHandgripTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();


    if (mode == 0)
    {
        armControlLeft->releaseAllFingers();
    }

    if (mode == 1)
    {
        armControlLeft->fingersToInitialPositions();
        this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (mode == 2)
    {
        armControlLeft->fingersGrasp();
        this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (mode == 3)
    {
        armControlLeft->fingersToInitialPositions();
        this_thread::sleep_for(std::chrono::milliseconds(500));
        armControlLeft->fingersGrasp();
    }

    if (mode == 4)
    {
        armControlLeft->fingersToInitialPositions();
        this_thread::sleep_for(std::chrono::milliseconds(500));
        armControlLeft->releaseAllFingers();
    }

    setCompleted();
}

int LeftHandgripTask::getMode() const
{
    return mode;
}

void LeftHandgripTask::setMode(int value)
{
    mode = value;
}
