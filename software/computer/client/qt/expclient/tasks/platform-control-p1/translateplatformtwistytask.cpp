#include "valter.h"
#include "translateplatformtwistytask.h"

TranslatePlatformTwistyTask::TranslatePlatformTwistyTask()
{
    qDebugOn = true;
    taskName = "TranslatePlatformTwistyTask";
    blocking = false;
    attachable = true;
}

bool TranslatePlatformTwistyTask::checkFeasibility()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    return true;
}

bool TranslatePlatformTwistyTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];

    return true;
}

void TranslatePlatformTwistyTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&TranslatePlatformTwistyTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s,%s~%s~%s",
                                                                                                  getTaskId(),
                                                                                                  getTaskName().c_str(),
                                                                                                  (blocking) ? "blocking" : "non blocking",
                                                                                                  "attachable",
                                                                                                  ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))),
                                                                                                  getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void TranslatePlatformTwistyTask::stopExecution()
{

}

void TranslatePlatformTwistyTask::reportCompletion()
{

}

ITask *TranslatePlatformTwistyTask::create()
{
    return (ITask*)new TranslatePlatformTwistyTask();
}

void TranslatePlatformTwistyTask::executionWorker()
{
    //TODO: logic
    //temp stub
    this_thread::sleep_for(std::chrono::milliseconds(1000));
    setCompleted();
}
