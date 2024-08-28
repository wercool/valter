#include "valter.h"
#include "leftforearmrollmotoractivatetask.h"

LeftForearmRollMotorActivateTask::LeftForearmRollMotorActivateTask()
{
    completed = false;
    taskName = "LeftForearmRollMotorActivateTask";
    setBlocking(true);
}

ITask *LeftForearmRollMotorActivateTask::create()
{
    return (ITask*)new LeftForearmRollMotorActivateTask();
}

bool LeftForearmRollMotorActivateTask::checkFeasibility()
{
    return true;
}

bool LeftForearmRollMotorActivateTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    unsigned int state_value = atoi(((string)taskInitiationParts[1]).c_str());
    setState(state_value);

    return true;
}

void LeftForearmRollMotorActivateTask::execute()
{
    executing = true;
    new std::thread(&LeftForearmRollMotorActivateTask::executionWorker, this);
}

void LeftForearmRollMotorActivateTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void LeftForearmRollMotorActivateTask::reportCompletion()
{
    qDebug("Task#%lu (%s) completed; Left Forearm Roll Motor %s.", getTaskId(), getTaskName().c_str(), getState() == 1 ? "activated" : "deactivated");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void LeftForearmRollMotorActivateTask::executionWorker()
{
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
    qDebug("Task#%lu (%s) %s Left Forearm Roll Motor...", getTaskId(), getTaskName().c_str(), getState() == 1 ? "activating" : "deactivating");

    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();

    if (getState() == 1) {
        armControlLeft->setForearmRollMotorOnOff(true);
    } else {
        armControlLeft->setForearmRollMotorOnOff(false);
    }

    setCompleted();
}

unsigned int LeftForearmRollMotorActivateTask::getState() const
{
    return state;
}

void LeftForearmRollMotorActivateTask::setState(unsigned int state_value)
{
    state = state_value;
}
