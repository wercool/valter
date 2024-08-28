#include "valter.h"
#include "headmotorsactivatetask.h"

HeadMotorsActivateTask::HeadMotorsActivateTask()
{
    completed = false;
    taskName = "HeadMotorsActivateTask";
    setBlocking(true);
}

ITask *HeadMotorsActivateTask::create()
{
    return (ITask*)new HeadMotorsActivateTask();
}

bool HeadMotorsActivateTask::checkFeasibility()
{
    return true;
}

bool HeadMotorsActivateTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    unsigned int state_value = atoi(((string)taskInitiationParts[1]).c_str());
    qDebug("Task#%lu (%s): received state (str) = '%s', state = %d", getTaskId(), getTaskName().c_str(), taskInitiationParts[1].c_str(), state_value);
    setState(state_value);

    return true;
}

void HeadMotorsActivateTask::execute()
{
    executing = true;
    new std::thread(&HeadMotorsActivateTask::executionWorker, this);
}

void HeadMotorsActivateTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void HeadMotorsActivateTask::reportCompletion()
{
    qDebug("Task#%lu (%s) completed; Head Motors %s, state = %d.", getTaskId(), getTaskName().c_str(), getState() == 1 ? "activated" : "deactivated", getState());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void HeadMotorsActivateTask::executionWorker()
{
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
    qDebug("Task#%lu (%s) %s head motors...", getTaskId(), getTaskName().c_str(), getState() == 1 ? "activating" : "deactivating");

    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();

    if (getState() == 1) {
        bodyControlP1->setHead24VOnOff(true);
        bodyControlP1->setHeadYawMotorOnOff(true);
        bodyControlP1->setHeadPitchMotorOnOff(true);
    } else {
        bodyControlP1->setHead24VOnOff(false);
        bodyControlP1->setHeadYawMotorOnOff(false);
        bodyControlP1->setHeadPitchMotorOnOff(false);
    }

    setCompleted();
}

unsigned int HeadMotorsActivateTask::getState() const
{
    return state;
}

void HeadMotorsActivateTask::setState(unsigned int state_value)
{
    state = state_value;
}
