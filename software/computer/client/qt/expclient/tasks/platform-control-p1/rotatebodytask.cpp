#include "valter.h"
#include "rotatebodytask.h"

signed int RotateBodyTask::prevDirection = -1;

RotateBodyTask::RotateBodyTask()
{
    qDebugOn = true;
    taskName = "RotateBodyTasks";
    blocking = false;

    direction = -1; //1- CW (right), 2 - CCW (left)
    angle = -1;

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    initialTurretMotorMaxDuty = platformControlP1->getTurretMotorDutyMax();
}

bool RotateBodyTask::checkFeasibility()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    if (direction < 0)
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be performed. Rotation direction is undefined.", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    if (angle < 0)
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be performed. Rotation angle is undefined.", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    if (direction != prevDirection)
    {
        if (platformControlP1->getTurretMotorActivated())
        {
            string msg = Valter::format_string("Task#%lu (%s) could not be performed. Saltatory inversion of rotation direction.", getTaskId(), getTaskName().c_str());
            qDebug("%s", msg.c_str());
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
            return false;
        }
    }
    return true;
}

bool RotateBodyTask::initialize()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getTurretPositionRead())
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be executed. Body rotation Encoder readings is OFF", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    qDebug("Task#%lu (%s) rotation direction = %s, angle = %f", getTaskId(), getTaskName().c_str(), (direction > 0 && direction != -1) ? "right" : "left", angle);
    return true;
}

void RotateBodyTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&RotateBodyTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void RotateBodyTask::stopExecution()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->setTurretMotorActivated(false);
    stopped = true;
    platformControlP1->setTurretMotorDutyMax(initialTurretMotorMaxDuty);
}

void RotateBodyTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *RotateBodyTask::create()
{
    return (ITask*)new RotateBodyTask();
}

void RotateBodyTask::setDirection(int value)
{
    direction = value;
    prevDirection = direction;
}

void RotateBodyTask::setAngle(float value)
{
    angle = value;
}

void RotateBodyTask::executionWorker()
{
    //TODO: logic
    //temp stub
    this_thread::sleep_for(std::chrono::milliseconds(500));
    stopExecution();
    this_thread::sleep_for(std::chrono::milliseconds(100));
    setCompleted();
}
