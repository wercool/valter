#include "valter.h"
#include "setrightarmrollpositiontask.h"

SetRightArmRollPositionTask::SetRightArmRollPositionTask()
{
    qDebugOn = true;
    taskName = "SetRightArmRollPositionTask";
    blocking = false;
}

bool SetRightArmRollPositionTask::checkFeasibility()
{
//    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
//    if (!bodyControlP1->prepareLeftArmYawMovement())
//    {
//        string msg = Valter::format_string("Task#%lu bodyControlP1->prepareLeftArmYawMovement() returned false in checkFeasibility().", getTaskId());
//        qDebug("%s", msg.c_str());
//        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

//        return false;
//    }
//    if (angle < 0 || angle > 55)
//    {
//        string msg = Valter::format_string("Task#%lu target Left Arm Yaw angle %f in unreachable.", getTaskId(), angle);
//        qDebug("%s", msg.c_str());
//        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

//        return false;
//    }
    return true;
}

bool SetRightArmRollPositionTask::initialize()
{
//    //script line parsing
//    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
//    std::string taskName = taskInitiationParts[0];
//    float angle = atof(((string)taskInitiationParts[1]).c_str());
//    setAngle(angle);

///************************************ emulation *********************start***************************/
////    return true;
///************************************ emulation *********************finish**************************/

//    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
//    if (!platformControlP1->getPower5VOnState())
//    {
//        qDebug("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
//        return false;
//    }
    return true;
}

void SetRightArmRollPositionTask::execute()
{
//    if (initialize())
//    {
//        if (checkFeasibility())
//        {
//            new std::thread(&SetLeftArmYawPositionTask::executionWorker, this);
//            this_thread::sleep_for(std::chrono::milliseconds(100));
//            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
//            return;
//        }
//    }
//    this_thread::sleep_for(std::chrono::milliseconds(100));
//    stopExecution();
//    setCompleted();
}

void SetRightArmRollPositionTask::stopExecution()
{
//    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
//    bodyControlP1->setLeftArmYawMotorActivated(false);
//    stopped = true;
//    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetRightArmRollPositionTask::reportCompletion()
{
//    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
//    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetRightArmRollPositionTask::create()
{
    return (ITask*)new SetRightArmRollPositionTask();
}

void SetRightArmRollPositionTask::executionWorker()
{

}

float SetRightArmRollPositionTask::getAngle() const
{
    return angle;
}

void SetRightArmRollPositionTask::setAngle(float value)
{
    angle = value;
}
