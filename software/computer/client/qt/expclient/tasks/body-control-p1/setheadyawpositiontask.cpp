#include "valter.h"
#include "setheadyawpositiontask.h"

SetHeadYawPositionTask::SetHeadYawPositionTask()
{
    qDebugOn = true;
    taskName = "SetHeadYawPositionTask";
    blocking = false;
    attachable = true;
}

bool SetHeadYawPositionTask::checkFeasibility()
{
    if (!TaskManager::getInstance()->getEmulation())
    {
        PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
        if (!platformControlP1->getPower5VOnState())
        {
            string msg = Valter::format_string("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
            qDebug("%s", msg.c_str());
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
            return false;
        }
        BodyControlP1  *bodyControlP1 = BodyControlP1::getInstance();
        if (!bodyControlP1->getHead24VState())
        {
            string msg = Valter::format_string("Task#%lu (%s) could not be executed. 24V head power is OFF", getTaskId(), getTaskName().c_str());
            qDebug("%s", msg.c_str());
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
            return false;
        }
        if (!bodyControlP1->getHeadYawMotorState())
        {
            string msg = Valter::format_string("Task#%lu (%s) could not be executed. Head Yaw motor is DISABLED", getTaskId(), getTaskName().c_str());
            qDebug("%s", msg.c_str());
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
            return false;
        }
    }

    if (angle < -80 || angle > 80)
    {
        string msg = Valter::format_string("Task#%lu target Head Yaw angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetHeadYawPositionTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    //T_BCP1_SetHeadYawPositionTask_15
    float _angle = atof(((string)taskInitiationParts[1]).c_str());
    setAngle(_angle);

    stopped = !checkFeasibility();
    //right(CW) - true, left(CCW) - false (in CW angle is positive)
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    direction = (_angle > bodyControlP1->getHeadYawPosition()) ? true : false;

    string msg = Valter::format_string("Task#%lu Head Yaw direction [(a ? cur) %.2f ? %.2f] == [%s]", getTaskId(), _angle, bodyControlP1->getHeadYawPosition(), (direction ? "RIGHT" : "LEFT"));
    qDebug("%s", msg.c_str());

    bodyControlP1->setHeadYawMotorActivated(false);
    bodyControlP1->setHeadYawDirection(direction);
    bodyControlP1->setHeadYawMotorActivated(true);

    if (executing)
    {
        string msg = Valter::format_string("Task#%lu Head Yaw angle %f set via attach.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
    }

    return true;
}

void SetHeadYawPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetHeadYawPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s, %s~%s~%s",
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

void SetHeadYawPositionTask::stopExecution()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadYawMotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetHeadYawPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s",
                                                                                          getTaskId(),
                                                                                          getTaskName().c_str(),
                                                                                          (blocking) ? "blocking" : "non blocking",
                                                                                          ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))),
                                                                                          getTaskScriptLine().c_str()));
}

ITask *SetHeadYawPositionTask::create()
{
    return (ITask*)new SetHeadYawPositionTask();
}

void SetHeadYawPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());

    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();

    string stopExecutionReason = "";

    /************************************ emulation *********************start***************************/
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    while (!stopped)
    {
        if (!executing)
        {
            if (!checkFeasibility())
            {
                stopExecutionReason = Valter::format_string("Task#%lu (%s) has been terminated. checkFeasibility() returns FALSE.", getTaskId(), getTaskName().c_str());
                qDebug("%s", stopExecutionReason.c_str());
                stopExecution();
                continue;
            }
            else
            {
                bodyControlP1->requestHeadYawPosition();

                executing = true;

                string msg = Valter::format_string("Task#%lu (%s) executing now..", getTaskId(), getTaskName().c_str());
                qDebug("%s", msg.c_str());
            }
        }
        else
        {
            bodyControlP1->requestHeadYawPosition();

            if ((abs(angle - bodyControlP1->getHeadYawPosition()) < sigma) || (bodyControlP1->getHeadYawPosition() > 80 && direction) || (bodyControlP1->getHeadYawPosition() < -80 && !direction))
            {
                if (bodyControlP1->getHeadYawMotorActivated())
                {
                    string msg = Valter::format_string("Task#%lu (%s) position [%.2f] reached", getTaskId(), getTaskName().c_str(), bodyControlP1->getHeadYawPosition());
                    qDebug("%s", msg.c_str());
                }
                bodyControlP1->setHeadYawMotorActivated(false);
            }
/************************************ emulation *********************start***************************/
string msg = Valter::format_string("Task#%lu (%s) positioning [%.2f]...", getTaskId(), getTaskName().c_str(), bodyControlP1->getHeadYawPosition());
qDebug("%s", msg.c_str());
/************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    bodyControlP1->setHeadYawMotorActivated(false);

    if (!getCompleted())
    {
        string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal.%s", getTaskId(), ((stopExecutionReason.length() > 0) ? Valter::format_string(" REASON: %s", stopExecutionReason.c_str()).c_str() : ""));
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        setCompleted();
    }
}

float SetHeadYawPositionTask::getAngle() const
{
    return angle;
}

void SetHeadYawPositionTask::setAngle(float value)
{
    std::lock_guard<std::mutex> lock(values_mutex);
    angle = value;
}
