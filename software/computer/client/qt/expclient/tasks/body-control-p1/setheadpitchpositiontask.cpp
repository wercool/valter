#include "valter.h"
#include "setheadpitchpositiontask.h"

SetHeadPitchPositionTask::SetHeadPitchPositionTask()
{
    qDebugOn = true;
    taskName = "SetHeadPitchPositionTask";
    blocking = false;
    attachable = true;
}

bool SetHeadPitchPositionTask::checkFeasibility()
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
        if (!bodyControlP1->getHeadPitchMotorState())
        {
            string msg = Valter::format_string("Task#%lu (%s) could not be executed. Head Pitch motor is DISABLED", getTaskId(), getTaskName().c_str());
            qDebug("%s", msg.c_str());
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
            return false;
        }
    }

    if (angle < 0 || angle > 45)
    {
        string msg = Valter::format_string("Task#%lu target Head Pitch angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }

    return true;
}

bool SetHeadPitchPositionTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    //T_BCP1_SetHeadPitchPositionTask_15
    float _angle = atof(((string)taskInitiationParts[1]).c_str());
    setAngle(_angle);

    if (executing)
    {
        string msg = Valter::format_string("Task#%lu Head Pitch angle %f set via attach.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
    }

    return true;
}

void SetHeadPitchPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetHeadPitchPositionTask::executionWorker, this);
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

void SetHeadPitchPositionTask::stopExecution()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadPitchMotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetHeadPitchPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s",
                                                                                          getTaskId(),
                                                                                          getTaskName().c_str(),
                                                                                          (blocking) ? "blocking" : "non blocking",
                                                                                          ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))),
                                                                                          getTaskScriptLine().c_str()));
}

ITask *SetHeadPitchPositionTask::create()
{
    return (ITask*)new SetHeadPitchPositionTask();
}

void SetHeadPitchPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());

    bool prevMotorDirection  = true; //right(CW)
    bool resetStart = false;

    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();

    string stopExecutionReason = "";

    /************************************ emulation *********************start***************************/
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    while (!stopped)
    {
        stopped = !checkFeasibility();

        if (!stopped)
        {
            //right(CW) - true, left(CCW) - false (in CW angle is positive)
            bool direction = (angle > bodyControlP1->getHeadPitchPosition()) ? true : false;

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
                    executing = true;
                    resetStart = true;

                    string msg = Valter::format_string("Task#%lu (%s) executing now..", getTaskId(), getTaskName().c_str());
                    qDebug("%s", msg.c_str());
                }
            }
            else
            {
                if (direction != prevMotorDirection || resetStart)
                {
                    resetStart = false;
                    bodyControlP1->setHeadPitchMotorActivated(false);
                    bodyControlP1->setHeadPitchDirection(direction);
                    bodyControlP1->setHeadPitchMotorActivated(true);
                    prevMotorDirection = direction;
                    string msg = Valter::format_string("Task#%lu (%s) direction changed [%s]", getTaskId(), getTaskName().c_str(), (direction ? "R" : "L"));
                    qDebug("%s", msg.c_str());
                }

                if ((abs(angle - bodyControlP1->getHeadPitchPosition()) < sigma))
                {
                    if (bodyControlP1->getHeadPitchMotorActivated())
                    {
                        string msg = Valter::format_string("Task#%lu (%s) position [%.2f] reached", getTaskId(), getTaskName().c_str(), bodyControlP1->getHeadPitchPosition());
                        qDebug("%s", msg.c_str());
                    }
                    bodyControlP1->setHeadPitchMotorActivated(false);
                }
/************************************ emulation *********************start***************************/
string msg = Valter::format_string("Task#%lu (%s) positioning [%.2f]...", getTaskId(), getTaskName().c_str(), bodyControlP1->getHeadPitchPosition());
qDebug("%s", msg.c_str());
/************************************ emulation *********************finish**************************/
            }
            this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    }

    bodyControlP1->setHeadPitchMotorActivated(false);

    if (!getCompleted())
    {
        string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal.%s", getTaskId(), ((stopExecutionReason.length() > 0) ? Valter::format_string(" REASON: %s", stopExecutionReason.c_str()).c_str() : ""));
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        setCompleted();
    }
}

float SetHeadPitchPositionTask::getAngle() const
{
    return angle;
}

void SetHeadPitchPositionTask::setAngle(float value)
{
    angle = value;
}
