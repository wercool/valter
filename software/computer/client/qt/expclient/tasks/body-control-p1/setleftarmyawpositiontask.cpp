#include "valter.h"
#include "setleftarmyawpositiontask.h"

SetLeftArmYawPositionTask::SetLeftArmYawPositionTask()
{
    qDebugOn = true;
    taskName = "SetLeftArmYawPositionTask";
    blocking = false;
}

bool SetLeftArmYawPositionTask::checkFeasibility()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (!bodyControlP1->prepareLeftArmYawMovement())
    {
        string msg = Valter::format_string("Task#%lu bodyControlP1->prepareLeftArmYawMovement() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    if (angle < 0 || angle > 55)
    {
        string msg = Valter::format_string("Task#%lu target Left Arm Yaw angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetLeftArmYawPositionTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    float angle = atof(((string)taskInitiationParts[1]).c_str());
    setAngle(angle);

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        qDebug("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        return false;
    }
    return true;
}

void SetLeftArmYawPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetLeftArmYawPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetLeftArmYawPositionTask::stopExecution()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setLeftArmYawMotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetLeftArmYawPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetLeftArmYawPositionTask::create()
{
    return (ITask*)new SetLeftArmYawPositionTask();
}

void SetLeftArmYawPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();

/************************************ emulation *********************start***************************/
//    bodyControlP1->setLeftArmYawADCPosition(580);
/************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    //move to open - true
    bool direction = (angle > bodyControlP1->getLeftArmYawPosition()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 0.98) : (angle / 0.98); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(angle - bodyControlP1->getLeftArmYawPosition()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //move Left Arm Yaw (open)
            {
                if (bodyControlP1->prepareLeftArmYawMovement())
                {
                    //move to open (CCW)
                    if (bodyControlP1->setLeftArmYawMovementDirection(true))
                    {
                        bodyControlP1->setLeftArmYawMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason bodyControlP1->prepareLeftArmYawMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    return;
                }
            }
            else
            {
                if (bodyControlP1->prepareLeftArmYawMovement())
                {
                    //closing (CW)
                    if (bodyControlP1->setLeftArmYawMovementDirection(false))
                    {
                        bodyControlP1->setLeftArmYawMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason bodyControlP1->prepareLeftArmYawMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    return;
                }
            }
        }
        else
        {

            if (qDebugOn)
            {
                qDebug("Task#%lu: getLeftArmYawPosition() (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), bodyControlP1->getLeftArmYawPosition(), angle, cutoffAngle, abs(angle - bodyControlP1->getLeftArmYawPosition()), sigma, (bodyControlP1->getLeftArmYawMovementDirection() ? "opening" : "closing"));
            }

            if ((abs(angle - bodyControlP1->getLeftArmYawPosition()) < sigma))
            {
                bodyControlP1->setLeftArmYawMotorActivated(false);
                setCompleted();
                return;
            }

/************************************ emulation *********************start***************************/
//            int positionADC = bodyControlP1->getLeftArmYawADCPosition();
//            if (direction)
//            {
//                positionADC -= 5;
//            }
//            else
//            {
//                positionADC += 5;
//            }
//            bodyControlP1->setLeftArmYawADCPosition(positionADC);
/************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    qDebug("%s", msg.c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));


    setCompleted();
}

float SetLeftArmYawPositionTask::getAngle() const
{
    return angle;
}

void SetLeftArmYawPositionTask::setAngle(float value)
{
    angle = value;
}
