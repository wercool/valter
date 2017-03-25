#include "valter.h"
#include "setrightarmpositiontask.h"


SetRightArmPositionTask::SetRightArmPositionTask()
{
    qDebugOn = true;
    taskName = "SetRightArmPositionTask";
    blocking = false;
}

bool SetRightArmPositionTask::checkFeasibility()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (!armControlRight->prepareRightArmMovement())
    {
        string msg = Valter::format_string("Task#%lu prepareRightArmMovement() returned false in checkFeasibility().", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    if (angle < 0 || angle > 90)
    {
        string msg = Valter::format_string("Task#%lu target Right Arm angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetRightArmPositionTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    float angle = atof(((string)taskInitiationParts[1]).c_str());
    setAngle(angle);

/************************************ emulation *********************start***************************/
//    return true;
/************************************ emulation *********************finish**************************/

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        qDebug("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        return false;
    }
    return true;
}

void SetRightArmPositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetRightArmPositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetRightArmPositionTask::stopExecution()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightArmMotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetRightArmPositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetRightArmPositionTask::create()
{
    return (ITask*)new SetRightArmPositionTask();
}

void SetRightArmPositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();

/************************************ emulation *********************start***************************/
//    for (int i = 0; i < 15; i++)
//    {
//        armControlRight->setArmADCPosition(ArmControlRight::armAngleADCZero);
//    }
//    qDebug("Current armControlRight->getArmPosition() = %.2f", armControlRight->getArmPosition());
/************************************ emulation *********************finish**************************/

    float sigma = 2.0; //precision in degrees

    //move up (angle increased) - true
    bool direction = (angle > armControlRight->getArmPosition()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 0.95) : (angle / 0.95); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(angle - armControlRight->getArmPosition()) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //move Right Arm up
            {
                if (armControlRight->prepareRightArmMovement())
                {
                    //move up
                    if (armControlRight->setRightArmMotorMovementDirection(false))
                    {
                        armControlRight->setRightArmMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlRight->prepareRightArmMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    continue;
                }
            }
            else
            {
                if (armControlRight->prepareRightArmMovement())
                {
                    //down
                    if (armControlRight->setRightArmMotorMovementDirection(true))
                    {
                        armControlRight->setRightArmMotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason armControlRight->prepareRightArmMovement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    continue;
                }
            }
        }
        else
        {

            if (qDebugOn)
            {
                qDebug("Task#%lu: getArmPosition() (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), armControlRight->getArmPosition(), angle, cutoffAngle, abs(angle - armControlRight->getArmPosition()), sigma, (armControlRight->getRightArmMotorMovementDirection() ? "down" : "up"));
            }

            if ((abs(angle - armControlRight->getArmPosition()) < sigma))
            {
                armControlRight->setRightArmMotorActivated(false);
                setCompleted();
                break;
            }

/************************************ emulation *********************start***************************/
//            int positionADC = ArmControlRight::armAngleADCMax - round(armControlRight->getArmPosition() * ArmControlRight::armDegreesDiv);
//            int noise = rand() % 15;
//            if (noise == 0)
//            {
//                qDebug("NOISE ++++++++++++++++++++++++++++++++++++++++++");
//                positionADC = 1023;
//            }
//            if (noise == 1)
//            {
//                positionADC = 0;
//                qDebug("NOISE ------------------------------------------");
//            }
//            if (noise == 2)
//            {
//                qDebug("NOISE +++++++++++");
//                positionADC += 100;
//            }
//            if (noise == 3)
//            {
//                positionADC -= 100;
//                qDebug("NOISE -----------");
//            }
//            if (direction)
//            {
//                positionADC -= 5;
//            }
//            else
//            {
//                positionADC += 5;
//            }
//            armControlRight->setArmADCPosition(positionADC);
/************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!getCompleted())
    {
        string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));


        setCompleted();
    }
}

float SetRightArmPositionTask::getAngle() const
{
    return angle;
}

void SetRightArmPositionTask::setAngle(float value)
{
    angle = value;
}
