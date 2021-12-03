#include "valter.h"
#include "rotateplatformtask.h"

signed int RotatePlatformTask::prevDirection = -1;

RotatePlatformTask::RotatePlatformTask()
{
    qDebugOn = true;
    taskName = "RotatePlatformTask";
    blocking = true;

    direction = -1;
    angle = -1;

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    initialLeftMotorMaxDuty = platformControlP1->getLeftMotorDutyMax();
    initialRightMotorMaxDuty = platformControlP1->getRightMotorDutyMax();
}

bool RotatePlatformTask::checkFeasibility()
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
        if (platformControlP1->getRightMotorActivated() || platformControlP1->getLeftMotorActivated())
        {
            string msg = Valter::format_string("Task#%lu (%s) could not be performed. Saltatory inversion of rotation direction.", getTaskId(), getTaskName().c_str());
            qDebug("%s", msg.c_str());
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
            return false;
        }
    }
    return true;
}

bool RotatePlatformTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    signed int direction = atoi(((string)taskInitiationParts[1]).c_str()); //1 - right; initial -1 - undefined
    float angle = atof(((string)taskInitiationParts[2]).c_str()); //degrees
    setDirection(direction);
    setAngle(angle);

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    if (!platformControlP1->getLeftWheelEncoderRead())
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be executed. Left Wheel Encoder readings is OFF", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    if (!platformControlP1->getRightWheelEncoderRead())
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be executed. Right Wheel Encoder readings is OFF", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    qDebug("Task#%lu (%s) rotation direction = %s, angle = %f", getTaskId(), getTaskName().c_str(), (direction > 0 && direction != -1) ? "right" : "left", angle);
    return true;
}

void RotatePlatformTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&RotatePlatformTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void RotatePlatformTask::stopExecution()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
    stopped = true;
    platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
    platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);
}

void RotatePlatformTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *RotatePlatformTask::create()
{
    return (ITask*)new RotatePlatformTask();
}

void RotatePlatformTask::setAngle(float value)
{
    angle = (value * M_PI) / 180;
}

void RotatePlatformTask::setDirection(int value)
{
    direction = value;
    prevDirection = direction;
}

void RotatePlatformTask::executionWorker()
{
/************************************ emulation *********************start***************************/
//int lwen, rwen;
//lwen = 0;
//rwen = 0;
/************************************ emulation *********************finish**************************/
    int cutOffDistanceTicks = (int)round((((double)PlatformControlP1::vagueEncoderTicksPer360Turn * angle) / (2 * M_PI)) * 0.98);

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    while (!stopped)
    {
        if (!executing)
        {
            if (platformControlP1->preparePlatformMovement())
            {
                if (direction == 1) //turn right
                {
                    //left forward, right backward
                    if (platformControlP1->setLeftMotorDirection(true) && platformControlP1->setRightMotorDirection(false))
                    {
                        platformControlP1->resetLeftWheelEncoder();
                        platformControlP1->resetRightWheelEncoder();

                        platformControlP1->setLeftMotorActivated(true);
                        platformControlP1->setRightMotorActivated(true);
                    }
                    else
                    {
                        string msg = Valter::format_string("Task#%lu (%s)has been terminated. Saltatory inversion of movement direction while execution.", getTaskId(), getTaskName().c_str());
                        qDebug("%s", msg.c_str());
                        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
                        stopExecution();
                        continue;
                    }
                }
                else if (direction == 0) //turn left
                {
                    //left backward, right forward
                    if (platformControlP1->setLeftMotorDirection(false) && platformControlP1->setRightMotorDirection(true))
                    {
                        platformControlP1->resetLeftWheelEncoder();
                        platformControlP1->resetRightWheelEncoder();

                        platformControlP1->setLeftMotorActivated(true);
                        platformControlP1->setRightMotorActivated(true);
                    }
                    else
                    {
                        string msg = Valter::format_string("Task#%lu (%s)has been terminated. Saltatory inversion of movement direction while execution.", getTaskId(), getTaskName().c_str());
                        qDebug("%s", msg.c_str());
                        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
                        stopExecution();
                        continue;
                    }
                }
            }
            executing = true;
        }
        else
        {
            platformControlP1->setRightWheelEncoderGetOnce(true);
            platformControlP1->setLeftWheelEncoderGetOnce(true);
/************************************ emulation *********************start***************************/
//int randNum = rand() % 3; // Generate a random number between 0 and 1
//if (randNum == 0)
//{
//    lwen++;
//}
//if (randNum == 1)
//{
//    rwen++;
//}
//if (randNum == 2)
//{
//    rwen += 2;
//}
//this_thread::sleep_for(std::chrono::milliseconds(50));
/************************************ emulation *********************finish**************************/

/************************************ emulation *********************start***************************/
//if (lwen >=  cutOffDistanceTicks || rwen >= cutOffDistanceTicks)
/************************************ emulation *********************finish**************************/
            if (platformControlP1->getLeftWheelEncoder() >= cutOffDistanceTicks || platformControlP1->getRightWheelEncoder() >= cutOffDistanceTicks)
            {
                platformControlP1->setLeftMotorActivated(false);
                platformControlP1->setRightMotorActivated(false);
                platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
                platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);
                setCompleted();
                break;
            }
            else
            {
                int correctedLeftMotorDuty = platformControlP1->getLeftMotorDutyMax();
                int correctedRightMotorDuty = platformControlP1->getRightMotorDutyMax();
/************************************ emulation *********************start***************************/
//if (lwen > rwen)
/************************************ emulation *********************finish**************************/
                if (platformControlP1->getLeftWheelEncoder() > platformControlP1->getRightWheelEncoder())
                {
                    --correctedLeftMotorDuty;
                    ++correctedRightMotorDuty;
                    if (correctedLeftMotorDuty > 0)
                    {
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                        qDebug("RIGHT CORRECTION RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                    }
/************************************ emulation *********************start***************************/
//rwen++;
/************************************ emulation *********************finish**************************/
                }
/************************************ emulation *********************start***************************/
//if (rwen > lwen)
/************************************ emulation *********************finish**************************/
                if (platformControlP1->getRightWheelEncoder() > platformControlP1->getLeftWheelEncoder())
                {
                    ++correctedLeftMotorDuty;
                    --correctedRightMotorDuty;
                    if (correctedRightMotorDuty > 0)
                    {
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                        qDebug("LEFT CORRECTION RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                    }
/************************************ emulation *********************start***************************/
//lwen++;
/************************************ emulation *********************finish**************************/
                }
/************************************ emulation *********************start***************************/
//if (rwen == lwen)
/************************************ emulation *********************finish**************************/
                if (platformControlP1->getRightWheelEncoder() == platformControlP1->getLeftWheelEncoder())
                {
                    platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
                    platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);
                    qDebug("CORRECTED RightWheelEncoder = LeftWheelEncoder");
                }
            }
/************************************ emulation *********************start***************************/
//qDebug("LEN:%d, REN:%d, cutOffDistanceTicks = %d", lwen, rwen, cutOffDistanceTicks);
/************************************ emulation *********************finish**************************/
            qDebug("LEN:%d, REN:%d, cutOffDistanceTicks = %d", platformControlP1->getLeftWheelEncoder(), platformControlP1->getRightWheelEncoder(), cutOffDistanceTicks);
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
    platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
    platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);

    if (!getCompleted())
    {
        string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        setCompleted();
    }
}
