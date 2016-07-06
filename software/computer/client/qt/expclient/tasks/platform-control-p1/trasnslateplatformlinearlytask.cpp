#include "valter.h"
#include "trasnslateplatformlinearlytask.h"

signed int TrasnslatePlatformLinearlyTask::prevDirection = -1;

TrasnslatePlatformLinearlyTask::TrasnslatePlatformLinearlyTask()
{
    direction = -1;
    distance  = -1;

    qDebugOn = true;
    taskName = "TrasnslatePlatformLinearlyTask";
    blocking = false;

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    initialLeftMotorMaxDuty = platformControlP1->getLeftMotorDutyMax();
    initialRightMotorMaxDuty = platformControlP1->getRightMotorDutyMax();
}

bool TrasnslatePlatformLinearlyTask::checkFeasibility()
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
        string msg = Valter::format_string("Task#%lu (%s) could not be performed. Direction is undefined.", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    if (distance < 0)
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be performed. Translation distance is undefined.", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    if (direction != prevDirection)
    {
        if (platformControlP1->getRightMotorActivated() || platformControlP1->getLeftMotorActivated())
        {
            string msg = Valter::format_string("Task#%lu (%s) could not be performed. Saltatory inversion of movement direction.", getTaskId(), getTaskName().c_str());
            qDebug("%s", msg.c_str());
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

            return false;
        }
    }
    return true;
}

bool TrasnslatePlatformLinearlyTask::initialize()
{
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
    qDebug("Task#%lu (%s) direction = %s, distance = %f", getTaskId(), getTaskName().c_str(), (direction > 0 && direction != -1) ? "forward" : "backward", distance);
    return true;
}

void TrasnslatePlatformLinearlyTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&TrasnslatePlatformLinearlyTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void TrasnslatePlatformLinearlyTask::stopExecution()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
    stopped = true;
    platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
    platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);
}

void TrasnslatePlatformLinearlyTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *TrasnslatePlatformLinearlyTask::create()
{
    return (ITask*)new TrasnslatePlatformLinearlyTask();
}

void TrasnslatePlatformLinearlyTask::executionWorker()
{
    /************************************ emulation *********************start***************************/
    int lwen, rwen;
    lwen = 0;
    rwen = 0;
    /************************************ emulation *********************finish**************************/
    int cutOffDistanceTicks = (int)round(47 * distance * 0.98);
    //1m ~ 47 vague encoder ticks
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    while (!stopped)
    {
        if (!executing)
        {
            /************************************ emulation *********************start***************************/
            if (true)
            /************************************ emulation *********************finish**************************/
            //if (platformControlP1->preparePlatformMovement())
            {
                if (direction == 1)
                {
                    //left and right forward
                    if (platformControlP1->setLeftMotorDirection(true) && platformControlP1->setRightMotorDirection(true))
                    {
                        platformControlP1->setLeftMotorActivated(true);
                        platformControlP1->setRightMotorActivated(true);
                    }
                    else
                    {
                        string msg = Valter::format_string("Task#%lu (%s)has been terminated. Saltatory inversion of movement direction while execution.", getTaskId(), getTaskName().c_str());
                        qDebug("%s", msg.c_str());
                        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                        stopExecution();
                    }
                }
                else if (direction == 0)
                {
                    //left and right forward
                    if (platformControlP1->setLeftMotorDirection(false) && platformControlP1->setRightMotorDirection(false))
                    {
                        platformControlP1->setLeftMotorActivated(true);
                        platformControlP1->setRightMotorActivated(true);
                    }
                    else
                    {
                        string msg = Valter::format_string("Task#%lu (%s)has been terminated. Saltatory inversion of movement direction while execution.", getTaskId(), getTaskName().c_str());
                        qDebug("%s", msg.c_str());
                        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                        stopExecution();
                    }
                }
            }
            executing = true;
        }
        else
        {
            /************************************ emulation *********************start***************************/
            int randNum = rand() % 3; // Generate a random number between 0 and 1
            if (randNum == 0)
            {
                lwen++;
            }
            if (randNum == 1)
            {
                rwen++;
            }
            if (randNum == 2)
            {
                rwen += 2;
            }
            this_thread::sleep_for(std::chrono::milliseconds(50));
            qDebug("LEN:%d, REN:%d, (int)round((double)47 * cutOffDistance) = %d, rand = %d", lwen, rwen, cutOffDistanceTicks, randNum);
            /************************************ emulation *********************finish**************************/

            /************************************ emulation *********************start***************************/
            if (lwen >=  cutOffDistanceTicks || rwen >= cutOffDistanceTicks)
            /************************************ emulation *********************finish**************************/
//            if (platformControlP1->getLeftWheelEncoder() >= cutOffDistance || platformControlP1->getRightWheelEncoder() >= cutOffDistance)
            {
                platformControlP1->setLeftMotorActivated(false);
                platformControlP1->setRightMotorActivated(false);
                platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
                platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);
                setCompleted();
                return;
            }
            else
            {
                /************************************ emulation *********************start***************************/
                if (lwen > rwen)
                /************************************ emulation *********************finish**************************/
                //if (platformControlP1->getLeftWheelEncoder() > platformControlP1->getRightWheelEncoder())
                {
                    int correctedLeftMotorDuty = platformControlP1->getLeftMotorDutyMax();
                    int correctedRightMotorDuty = platformControlP1->getRightMotorDutyMax();
                    --correctedLeftMotorDuty;
                    ++correctedRightMotorDuty;
                    if (correctedLeftMotorDuty > 0)
                    {
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                        qDebug("RIGHT CORRECTION RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                    }
                    /************************************ emulation *********************start***************************/
                    rwen++;
                    /************************************ emulation *********************finish**************************/
                }
                /************************************ emulation *********************start***************************/
                if (rwen > lwen)
                /************************************ emulation *********************finish**************************/
                //if (platformControlP1->getRightWheelEncoder() > platformControlP1->getLeftWheelEncoder())
                {
                    int correctedLeftMotorDuty = platformControlP1->getLeftMotorDutyMax();
                    int correctedRightMotorDuty = platformControlP1->getRightMotorDutyMax();
                    ++correctedLeftMotorDuty;
                    --correctedRightMotorDuty;
                    if (correctedRightMotorDuty > 0)
                    {
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                        qDebug("LEFT CORRECTION RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                    }
                    /************************************ emulation *********************start***************************/
                    lwen++;
                    /************************************ emulation *********************finish**************************/
                }
                /************************************ emulation *********************start***************************/
                if (rwen == lwen)
                /************************************ emulation *********************finish**************************/
                //if (platformControlP1->getRightWheelEncoder() > platformControlP1->getLeftWheelEncoder())
                {
                    platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
                    platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);
                    qDebug("CORRECTED RightWheelEncoder = LeftWheelEncoder");
                }
            }

//            qDebug("LEN:%d, REN:%d, (int)round((double)47 * cutOffDistance) = %d", platformControlP1->getLeftWheelEncoder(), platformControlP1->getRightWheelEncoder(), cutOffDistanceTicks);
        }
    }

    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
    platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
    platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);

    string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    qDebug("%s", msg.c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

    setCompleted();
}

void TrasnslatePlatformLinearlyTask::setDirection(signed int value)
{
    direction = value;
    prevDirection = direction;
}

void TrasnslatePlatformLinearlyTask::setDistance(float value)
{
    distance = value;
}
