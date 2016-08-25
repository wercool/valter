#include "valter.h"
#include "translateplatformtwistytask.h"

TranslatePlatformTwistyTask::TranslatePlatformTwistyTask()
{
    qDebugOn = true;
    taskName = "TranslatePlatformTwistyTask";
    blocking = false;
    attachable = true;

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    initialLeftMotorMaxDuty = platformControlP1->getLeftMotorDutyMax();
    initialRightMotorMaxDuty = platformControlP1->getRightMotorDutyMax();
}

bool TranslatePlatformTwistyTask::checkFeasibility()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->getPower5VOnState())
    {
        string msg = Valter::format_string("Task#%lu (%s) could not be executed. 5V power is OFF", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));
        return false;
    }
    return true;
}

bool TranslatePlatformTwistyTask::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    //T_PCP1_TranslatePlatformTwistyTask_0.1_0.0
    float _linearVelocity = atof(((string)taskInitiationParts[1]).c_str());
    float _angularVelocity = atof(((string)taskInitiationParts[2]).c_str());
    setLinearVelocity(_linearVelocity);
    setAngularVelocity(_angularVelocity);

    return true;
}

void TranslatePlatformTwistyTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&TranslatePlatformTwistyTask::executionWorker, this);
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

void TranslatePlatformTwistyTask::stopExecution()
{
    stopped = true;
}

void TranslatePlatformTwistyTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s",
                                                                                          getTaskId(),
                                                                                          getTaskName().c_str(),
                                                                                          (blocking) ? "blocking" : "non blocking",
                                                                                          ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))),
                                                                                          getTaskScriptLine().c_str()));
}

ITask *TranslatePlatformTwistyTask::create()
{
    return (ITask*)new TranslatePlatformTwistyTask();
}

void TranslatePlatformTwistyTask::executionWorker()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    float curVelocityL = 0.0;
    float curVelocityR = 0.0;

    float targetVelocityL = 0.0;
    float targetVelocityR = 0.0;

    int prevLeftWheelEncoder = 0;
    int prevRightWheelEncoder = 0;

    unsigned long curTime = 0;
    unsigned long prevTime = 0;
    unsigned long dTime = 0;

    while (!stopped)
    {
        curTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (prevTime > 0)
        {
            dTime = curTime - prevTime;
        }
        if (!executing)
        {
            if (platformControlP1->preparePlatformMovement())
            {
                //left and right forward (in twisty mode platform moves only forward or turns on the spot)
                if (platformControlP1->setLeftMotorDirection(true) && platformControlP1->setRightMotorDirection(true))
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
                }
            }
            else
            {
                string msg = Valter::format_string("Task#%lu (%s)has been terminated. preparePlatformMovement() returns FALSE.", getTaskId(), getTaskName().c_str());
                qDebug("%s", msg.c_str());
                stopExecution();
            }
            executing = true;
            prevTime = curTime;
        }
        else
        {
            /************************************ emulation *********************start***************************/
int lwen, rwen;
lwen = 0;
rwen = 0;
            /************************************ emulation *********************finish**************************/
            if (!platformControlP1->getLeftMotorAccelerating()  &&
                !platformControlP1->getRightMotorAccelerating() &&
                !platformControlP1->getLeftMotorDecelerating()  &&
                !platformControlP1->getRightMotorDecelerating())
                //motors steady mode
            {
/************************************ emulation *********************start***************************/
int randNum = rand() % 3; // Generate a random number between 0 and 2
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
platformControlP1->setLeftWheelEncoder(lwen);
platformControlP1->setRightWheelEncoder(rwen);
/************************************ emulation *********************finish**************************/
                if (dTime > 1000)
                {
                    curVelocityL = ((double)(platformControlP1->getLeftWheelEncoder() - prevLeftWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * (1000 / (double)dTime);
                    curVelocityR = ((double)(platformControlP1->getRightWheelEncoder() - prevRightWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * (1000 / (double)dTime);

                    targetVelocityL = linearVelocity + angularVelocity * (Valter::wheelBase / 1000) / 2;
                    targetVelocityR = linearVelocity - angularVelocity * (Valter::wheelBase / 1000) / 2;

                    if (curVelocityL < targetVelocityL || curVelocityR > targetVelocityR)
                    {
                        int correctedLeftMotorDuty = platformControlP1->getLeftMotorDutyMax();
                        int correctedRightMotorDuty = platformControlP1->getRightMotorDutyMax();
                        ++correctedLeftMotorDuty;
                        --correctedRightMotorDuty;
                        if (correctedRightMotorDuty > 0)
                        {
                            platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                            platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                            qDebug("LEFT CORRECTION [left motor duty increased] RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                        }
                    }

                    if (curVelocityR < targetVelocityR || curVelocityL > targetVelocityL)
                    {
                        int correctedLeftMotorDuty = platformControlP1->getLeftMotorDutyMax();
                        int correctedRightMotorDuty = platformControlP1->getRightMotorDutyMax();
                        --correctedLeftMotorDuty;
                        ++correctedRightMotorDuty;
                        if (correctedLeftMotorDuty > 0)
                        {
                            platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                            platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                            qDebug("RIGHT CORRECTION [right motor duty increased] RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                        }
                    }

/************************************ emulation *********************start***************************/
prevLeftWheelEncoder  = lwen;
prevRightWheelEncoder = rwen;
/************************************ emulation *********************finish**************************/
//                    prevLeftWheelEncoder  = platformControlP1->getLeftWheelEncoder();
//                    prevRightWheelEncoder = platformControlP1->getRightWheelEncoder();
                    prevTime = curTime;
                }
            }

            qDebug("[%lu, %s] LEN:%d, REN:%d", taskId, taskName.c_str(), platformControlP1->getLeftWheelEncoder(), platformControlP1->getRightWheelEncoder());
            this_thread::sleep_for(std::chrono::milliseconds(100));
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

float TranslatePlatformTwistyTask::getAngularVelocity() const
{
    return angularVelocity;
}

void TranslatePlatformTwistyTask::setAngularVelocity(float value)
{
    angularVelocity = value;
}

float TranslatePlatformTwistyTask::getLinearVelocity() const
{
    return linearVelocity;
}

void TranslatePlatformTwistyTask::setLinearVelocity(float value)
{
    linearVelocity = value;
}
