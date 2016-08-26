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

    int maxAllowedDuty = 36;

    float curVelocityL = 0.0;
    float curVelocityR = 0.0;

    float targetVelocityL = 0.0;
    float targetVelocityR = 0.0;

    int prevLeftWheelEncoder = 0;
    int prevRightWheelEncoder = 0;

    int sleepTime = 500;

    string stopExecutionReason = "";
    int assumedStoppedCnt = 0;

/************************************ emulation *********************start***************************/
int lwen, rwen;
lwen = 0;
rwen = 0;
/************************************ emulation *********************finish**************************/

    while (!stopped)
    {
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
        }
        else
        {
            if (!platformControlP1->getLeftMotorAccelerating()  &&
                !platformControlP1->getRightMotorAccelerating() &&
                !platformControlP1->getLeftMotorDecelerating()  &&
                !platformControlP1->getRightMotorDecelerating())
                //motors steady mode
            {
/************************************ emulation *********************start***************************/
int randNum = rand() % 10;
if (randNum == 0)
{
    lwen++;
}
if (randNum == 1)
{
    rwen++;
}
/************************************ emulation *********************finish**************************/

                float lv = getLinearVelocity();
                float av = getAngularVelocity();
                float av_b = ((double)Valter::wheelBase / 1000) / 2;
                double t = (1000 / (double)sleepTime);

/************************************ emulation *********************start***************************/
curVelocityL = ((double)(lwen - prevLeftWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * t;
curVelocityR = ((double)(rwen - prevRightWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * t;
/************************************ emulation *********************finish**************************/
//                curVelocityL = ((double)(platformControlP1->getLeftWheelEncoder() - prevLeftWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * t;
//                curVelocityR = ((double)(platformControlP1->getRightWheelEncoder() - prevRightWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * t;

                targetVelocityL = lv + av * av_b;
                targetVelocityR = lv - av * av_b;

                if (lv == 0 && abs(av) > 0)
                {
                    platformControlP1->setLeftMotorActivated(false);
                    platformControlP1->setRightMotorActivated(false);
                    continue;
                }

                int correctedLeftMotorDuty = platformControlP1->getLeftMotorDutyMax();
                int correctedRightMotorDuty = platformControlP1->getRightMotorDutyMax();

                if (curVelocityL < targetVelocityL && curVelocityR > targetVelocityR)
                {
                    if (correctedRightMotorDuty > 1 && correctedLeftMotorDuty < maxAllowedDuty)
                    {
                        ++correctedLeftMotorDuty;
                        --correctedRightMotorDuty;
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                        qDebug("LEFT CORRECTION [left motor duty increased] RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                    }
                }

                if (curVelocityR < targetVelocityR && curVelocityL > targetVelocityL)
                {
                    if (correctedLeftMotorDuty > 1 && correctedRightMotorDuty < maxAllowedDuty)
                    {
                        --correctedLeftMotorDuty;
                        ++correctedRightMotorDuty;
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                        qDebug("RIGHT CORRECTION [right motor duty increased] RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                    }
                }

                if (curVelocityR < targetVelocityR && curVelocityL < targetVelocityL)
                {
                    if (correctedLeftMotorDuty < maxAllowedDuty && correctedRightMotorDuty < maxAllowedDuty)
                    {
                        ++correctedLeftMotorDuty;
                        ++correctedRightMotorDuty;
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                        qDebug("(+)L&R CORRECTION RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                    }
                }

                if (curVelocityR > targetVelocityR && curVelocityL > targetVelocityL)
                {
                    if (correctedLeftMotorDuty > 1 && correctedRightMotorDuty > 1)
                    {
                        --correctedLeftMotorDuty;
                        --correctedRightMotorDuty;
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                        qDebug("(-)L&R CORRECTION RDuty=%d, LDuty=%d", correctedRightMotorDuty, correctedLeftMotorDuty);
                    }
                }

/************************************ emulation *********************start***************************/
prevLeftWheelEncoder  = lwen;
prevRightWheelEncoder = rwen;
/************************************ emulation *********************finish**************************/
//                prevLeftWheelEncoder  = platformControlP1->getLeftWheelEncoder();
//                prevRightWheelEncoder = platformControlP1->getRightWheelEncoder();



                //check fault conditions
                if (platformControlP1->getLeftMotorDuty() < 15 && platformControlP1->getRightMotorDuty() > 25)
                {
                    stopExecutionReason = "getLeftMotorDuty() has reached duty < 15 while getRightMotorDuty() > 25 - which is a fault condition.";
                    stopExecution();
                    break;
                }
                if (platformControlP1->getRightMotorDuty() < 15 && platformControlP1->getLeftMotorDuty() > 25)
                {
                    stopExecutionReason = "getRightMotorDuty() has reached duty < 15 while getLeftMotorDuty() > 25 - which is a fault condition.";
                    stopExecution();
                    break;
                }
                if (platformControlP1->getRightMotorDuty() > maxAllowedDuty)
                {
                    stopExecutionReason = "getRightMotorDuty() has reached duty > maxAllowedDuty - which is a fault condition, too fast.";
                    stopExecution();
                    break;
                }
                if (platformControlP1->getLeftMotorDuty() > maxAllowedDuty)
                {
                    stopExecutionReason = "getLeftMotorDuty() has reached duty < maxAllowedDuty - which is a fault condition, too fast.";
                    stopExecution();
                    break;
                }
                if (linearVelocity < 0.005 && abs(angularVelocity) < 0.01)
                {
                    stopExecutionReason = "linearVelocity < 0.005 AND abs(angularVelocity) < 0.01 condition means platform is stopped.";
                    stopExecution();
                    break;
                }
                if (!platformControlP1->getLeftMotorAccelerating() && !platformControlP1->getRightMotorAccelerating())
                {
                    if (platformControlP1->getLeftMotorDuty() < 17 && platformControlP1->getRightMotorDuty() < 17)
                    {
                        assumedStoppedCnt++;
                        if (assumedStoppedCnt > 10)
                        {
                            stopExecutionReason = "getLeftMotorDuty() and getRightMotorDuty() < 17 means platoform is stopped.";
                            stopExecution();
                            break;
                        }
                    }
                    else
                    {
                        assumedStoppedCnt = 0;
                    }
                }


/************************************ emulation *********************start***************************/
qDebug("[%lu, %s] LEN:%d, REN:%d, curVelocityL:%f/targetVelocityL:%f, curVelocityR:%f/targetVelocityR:%f", taskId, taskName.c_str(), lwen, rwen, curVelocityL, targetVelocityL, curVelocityR, targetVelocityR);
/************************************ emulation *********************finish**************************/
                //qDebug("[%lu, %s] LEN:%d, REN:%d, curVelocityL:%f/targetVelocityL:%f, curVelocityR:%f/targetVelocityR:%f", taskId, taskName.c_str(), platformControlP1->getLeftWheelEncoder(), platformControlP1->getRightWheelEncoder(), curVelocityL, targetVelocityL, curVelocityR, targetVelocityR);
                this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
            }
            else
            {
                this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
    platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
    platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);

    string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal.%s", getTaskId(), ((stopExecutionReason.length() > 0) ? Valter::format_string(" REASON: %s", stopExecutionReason.c_str()).c_str() : ""));
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
    std::lock_guard<std::mutex> lock(_mutex);
    angularVelocity = value;
}

float TranslatePlatformTwistyTask::getLinearVelocity() const
{
    return linearVelocity;
}

void TranslatePlatformTwistyTask::setLinearVelocity(float value)
{
    std::lock_guard<std::mutex> lock(_mutex);
    linearVelocity = value;
}
