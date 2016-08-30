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
    int minAllowedDuty = 18;

    float curVelocityL = 0.0;
    float curVelocityR = 0.0;

    float targetVelocityL = 0.0;
    float targetVelocityR = 0.0;

    int prevLeftWheelEncoder = 0;
    int prevRightWheelEncoder = 0;

    int sleepTime = 500;

    string stopExecutionReason = "";

    bool rotationMode = false;
    bool movementInitialized = false;

    bool prevRotationDirection = true;

    double t = (1000 / (double)sleepTime);
    float av_b = ((double)Valter::wheelBase / 1000) / 2;

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
                if (getLinearVelocity() > 0)
                {
                    rotationMode = false;
                }
                if (abs(getAngularVelocity()) > 0 && getLinearVelocity() == 0)
                {
                    rotationMode = true;
                }

                //linear movement
                if (!rotationMode)
                {
                    movementInitialized = initLinearMovement();
                }
                //rotation
                else
                {
                    prevRotationDirection = (getAngularVelocity() > 0) ? true : false;
                    movementInitialized = initRotation();
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
            if (!platformControlP1->getLeftMotorAccelerating()  &&
                !platformControlP1->getRightMotorAccelerating() &&
                !platformControlP1->getLeftMotorDecelerating()  &&
                !platformControlP1->getRightMotorDecelerating())
                //motors steady mode
            {
                bool curRotationDirection = prevRotationDirection;
                if (abs(getAngularVelocity()) > 0)
                {
                    curRotationDirection = (getAngularVelocity() > 0) ? true : false;
                }


                if ((getLinearVelocity() == 0 && abs(getAngularVelocity()) > 0 && !rotationMode) //platform is moving twisty and has to be rotated at a spot
                 || (getLinearVelocity() == 0 && prevRotationDirection != curRotationDirection)) //platform rotation direction has been changed
                {
                    movementInitialized = false;
                    platformControlP1->setLeftMotorActivated(false);
                    platformControlP1->setRightMotorActivated(false);
                    this_thread::sleep_for(std::chrono::milliseconds(100));
                    prevRotationDirection = curRotationDirection;
                    rotationMode = true;
                    continue;
                }

                if (getLinearVelocity() > 0 && rotationMode) //platform has to move twisty
                {
                    movementInitialized = false;
                    platformControlP1->setLeftMotorActivated(false);
                    platformControlP1->setRightMotorActivated(false);
                    this_thread::sleep_for(std::chrono::milliseconds(100));
                    rotationMode = false;
                    continue;
                }

                int correctedLeftMotorDuty = platformControlP1->getLeftMotorDutyMax();
                int correctedRightMotorDuty = platformControlP1->getRightMotorDutyMax();

                //linear movement
                if (!rotationMode)
                {
                    if (!movementInitialized)
                    {
                        movementInitialized = initLinearMovement();
                        continue;
                    }

                    float lv = getLinearVelocity();
                    float av = getAngularVelocity();

                    targetVelocityL = lv + av * av_b;
                    targetVelocityR = lv - av * av_b;

/************************************ emulation *********************start***************************/
curVelocityL = ((double)(lwen - prevLeftWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * t;
curVelocityR = ((double)(rwen - prevRightWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * t;
/************************************ emulation *********************finish**************************/
//                    curVelocityL = ((double)(platformControlP1->getLeftWheelEncoder() - prevLeftWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * t;
//                    curVelocityR = ((double)(platformControlP1->getRightWheelEncoder() - prevRightWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPerMeter) * t;
                }
                //rotation mode (wheels rotate in opposite directions)
                else
                {
                    if (!movementInitialized)
                    {
                        movementInitialized = initRotation();
                        continue;
                    }

                    float av = getAngularVelocity();

                    targetVelocityL = targetVelocityR = av / (2 * M_PI);


/************************************ emulation *********************start***************************/
curVelocityL = ((double)(lwen - prevLeftWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPer360Turn) * t;
curVelocityR = ((double)(rwen - prevRightWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPer360Turn) * t;
/************************************ emulation *********************finish**************************/
//                    curVelocityL = ((double)(platformControlP1->getLeftWheelEncoder() - prevLeftWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPer360Turn) * t;
//                    curVelocityR = ((double)(platformControlP1->getRightWheelEncoder() - prevRightWheelEncoder) / (double)PlatformControlP1::vagueEncoderTicksPer360Turn) * t;
                }

                if (curVelocityL != targetVelocityL)
                {
                    if (correctedLeftMotorDuty < maxAllowedDuty && correctedLeftMotorDuty > minAllowedDuty)
                    {
                        correctedLeftMotorDuty += (curVelocityL > targetVelocityL) ? -1 : 1;
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        qDebug("LEFT CORRECTION LDuty=%d", correctedLeftMotorDuty);
                    }
                }

                if (curVelocityR != targetVelocityR)
                {
                    if (correctedLeftMotorDuty < maxAllowedDuty && correctedRightMotorDuty > minAllowedDuty)
                    {
                        correctedRightMotorDuty += (curVelocityR > targetVelocityR) ? -1 : 1;
                        platformControlP1->setLeftMotorDutyMax(correctedLeftMotorDuty);
                        platformControlP1->setRightMotorDutyMax(correctedRightMotorDuty);
                        qDebug("RIGHT CORRECTION RDuty=%d", correctedRightMotorDuty);
                    }
                }


/************************************ emulation *********************start***************************/
prevLeftWheelEncoder  = lwen;
prevRightWheelEncoder = rwen;
/************************************ emulation *********************finish**************************/
//                prevLeftWheelEncoder  = platformControlP1->getLeftWheelEncoder();
//                prevRightWheelEncoder = platformControlP1->getRightWheelEncoder();



                //check fault conditions
                if (platformControlP1->getLeftMotorDuty() < minAllowedDuty)
                {
                    stopExecutionReason = "getLeftMotorDuty() has reached duty < minAllowedDuty - which is a fault condition.";
                    stopExecution();
                    break;
                }
                if (platformControlP1->getRightMotorDuty() < minAllowedDuty)
                {
                    stopExecutionReason = "getRightMotorDuty() has reached duty < minAllowedDuty - which is a fault condition.";
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


/************************************ emulation *********************start***************************/
qDebug("[%lu, %s] LEN:%d, REN:%d, curVelocityL:%f/targetVelocityL:%f, curVelocityR:%f/targetVelocityR:%f", taskId, taskName.c_str(), lwen, rwen, curVelocityL, targetVelocityL, curVelocityR, targetVelocityR);
/************************************ emulation *********************finish**************************/
                //qDebug("[%lu, %s] LEN:%d, REN:%d, curVelocityL:%f/targetVelocityL:%f, curVelocityR:%f/targetVelocityR:%f", taskId, taskName.c_str(), platformControlP1->getLeftWheelEncoder(), platformControlP1->getRightWheelEncoder(), curVelocityL, targetVelocityL, curVelocityR, targetVelocityR);
                this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
            }
            else
            {
                this_thread::sleep_for(std::chrono::milliseconds(100));
                qDebug("Task#%lu (%s) motors are not in steady mode...", getTaskId(), getTaskName().c_str());
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


bool TranslatePlatformTwistyTask::initLinearMovement()
{
    qDebug("Task#%lu (%s) initLinearMovement()", getTaskId(), getTaskName().c_str());
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (platformControlP1->setLeftMotorDirection(true) && platformControlP1->setRightMotorDirection(true))
    {
        platformControlP1->resetLeftWheelEncoder();
        platformControlP1->resetRightWheelEncoder();

        platformControlP1->setLeftMotorDutyMax(20);
        platformControlP1->setRightMotorDutyMax(20);

        platformControlP1->setLeftMotorActivated(true);
        platformControlP1->setRightMotorActivated(true);

        return true;
    }
    else
    {
        string msg = Valter::format_string("Task#%lu (%s)has been terminated. Saltatory inversion of movement direction while execution.", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        stopExecution();

        return false;
    }
}

bool TranslatePlatformTwistyTask::initRotation()
{
    qDebug("Task#%lu (%s) initRotation()", getTaskId(), getTaskName().c_str());
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (platformControlP1->setLeftMotorDirection(((getAngularVelocity() > 0) ? true : false)) && platformControlP1->setRightMotorDirection(((getAngularVelocity() > 0) ? false : true)))
    {
        platformControlP1->resetLeftWheelEncoder();
        platformControlP1->resetRightWheelEncoder();

        platformControlP1->setLeftMotorDutyMax(20);
        platformControlP1->setRightMotorDutyMax(20);

        platformControlP1->setLeftMotorActivated(true);
        platformControlP1->setRightMotorActivated(true);

        return true;
    }
    else
    {
        string msg = Valter::format_string("Task#%lu (%s)has been terminated. Saltatory inversion of movement direction while execution.", getTaskId(), getTaskName().c_str());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        stopExecution();

        return false;
    }
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
