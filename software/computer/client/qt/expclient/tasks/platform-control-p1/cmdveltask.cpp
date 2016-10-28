#include "valter.h"
#include "cmdveltask.h"

CmdVelTask::CmdVelTask()
{
    qDebugOn = true;
    taskName = "CmdVelTask";
    blocking = false;
    attachable = true;

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    initialLeftMotorMaxDuty = platformControlP1->getLeftMotorDutyMax();
    initialRightMotorMaxDuty = platformControlP1->getRightMotorDutyMax();
}

bool CmdVelTask::checkFeasibility()
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

bool CmdVelTask::initialize()
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

void CmdVelTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&CmdVelTask::executionWorker, this);
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

void CmdVelTask::stopExecution()
{
    stopped = true;
}

void CmdVelTask::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s",
                                                                                          getTaskId(),
                                                                                          getTaskName().c_str(),
                                                                                          (blocking) ? "blocking" : "non blocking",
                                                                                          ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))),
                                                                                          getTaskScriptLine().c_str()));
}

ITask *CmdVelTask::create()
{
    return (ITask*)new CmdVelTask();
}

float CmdVelTask::getLinearVelocity() const
{
    return linearVelocity;
}

void CmdVelTask::setLinearVelocity(float value)
{
    std::lock_guard<std::mutex> lock(values_mutex);
    linearVelocity = value;
}

float CmdVelTask::getAngularVelocity() const
{
    return angularVelocity;
}

void CmdVelTask::setAngularVelocity(float value)
{
    std::lock_guard<std::mutex> lock(values_mutex);
    angularVelocity = value;
}


int CmdVelTask::getLeftWheelEncoderReading()
{
    //return PlatformControlP2::getInstance()->getLeftEncoder();
    return PlatformControlP1::getInstance()->getLeftWheelEncoder();
}

int CmdVelTask::getRightWheelEncoderReading()
{
    //return PlatformControlP2::getInstance()->getRightEncoder();
    return PlatformControlP1::getInstance()->getRightWheelEncoder();
}

void CmdVelTask::executionWorker()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

    int maxAllowedDuty = 25;
    int minAllowedDuty = 15;

    int leftMotorDuty  = minAllowedDuty;
    int rightMotorDuty = minAllowedDuty;


    //******************************************************************************************encoder based
    /*
    int ticksPerMeter = PlatformControlP1::vagueEncoderTicksPerMeter;
    int curVelocityTicksL = 0;
    int curVelocityTicksR = 0;

    int velocityTicksL = 0;
    int velocityTicksR = 0;

    int curLeftEncoder = 0;
    int curRightEncoder = 0;

    int prevLeftEncoder = 0;
    int prevRightEncoder = 0;

    float wheelSeparation = ((double)Valter::wheelBase / 1000) / 2;
    */


    string stopExecutionReason = "";

    while (!stopped)
    {
        stopped = !checkFeasibility();

        if (!stopped)
        {
            if (!executing)
            {
                if (!platformControlP1->preparePlatformMovement())
                {
                    stopExecutionReason = Valter::format_string("Task#%lu (%s)has been terminated. preparePlatformMovement() returns FALSE.", getTaskId(), getTaskName().c_str());
                    qDebug("%s", stopExecutionReason.c_str());
                    stopExecution();
                    continue;
                }
                else
                {
                    executing = true;
                }
            }
            else
            {
                float linVel = getLinearVelocity();
                float angVel = getAngularVelocity();

                //******************************************************************************************experimental
                //robot spins 2*pi at the duty of [maxAllowedDuty] about [st] seconds, so the maxAngVel = (2*pi)/st (rad/sec)
                //robot linear speed at the duty of [maxAllowedDuty] maxLinVel = x(m/s)

                //angVelDuty = (abs(angVel) * maxAllowedDuty) / maxAngVel;
                //linVelDuty = (abs(linVel) * maxAllowedDuty) / maxLinVel;

                //leftMotorDuty  = round(linVelDuty - angVelDuty);
                //rightMotorDuty = round(linVelDuty + angVelDuty);


                //******************************************************************************************encoder based
                /*
                velocityTicksL = round(linVel * ticksPerMeter - angVel * wheelSeparation * ticksPerMeter);
                velocityTicksR = round(linVel * ticksPerMeter + angVel * wheelSeparation * ticksPerMeter);

                qDebug("tL %d tR %d", velocityTicksL, velocityTicksR);

                curLeftEncoder  = getLeftWheelEncoderReading();
                curRightEncoder = getRightWheelEncoderReading();

                curVelocityTicksL = curLeftEncoder  - prevLeftEncoder;
                curVelocityTicksR = curRightEncoder - prevRightEncoder;

                qDebug("cL %d cR %d", curVelocityTicksL, curVelocityTicksR);

                prevLeftEncoder  = getLeftWheelEncoderReading();
                prevRightEncoder = getRightWheelEncoderReading();
                */
            }
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
    platformControlP1->setLeftMotorDutyMax(initialLeftMotorMaxDuty);
    platformControlP1->setRightMotorDutyMax(initialRightMotorMaxDuty);

    if (!getCompleted())
    {
        string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal.%s", getTaskId(), ((stopExecutionReason.length() > 0) ? Valter::format_string(" REASON: %s", stopExecutionReason.c_str()).c_str() : ""));
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        setCompleted();
    }
}

