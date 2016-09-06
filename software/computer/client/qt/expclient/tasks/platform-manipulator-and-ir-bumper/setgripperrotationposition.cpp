#include "valter.h"
#include "setgripperrotationposition.h"

SetGripperRotationPosition::SetGripperRotationPosition()
{
    qDebugOn = true;
    taskName = "SetGripperRotationPosition";
    blocking = false;
}

bool SetGripperRotationPosition::checkFeasibility()
{
    if (angle > 177 || angle < -89)
    {
        string msg = Valter::format_string("Task#%lu (%s) target angle %f in unreachable.", getTaskId(), getTaskName().c_str(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetGripperRotationPosition::initialize()
{
    //script line parsing
    std::vector<std::string> taskInitiationParts = Valter::split(taskScriptLine, '_');
    std::string taskName = taskInitiationParts[0];
    float angle = atof(((string)taskInitiationParts[1]).c_str());
    setAngle(angle);

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

void SetGripperRotationPosition::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetGripperRotationPosition::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
            return;
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetGripperRotationPosition::stopExecution()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperRotateStop();
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetGripperRotationPosition::reportCompletion()
{
    qDebug("Task#%lu (%s) %s.", getTaskId(), getTaskName().c_str(), (stopped) ? "stopped" : "completed");
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetGripperRotationPosition::create()
{
    return (ITask*)new SetGripperRotationPosition();
}

float SetGripperRotationPosition::getAngle() const
{
    return angle;
}

void SetGripperRotationPosition::setAngle(float value)
{
    angle = value;
}

void SetGripperRotationPosition::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    /************************************ emulation *********************start***************************/
    platformManipulatorAndIRBumper->setGripperADCRotation(355); //0 degrees = 355;
    /************************************ emulation *********************finish**************************/

    float sigma = 0.5; //precision in degrees

    //true CW
    bool direction = (angle > platformManipulatorAndIRBumper->getGripperRotation()) ? true : false;

    float cutoffAngle = angle * 0.99; //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(abs(angle) - abs(platformManipulatorAndIRBumper->getGripperRotation())) < sigma)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //rotate gripper CW
            {
                platformManipulatorAndIRBumper->manGripperRotateCW();
            }
            else
            {
                platformManipulatorAndIRBumper->manGripperRotateCCW();
            }
            executing = true;
        }
        else
        {

            if (qDebugOn)
            {
                qDebug("Task#%lu: GripperRotation (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), platformManipulatorAndIRBumper->getGripperRotation(), angle, cutoffAngle, abs(abs(angle) - abs(platformManipulatorAndIRBumper->getGripperRotation())), sigma, (direction ? "CW" : "CCW"));
            }

            if ((direction && platformManipulatorAndIRBumper->getGripperRotation() >= cutoffAngle) || ((!direction && platformManipulatorAndIRBumper->getGripperRotation() <= cutoffAngle)) || (abs(angle - platformManipulatorAndIRBumper->getGripperRotation()) < sigma))
            {
                platformManipulatorAndIRBumper->manGripperRotateStop();
                setCompleted();
                break;
            }

            /************************************ emulation *********************start***************************/
            int positionADC = platformManipulatorAndIRBumper->getGripperADCRotation();
            if (direction)
            {
                positionADC += 1;
            }
            else
            {
                positionADC -= 1;
            }
            platformManipulatorAndIRBumper->setGripperADCRotation(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!getCompleted())
    {
        string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        setCompleted();
    }
}
