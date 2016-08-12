#include "valter.h"
#include "setlink2positiontask.h"

SetLink2PositionTask::SetLink2PositionTask()
{
    qDebugOn = true;
    taskName = "SetLink2PositionTask";
    blocking = false;
}

bool SetLink2PositionTask::checkFeasibility()
{
    if (angle < 0 || angle > 90)
    {
        string msg = Valter::format_string("Task#%lu target Link2 angle %f in unreachable.", getTaskId(), angle);
        qDebug("%s", msg.c_str());
        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

        return false;
    }
    return true;
}

bool SetLink2PositionTask::initialize()
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

void SetLink2PositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetLink2PositionTask::executionWorker, this);
            this_thread::sleep_for(std::chrono::milliseconds(100));
            TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
        }
    }
    this_thread::sleep_for(std::chrono::milliseconds(100));
    stopExecution();
    setCompleted();
}

void SetLink2PositionTask::stopExecution()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink2MotorActivated(false);
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void SetLink2PositionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) completed.", getTaskId(), getTaskName().c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

ITask *SetLink2PositionTask::create()
{
    return (ITask*)new SetLink2PositionTask();
}

void SetLink2PositionTask::executionWorker()
{
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    /************************************ emulation *********************start***************************/
//    platformManipulatorAndIRBumper->setLink2ADCPosition(650); //0 degrees = 278; 45 deg - 650
    /************************************ emulation *********************finish**************************/

    float sigma = 1.0; //precision in degrees

    //descending (angle increased) - true
    bool direction = (angle > platformManipulatorAndIRBumper->getLink2Position()) ? true : false;

    float cutoffAngle = (direction) ? (angle * 0.98) : (angle / 0.98); //<<<<<<<<<<<<<<< dynamic parameter

    if (abs(angle - platformManipulatorAndIRBumper->getLink2Position()) < sigma)
    {
        setCompleted();
        return;
    }

    while (!stopped)
    {
        if (!executing)
        {
            if (direction) //move Link2 descending
            {

                if (platformManipulatorAndIRBumper->prepareManLink2Movement())
                {
                    //descent
                    if (platformManipulatorAndIRBumper->setLink2MovementDirection(false))
                    {
                        platformManipulatorAndIRBumper->setLink2MotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason platformManipulatorAndIRBumper->prepareManLink2Movement()", getTaskId());
                    qDebug("%s", msg.c_str());
                    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

                    stopExecution();
                    return;
                }
            }
            else
            {
                if (platformManipulatorAndIRBumper->prepareManLink2Movement())
                {
                    //ascent
                    if (platformManipulatorAndIRBumper->setLink2MovementDirection(true))
                    {
                        platformManipulatorAndIRBumper->setLink2MotorActivated(true);
                        executing = true;
                    }
                }
                else
                {
                    string msg = Valter::format_string("Task#%lu could not be completed; reason platformManipulatorAndIRBumper->prepareManLink2Movement()", getTaskId());
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
                qDebug("Task#%lu: link2Position (deg) = %f, target (deg) = %f, cutoffAngle = %f, dSigma = %f ? %f, direction: %s", getTaskId(), platformManipulatorAndIRBumper->getLink2Position(), angle, cutoffAngle, abs(angle - platformManipulatorAndIRBumper->getLink2Position()), sigma, (platformManipulatorAndIRBumper->getLink2MovementDirection() ? "ascent" : "descent"));
            }

            if ((direction && platformManipulatorAndIRBumper->getLink2Position() >= cutoffAngle) || (!direction && platformManipulatorAndIRBumper->getLink2Position() <= cutoffAngle) || (abs(angle - platformManipulatorAndIRBumper->getLink2Position()) <= sigma))
            {
                if (platformManipulatorAndIRBumper->getLink2MotorActivated())
                {
                    platformManipulatorAndIRBumper->setLink2MotorActivated(false);
                    setCompleted();
                    return;
                }
            }

            /************************************ emulation *********************start***************************/
//            int positionADC = platformManipulatorAndIRBumper->getLink2ADCPosition();
//            if (direction)
//            {
//                positionADC += 5;
//            }
//            else
//            {
//                positionADC -= 5;
//            }
//            platformManipulatorAndIRBumper->setLink2ADCPosition(positionADC);
            /************************************ emulation *********************finish**************************/
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    string msg = Valter::format_string("Task#%lu has been stopped via stopExecution() signal", getTaskId());
    qDebug("%s", msg.c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", getTaskId(), msg.c_str()));

    setCompleted();
}

float SetLink2PositionTask::getAngle() const
{
    return angle;
}

void SetLink2PositionTask::setAngle(float value)
{
    angle = value;
}
