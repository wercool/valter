#include "valter.h"
#include "setlink2positiontask.h"

SetLink2PositionTask::SetLink2PositionTask(float targetAngle)
{
    angle = targetAngle;
}

bool SetLink2PositionTask::checkFeasibility()
{
    return true;
}

bool SetLink2PositionTask::initialize()
{
    return true;
}

void SetLink2PositionTask::execute()
{
    if (initialize())
    {
        if (checkFeasibility())
        {
            new std::thread(&SetLink2PositionTask::executionWorker, this);
        }
    }
}

void SetLink2PositionTask::stopExecution()
{
    stopped = true;
}

void SetLink2PositionTask::reportCompletion()
{
     qDebug("Task#%lu task assumed as completed", getTaskId());
}

void SetLink2PositionTask::executionWorker()
{
    PlatformManipulatorAndIRBumper *root = PlatformManipulatorAndIRBumper::getInstance();
root->setLink2ADCPosition(136); //45 degrees = 542;
    float sigma = 0.5; //precision in degrees

    executing = true;
    while (!stopped)
    {
        qDebug("Task#%lu: link2Position (deg) = %f, target (deg) = %f, dSigma = %f ? %f", getTaskId(), root->getLink2Position(), angle, abs(angle - root->getLink2Position()), sigma);

        if (abs(angle - root->getLink2Position()) < sigma)
        {
            setCompleted(true);
            return;
        }
        /************************************ emulation *********************start***************************/
        int positionADC = root->getLink2ADCPosition();
        root->setLink2ADCPosition(++positionADC);
        /************************************ emulation *********************finish**************************/

        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    qDebug("Task#%lu has been stopped via stopExecution() signal", getTaskId());
}
