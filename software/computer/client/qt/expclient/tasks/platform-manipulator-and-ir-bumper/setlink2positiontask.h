#ifndef SETLINK2POSITIONTASK_H
#define SETLINK2POSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLink2PositionTask : public ITask
{
public:
    SetLink2PositionTask(float targetAngle);

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    // ITask interface
protected:
    void executionWorker();

private:
    float angle;
};

#endif // SETLINK2POSITIONTASK_H
