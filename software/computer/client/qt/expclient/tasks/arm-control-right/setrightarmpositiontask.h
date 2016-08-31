#ifndef SETRIGHTARMPOSITIONTASK_H
#define SETRIGHTARMPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetRightArmPositionTask : public ITask
{
public:
    SetRightArmPositionTask();
    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    float getAngle() const;
    void setAngle(float value);

protected:
    void executionWorker();

private:
    float angle;
};

#endif // SETRIGHTARMPOSITIONTASK_H
