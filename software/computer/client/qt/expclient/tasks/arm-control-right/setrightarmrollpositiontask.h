#ifndef SETRIGHTARMROLLPOSITIONTASK_H
#define SETRIGHTARMROLLPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetRightArmRollPositionTask : public ITask
{
public:
    SetRightArmRollPositionTask();
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

#endif // SETRIGHTARMROLLPOSITIONTASK_H
