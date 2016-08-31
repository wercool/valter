#ifndef SETRIGHTFOREARMPOSITIONTASK_H
#define SETRIGHTFOREARMPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetRightForearmPositionTask : public ITask
{
public:
    SetRightForearmPositionTask();
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

#endif // SETRIGHTFOREARMPOSITIONTASK_H
