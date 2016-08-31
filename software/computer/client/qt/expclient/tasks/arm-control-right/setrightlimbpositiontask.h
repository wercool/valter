#ifndef SETRIGHTLIMBPOSITIONTASK_H
#define SETRIGHTLIMBPOSITIONTASK_H

#include "tasks/itask.h"
#include <thread>

class SetRightLimbPositionTask : public ITask
{
public:
    SetRightLimbPositionTask();
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

#endif // SETRIGHTLIMBPOSITIONTASK_H
