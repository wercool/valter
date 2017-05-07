#ifndef SETLEFTARMROLLPOSITIONTASK_H
#define SETLEFTARMROLLPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLeftArmRollPositionTask : public ITask
{
public:
    SetLeftArmRollPositionTask();

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

#endif // SETLEFTARMROLLPOSITIONTASK_H
