#ifndef SETLINK1POSITIONTASK_H
#define SETLINK1POSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLink1PositionTask : public ITask
{
public:
    SetLink1PositionTask();
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

#endif // SETLINK1POSITIONTASK_H
