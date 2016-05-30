#ifndef SETLINK2POSITIONTASK_H
#define SETLINK2POSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLink2PositionTask : public ITask
{
public:
    SetLink2PositionTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    float getAngle() const;
    void setAngle(float value);

protected:
    void executionWorker();

private:
    float angle;
    static float prevAngle;
};

#endif // SETLINK2POSITIONTASK_H
