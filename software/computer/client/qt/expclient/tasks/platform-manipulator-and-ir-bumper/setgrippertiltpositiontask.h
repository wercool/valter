#ifndef SETGRIPPERTILTPOSITIONTASK_H
#define SETGRIPPERTILTPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetGripperTiltPositionTask : public ITask
{
public:
    SetGripperTiltPositionTask();

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

#endif // SETGRIPPERTILTPOSITIONTASK_H
