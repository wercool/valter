#ifndef SETLEFTARMPOSITIONTASK_H
#define SETLEFTARMPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLeftArmPositionTask : public ITask
{
public:
    SetLeftArmPositionTask();
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

#endif // SETLEFTARMPOSITIONTASK_H
