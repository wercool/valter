#ifndef SETLEFTFOREARMPOSITIONTASK_H
#define SETLEFTFOREARMPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLeftForearmPositionTask : public ITask
{
public:
    SetLeftForearmPositionTask();
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

#endif // SETLEFTFOREARMPOSITIONTASK_H
