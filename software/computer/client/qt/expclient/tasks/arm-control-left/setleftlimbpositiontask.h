#ifndef SETLEFTLIMBPOSITIONTASK_H
#define SETLEFTLIMBPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLeftLimbPositionTask : public ITask
{
public:
    SetLeftLimbPositionTask();
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

#endif // SETLEFTLIMBPOSITIONTASK_H
