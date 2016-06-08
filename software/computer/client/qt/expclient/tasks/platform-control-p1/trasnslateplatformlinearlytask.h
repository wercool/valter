#ifndef TRASNSLATEPLATFORMLINEARLYTASK_H
#define TRASNSLATEPLATFORMLINEARLYTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class TrasnslatePlatformLinearlyTask : public ITask
{
public:
    TrasnslatePlatformLinearlyTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    void setDistance(float value);

protected:
    void executionWorker();

private:
    float distance; //translate a distance in meters
};

#endif // TRASNSLATEPLATFORMLINEARLYTASK_H
