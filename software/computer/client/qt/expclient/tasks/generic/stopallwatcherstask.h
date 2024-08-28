#ifndef STOPALLWATCHERSTASK_H
#define STOPALLWATCHERSTASK_H


#include <QtDebug>

#include "tasks/itask.h"
#include <thread>


class StopAllWatchersTask : public ITask
{
public:
    StopAllWatchersTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

protected:
    void executionWorker();
};

#endif // STOPALLWATCHERSTASK_H
