#ifndef NOOP_H
#define NOOP_H


#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class NoOpTask : public ITask
{
public:
    NoOpTask();

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

#endif // NOOP_H
