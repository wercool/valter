#ifndef DELAYTASK_H
#define DELAYTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class DelayTask : public ITask
{
public:
    DelayTask(unsigned int delay_ms_value);

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

protected:
    void executionWorker();

    unsigned int delay_ms;
};

#endif // DELAYTASK_H
