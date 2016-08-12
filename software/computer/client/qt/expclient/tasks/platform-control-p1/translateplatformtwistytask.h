#ifndef TRANSLATEPLATFORMTWISTYTASK_H
#define TRANSLATEPLATFORMTWISTYTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class TranslatePlatformTwistyTask : public ITask
{
public:
    TranslatePlatformTwistyTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

protected:
    void executionWorker();
};

#endif // TRANSLATEPLATFORMTWISTYTASK_H
