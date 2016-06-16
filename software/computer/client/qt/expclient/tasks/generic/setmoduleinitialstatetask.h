#ifndef SETMODULEINITIALSTATETASK_H
#define SETMODULEINITIALSTATETASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>
#include "valter.h"

class SetModuleInitialStateTask : public ITask
{
public:
    SetModuleInitialStateTask();

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

#endif // SETMODULEINITIALSTATETASK_H
