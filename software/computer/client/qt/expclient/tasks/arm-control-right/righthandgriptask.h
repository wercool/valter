#ifndef RIGHTHANDGRIPTASK_H
#define RIGHTHANDGRIPTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class RightHandgripTask : public ITask
{
public:
    RightHandgripTask();
    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    int getMode() const;
    void setMode(int value);

protected:
    void executionWorker();

private:
    int mode;
};

#endif // RIGHTHANDGRIPTASK_H
