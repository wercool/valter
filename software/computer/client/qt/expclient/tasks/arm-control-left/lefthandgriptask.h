#ifndef LEFTHANDGRIPTASK_H
#define LEFTHANDGRIPTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class LeftHandgripTask : public ITask
{
public:
    LeftHandgripTask();
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

#endif // LEFTHANDGRIPTASK_H
