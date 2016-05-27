#ifndef ITASK_H
#define ITASK_H

#include <thread>
#include <chrono>
using namespace std;
using namespace chrono;

class ITask
{

public:
    ITask();
    virtual bool checkFeasibility(void) = 0;
    virtual bool initialize(void)       = 0;
    virtual void execute(void)          = 0;
    virtual void stopExecution(void)    = 0;

    unsigned long getTaskId();

    bool getExecuting() const;
    bool getCompleted() const;

    bool getBlocking() const;
    void setBlocking(bool value);


protected:
    bool stopped;
    bool blocking;
    bool executing;
    bool completed;

    unsigned long taskId;

    virtual void executionWorker(void) = 0;
};

#endif // ITASK_H
