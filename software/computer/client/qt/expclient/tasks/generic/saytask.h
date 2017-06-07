#ifndef SAYTASK_H
#define SAYTASK_H

#include <string>

class SayTask : public ITask
{
public:
    SayTask();

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

#endif // SAYTASK_H
