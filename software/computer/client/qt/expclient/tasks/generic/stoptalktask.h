#ifndef STOPTALKTASK_H
#define STOPTALKTASK_H

#include <string>
#include "tasks/itask.h"

class StopTalkTask : public ITask
{
public:
    StopTalkTask();

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

#endif // STOPTALKTASK_H
