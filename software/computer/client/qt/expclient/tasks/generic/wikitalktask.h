#ifndef WIKITALKTASK_H
#define WIKITALKTASK_H

#include <string>
#include "tasks/itask.h"

class WikiTalkTask : public ITask
{
public:
    WikiTalkTask();

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

#endif // WIKITALKTASK_H
