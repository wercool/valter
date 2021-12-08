#ifndef TALK_H
#define TALK_H

#include <string>
#include "tasks/itask.h"

class TalkTask : public ITask
{
public:
    TalkTask();

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

#endif // TALK_H
