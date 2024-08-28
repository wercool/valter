#ifndef HEADMOTORSACTIVATETASK_H
#define HEADMOTORSACTIVATETASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class HeadMotorsActivateTask : public ITask
{
public:
    HeadMotorsActivateTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    unsigned int getState() const;
    void setState(unsigned int state_value);

protected:
    void executionWorker();

    unsigned int state; //1 - active, 0 - inactive
};

#endif // HEADMOTORSACTIVATETASK_H
