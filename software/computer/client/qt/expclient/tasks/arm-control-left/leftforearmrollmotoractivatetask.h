#ifndef LEFTFOREARMROLLMOTORACTIVATETASK_H
#define LEFTFOREARMROLLMOTORACTIVATETASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class LeftForearmRollMotorActivateTask : public ITask
{
public:
    LeftForearmRollMotorActivateTask();

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

#endif // LEFTFOREARMROLLMOTORACTIVATETASK_H
