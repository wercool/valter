#ifndef ROTATEPLATFORMTASK_H
#define ROTATEPLATFORMTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class RotatePlatformTask : public ITask
{
public:
    RotatePlatformTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    void setDirection(int value);

protected:
    void executionWorker();

private:
    int direction; // -1 = not defined, 1 = CW (right), 0 = CCW (left)
    static signed int prevDirection;
};

#endif // ROTATEPLATFORMTASK_H
