#ifndef ROTATEBODYTASKS_H
#define ROTATEBODYTASKS_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class RotateBodyTask : public ITask
{
public:
    RotateBodyTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    void setDirection(int value);
    void setAngle(float value);

protected:
    void executionWorker();

private:
    int direction; // -1 = not defined, 1 = CW (right), 0 = CCW (left)
    float angle;
    static signed int prevDirection;
};

#endif // ROTATEBODYTASKS_H
