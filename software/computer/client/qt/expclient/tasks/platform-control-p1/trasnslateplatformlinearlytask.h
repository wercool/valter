#ifndef TRASNSLATEPLATFORMLINEARLYTASK_H
#define TRASNSLATEPLATFORMLINEARLYTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class TrasnslatePlatformLinearlyTask : public ITask
{
public:
    TrasnslatePlatformLinearlyTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    void setDistance(float value);

    void setDirection(signed int value); // -1 = not defined, 1 = forward, 0 = backward

protected:
    void executionWorker();

private:
    float distance; //translate a distance in meters
    int direction; // -1 = not defined, 1 = forward, 0 = backward
    static signed int prevDirection;
    int initialLeftMotorMaxDuty;
    int initialRightMotorMaxDuty;
};

#endif // TRASNSLATEPLATFORMLINEARLYTASK_H
