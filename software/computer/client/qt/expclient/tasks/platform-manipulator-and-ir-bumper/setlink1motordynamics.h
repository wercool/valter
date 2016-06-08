#ifndef SETLINK1MOTORDYNAMICS_H
#define SETLINK1MOTORDYNAMICS_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLink1MotorDynamics : public ITask
{
public:
    SetLink1MotorDynamics();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    void setMotorDutyMax(unsigned int value);
    void setMotorDeceleration(unsigned int value);
    void setMotorAcceleration(unsigned int value);

protected:
    void executionWorker();

private:
    unsigned int motorDutyMax = 0;
    unsigned int motorDeceleration = 0;
    unsigned int motorAcceleration = 0;
};

#endif // SETLINK1MOTORDYNAMICS_H
