#ifndef SETLINK2MOTORDYNAMICS_H
#define SETLINK2MOTORDYNAMICS_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLink2MotorDynamics : public ITask
{
public:
    SetLink2MotorDynamics();

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
    unsigned int motorDutyMax;
    unsigned int motorDeceleration;
    unsigned int motorAcceleration;
};

#endif // SETLINK2MOTORDYNAMICS_H
