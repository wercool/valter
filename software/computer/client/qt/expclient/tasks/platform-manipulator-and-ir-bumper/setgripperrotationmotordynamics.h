#ifndef SETGRIPPERROTATIONMOTORDYNAMICS_H
#define SETGRIPPERROTATIONMOTORDYNAMICS_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetGripperRotationMotorDynamics : public ITask
{
public:
    SetGripperRotationMotorDynamics();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    void setMotorDutyMax(unsigned int value);

protected:
    void executionWorker();

private:
    unsigned int motorDutyMax = 0;
};

#endif // SETGRIPPERROTATIONMOTORDYNAMICS_H
