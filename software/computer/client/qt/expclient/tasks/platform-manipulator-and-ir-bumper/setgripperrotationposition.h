#ifndef SETGRIPPERROTATIONPOSITION_H
#define SETGRIPPERROTATIONPOSITION_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetGripperRotationPosition : public ITask
{
public:
    SetGripperRotationPosition();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    float getAngle() const;
    void setAngle(float value);

protected:
    void executionWorker();

private:
    float angle;
};

#endif // SETGRIPPERROTATIONPOSITION_H
