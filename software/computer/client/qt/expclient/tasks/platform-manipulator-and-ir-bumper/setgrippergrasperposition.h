#ifndef SETGRIPPERGRASPERPOSITION_H
#define SETGRIPPERGRASPERPOSITION_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetGripperGrasperPosition : public ITask
{
public:
    SetGripperGrasperPosition();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    float getPosition() const;
    void setPosition(float value);

protected:
    void executionWorker();

private:
    float position;
};

#endif // SETGRIPPERGRASPERPOSITION_H
