#ifndef SETRIGHTARMYAWPOSITIONTASK_H
#define SETRIGHTARMYAWPOSITIONTASK_H


#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetRightArmYawPositionTask : public ITask
{
public:
    SetRightArmYawPositionTask();
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

#endif // SETRIGHTARMYAWPOSITIONTASK_H
