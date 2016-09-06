#ifndef SETLEFTARMYAWPOSITIONTASK_H
#define SETLEFTARMYAWPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>

class SetLeftArmYawPositionTask : public ITask
{
public:
    SetLeftArmYawPositionTask();
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

#endif // SETLEFTARMYAWPOSITIONTASK_H
