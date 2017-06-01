#ifndef SETHEADYAWPOSITIONTASK_H
#define SETHEADYAWPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>
#include <mutex>

class SetHeadYawPositionTask : public ITask
{
public:
    SetHeadYawPositionTask();

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
    float angle;  //in degrees;

    std::mutex values_mutex;
};

#endif // SETHEADYAWPOSITIONTASK_H
