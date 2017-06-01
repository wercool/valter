#ifndef SETHEADPITCHPOSITIONTASK_H
#define SETHEADPITCHPOSITIONTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>
#include <mutex>

class SetHeadPitchPositionTask : public ITask
{
public:
    SetHeadPitchPositionTask();

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

#endif // SETHEADPITCHPOSITIONTASK_H
