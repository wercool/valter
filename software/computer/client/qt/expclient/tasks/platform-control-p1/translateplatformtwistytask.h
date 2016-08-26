#ifndef TRANSLATEPLATFORMTWISTYTASK_H
#define TRANSLATEPLATFORMTWISTYTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>
#include <mutex>

class TranslatePlatformTwistyTask : public ITask
{
public:
    TranslatePlatformTwistyTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

    static ITask *create();

    float getLinearVelocity() const;
    void setLinearVelocity(float value);

    float getAngularVelocity() const;
    void setAngularVelocity(float value);

protected:
    void executionWorker();

private:
    float linearVelocity;   //in m/s; along X;   1m ~ 47 vague encoder ticks
    float angularVelocity;  //in rad/s; about Z;
    int initialLeftMotorMaxDuty;
    int initialRightMotorMaxDuty;

    std::mutex _mutex;
};

#endif // TRANSLATEPLATFORMTWISTYTASK_H
