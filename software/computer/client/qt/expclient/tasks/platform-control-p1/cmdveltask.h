#ifndef CMDVELTASK_H
#define CMDVELTASK_H

#include <QtDebug>

#include "tasks/itask.h"
#include <thread>
#include <mutex>

class CmdVelTask : public ITask
{
public:
    CmdVelTask();

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
    int getLeftWheelEncoderReading();
    int getRightWheelEncoderReading();

private:
    float linearVelocity;   //in m/s; along X;   1m ~ 47 vague encoder ticks
    float angularVelocity;  //in rad/s; about Z;

    int initialLeftMotorMaxDuty;
    int initialRightMotorMaxDuty;

    std::mutex values_mutex;
};

#endif // CMDVELTASK_H
