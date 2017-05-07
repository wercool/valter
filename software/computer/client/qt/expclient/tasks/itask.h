#ifndef ITASK_H
#define ITASK_H

#include <thread>
#include <chrono>
#include <string>

using namespace std;
using namespace chrono;

class ITask
{

public:
    ITask();

    virtual bool checkFeasibility(void)                 = 0;
    virtual bool initialize(void)                       = 0;
    virtual void execute(void)                          = 0;
    virtual void stopExecution(void)                    = 0;
    virtual void reportCompletion(void)                 = 0;

    unsigned long getTaskId() const;

    bool getExecuting() const;

    void setCompleted();
    bool getCompleted() const;

    void setBlocking(bool value);
    bool getBlocking() const;

    bool getAttachable() const;
    void setAttachable(bool value);

    bool getQDebugOn() const;
    void setQDebugOn(bool value);

    std::string getTaskName() const;

    bool getStopped() const;

    std::thread *getExecutionWorkerThread() const;
    void setExecutionWorkerThread(std::thread *value);

    std::string getTaskScriptLine() const;
    void setTaskScriptLine(const std::string &value);

protected:
    bool stopped;
    bool executing;
    bool completed;

    bool blocking;
    bool attachable;

    unsigned long taskId;
    std::string taskName;
    std::string taskScriptLine;

    bool qDebugOn;

    virtual void executionWorker(void) = 0;

    std::thread *executionWorkerThread;
};

#endif // ITASK_H
