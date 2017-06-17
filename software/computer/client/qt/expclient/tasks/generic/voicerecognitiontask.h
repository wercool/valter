#ifndef VOICERECOGNITIONTASK_H
#define VOICERECOGNITIONTASK_H

#include "tasks/itask.h"

class VoiceRecognitionTask : public ITask
{
public:
    VoiceRecognitionTask();

    // ITask interface
public:
    bool checkFeasibility();
    bool initialize();
    void execute();
    void stopExecution();
    void reportCompletion();

protected:
    void executionWorker();
};

#endif // VOICERECOGNITIONTASK_H
