#include "valter.h"
#include "voicerecognitiontask.h"

VoiceRecognitionTask::VoiceRecognitionTask()
{
    completed = false;
    taskName = "VOICERECOGNITION";
    blocking = false;
}

bool VoiceRecognitionTask::checkFeasibility()
{
    return true;
}

bool VoiceRecognitionTask::initialize()
{
    return true;
}

void VoiceRecognitionTask::execute()
{
    executing = true;
    new std::thread(&VoiceRecognitionTask::executionWorker, this);
}

void VoiceRecognitionTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void VoiceRecognitionTask::reportCompletion()
{
    qDebug("Task#%lu (%s) is completed.", getTaskId(), getTaskName().c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void VoiceRecognitionTask::executionWorker()
{
    std::string cmd = "/home/maska/git/valter/software/raspberrypi3/ValterTalks/start";
    Valter::getInstance()->executeShellCmdLinuxAndDetach(cmd);
    setCompleted();
}
