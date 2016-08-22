#include "valter.h"
#include "taskmanager.h"
#include <QtDebug>

#include "tcphandlers/taskmanager.tcphandler.cpp"


TaskManager* TaskManager::pTaskManager = NULL;
bool TaskManager::instanceFlag = false;
std::map<unsigned long, ITask*> queuedTasksMap = {};
std::map<std::string, ITask*> executingTasksMap = {};
ITask *processingTask = NULL;

TaskManager *TaskManager::getInstance()
{
    if(!instanceFlag)
    {
        pTaskManager = new TaskManager();
        instanceFlag = true;
        return pTaskManager;
    }
    else
    {
        return pTaskManager;
    }
}

TaskManager::TaskManager()
{
    initTcpInterface();
    qDebug("Task Manager initialized");
    Valter::log("Task Manager singleton initialized");
    queueStopped = false;
    incomingScriptProcessing = false;
    stopTopTask = false;
    new std::thread(&TaskManager::tasksQueueWorker, this);
}

bool TaskManager::getStopTopTask() const
{
    return stopTopTask;
}

void TaskManager::setStopTopTask(bool value)
{
    stopTopTask = value;
}

bool TaskManager::getIncomingScriptProcessing() const
{
    return incomingScriptProcessing;
}

void TaskManager::setIncomingScriptProcessing(bool value)
{
    incomingScriptProcessing = value;
}

unsigned int TaskManager::addTask(ITask *task)
{
    std::lock_guard<std::mutex> guard(tasks_mutex);
    queuedTasksMap.insert(pair<unsigned long, ITask*>(task->getTaskId(), task));
    qDebug("Tasks Queue length: %d", static_cast<int>(queuedTasksMap.size()));
    this_thread::sleep_for(std::chrono::milliseconds(10));
    unsigned long taskId = task->getTaskId();
    if (task->getAttachable())
    {
        if (executingTasksMap.find(task->getTaskName()) != executingTasksMap.end())
        {
            return taskId;
        }
    }
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", task->getTaskId(), task->getTaskName().c_str(), (task->getBlocking()) ? "blocking" : "non blocking", ((task->getCompleted()) ? "completed" : ((task->getExecuting()) ? "executing" : "queued")), task->getTaskScriptLine().c_str()));
    qDebug("Task [%lu] has been queued...", taskId);
    return taskId;
}

ITask* TaskManager::getTaskById(unsigned long id)
{
    if ( queuedTasksMap.find(id) == queuedTasksMap.end() )
    {
        return NULL;
    }
    else
    {
        return queuedTasksMap[id];
    }
}

ITask *TaskManager::getProcessingTask()
{
    return processingTask;
}

void TaskManager::stopTask(unsigned long taskId)
{
    qDebug("Task [%lu] will be stopped...", taskId);
    for(std::map<unsigned long, ITask*>::iterator it = queuedTasksMap.begin(); it != queuedTasksMap.end(); it++)
    {
        if (((ITask*)it->second)->getTaskId() == taskId)
        {
            ((ITask*)it->second)->stopExecution();
            qDebug("Task [%s %lu] has been stopped...", ((ITask*)it->second)->getTaskName().c_str(), ((ITask*)it->second)->getTaskId());
            return;
        }
    }
}

bool TaskManager::getQueueStopped() const
{
    return queueStopped;
}

void TaskManager::setQueueStopped(bool value)
{
    queueStopped = value;
}

void TaskManager::wipeQueuedCompletedTaskFromQueue(unsigned long id, bool onlyFromQueuedTasks)
{
    ITask *task = getTaskById(id);
    if (task != NULL)
    {
        std::lock_guard<std::mutex> guard(tasks_mutex);
        if (!onlyFromQueuedTasks)
        {
            if (executingTasksMap.find(task->getTaskName()) != executingTasksMap.end())
            {
                executingTasksMap.erase(task->getTaskName());
            }
        }
        if (queuedTasksMap.find(id) != queuedTasksMap.end())
        {
            queuedTasksMap.erase(id);
        }
    }
    qDebug("Tasks Queue length: %d", static_cast<int>(queuedTasksMap.size()));
}

void TaskManager::processScript(string script)
{
    setIncomingScriptProcessing(true);
    qDebug("processScript:\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvTASKS SCRIPTvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n%s\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^TASKS SCRIPT^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", script.c_str());
    std::vector<std::string> scriptInstructions = Valter::split(script, '\n');
    qDebug("\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvTASKS SCRIPT PROCESSINGvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n");
    for (int i = 0; i < static_cast<int>(scriptInstructions.size()); i++)
    {
        if (((string)scriptInstructions[i]).length() > 0)
        {
            //task start from scriptInstructionParts[0]
            std::vector<std::string> scriptInstructionParts = Valter::split(scriptInstructions[i], '_');

            if (((string)scriptInstructionParts[0]).compare("CLEARQUEUE") == 0)
            {
                TaskManager::getInstance()->clearQueue();
                continue;
            }
            if (((string)scriptInstructionParts[0]).compare("STOPTASK") == 0)
            {
                unsigned long taskid = atoll(((string)scriptInstructionParts[1]).c_str());
                qDebug(">>>>>>>>>>> STOP TASK >>>>>>> [(string)%s = %lu]", ((string)scriptInstructionParts[1]).c_str(), taskid);
                TaskManager::getInstance()->stopTask(taskid);
                continue;
            }
            if (((string)scriptInstructionParts[0]).compare("STOPTOPTASK") == 0)
            {
                TaskManager::getInstance()->setStopTopTask(true);
                continue;
            }
            if (((string)scriptInstructionParts[0]).compare("DELAY") == 0)
            {
                ITask *task = new DelayTask(atoi(((string)scriptInstructionParts[1]).c_str()));
                task->setTaskScriptLine(scriptInstructions[i]);
                TaskManager::getInstance()->addTask(task);
                continue;
            }
            if (((string)scriptInstructionParts[0]).compare("INITIAL") == 0)
            {
                ITask *task = new SetModuleInitialStateTask();
                task->setTaskScriptLine(scriptInstructions[i]);
                TaskManager::getInstance()->addTask(task);
                continue;
            }

            //Task for particular Control Device
            routeTaskRequest(scriptInstructions[i]);
        }
    }
    qDebug("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^TASKS SCRIPT PROCESSING^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
    setIncomingScriptProcessing(false);
}

void TaskManager::clearQueue()
{
    for(std::map<unsigned long, ITask*>::iterator it = queuedTasksMap.begin(); it != queuedTasksMap.end(); it++)
    {
        ((ITask*)it->second)->stopExecution();
    }
    std::lock_guard<std::mutex> guard(tasks_mutex);
    executingTasksMap.clear();
    queuedTasksMap.clear();
}

void TaskManager::sendScript(string script)
{
    vector<string> remoteControlDeviceTCPInterfacesIpAdresses = Valter::getInstance()->getRemoteControlDeviceTCPInterfacesIpAddressesVector();
    for(std::vector<string>::size_type i = 0; i != remoteControlDeviceTCPInterfacesIpAdresses.size(); i++)
    {
        sendScriptToRemoteTaskManager(script, remoteControlDeviceTCPInterfacesIpAdresses[i]);
    }
}

bool TaskManager::sendScriptToRemoteTaskManager(string script, string ipAddress)
{
    TCPStream* stream = getTcpInterface()->getCommandInterfaceConnector()->connect(ipAddress.c_str(), getTcpInterface()->getPort());
    int length;
    char response[256];
    if (stream)
    {
        //qDebug("sent - %s", command.c_str());
        stream->send(script.c_str(), script.size());
        length = stream->receive(response, sizeof(response));
        response[length] = '\0';
        //qDebug("received - %s", response);
        delete stream;

        return true;
    }
    return false;
}

bool TaskManager::sendMessageToCentralHostTaskManager(string message)
{
    //FORMAT
    //RTMM~{task id}~{task name}~{task type}~{task status}~{script line}~{notes}
    if (getTcpInterface()->getCentralCommandHostIP().compare("") != 0)
    {
        TCPStream* stream = getTcpInterface()->getCommandInterfaceConnector()->connect(getTcpInterface()->getCentralCommandHostIP().c_str(), getTcpInterface()->getCentralCommandHostIPPort());
        int length;
        char response[256];

        message = "RTMM~" + message;

        if (stream)
        {
            //qDebug("sent - %s", command.c_str());
            stream->send(message.c_str(), message.size());
            length = stream->receive(response, sizeof(response));
            response[length] = '\0';
            //qDebug("received - %s", response);
            delete stream;

            return true;
        }
        else
        {
            clearQueue();
        }
    }
    return false;
}

unsigned int TaskManager::routeTaskRequest(string taskMessage)
{
    qDebug("routeTaskRequest: %s", taskMessage.c_str());
    std::vector<std::string> taskMessageParts = Valter::split(taskMessage, '_');
    if (((string)taskMessageParts[0]).compare("T") == 0)
    {
        IValterModule* valterModule = Valter::getInstance()->getValterModulePtrByShortName(taskMessageParts[1]);
        std::string taskScriptLine = "";
        for (int i = 2; i <static_cast<int>(taskMessageParts.size()); i++)
        {
            if (i > 2)
            {
                taskScriptLine.append("_");
            }
            taskScriptLine.append(taskMessageParts[i]);
        }
        if (valterModule->getControlDeviceIsSet())
        {
            if (valterModule->getControlDevice()->getStatus() == ControlDevice::StatusActive)
            {
                //qDebug("%s", valterModule->getControlDevice()->getControlDeviceId().c_str());
                return valterModule->executeTask(taskScriptLine);
            }
        }
        else
        {
            qDebug("Task %s could not be executed - NO COTNROL DEVICE CONNECTED", taskScriptLine.c_str());
        }
    }
    return 0;
}

void TaskManager::tasksQueueWorker()
{
    int msgCnt = 0;
    string debugMsg = "";

    while (!queueStopped)
    {
        if (!getIncomingScriptProcessing())
        {
            try
            {
                if (!queuedTasksMap.empty())
                {
                    for(std::map<unsigned long, ITask*>::iterator it = queuedTasksMap.begin(); it != queuedTasksMap.end(); it++)
                    {
                        processingTask = it->second;
                        if (processingTask->getCompleted())
                        {
                            wipeQueuedCompletedTaskFromQueue(processingTask->getTaskId(), false);
                            processingTask->reportCompletion();
                            break;
                        }
                        //getExecuting() has a major impact (if not executed right now - just start it if no concurrent [same command] is still running)
                        if (!processingTask->getExecuting() && !processingTask->getCompleted()  && !processingTask->getStopped())
                        {
                            if (executingTasksMap.find(processingTask->getTaskName()) != executingTasksMap.end())
                            {
                                ITask *runningTask = executingTasksMap[processingTask->getTaskName()];
                                if (processingTask->getTaskId() != runningTask->getTaskId())
                                {
                                    if (runningTask->getAttachable())
                                    {
                                        qDebug("Task#%lu (%s) will be attached to Task#%lu (%s)", processingTask->getTaskId(), processingTask->getTaskName().c_str(), runningTask->getTaskId(), runningTask->getTaskName().c_str());
                                        runningTask->setTaskScriptLine(processingTask->getTaskScriptLine());
                                        wipeQueuedCompletedTaskFromQueue(processingTask->getTaskId(), true);
                                        string msg = Valter::format_string("%s has been attached [%s]", processingTask->getTaskName().c_str(), processingTask->getTaskScriptLine().c_str());
                                        qDebug("%s", msg.c_str());
                                        TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~notes~%s", runningTask->getTaskId(), msg.c_str()));
                                    }
                                    else
                                    {
                                        string msg = Valter::format_string("Task#%lu (%s) is postponed because the concurent Task#%lu (%s) is beeing executed right now. Treated as BLOCKING task.", processingTask->getTaskId(), processingTask->getTaskName().c_str(), runningTask->getTaskId(), runningTask->getTaskName().c_str());
                                        if (debugMsg.compare(msg) == 0)
                                        {
                                            msgCnt++;
                                            if (msgCnt > 100)
                                            {
                                                msgCnt = 0;
                                                qDebug("%s", msg.c_str());
                                            }
                                        }
                                        else
                                        {
                                            msgCnt = 0;
                                            debugMsg = msg;
                                            qDebug("%s", msg.c_str());
                                        }
                                        break;
                                    }
                                }
                                continue;
                            }
                            executingTasksMap.insert(pair<std::string, ITask*>(processingTask->getTaskName(), processingTask));
                            processingTask->execute();
                            if (processingTask->getBlocking())
                            {
                                break;
                            }
                        }
                        else //processing task is being executed or completed or stopped
                        {
                            if (processingTask->getExecuting())
                            {
                                if (getStopTopTask())
                                {
                                    processingTask->stopExecution();
                                    setStopTopTask(false);
                                    continue;
                                }
                                if (processingTask->getBlocking())
                                {
                                    break;
                                }
                            }
                            else if (processingTask->getStopped())
                            {
                                processingTask->stopExecution();
                            }
                        }
                    }

                    this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                else
                {
                    this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            catch(const std::system_error& e)
            {
                qDebug("Caught system_error with code %d  meaning %s",  e.code().value(), e.what());
            }
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    qDebug("STOPPED: TaskManager::tasksQueueWorker");
}

std::map<long, string> TaskManager::getRtmms() const
{
    return rtmms;
}

TCPInterface *TaskManager::getTcpInterface() const
{
    return tcpInterface;
}

void TaskManager::setTcpInterface(TCPInterface *value)
{
    tcpInterface = value;
}

void TaskManager::initTcpInterface()
{
    if (!getTcpInterface())
    {
        TCPInterface *tcpInterface = new TCPInterface(55555);
        setTcpInterface(tcpInterface);
        initTcpCommandAcceptorInterface();
    }
}

void TaskManager::initTcpCommandAcceptorInterface()
{
    getTcpInterface()->setConnectionHandler((Thread*)new TaskManagerTCPConnectionHandler(getTcpInterface()->queue));
    getTcpInterface()->startListening();
}

void TaskManager::addUpdateRTMM(string rtmm)
{
    vector<string>rtmm_values = Valter::split(rtmm, '~');

    //RTMM~{task id}~{notes}~{......}
    if (((string)rtmm_values[2]).compare("notes") == 0)
    {
        long taskId = atol(((string)rtmm_values[1]).c_str());
        string storedTaskDesk = getRTMMDesc(taskId);
        vector<string>existing_rtmm_values = Valter::split(storedTaskDesk, '~');
        existing_rtmm_values[4] = (((string)existing_rtmm_values[4]).length() > 0) ? (existing_rtmm_values[4] + "\n" + rtmm_values[3]) : rtmm_values[3];
        string taskDesc = existing_rtmm_values[0] + "~" + existing_rtmm_values[1] + "~" + existing_rtmm_values[2] + "~" + existing_rtmm_values[3] + "~" + existing_rtmm_values[4];
        rtmms[taskId] = taskDesc;
    }
    else
    {
        long taskId = atol(((string)rtmm_values[1]).c_str());
        //RTMM~{task id}~{task name}~{task type}~{task status}~{script line}~{notes}
        //stored as
        //rtmms[{task id}] = {task name}~{task type}~{task status}~{script line}~{notes}
        string storedTaskDesk = getRTMMDesc(taskId);
        vector<string>existing_rtmm_values = Valter::split(storedTaskDesk, '~');

        string taskDesc = rtmm_values[2] + "~" + rtmm_values[3] + "~" + rtmm_values[4] + "~" + rtmm_values[5] + "~" + ((rtmm_values.size() > 6) ? (((existing_rtmm_values.size() > 4) ? existing_rtmm_values[4] + "\n" + rtmm_values[6] : rtmm_values[6])) : ((existing_rtmm_values.size() > 4) ? existing_rtmm_values[4] : ""));
        rtmms[taskId] = taskDesc;
    }
}

string TaskManager::getRTMMDesc(long taskId)
{
    if (rtmms.find(taskId) != rtmms.end())
    {
        return rtmms[taskId];
    }
    return "";
}

void TaskManager::removeRTMM(long taskId)
{
    std::mutex rtmms_mutex;
    std::lock_guard<std::mutex> guard(rtmms_mutex);
    rtmms.erase(taskId);
}

void TaskManager::clearRTMM()
{
    std::mutex rtmms_mutex;
    std::lock_guard<std::mutex> guard(rtmms_mutex);
    rtmms.clear();
}
