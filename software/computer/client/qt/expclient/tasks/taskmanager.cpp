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
    unsigned taskId = task->getTaskId();
    qDebug("Task [%d] has been queued...", taskId);
    getTcpInterface()->sendCDRToCentralCommandHost("!!!!!!!!!!!");
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

bool TaskManager::getQueueStopped() const
{
    return queueStopped;
}

void TaskManager::setQueueStopped(bool value)
{
    queueStopped = value;
}

void TaskManager::wipeQueuedCompletedTaskFromQueue(unsigned long id)
{
    ITask *task = getTaskById(id);
    if (task != NULL)
    {
        std::lock_guard<std::mutex> guard(tasks_mutex);
        if (executingTasksMap.find(task->getTaskName()) != executingTasksMap.end())
        {
            executingTasksMap.erase(task->getTaskName());
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
    qDebug("processScript:\n=====================SCRIPT============start\n%s\n=====================SCRIPT============end\n", script.c_str());
    std::vector<std::string> scriptInstructions = Valter::split(script, '\n');
    for (int i = 0; i < static_cast<int>(scriptInstructions.size()); i++)
    {
        if (((string)scriptInstructions[i]).length() > 0)
        {
            std::vector<std::string> scriptInstructionParts = Valter::split(scriptInstructions[i], '_');
            if (((string)scriptInstructionParts[0]).compare("CLEARQUEUE") == 0)
            {
                TaskManager::getInstance()->clearQueue();
                continue;
            }
            if (((string)scriptInstructionParts[0]).compare("STOPTOPTASK") == 0)
            {
                TaskManager::getInstance()->setStopTopTask(true);
                continue;
            }
            if (((string)scriptInstructionParts[0]).compare("DELAY") == 0)
            {
                TaskManager::getInstance()->addTask(new DelayTask(atoi(((string)scriptInstructionParts[1]).c_str())));
                continue;
            }
            if (((string)scriptInstructionParts[0]).compare("INITIAL") == 0)
            {
                TaskManager::getInstance()->addTask(new SetModuleInitialStateTask());
                continue;
            }

            routeTaskRequest(scriptInstructions[i]);
        }
    }
    qDebug("\n=====================SCRIPT============processing completed\n");
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
                        if (!processingTask->getExecuting() && !processingTask->getCompleted()  && !processingTask->getStopped())
                        {
                            if (executingTasksMap.find(processingTask->getTaskName()) != executingTasksMap.end())
                            {
                                continue;
                            }
                            executingTasksMap.insert(pair<std::string, ITask*>(processingTask->getTaskName(), processingTask));
                            processingTask->execute();
                            if (processingTask->getBlocking())
                            {
                                break;
                            }
                        }
                        else
                        {
                            if (processingTask->getExecuting())
                            {
                                if (getStopTopTask())
                                {
                                    processingTask->stopExecution();
                                    setStopTopTask(false);
                                }
                                if (processingTask->getBlocking())
                                {
                                    break;
                                }
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
