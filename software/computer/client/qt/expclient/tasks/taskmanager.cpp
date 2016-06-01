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
    new std::thread(&TaskManager::tasksQueueWorker, this);
}

unsigned int TaskManager::addTask(ITask *task)
{
    std::lock_guard<std::mutex> guard(tasks_mutex);
    queuedTasksMap.insert(pair<unsigned long, ITask*>(task->getTaskId(), task));
    qDebug("Tasks Queue length: %d", static_cast<int>(queuedTasksMap.size()));
    this_thread::sleep_for(std::chrono::milliseconds(10));
    return task->getTaskId();
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
    qDebug("processScript:\n=====================SCRIPT============start===============\n%s\n=====================SCRIPT============end=================\n", script.c_str());
    std::vector<std::string> scriptInstructions = Valter::split(script, '\n');
    for (int i = 0; i < static_cast<int>(scriptInstructions.size()); i++)
    {
        if (((string)scriptInstructions[i]).length() > 0)
        {
            std::vector<std::string> scriptInstructionParts = Valter::split(scriptInstructions[i], '_');
            if (((string)scriptInstructionParts[0]).compare("DELAY") == 0)
            {
                TaskManager::getInstance()->addTask(new DelayTask(atoi(((string)scriptInstructionParts[1]).c_str())));
            }
            else
            {
                routeTaskRequest(scriptInstructions[i]);
            }
        }
    }
}

void TaskManager::clearQueue()
{
    std::lock_guard<std::mutex> guard(tasks_mutex);
    executingTasksMap.clear();
    queuedTasksMap.clear();
}

bool TaskManager::sendScriptToRemoteTaskManager(string script, string ipAddress)
{
    TCPStream* stream = getTcpInterface()->getCommanfInterfaceConnector()->connect(ipAddress.c_str(), getTcpInterface()->getPort());
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
        for (int i = 2; i < static_cast<int>(taskMessageParts.size()); i++)
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
    }
    return 0;
}

void TaskManager::tasksQueueWorker()
{
    while (!queueStopped)
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
