#include "valter.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qnamespace.h>

class ArmControlRightTCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    ArmControlRightTCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

    void* run()
    {
        // Remove 1 item at a time and process it. Blocks if no items are
        // available to process.
        for (int i = 0;; i++)
        {
            //qDebug("thread %lu, loop %d - waiting for item...", (long unsigned int)self(), i);
            WorkItem* item = m_queue.remove();
            //qDebug("thread %lu, loop %d - got one item", (long unsigned int)self(), i);
            TCPStream* stream = item->getStream();

            // Echo messages back the client until the connection is
            // closed
            char input[256];
            for (int i = 0; i < 255; i++)
            {
                input[i] = '\0';
            }

            string output;
            int len;

            while ((len = stream->receive(input, sizeof(input)-1)) > 0 )
            {
                output = "OK";
                stream->send(output.c_str(), (sizeof(output.c_str())-1));
                //qDebug("thread %lu, echoed '%s' back to the client", (long unsigned int)self(), input);
            }
            delete item;

            std::string cmd(input);

            executeCommand(cmd);
        }

        // Should never get here
        return NULL;
    }

    void executeCommand(string cmd)
    {
        ArmControlRight *armControlRight = ArmControlRight::getInstance();

        if (cmd.find("CDR~") != std::string::npos)
        {
            int cdr_pos = cmd.find("~") + 1;
            string cdr = cmd.substr(cdr_pos);

            qDebug("TCP[%s] → %s", armControlRight->getControlDeviceId().c_str(), cdr.c_str());

            armControlRight->processControlDeviceResponse(cdr);
        }

        //control/service messages
        if (cmd.find("setCentralCommandHostInfo") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            armControlRight->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            armControlRight->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            qDebug("Central Command Host IP Address:%s Port:%d", armControlRight->getTcpInterface()->getCentralCommandHostIP().c_str(), armControlRight->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }

        //GUI elements events handler

        MainWindow *mainWindow = MainWindow::getInstance();
        Ui::MainWindow* ui = mainWindow->getUi();

        if (cmd.find("on_armControlRightLoadDefaultsButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightLoadDefaultsButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightRedrawGUICheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->armControlRightRedrawGUICheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_armControlRightStartAllWatchersButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightStartAllWatchersButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightStopAllWatchersButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightStopAllWatchersButton_clicked();
            return;
        }
    }
};
