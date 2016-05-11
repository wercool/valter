#include "valter.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

class PlatformControlP1TCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    PlatformControlP1TCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

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
        PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();

        if (cmd.find("CDR~") != std::string::npos)
        {
            int cdr_pos = cmd.find("~") + 1;
            string cdr = cmd.substr(cdr_pos);

            qDebug("TCP[%s] → %s", platformControlP1->getControlDeviceId().c_str(), cdr.c_str());

            platformControlP1->processControlDeviceResponse(cdr);
        }

        //control/service messages
        if (cmd.find("setCentralCommandHostInfo") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            platformControlP1->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            platformControlP1->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            qDebug("Central Command Host IP Address:%s Port:%d", platformControlP1->getTcpInterface()->getCentralCommandHostIP().c_str(), platformControlP1->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }

        //GUI elements events handler

        MainWindow *mainWindow = MainWindow::getInstance();
        Ui::MainWindow* ui = mainWindow->getUi();

        if (cmd.find("on_platformControlP1LoadDefaultsButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP1LoadDefaultsButton_clicked();
            return;
        }
        if (cmd.find("on_platformMoveForwardButton_pressed") != std::string::npos)
        {
            mainWindow->on_platformMoveForwardButton_pressed();
            return;
        }
        if (cmd.find("on_platformMoveForwardButton_released") != std::string::npos)
        {
            mainWindow->on_platformMoveForwardButton_released();
            return;
        }
        if (cmd.find("on_platformBackwardForwardButton_pressed") != std::string::npos)
        {
            mainWindow->on_platformBackwardForwardButton_pressed();
            return;
        }
        if (cmd.find("on_platformBackwardForwardButton_released") != std::string::npos)
        {
            mainWindow->on_platformBackwardForwardButton_released();
            return;
        }
        if (cmd.find("on_platformRotateLeftButton_pressed") != std::string::npos)
        {
            mainWindow->on_platformRotateLeftButton_pressed();
            return;
        }
        if (cmd.find("on_platformRotateLeftButton_released") != std::string::npos)
        {
            mainWindow->on_platformRotateLeftButton_released();
            return;
        }
        if (cmd.find("on_platformRotateRightButton_pressed") != std::string::npos)
        {
            mainWindow->on_platformRotateRightButton_pressed();
            return;
        }
        if (cmd.find("on_platformRotateRightButton_released") != std::string::npos)
        {
            mainWindow->on_platformRotateRightButton_released();
            return;
        }
        if (cmd.find("on_platformMoveForwardRightButton_pressed") != std::string::npos)
        {
            mainWindow->on_platformMoveForwardRightButton_pressed();
            return;
        }
        if (cmd.find("on_platformMoveForwardRightButton_released") != std::string::npos)
        {
            mainWindow->on_platformMoveForwardRightButton_released();
            return;
        }
        if (cmd.find("on_platformMoveForwardLeftButton_pressed") != std::string::npos)
        {
            mainWindow->on_platformMoveForwardLeftButton_pressed();
            return;
        }
        if (cmd.find("on_platformMoveForwardLeftButton_released") != std::string::npos)
        {
            mainWindow->on_platformMoveForwardLeftButton_released();
            return;
        }
        if (cmd.find("on_platformMoveBackwardRightButton_pressed") != std::string::npos)
        {
            mainWindow->on_platformMoveBackwardRightButton_pressed();
            return;
        }
        if (cmd.find("on_platformMoveBackwardRightButton_released") != std::string::npos)
        {
            mainWindow->on_platformMoveBackwardRightButton_released();
            return;
        }
        if (cmd.find("on_platformMoveBackwardLeftButton_pressed") != std::string::npos)
        {
            mainWindow->on_platformMoveBackwardLeftButton_pressed();
            return;
        }
        if (cmd.find("on_platformMoveBackwardLeftButton_released") != std::string::npos)
        {
            mainWindow->on_platformMoveBackwardLeftButton_released();
            return;
        }
        if (cmd.find("on_platformControlP1WheelMotorsDutySlider_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->platformControlP1WheelMotorsDutySlider->setValue(value);
            return;
        }
        if (cmd.find("on_platformMovementDecelerationSlider_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->platformMovementDecelerationSlider->setValue(value);
            return;
        }
        if (cmd.find("on_platformMovementAccelerationSlider_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->platformMovementAccelerationSlider->setValue(value);
            return;
        }
        if (cmd.find("on_leftMotorPlatformControlP1DutySlider_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftMotorPlatformControlP1DutySlider->setValue(value);
            return;
        }
        if (cmd.find("on_rightMotorPlatformControlP1DutySlider_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightMotorPlatformControlP1DutySlider->setValue(value);
            return;
        }
        if (cmd.find("on_turretRotationDutySlider_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->turretRotationDutySlider->setValue(value);
            return;
        }
    }
};
