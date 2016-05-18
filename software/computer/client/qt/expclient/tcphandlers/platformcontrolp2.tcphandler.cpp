#include "valter.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qnamespace.h>

class PlatformControlP2TCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    PlatformControlP2TCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

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
        PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();

        if (cmd.find("CDR~") != std::string::npos)
        {
            int cdr_pos = cmd.find("~") + 1;
            string cdr = cmd.substr(cdr_pos);

            qDebug("TCP[%s] → %s", platformControlP2->getControlDeviceId().c_str(), cdr.c_str());

            platformControlP2->processControlDeviceResponse(cdr);
        }

        //control/service messages
        if (cmd.find("setCentralCommandHostInfo") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            platformControlP2->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            platformControlP2->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            platformControlP2->getTcpInterface()->setConnected(true);
            qDebug("[%s] Central Command Host IP Address:%s Port:%d", platformControlP2->getControlDeviceId().c_str(), platformControlP2->getTcpInterface()->getCentralCommandHostIP().c_str(), platformControlP2->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }
        if (cmd.find("stopCDTtoCentralCommandHost") != std::string::npos)
        {
            platformControlP2->getTcpInterface()->setConnected(false);
            return;
        }

        //GUI elements events handler

        MainWindow *mainWindow = MainWindow::getInstance();
        Ui::MainWindow* ui = mainWindow->getUi();

        if (cmd.find("on_loadDefaultsPlatfromControlP2Button_clicked") != std::string::npos)
        {
            mainWindow->on_loadDefaultsPlatfromControlP2Button_clicked();
            return;
        }
        if (cmd.find("on_platfromControlP2RedrawGUICheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platfromControlP2RedrawGUICheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP2LeftWheelEncoderCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformControlP2LeftWheelEncoderCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP2RightWheelEncoderCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformControlP2RightWheelEncoderCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP2LeftWheelEncoderAutoresetCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformControlP2LeftWheelEncoderAutoresetCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP2RightWheelEncoderAutoresetCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformControlP2RightWheelEncoderAutoresetCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP2LeftWheelEncoderGetButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP2LeftWheelEncoderGetButton_clicked();
            return;
        }
        if (cmd.find("on_platformControlP2RightWheelEncoderGetButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP2RightWheelEncoderGetButton_clicked();
            return;
        }
        if (cmd.find("on_platformControlP2LeftWheelEncoderResetButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP2LeftWheelEncoderResetButton_clicked();
            return;
        }
        if (cmd.find("on_platformControlP2RightWheelEncoderResetButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP2RightWheelEncoderResetButton_clicked();
            return;
        }
        if (cmd.find("on_chargerMotorDutyScrollBar_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->chargerMotorDutyScrollBar->setValue(value);
            return;
        }
        if (cmd.find("on_chargerMotorPushDurationScrollBar_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->chargerMotorPushDurationScrollBar->setValue(value);
            return;
        }
        if (cmd.find("on_chargerMotorRotateCCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_chargerMotorRotateCCWButton_pressed();
            return;
        }
        if (cmd.find("on_chargerMotorRotateCCWButton_released") != std::string::npos)
        {
            mainWindow->on_chargerMotorRotateCCWButton_released();
            return;
        }
        if (cmd.find("on_chargerMotorRotateCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_chargerMotorRotateCWButton_pressed();
            return;
        }
        if (cmd.find("on_chargerMotorRotateCWButton_released") != std::string::npos)
        {
            mainWindow->on_chargerMotorRotateCWButton_released();
            return;
        }
        if (cmd.find("on_chargerMotorPushCCWButton_clicked") != std::string::npos)
        {
            mainWindow->on_chargerMotorPushCCWButton_clicked();
            return;
        }
        if (cmd.find("on_chargerMotorPushCWButton_clicked") != std::string::npos)
        {
            mainWindow->on_chargerMotorPushCWButton_clicked();
            return;
        }
        if (cmd.find("on_platfromControlP2IRScannerADCCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platfromControlP2IRScannerADCCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP2GetBottomIRReadingButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP2GetBottomIRReadingButton_clicked();
            return;
        }
        if (cmd.find("on_irScanningButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->irScanningButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_irScannerAngleScrollBar_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->irScannerAngleScrollBar->setValue(value);
            return;
        }
        if (cmd.find("on_platformControlP2ResetIRScannerServoButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP2ResetIRScannerServoButton_clicked();
            return;
        }
        if (cmd.find("on_beepDurationScrollBar_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->beepDurationScrollBar->setValue(value);
            return;
        }
        if (cmd.find("on_beepButton_clicked") != std::string::npos)
        {
            mainWindow->on_beepButton_clicked();
            return;
        }
        if (cmd.find("on_alarmButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->alarmButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_chargerLedsButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->chargerLedsButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_bottomFronLedsButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->bottomFronLedsButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_bottomRearLedsButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->bottomRearLedsButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
    }
};
