#include "valter.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qnamespace.h>

class ArmControlLeftTCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    ArmControlLeftTCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

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
                stream->send(output.c_str(), output.length());
                //qDebug("thread %lu, sent '%s' back to the client", (long unsigned int)self(), output.c_str());

                std::string cmd(input);
                executeCommand(cmd);
            }
            delete item;
        }

        // Should never get here
        return NULL;
    }

    void executeCommand(string cmd)
    {
        ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();

        if (cmd.find("CDR~") != std::string::npos)
        {
            int cdr_pos = cmd.find("~") + 1;
            string cdr = cmd.substr(cdr_pos);

            qDebug("TCP[%s] → %s", armControlLeft->getControlDeviceId().c_str(), cdr.c_str());

            armControlLeft->processControlDeviceResponse(cdr);
            return;
        }

        //control/service messages
        if (cmd.find("setCentralCommandHostInfo") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            armControlLeft->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            armControlLeft->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            armControlLeft->getTcpInterface()->setConnected(true);
            armControlLeft->getTcpInterface()->sendCDRToCentralCommandHost(Valter::format_string("CDR~REMOTECD:%s:%s:%d:%s", armControlLeft->getControlDeviceId().c_str(), armControlLeft->getTcpInterface()->getLocalHostIP().c_str(), armControlLeft->getTcpInterface()->getPort(), armControlLeft->getControlDevice()->getStatus().c_str()));
            qDebug("[%s] Central Command Host IP Address:%s Port:%d", armControlLeft->getControlDeviceId().c_str(), armControlLeft->getTcpInterface()->getCentralCommandHostIP().c_str(), armControlLeft->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }
        if (cmd.find("stopCDTtoCentralCommandHost") != std::string::npos)
        {
            armControlLeft->getTcpInterface()->setConnected(false);
            return;
        }

        //GUI elements events handler

        MainWindow *mainWindow = MainWindow::getInstance();
        Ui::MainWindow* ui = mainWindow->getUi();

        if (cmd.find("on_armControlLeftLoadDefaultsButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftLoadDefaultsButton_clicked();
            return;
        }
//        if (cmd.find("on_armControlLeftRedrawGUICheckBox_clicked") != std::string::npos)
//        {
//            int substr_pos = cmd.find("@") + 1;
//            string value_str = cmd.substr(substr_pos);
//            ui->armControlLeftRedrawGUICheckBox->setChecked((value_str.compare("true")) ? true : false);
//            return;
//        }
        if (cmd.find("on_armControlLeftStopAllWatchersButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftStopAllWatchersButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftStartAllWatchersButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftStartAllWatchersButton_clicked();
            return;
        }
        if (cmd.find("on_leftForearmMoveUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftForearmMoveUpButton_pressed();
            return;
        }
        if (cmd.find("on_leftForearmMoveUpButton_released") != std::string::npos)
        {
            mainWindow->on_leftForearmMoveUpButton_released();
            return;
        }
        if (cmd.find("on_leftForearmMoveDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftForearmMoveDownButton_pressed();
            return;
        }
        if (cmd.find("on_leftForearmMoveDownButton_released") != std::string::npos)
        {
            mainWindow->on_leftForearmMoveDownButton_released();
            return;
        }
        if (cmd.find("on_leftForearmMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftForearmMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftForearmMotorDecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftForearmMotorDecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftForearmAccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftForearmAccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftArmMoveUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftArmMoveUpButton_pressed();
            return;
        }
        if (cmd.find("on_leftArmMoveUpButton_released") != std::string::npos)
        {
            mainWindow->on_leftArmMoveUpButton_released();
            return;
        }
        if (cmd.find("on_leftArmMoveDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftArmMoveDownButton_pressed();
            return;
        }
        if (cmd.find("on_leftArmMoveDownButton_released") != std::string::npos)
        {
            mainWindow->on_leftArmMoveDownButton_released();
            return;
        }
        if (cmd.find("on_armControlLeftArmMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlLeftArmMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlLeftArmMotorDecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlLeftArmMotorDecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlLeftArmMotorAccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlLeftArmMotorAccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftLimbMoveUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftLimbMoveUpButton_pressed();
            return;
        }
        if (cmd.find("on_leftLimbMoveUpButton_released") != std::string::npos)
        {
            mainWindow->on_leftLimbMoveUpButton_released();
            return;
        }
        if (cmd.find("on_leftLimbMoveDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftLimbMoveDownButton_pressed();
            return;
        }
        if (cmd.find("on_leftLimbMoveDownButton_released") != std::string::npos)
        {
            mainWindow->on_leftLimbMoveDownButton_released();
            return;
        }
        if (cmd.find("on_leftLimbMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftLimbMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftLimbMotorDecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftLimbMotorDecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftLimbMotorAccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftLimbMotorAccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftHandYawCCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftHandYawCCWButton_pressed();
            return;
        }
        if (cmd.find("on_leftHandYawCCWButton_released") != std::string::npos)
        {
            mainWindow->on_leftHandYawCCWButton_released();
            return;
        }
        if (cmd.find("on_leftHandYawCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftHandYawCWButton_pressed();
            return;
        }
        if (cmd.find("on_leftHandYawCWButton_released") != std::string::npos)
        {
            mainWindow->on_leftHandYawCWButton_released();
            return;
        }
        if (cmd.find("on_leftHandPitchUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftHandPitchUpButton_pressed();
            return;
        }
        if (cmd.find("on_leftHandPitchUpButton_released") != std::string::npos)
        {
            mainWindow->on_leftHandPitchUpButton_released();
            return;
        }
        if (cmd.find("on_leftHandPitchDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftHandPitchDownButton_pressed();
            return;
        }
        if (cmd.find("on_leftHandPitchDownButton_released") != std::string::npos)
        {
            mainWindow->on_leftHandPitchDownButton_released();
            return;
        }
        if (cmd.find("on_leftForearmRollMotorOnButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftForearmRollMotorOnButton_clicked();
            return;
        }
        if (cmd.find("on_leftForearmRollMotorOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftForearmRollMotorOffButton_clicked();
            return;
        }
        if (cmd.find("on_leftForearmRollCCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftForearmRollCCWButton_pressed();
            return;
        }
        if (cmd.find("on_leftForearmRollCCWButton_released") != std::string::npos)
        {
            mainWindow->on_leftForearmRollCCWButton_released();
            return;
        }
        if (cmd.find("on_leftForearmRollCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftForearmRollCWButton_pressed();
            return;
        }
        if (cmd.find("on_leftForearmRollCWButton_released") != std::string::npos)
        {
            mainWindow->on_leftForearmRollCWButton_released();
            return;
        }
        if (cmd.find("on_leftForearmRollStepDelaySpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftForearmRollStepDelaySpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_leftForearmRollStepSwitchDelaySpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftForearmRollStepSwitchDelaySpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_leftForearmResetPositionButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftForearmResetPositionButton_clicked();
            return;
        }
        if (cmd.find("on_leftForearmYawCCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftForearmYawCCWButton_pressed();
            return;
        }
        if (cmd.find("on_leftForearmYawCCWButton_released") != std::string::npos)
        {
            mainWindow->on_leftForearmYawCCWButton_released();
            return;
        }
        if (cmd.find("on_leftForearmYawCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftForearmYawCWButton_pressed();
            return;
        }
        if (cmd.find("on_leftForearmYawCWButton_released") != std::string::npos)
        {
            mainWindow->on_leftForearmYawCWButton_released();
            return;
        }
        if (cmd.find("on_leftArmLedsOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftArmLedsOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_leftHandSensorsTable_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->leftHandSensorsTable->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->leftHandSensorsTable->viewport()->update();
            mainWindow->on_leftHandSensorsTable_itemClicked(item);
            return;
        }
        if (cmd.find("on_leftArmReadingsTable_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->leftArmReadingsTable->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->leftArmReadingsTable->viewport()->update();
            mainWindow->on_leftArmReadingsTable_itemClicked(item);
            return;
        }
        if (cmd.find("on_leftHandSensorsTrackAllButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftHandSensorsTrackAllButton_clicked();
            return;
        }
        if (cmd.find("on_leftHandSensorsTrackNoneButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftHandSensorsTrackNoneButton_clicked();
            return;
        }
        if (cmd.find("on_leftArmReadingsTrackAllButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftArmReadingsTrackAllButton_clicked();
            return;
        }
        if (cmd.find("on_leftArmReadingsTrackNoneButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftArmReadingsTrackNoneButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftPalmReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftPalmReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftPalmActivatedButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftPalmActivatedButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftPalmGraspButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftPalmGraspButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftPalmSqueezeButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftPalmSqueezeButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftFinger6PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlLeftFinger6PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlLeftFinger7PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlLeftFinger7PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlLeftFinger8PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlLeftFinger8PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlLeftFinger9PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlLeftFinger9PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlLeftFinger10PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlLeftFinger10PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlLeftFinger11PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlLeftFinger11PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlLeftFinger6ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftFinger6ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftFinger7ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftFinger7ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftFinger8ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftFinger8ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftFinger9ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftFinger9ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftFinger10ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftFinger10ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlLeftFinger11ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlLeftFinger11ReleaseButton_clicked();
            return;
        }
    }
};
