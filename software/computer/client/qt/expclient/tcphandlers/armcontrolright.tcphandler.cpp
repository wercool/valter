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
        ArmControlRight *armControlRight = ArmControlRight::getInstance();

        if (cmd.find("CDR~") != std::string::npos)
        {
            int cdr_pos = cmd.find("~") + 1;
            string cdr = cmd.substr(cdr_pos);

            qDebug("TCP[%s] → %s", armControlRight->getControlDeviceId().c_str(), cdr.c_str());

            armControlRight->processControlDeviceResponse(cdr);
            return;
        }

        //control/service messages
        if (cmd.find("setCentralCommandHostInfo") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            armControlRight->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            armControlRight->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            armControlRight->getTcpInterface()->setConnected(true);
            armControlRight->getTcpInterface()->sendCDRToCentralCommandHost(Valter::format_string("CDR~REMOTECD:%s:%s:%d:%s", armControlRight->getControlDeviceId().c_str(), armControlRight->getTcpInterface()->getLocalHostIP().c_str(), armControlRight->getTcpInterface()->getPort(), armControlRight->getControlDevice()->getStatus().c_str()));
            qDebug("[%s] Central Command Host IP Address:%s Port:%d", armControlRight->getControlDeviceId().c_str(), armControlRight->getTcpInterface()->getCentralCommandHostIP().c_str(), armControlRight->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }
        if (cmd.find("stopCDTtoCentralCommandHost") != std::string::npos)
        {
            armControlRight->getTcpInterface()->setConnected(false);
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
//        if (cmd.find("on_armControlRightRedrawGUICheckBox_clicked") != std::string::npos)
//        {
//            int substr_pos = cmd.find("@") + 1;
//            string value_str = cmd.substr(substr_pos);
//            ui->armControlRightRedrawGUICheckBox->setChecked((value_str.compare("true")) ? true : false);
//            return;
//        }
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
        if (cmd.find("on_rightForearmMoveUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightForearmMoveUpButton_pressed();
            return;
        }
        if (cmd.find("on_rightForearmMoveUpButton_released") != std::string::npos)
        {
            mainWindow->on_rightForearmMoveUpButton_released();
            return;
        }
        if (cmd.find("on_rightForearmMoveDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightForearmMoveDownButton_pressed();
            return;
        }
        if (cmd.find("on_rightForearmMoveDownButton_released") != std::string::npos)
        {
            mainWindow->on_rightForearmMoveDownButton_released();
            return;
        }
        if (cmd.find("on_rightForearmMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightForearmMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightForearmMotorDecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightForearmMotorDecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightForearmAccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightForearmAccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightArmMoveUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightArmMoveUpButton_pressed();
            return;
        }
        if (cmd.find("on_rightArmMoveUpButton_released") != std::string::npos)
        {
            mainWindow->on_rightArmMoveUpButton_released();
            return;
        }
        if (cmd.find("on_rightArmMoveDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightArmMoveDownButton_pressed();
            return;
        }
        if (cmd.find("on_rightArmMoveDownButton_released") != std::string::npos)
        {
            mainWindow->on_rightArmMoveDownButton_released();
            return;
        }
        if (cmd.find("on_armControlRightArmMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlRightArmMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlRightArmMotorDecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlRightArmMotorDecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlRightMotorAccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlRightMotorAccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightLimbMoveUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightLimbMoveUpButton_pressed();
            return;
        }
        if (cmd.find("on_rightLimbMoveUpButton_released") != std::string::npos)
        {
            mainWindow->on_rightLimbMoveUpButton_released();
            return;
        }
        if (cmd.find("on_rightLimbMoveDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightLimbMoveDownButton_pressed();
            return;
        }
        if (cmd.find("on_rightLimbMoveDownButton_released") != std::string::npos)
        {
            mainWindow->on_rightLimbMoveDownButton_released();
            return;
        }
        if (cmd.find("on_rightLimbMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightLimbMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightLimbMotorDecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightLimbMotorDecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightLimbMotorAccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightLimbMotorAccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightHandYawCCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightHandYawCCWButton_pressed();
            return;
        }
        if (cmd.find("on_rightHandYawCCWButton_released") != std::string::npos)
        {
            mainWindow->on_rightHandYawCCWButton_released();
            return;
        }
        if (cmd.find("on_rightHandYawCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightHandYawCWButton_pressed();
            return;
        }
        if (cmd.find("on_rightHandYawCWButton_released") != std::string::npos)
        {
            mainWindow->on_rightHandYawCWButton_released();
            return;
        }
        if (cmd.find("on_rightHandPitchUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightHandPitchUpButton_pressed();
            return;
        }
        if (cmd.find("on_rightHandPitchUpButton_released") != std::string::npos)
        {
            mainWindow->on_rightHandPitchUpButton_released();
            return;
        }
        if (cmd.find("on_rightHandPitchDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightHandPitchDownButton_pressed();
            return;
        }
        if (cmd.find("on_rightHandPitchDownButton_released") != std::string::npos)
        {
            mainWindow->on_rightHandPitchDownButton_released();
            return;
        }
        if (cmd.find("on_rightForearmRollMotorOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightForearmRollMotorOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_rightForearmRollCCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightForearmRollCCWButton_pressed();
            return;
        }
        if (cmd.find("on_rightForearmRollCCWButton_released") != std::string::npos)
        {
            mainWindow->on_rightForearmRollCCWButton_released();
            return;
        }
        if (cmd.find("on_rightForearmRollCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightForearmRollCWButton_pressed();
            return;
        }
        if (cmd.find("on_rightForearmRollCWButton_released") != std::string::npos)
        {
            mainWindow->on_rightForearmRollCWButton_released();
            return;
        }
        if (cmd.find("on_rightForearmRollStepDelaySpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightForearmRollStepDelaySpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_rightForearmRollStepSwitchDelaySpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightForearmRollStepSwitchDelaySpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_rightForearmResetPositionButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightForearmResetPositionButton_clicked();
            return;
        }
        if (cmd.find("on_rightForearmYawCCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightForearmYawCCWButton_pressed();
            return;
        }
        if (cmd.find("on_rightForearmYawCCWButton_released") != std::string::npos)
        {
            mainWindow->on_rightForearmYawCCWButton_released();
            return;
        }
        if (cmd.find("on_rightForearmYawCWButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightForearmYawCWButton_pressed();
            return;
        }
        if (cmd.find("on_rightForearmYawCWButton_released") != std::string::npos)
        {
            mainWindow->on_rightForearmYawCWButton_released();
            return;
        }
        if (cmd.find("on_rightArmLedsOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightArmLedsOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_rightHandSensorsTable_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->rightHandSensorsTable->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->rightHandSensorsTable->viewport()->update();
            mainWindow->on_rightHandSensorsTable_itemClicked(item);
            return;
        }
        if (cmd.find("on_rightArmReadingsTable_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->rightArmReadingsTable->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->rightArmReadingsTable->viewport()->update();
            mainWindow->on_rightArmReadingsTable_itemClicked(item);
            return;
        }
        if (cmd.find("on_rightHandSensorsTrackAllButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightHandSensorsTrackAllButton_clicked();
            return;
        }
        if (cmd.find("on_rightHandSensorsTrackNoneButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightHandSensorsTrackNoneButton_clicked();
            return;
        }
        if (cmd.find("on_rightArmReadingsTrackAllButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightArmReadingsTrackAllButton_clicked();
            return;
        }
        if (cmd.find("on_rightArmReadingsTrackNoneButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightArmReadingsTrackNoneButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightFinger0ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightFinger0ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightFinger1ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightFinger1ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightFinger2ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightFinger2ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightFinger3ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightFinger3ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightFinger4ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightFinger4ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightFinger5ReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightFinger5ReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightPalmReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightPalmReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightPalmActivatedButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightPalmActivatedButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightPalmGraspButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightPalmGraspButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightPalmSqueezeButton_clicked") != std::string::npos)
        {
            mainWindow->on_armControlRightPalmSqueezeButton_clicked();
            return;
        }
        if (cmd.find("on_armControlRightFinger0PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlRightFinger0PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlRightFinger1PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlRightFinger1PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlRightFinger2PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlRightFinger2PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlRightFinger3PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlRightFinger3PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlRightFinger4PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlRightFinger4PositionScoller->setValue(value);
            return;
        }
        if (cmd.find("on_armControlRightFinger5PositionScoller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->armControlRightFinger5PositionScoller->setValue(value);
            return;
        }
    }
};
