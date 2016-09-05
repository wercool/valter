#include "valter.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qnamespace.h>

class PlatformControlP1TCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    PlatformControlP1TCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

    void* run()
    {
        // Remove 1 item at a time and process it. Blocks if no items are
        // available to process.
        PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
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

            bool isGVR = false;

            while ((len = stream->receive(input, sizeof(input)-1)) > 0 )
            {
                std::string request(input);
                qDebug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s", request.c_str());
                if (request.find("GVR~") == 0)
                {
                    isGVR = true;
                    int gvr_pos = request.find("~") + 1;
                    string gvr = request.substr(gvr_pos);
                    output = platformControlP1->getValue(gvr);
                }
                else
                {
                    isGVR = false;
                    output = "OK";
                }
                stream->send(output.c_str(), (sizeof(output.c_str())-1));
                //qDebug("thread %lu, echoed '%s' back to the client", (long unsigned int)self(), input);
            }
            delete item;

            if (!isGVR)
            {
                std::string cmd(input);
                qDebug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %s", cmd.c_str());
                executeCommand(cmd);
            }
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
            return;
        }

        //control/service messages
        if (cmd.find("setCentralCommandHostInfo") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            platformControlP1->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            platformControlP1->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            platformControlP1->getTcpInterface()->setConnected(true);
            platformControlP1->getTcpInterface()->sendCDRToCentralCommandHost(Valter::format_string("CDR~REMOTECD:%s:%s:%d:%s", platformControlP1->getControlDeviceId().c_str(), platformControlP1->getTcpInterface()->getLocalHostIP().c_str(), platformControlP1->getTcpInterface()->getPort(), platformControlP1->getControlDevice()->getStatus().c_str()));
            qDebug("[%s] Central Command Host IP Address:%s Port:%d", platformControlP1->getControlDeviceId().c_str(), platformControlP1->getTcpInterface()->getCentralCommandHostIP().c_str(), platformControlP1->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }
        if (cmd.find("stopCDTtoCentralCommandHost") != std::string::npos)
        {
            platformControlP1->getTcpInterface()->setConnected(false);
            return;
        }

        //GUI elements events handler

        MainWindow *mainWindow = MainWindow::getInstance();
        Ui::MainWindow* ui = mainWindow->getUi();

        if (cmd.find("on_stopAllButton_clicked") != std::string::npos)
        {
            mainWindow->on_stopAllButton_clicked();
            return;
        }

        if (cmd.find("on_platformControlP1LoadDefaultsButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP1LoadDefaultsButton_clicked();
            return;
        }
//        if (cmd.find("on_platformControlP1RedrawGUICheckBox_clicked") != std::string::npos)
//        {
//            int substr_pos = cmd.find("@") + 1;
//            string value_str = cmd.substr(substr_pos);
//            ui->platformControlP1RedrawGUICheckBox->setChecked((value_str.compare("true")) ? true : false);
//            return;
//        }
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
        if (cmd.find("on_platformMoveStopButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformMoveStopButton_clicked();
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
        if (cmd.find("on_decelerationTurretRotationSlider_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->decelerationTurretRotationSlider->setValue(value);
            return;
        }
        if (cmd.find("on_accelerationTurretRotationSlider_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->accelerationTurretRotationSlider->setValue(value);
            return;
        }
        if (cmd.find("on_platformControlP1LeftWheelEncoderGetButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP1LeftWheelEncoderGetButton_clicked();
            return;
        }
        if (cmd.find("on_platformControlP1RightWheelEncoderGetButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP1RightWheelEncoderGetButton_clicked();
            return;
        }
        if (cmd.find("on_platformControlP1LeftWheelEncoderResetButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP1LeftWheelEncoderResetButton_clicked();
            return;
        }
        if (cmd.find("on_platformControlP1RightWheelEncoderResetButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP1RightWheelEncoderResetButton_clicked();
            return;
        }
        if (cmd.find("on_platformControlP1MotorsPWMFrequencySetButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP1MotorsPWMFrequencySetButton_clicked();
            return;
        }
        if (cmd.find("on_turretRotateLeftButton_pressed") != std::string::npos)
        {
            mainWindow->on_turretRotateLeftButton_pressed();
            return;
        }
        if (cmd.find("on_turretRotateLeftButton_released") != std::string::npos)
        {
            mainWindow->on_turretRotateLeftButton_released();
            return;
        }
        if (cmd.find("on_turretRotateRightButton_pressed") != std::string::npos)
        {
            mainWindow->on_turretRotateRightButton_pressed();
            return;
        }
        if (cmd.find("on_turretRotateRightButton_released") != std::string::npos)
        {
            mainWindow->on_turretRotateRightButton_released();
            return;
        }
        if (cmd.find("on_bodyRotationStopButton_clicked") != std::string::npos)
        {
            mainWindow->on_bodyRotationStopButton_clicked();
            return;
        }
        if (cmd.find("on_wheelMotorsCurrentADCCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->wheelMotorsCurrentADCCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_leftMotorCurrentCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->leftMotorCurrentCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_rightMotorCurrentCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->rightMotorCurrentCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP1MotorsPWMFrequencySpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->platformControlP1MotorsPWMFrequencySpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_getBodyRotationPositionButton_clicked") != std::string::npos)
        {
            mainWindow->on_getBodyRotationPositionButton_clicked();
            return;
        }
        if (cmd.find("on_platformControlP1LeftWheelEncoderCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformControlP1LeftWheelEncoderCheckBox->setChecked((value_str.compare("true")) ? true : false);
            platformControlP1->setLeftWheelEncoderRead((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP1RightWheelEncoderCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformControlP1RightWheelEncoderCheckBox->setChecked((value_str.compare("true")) ? true : false);
            platformControlP1->setRightWheelEncoderRead((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP1LeftWheelEncoderAutoresetCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformControlP1LeftWheelEncoderAutoresetCheckBox->setChecked((value_str.compare("true")) ? true : false);
            mainWindow->on_platformControlP1LeftWheelEncoderAutoresetCheckBox_clicked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformControlP1RightWheelEncoderAutoresetCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformControlP1RightWheelEncoderAutoresetCheckBox->setChecked((value_str.compare("true")) ? true : false);
            mainWindow->on_platformControlP1RightWheelEncoderAutoresetCheckBox_clicked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_turretMotorCurrentCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->turretMotorCurrentCheckBox->setChecked((value_str.compare("true")) ? true : false);
            mainWindow->on_turretMotorCurrentCheckBox_clicked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_turretMotorCurrentADCCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->turretMotorCurrentADCCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_turretPositionReadCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->turretPositionReadCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_on5VPlatformControlP1pushButton_clicked") != std::string::npos)
        {
            mainWindow->on_on5VPlatformControlP1pushButton_clicked();
            return;
        }
        if (cmd.find("on_off5VPlatformControlP1pushButton_clicked") != std::string::npos)
        {
            mainWindow->on_off5VPlatformControlP1pushButton_clicked();
            return;
        }
        if (cmd.find("on_onLeftAccumulatorPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_onLeftAccumulatorPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_offLeftAccumulatorPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_offLeftAccumulatorPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_onRightAccumulatorPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_onRightAccumulatorPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_offRightAccumulatorPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_offRightAccumulatorPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_scan220VAOnCButton_clicked") != std::string::npos)
        {
            mainWindow->on_scan220VAOnCButton_clicked();
            return;
        }
        if (cmd.find("on_scan220VAOffCButton_clicked") != std::string::npos)
        {
            mainWindow->on_scan220VAOffCButton_clicked();
            return;
        }
        if (cmd.find("on_chargerVoltageADCCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->chargerVoltageADCCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_chargerButton_clicked") != std::string::npos)
        {
            mainWindow->on_chargerButton_clicked();
            return;
        }
        if (cmd.find("on_setChargeOnButton_clicked") != std::string::npos)
        {
            mainWindow->on_setChargeOnButton_clicked();
            return;
        }
        if (cmd.find("on_setChargeOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_setChargeOffButton_clicked();
            return;
        }
        if (cmd.find("on_onMainAccumulatorRelayPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_onMainAccumulatorRelayPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_offMainAccumulatorRelayPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_offMainAccumulatorRelayPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_onLeftAccumulatorRelayPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_onLeftAccumulatorRelayPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_offLeftAccumulatorRelayPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_offLeftAccumulatorRelayPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_onRightAccumulatorRelayPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_onRightAccumulatorRelayPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_offRightAccumulatorRelayPlatformControlP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_offRightAccumulatorRelayPlatformControlP1Button_clicked();
            return;
        }
        if (cmd.find("on_platformControlP1additionalReadingsTrackingDelay_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->platformControlP1additionalReadingsTrackingDelay->setValue(value);
            return;
        }
        if (cmd.find("on_platformControlP1ReadingsTable_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->platformControlP1ReadingsTable->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->platformControlP1ReadingsTable->viewport()->update();
            mainWindow->on_platformControlP1ReadingsTable_itemClicked(item);
            return;
        }
        if (cmd.find("on_platformControlP1UntrackAllAdditionalReadingsButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformControlP1UntrackAllAdditionalReadingsButton_clicked();
            return;
        }
    }
};
