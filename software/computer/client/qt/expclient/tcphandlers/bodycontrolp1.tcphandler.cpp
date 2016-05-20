#include "valter.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qnamespace.h>

class BodyControlP1TCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    BodyControlP1TCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

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
        BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();

        if (cmd.find("CDR~") != std::string::npos)
        {
            int cdr_pos = cmd.find("~") + 1;
            string cdr = cmd.substr(cdr_pos);

            qDebug("TCP[%s] → %s", bodyControlP1->getControlDeviceId().c_str(), cdr.c_str());

            bodyControlP1->processControlDeviceResponse(cdr);
        }

        //control/service messages
        if (cmd.find("setCentralCommandHostInfo") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            bodyControlP1->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            bodyControlP1->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            bodyControlP1->getTcpInterface()->setConnected(true);
            qDebug("[%s] Central Command Host IP Address:%s Port:%d", bodyControlP1->getControlDeviceId().c_str(), bodyControlP1->getTcpInterface()->getCentralCommandHostIP().c_str(), bodyControlP1->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }
        if (cmd.find("stopCDTtoCentralCommandHost") != std::string::npos)
        {
            bodyControlP1->getTcpInterface()->setConnected(false);
            return;
        }

        //GUI elements events handler

        MainWindow *mainWindow = MainWindow::getInstance();
        Ui::MainWindow* ui = mainWindow->getUi();

        if (cmd.find("on_bodyControlP1LoadDefaultsButton_clicked") != std::string::npos)
        {
            mainWindow->on_bodyControlP1LoadDefaultsButton_clicked();
            return;
        }
        if (cmd.find("on_bodyControlP1CheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->bodyControlP1CheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_bodyPitchUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_bodyPitchUpButton_pressed();
            return;
        }
        if (cmd.find("on_bodyPitchUpButton_released") != std::string::npos)
        {
            mainWindow->on_bodyPitchUpButton_released();
            return;
        }
        if (cmd.find("on_bodyPitchDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_bodyPitchDownButton_pressed();
            return;
        }
        if (cmd.find("on_bodyPitchDownButton_released") != std::string::npos)
        {
            mainWindow->on_bodyPitchDownButton_released();
            return;
        }
        if (cmd.find("on_bodyPitchMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->bodyPitchMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_bodyPitchMotorDecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->bodyPitchMotorDecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_bodyPitchMotorAccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->bodyPitchMotorAccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightArmYawOpenButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightArmYawOpenButton_pressed();
            return;
        }
        if (cmd.find("on_rightArmYawOpenButton_released") != std::string::npos)
        {
            mainWindow->on_rightArmYawOpenButton_released();
            return;
        }
        if (cmd.find("on_rightArmYawCloseButton_pressed") != std::string::npos)
        {
            mainWindow->on_rightArmYawCloseButton_pressed();
            return;
        }
        if (cmd.find("on_rightArmYawCloseButton_released") != std::string::npos)
        {
            mainWindow->on_rightArmYawCloseButton_released();
            return;
        }
        if (cmd.find("on_rightArmMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightArmMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightArmMotorDecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightArmMotorDecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightArmMotorAccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightArmMotorAccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftArmYawOpenButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftArmYawOpenButton_pressed();
            return;
        }
        if (cmd.find("on_leftArmYawOpenButton_released") != std::string::npos)
        {
            mainWindow->on_leftArmYawOpenButton_released();
            return;
        }
        if (cmd.find("on_leftArmYawCloseButton_pressed") != std::string::npos)
        {
            mainWindow->on_leftArmYawCloseButton_pressed();
            return;
        }
        if (cmd.find("on_leftArmYawCloseButton_released") != std::string::npos)
        {
            mainWindow->on_leftArmYawCloseButton_released();
            return;
        }
        if (cmd.find("on_leftArmMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftArmMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftArmMotorDecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftArmMotorDecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftArmMotorAccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftArmMotorAccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_bodyControlP1ShiftRegEnableButton_clicked") != std::string::npos)
        {
            mainWindow->on_bodyControlP1ShiftRegEnableButton_clicked();
            return;
        }
        if (cmd.find("on_bodyControlP1ShiftRegDisableButton_clicked") != std::string::npos)
        {
            mainWindow->on_bodyControlP1ShiftRegDisableButton_clicked();
            return;
        }
        if (cmd.find("on_bodyControlP1ShiftRegResetButton_clicked") != std::string::npos)
        {
            mainWindow->on_bodyControlP1ShiftRegResetButton_clicked();
            return;
        }
        if (cmd.find("on_bodyControlP1StopShiftRegResetButton_clicked") != std::string::npos)
        {
            mainWindow->on_bodyControlP1StopShiftRegResetButton_clicked();
            return;
        }
        if (cmd.find("on_leftArm12VOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftArm12VOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_rightArm12VButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightArm12VButton_clicked();
            return;
        }
        if (cmd.find("on_kinect1OnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_kinect1OnOffButton_clicked();
            return;
        }
        if (cmd.find("on_kinect2OnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_kinect2OnOffButton_clicked();
            return;
        }
        if (cmd.find("on_head24VOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_head24VOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_powerSource5V5OnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_powerSource5V5OnOffButton_clicked();
            return;
        }
        if (cmd.find("on_wifiOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_wifiOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_leftArm24VOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftArm24VOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_rightArm24VButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightArm24VButton_clicked();
            return;
        }
        if (cmd.find("on_headLedOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_headLedOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_leftAccumulatorOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftAccumulatorOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_rightAccumulatorOnOffButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightAccumulatorOnOffButton_clicked();
            return;
        }
        if (cmd.find("on_headYawMotorEnableDisableButton_clicked") != std::string::npos)
        {
            mainWindow->on_headYawMotorEnableDisableButton_clicked();
            return;
        }
        if (cmd.find("on_headYawRightRotateButton_pressed") != std::string::npos)
        {
            mainWindow->on_headYawRightRotateButton_pressed();
            return;
        }
        if (cmd.find("on_headYawRightRotateButton_released") != std::string::npos)
        {
            mainWindow->on_headYawRightRotateButton_released();
            return;
        }
        if (cmd.find("on_headYawLeftRotateButton_pressed") != std::string::npos)
        {
            mainWindow->on_headYawLeftRotateButton_pressed();
            return;
        }
        if (cmd.find("on_headYawLeftRotateButton_released") != std::string::npos)
        {
            mainWindow->on_headYawLeftRotateButton_released();
            return;
        }
        if (cmd.find("on_headYawStepDelaySpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->headYawStepDelaySpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_headYawStepSwitchDelaySpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->headYawStepSwitchDelaySpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_headYawStepDelayTestModeSpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->headYawStepDelayTestModeSpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_headYawRightDirectonCheckButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            bool checked = (value_str.compare("true")) ? true : false;
            ui->headYawRightDirectonCheckButton->setChecked(checked);
            return;
        }
        if (cmd.find("on_headYawLeftDirectonCheckButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            bool checked = (value_str.compare("true")) ? true : false;
            ui->headYawLeftDirectonCheckButton->setChecked(checked);
            return;
        }
        if (cmd.find("on_headYawMoveStepsSpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->headYawMoveStepsSpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_headYawTestModeExecuteButton_clicked") != std::string::npos)
        {
            mainWindow->on_headYawTestModeExecuteButton_clicked();
            return;
        }
        if (cmd.find("on_getHeadYawPositionButton_clicked") != std::string::npos)
        {
            mainWindow->on_getHeadYawPositionButton_clicked();
            return;
        }
        if (cmd.find("on_headPitchMotorEnableDisableButton_clicked") != std::string::npos)
        {
            mainWindow->on_headPitchMotorEnableDisableButton_clicked();
            return;
        }
        if (cmd.find("on_headPitchDownButton_pressed") != std::string::npos)
        {
            mainWindow->on_headPitchDownButton_pressed();
            return;
        }
        if (cmd.find("on_headPitchDownButton_released") != std::string::npos)
        {
            mainWindow->on_headPitchDownButton_released();
            return;
        }
        if (cmd.find("on_headPitchUpButton_pressed") != std::string::npos)
        {
            mainWindow->on_headPitchUpButton_pressed();
            return;
        }
        if (cmd.find("on_headPitchUpButton_released") != std::string::npos)
        {
            mainWindow->on_headPitchUpButton_released();
            return;
        }
        if (cmd.find("on_headPitchStepDelaySpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->headPitchStepDelaySpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_headPitchStepSwitchDelaySpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->headPitchStepSwitchDelaySpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_headPitchStepDelayTestModeSpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->headPitchStepDelayTestModeSpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_headPitchMoveStepsSpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->headPitchMoveStepsSpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_headPitchTestModeExecuteButton_clicked") != std::string::npos)
        {
            mainWindow->on_headPitchTestModeExecuteButton_clicked();
            return;
        }
        if (cmd.find("on_getHeadPitchPositionButton_clicked") != std::string::npos)
        {
            mainWindow->on_getHeadPitchPositionButton_clicked();
            return;
        }
        if (cmd.find("on_bodyControlP1ReadingsTable_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->bodyControlP1ReadingsTable->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->bodyControlP1ReadingsTable->viewport()->update();
            mainWindow->on_bodyControlP1ReadingsTable_itemClicked(item);
            return;
        }
        if (cmd.find("on_bodyCameraReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_bodyCameraReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_pitchHeadDownDirectionCheckButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            bool checked = (value_str.compare("true")) ? true : false;
            ui->pitchHeadDownDirectionCheckButton->setChecked(checked);
            return;
        }
        if (cmd.find("on_pitchHeadUpDirectionCheckButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            bool checked = (value_str.compare("true")) ? true : false;
            ui->pitchHeadUpDirectionCheckButton->setChecked(checked);
            return;
        }
    }
};
