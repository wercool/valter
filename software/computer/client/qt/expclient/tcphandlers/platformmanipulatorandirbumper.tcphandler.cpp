#include "valter.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qnamespace.h>

class PlatformManipulatorAndIRBumperTCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    PlatformManipulatorAndIRBumperTCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

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
        PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

        if (cmd.find("CDR~") != std::string::npos)
        {
            int cdr_pos = cmd.find("~") + 1;
            string cdr = cmd.substr(cdr_pos);

            qDebug("TCP[%s] → %s", platformManipulatorAndIRBumper->getControlDeviceId().c_str(), cdr.c_str());

            platformManipulatorAndIRBumper->processControlDeviceResponse(cdr);
        }

        //control/service messages
        if (cmd.find("setCentralCommandHostInfo") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            platformManipulatorAndIRBumper->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            platformManipulatorAndIRBumper->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            platformManipulatorAndIRBumper->getTcpInterface()->setConnected(true);
            platformManipulatorAndIRBumper->getTcpInterface()->sendCDRToCentralCommandHost(Valter::format_string("CDR~REMOTECD:%s:%s:%d:%s", platformManipulatorAndIRBumper->getControlDeviceId().c_str(), platformManipulatorAndIRBumper->getTcpInterface()->getCentralCommandHostIP().c_str(), platformManipulatorAndIRBumper->getTcpInterface()->getCentralCommandHostIPPort(), platformManipulatorAndIRBumper->getControlDevice()->getStatus().c_str()));
            qDebug("[%s] Central Command Host IP Address:%s Port:%d", platformManipulatorAndIRBumper->getControlDeviceId().c_str(), platformManipulatorAndIRBumper->getTcpInterface()->getCentralCommandHostIP().c_str(), platformManipulatorAndIRBumper->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }
        if (cmd.find("stopCDTtoCentralCommandHost") != std::string::npos)
        {
            platformManipulatorAndIRBumper->getTcpInterface()->setConnected(false);
            return;
        }

        //GUI elements events handler

        MainWindow *mainWindow = MainWindow::getInstance();
        Ui::MainWindow* ui = mainWindow->getUi();

        if (cmd.find("on_platformManipulatorAndIRBumperLoadDefaultsButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformManipulatorAndIRBumperLoadDefaultsButton_clicked();
            return;
        }
        if (cmd.find("on_platformManipulatorAndIRBumperRedrawGUICheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformManipulatorAndIRBumperRedrawGUICheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_powerOnOff24VPlatfromManipulatorAndIRBumperButton_clicked") != std::string::npos)
        {
            mainWindow->on_powerOnOff24VPlatfromManipulatorAndIRBumperButton_clicked();
            return;
        }
        if (cmd.find("on_manipulatorLiknk1AscentButton_pressed") != std::string::npos)
        {
            mainWindow->on_manipulatorLiknk1AscentButton_pressed();
            return;
        }
        if (cmd.find("on_manipulatorLiknk1AscentButton_released") != std::string::npos)
        {
            mainWindow->on_manipulatorLiknk1AscentButton_released();
            return;
        }
        if (cmd.find("on_manipulatorLiknk1DescentButton_pressed") != std::string::npos)
        {
            mainWindow->on_manipulatorLiknk1DescentButton_pressed();
            return;
        }
        if (cmd.find("on_manipulatorLiknk1DescentButton_released") != std::string::npos)
        {
            mainWindow->on_manipulatorLiknk1DescentButton_released();
            return;
        }
        if (cmd.find("on_manipulatorLiknk1MotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->manipulatorLiknk1MotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_manipulatorLiknk1DecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->manipulatorLiknk1DecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_manipulatorLiknk1AccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->manipulatorLiknk1AccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_manipulatorLiknk2AscentButton_pressed") != std::string::npos)
        {
            mainWindow->on_manipulatorLiknk2AscentButton_pressed();
            return;
        }
        if (cmd.find("on_manipulatorLiknk2AscentButton_released") != std::string::npos)
        {
            mainWindow->on_manipulatorLiknk2AscentButton_released();
            return;
        }
        if (cmd.find("on_manipulatorLiknk2DescentButton_pressed") != std::string::npos)
        {
            mainWindow->on_manipulatorLiknk2DescentButton_pressed();
            return;
        }
        if (cmd.find("on_manipulatorLiknk2DescentButton_released") != std::string::npos)
        {
            mainWindow->on_manipulatorLiknk2DescentButton_released();
            return;
        }
        if (cmd.find("on_manipulatorLiknk2MotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->manipulatorLiknk2MotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_manipulatorLiknk2DecelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->manipulatorLiknk2DecelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_manipulatorLiknk2AccelerationScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->manipulatorLiknk2AccelerationScroller->setValue(value);
            return;
        }
        if (cmd.find("on_platformManipulatorAngleASpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            double value = atof(value_str.c_str());
            ui->platformManipulatorAngleASpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_platformManipulatorAngleBSpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            double value = atof(value_str.c_str());
            ui->platformManipulatorAngleBSpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_platformManipulatorAngleGSpinBox_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            double value = atof(value_str.c_str());
            ui->platformManipulatorAngleGSpinBox->setValue(value);
            return;
        }
        if (cmd.find("on_platformManipulatorExecuteAngleSettingsButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformManipulatorExecuteAngleSettingsButton_clicked();
            return;
        }
        if (cmd.find("on_gripperTiltAscentButton_pressed") != std::string::npos)
        {
            mainWindow->on_gripperTiltAscentButton_pressed();
            return;
        }
        if (cmd.find("on_gripperTiltAscentButton_released") != std::string::npos)
        {
            mainWindow->on_gripperTiltAscentButton_released();
            return;
        }
        if (cmd.find("on_gripperTiltDescentButton_pressed") != std::string::npos)
        {
            mainWindow->on_gripperTiltDescentButton_pressed();
            return;
        }
        if (cmd.find("on_gripperTiltDescentButton_released") != std::string::npos)
        {
            mainWindow->on_gripperTiltDescentButton_released();
            return;
        }
        if (cmd.find("on_manGripperOpenButton_pressed") != std::string::npos)
        {
            mainWindow->on_manGripperOpenButton_pressed();
            return;
        }
        if (cmd.find("on_manGripperOpenButton_released") != std::string::npos)
        {
            mainWindow->on_manGripperOpenButton_released();
            return;
        }
        if (cmd.find("on_manGripperCloseButton_pressed") != std::string::npos)
        {
            mainWindow->on_manGripperCloseButton_pressed();
            return;
        }
        if (cmd.find("on_manGripperCloseButton_released") != std::string::npos)
        {
            mainWindow->on_manGripperCloseButton_released();
            return;
        }
        if (cmd.find("on_manGripperRotateCCW_pressed") != std::string::npos)
        {
            mainWindow->on_manGripperRotateCCW_pressed();
            return;
        }
        if (cmd.find("on_manGripperRotateCCW_released") != std::string::npos)
        {
            mainWindow->on_manGripperRotateCCW_released();
            return;
        }
        if (cmd.find("on_manGripperRotateCW_pressed") != std::string::npos)
        {
            mainWindow->on_manGripperRotateCW_pressed();
            return;
        }
        if (cmd.find("on_manGripperRotateCW_released") != std::string::npos)
        {
            mainWindow->on_manGripperRotateCW_released();
            return;
        }
        if (cmd.find("on_manGripperRotationMotorDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->manGripperRotationMotorDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_manipulatorAndIRBumperManipulatorReadingsTableWidget_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->viewport()->update();
            mainWindow->on_manipulatorAndIRBumperManipulatorReadingsTableWidget_itemClicked(item);
            return;
        }
        if (cmd.find("on_irBumperReadingsTable_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->irBumperReadingsTable->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->irBumperReadingsTable->viewport()->update();
            mainWindow->on_irBumperReadingsTable_itemClicked(item);
            return;
        }
        if (cmd.find("on_irBumperEnableButton_clicked") != std::string::npos)
        {
            mainWindow->on_irBumperEnableButton_clicked();
            return;
        }
        if (cmd.find("on_irBumperDisableButton_clicked") != std::string::npos)
        {
            mainWindow->on_irBumperDisableButton_clicked();
            return;
        }
        if (cmd.find("on_irBumperInitButton_clicked") != std::string::npos)
        {
            mainWindow->on_irBumperInitButton_clicked();
            return;
        }
        if (cmd.find("on_irBumperFrequencySpin_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->irBumperFrequencySpin->setValue(value);
            return;
        }
        if (cmd.find("on_irBumperDutySpin_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->irBumperDutySpin->setValue(value);
            return;
        }
        if (cmd.find("on_irBumperTrackAllButton_clicked") != std::string::npos)
        {
            mainWindow->on_irBumperTrackAllButton_clicked();
            return;
        }
        if (cmd.find("on_irBumperTrackNoneButton_clicked") != std::string::npos)
        {
            mainWindow->on_irBumperTrackNoneButton_clicked();
            return;
        }
    }
};
