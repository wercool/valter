#include "valter.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qnamespace.h>

class PlatformLocationP1TCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    PlatformLocationP1TCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

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
        PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();

        if (cmd.find("CDR~") != std::string::npos)
        {
            int cdr_pos = cmd.find("~") + 1;
            string cdr = cmd.substr(cdr_pos);

            qDebug("TCP[%s] → %s", platformLocationP1->getControlDeviceId().c_str(), cdr.c_str());

            platformLocationP1->processControlDeviceResponse(cdr);
        }

        //control/service messages
        if (cmd.find("setCentralCommandHostInfo") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            platformLocationP1->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            platformLocationP1->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            platformLocationP1->getTcpInterface()->setConnected(true);
            qDebug("[%s] Central Command Host IP Address:%s Port:%d", platformLocationP1->getControlDeviceId().c_str(), platformLocationP1->getTcpInterface()->getCentralCommandHostIP().c_str(), platformLocationP1->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }
        if (cmd.find("stopCDTtoCentralCommandHost") != std::string::npos)
        {
            platformLocationP1->getTcpInterface()->setConnected(false);
            return;
        }

        //GUI elements events handler

        MainWindow *mainWindow = MainWindow::getInstance();
        Ui::MainWindow* ui = mainWindow->getUi();

        if (cmd.find("on_loadDefaultsPlatformLocationP1Button_clicked") != std::string::npos)
        {
            mainWindow->on_loadDefaultsPlatformLocationP1Button_clicked();
            return;
        }
        if (cmd.find("on_platformLocationP1RedrawGUICheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->platformLocationP1RedrawGUICheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_platformLocationP1EnableSensorsButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformLocationP1EnableSensorsButton_clicked();
            return;
        }
        if (cmd.find("on_platformLocationP1DisableSensorsButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformLocationP1DisableSensorsButton_clicked();
            return;
        }
        if (cmd.find("on_LEDStatesButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->LEDStatesButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("platfromLocationP1LEDOnOff") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            mainWindow->platfromLocationP1LEDOnOff(value_str);
            return;
        }
        if (cmd.find("on_platformLocationP1AllLEDsONButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformLocationP1AllLEDsONButton_clicked();
            return;
        }
        if (cmd.find("on_platformLocationP1AllLEDsOFFButton_clicked") != std::string::npos)
        {
            mainWindow->on_platformLocationP1AllLEDsOFFButton_clicked();
            return;
        }
        if (cmd.find("on_USVoltageUpButton_clicked") != std::string::npos)
        {
            mainWindow->on_USVoltageUpButton_clicked();
            return;
        }
        if (cmd.find("on_USVoltageDownButton_clicked") != std::string::npos)
        {
            mainWindow->on_USVoltageDownButton_clicked();
            return;
        }
        if (cmd.find("on_USSignalDutyScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->USSignalDutyScroller->setValue(value);
            return;
        }
        if (cmd.find("on_USSignalBurstScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->USSignalBurstScroller->setValue(value);
            return;
        }
        if (cmd.find("on_USSignalDelayScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->USSignalDelayScroller->setValue(value);
            return;
        }
        if (cmd.find("on_enableAllIRSensorsButton_clicked") != std::string::npos)
        {
            mainWindow->on_enableAllIRSensorsButton_clicked();
            return;
        }
        if (cmd.find("on_disableAllIRSensorsButton_clicked") != std::string::npos)
        {
            mainWindow->on_disableAllIRSensorsButton_clicked();
            return;
        }
        if (cmd.find("on_enableAllUSSensorsButton_clicked") != std::string::npos)
        {
            mainWindow->on_enableAllUSSensorsButton_clicked();
            return;
        }
        if (cmd.find("on_disableAllUSSensorsButton_clicked") != std::string::npos)
        {
            mainWindow->on_disableAllUSSensorsButton_clicked();
            return;
        }
        if (cmd.find("on_irSensorsPresetsTable_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->irSensorsPresetsTable->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->irSensorsPresetsTable->viewport()->update();
            mainWindow->on_irSensorsPresetsTable_itemClicked(item);
            return;
        }
        if (cmd.find("on_usSensorsPresetsTable_itemClicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');
            int row = atoi(((string)value_str_values[0]).c_str());
            int column = atoi(((string)value_str_values[1]).c_str());
            bool checked = (value_str_values[2].compare("true")) ? true : false;
            QTableWidgetItem* item = ui->usSensorsPresetsTable->item(row, column);
            item->setCheckState((checked) ? Qt::Checked: Qt::Unchecked);
            ui->usSensorsPresetsTable->viewport()->update();
            mainWindow->on_usSensorsPresetsTable_itemClicked(item);
            return;
        }
        if (cmd.find("on_leftSonarAngleScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->leftSonarAngleScroller->setValue(value);
            return;
        }
        if (cmd.find("on_rightSonarAngleScroller_valueChanged") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            int value = atoi(value_str.c_str());
            ui->rightSonarAngleScroller->setValue(value);
            return;
        }
        if (cmd.find("on_leftSonarScanButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->leftSonarScanButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_rightSonarScanButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->rightSonarScanButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_leftSonarReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_leftSonarReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_rightSonarReleaseButton_clicked") != std::string::npos)
        {
            mainWindow->on_rightSonarReleaseButton_clicked();
            return;
        }
        if (cmd.find("on_updateAccelerometerButton_clicked") != std::string::npos)
        {
            mainWindow->on_updateAccelerometerButton_clicked();
            return;
        }
        if (cmd.find("on_accelerometerTrackCheckBox_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->accelerometerTrackCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_accelerometerRawCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->accelerometerRawCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_magnetometerTrackCheckBox_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->magnetometerTrackCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_magnetometerRawCheckBox_clicked") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->magnetometerRawCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_updateCompassHeadingButton_clicked") != std::string::npos)
        {
            mainWindow->on_updateCompassHeadingButton_clicked();
            return;
        }
        if (cmd.find("on_compassHeadingTrackCheckBox_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->compassHeadingTrackCheckBox->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_llLedToggleButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->llLedToggleButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_lrLedToggleButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->lrLedToggleButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_rlLedToggleButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->rlLedToggleButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_rrLedToggleButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->rrLedToggleButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_allSonarsLedsToggleButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->allSonarsLedsToggleButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
        if (cmd.find("on_manLedToggleButton_toggled") != std::string::npos)
        {
            int substr_pos = cmd.find("@") + 1;
            string value_str = cmd.substr(substr_pos);
            ui->manLedToggleButton->setChecked((value_str.compare("true")) ? true : false);
            return;
        }
    }
};
