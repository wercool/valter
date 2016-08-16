#ifndef TASKSTABGUI_H
#define TASKSTABGUI_H

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

void tasksTabRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    if (ui->tasksRTMMTableRefreshCheckBox->isChecked())
    {
        std::map<long, string> rtmms = TaskManager::getInstance()->getRtmms();

        if (rtmms.size() > 0)
        {
            ui->tasksTableWidget->clearContents();
            ui->tasksTableWidget->setRowCount(rtmms.size());

            typedef map<long, string>::iterator it_type;
            char i = 0;
            for(it_type iterator = rtmms.begin(); iterator != rtmms.end(); iterator++)
            {
                long taskId = iterator->first;
                vector<string> taskDesc = Valter::split(((string)iterator->second), '~');
                ui->tasksTableWidget->setItem(i, 0, new QTableWidgetItem(Valter::format_string("%lu", taskId).c_str()));//task id
                ui->tasksTableWidget->setItem(i, 1, new QTableWidgetItem(((string)taskDesc[0]).c_str()));//task name
                ui->tasksTableWidget->setItem(i, 2, new QTableWidgetItem(((string)taskDesc[1]).c_str()));//task type
                QTableWidgetItem *item = new QTableWidgetItem(((string)taskDesc[2]).c_str());
                if (((string)taskDesc[2]).compare("queued") == 0)
                {
                    item->setBackground(Qt::yellow);
                }
                if (((string)taskDesc[2]).compare("executing") == 0)
                {
                    item->setBackground(Qt::green);
                }
                if (((string)taskDesc[2]).compare("completed") == 0)
                {
                    item->setBackground(Qt::gray);
                }
                if (((string)taskDesc[2]).compare("stopped") == 0)
                {
                    item->setBackground(Qt::red);
                }
                ui->tasksTableWidget->setItem(i, 3, item);//task status
                ui->tasksTableWidget->setItem(i, 4, new QTableWidgetItem(((string)taskDesc[3]).c_str()));//task script line
                ui->tasksTableWidget->setItem(i, 5, new QTableWidgetItem(((string)taskDesc[4]).c_str()));//notes
                i++;
            }
        }
        else
        {
            ui->tasksTableWidget->clearContents();
            ui->tasksTableWidget->setRowCount(0);
        }
    }
}

#endif // TASKSTABGUI_H
