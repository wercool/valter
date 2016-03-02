#include <QtDebug>

#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowIcon(QIcon(":/valter_head_icon.png"));

    //controlDeviceTableWidget
    ui->controlDeviceTableWidget->setColumnCount(4);
    ui->controlDeviceTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->controlDeviceTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem("Device File Name"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem("Control Device Id"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem("Opened"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(3, new QTableWidgetItem("Status"));
    QHeaderView* controlDeviceTableWidgetHeaderView = new QHeaderView(Qt::Horizontal);
    controlDeviceTableWidgetHeaderView->setStretchLastSection(true);
    controlDeviceTableWidgetHeaderView->setSectionResizeMode(QHeaderView::Stretch);
    ui->controlDeviceTableWidget->setHorizontalHeader(controlDeviceTableWidgetHeaderView);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::refreshControlDeviceTableWidget()
{
    ui->controlDeviceTableWidget->clear();

    ui->controlDeviceTableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem("Device File Name"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem("Control Device Id"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem("Opened"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(3, new QTableWidgetItem("Status"));

    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    ui->controlDeviceTableWidget->setRowCount(controlDevicesMap.size());

    char i = 0;
    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        ui->controlDeviceTableWidget->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDevicePort()->getPort() .c_str()));
        ui->controlDeviceTableWidget->setItem(i, 1, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
        ui->controlDeviceTableWidget->setItem(i, 2, new QTableWidgetItem((controlDevice->getControlDevicePort()->isOpen() ? "true" :"false")));
        ui->controlDeviceTableWidget->setItem(i, 3, new QTableWidgetItem(controlDevice->getStatus().c_str()));
        i++;
    }

}


void MainWindow::on_scanControlDevicesBtn_clicked()
{
    Valter::getInstance()->scanControlDevices();
    refreshControlDeviceTableWidget();
}

void MainWindow::on_pushButton_clicked()
{
    int selectedControlDeviceRowIndex = ui->controlDeviceTableWidget->selectionModel()->currentIndex().row();
    if (selectedControlDeviceRowIndex >= 0)
    {
        QTableWidgetItem *item = new QTableWidgetItem();
        item = ui->controlDeviceTableWidget->item(selectedControlDeviceRowIndex,1);
        string controlDeviceId = item->text().toStdString();

        ControlDevice *controlDevice = Valter::getInstance()->getControlDeviceById(controlDeviceId);

        if (controlDevice->getControlDevicePort()->isOpen())
        {
            controlDevice->setStatus(ControlDevice::StatusReady);
            controlDevice->getControlDevicePort()->close();
        }
        else
        {
            controlDevice->getControlDevicePort()->open();
            controlDevice->setStatus(ControlDevice::StatusActive);
            controlDevice->spawnReadControlDeviceOutputThreadWorker();
        }
    }
    refreshControlDeviceTableWidget();
}
