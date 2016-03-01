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
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem("Control Device Name"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem("Connected"));
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


void MainWindow::on_scanControlDevicesBtn_clicked()
{
    Valter::getInstance()->scanControlDevices();
    vector<ControlDevice> controlDevices = Valter::getInstance()->getControlDevices();
    for(std::vector<int>::size_type i = 0; i != controlDevices.size(); i++)
    {
        ControlDevice controlDevice = controlDevices[i];
    }

}
