#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <valter.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT


public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void refreshControlDeviceTableWidget();

private slots:
    void on_scanControlDevicesBtn_clicked();

    void on_connectDisconnectControlDeviceButton_clicked();

    void on_controlDeviceTableWidget_clicked(const QModelIndex &index);

    void on_selectedControlDeviceListWidget_doubleClicked(const QModelIndex &index);

    void on_clearCommandButton_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
