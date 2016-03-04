#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <valter.h>

namespace Ui {
class MainWindow;
}
using namespace std;

class MainWindow : public QMainWindow
{
    Q_OBJECT


public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    static MainWindow* getInstance();
    void refreshControlDeviceTableWidget();
    void addMsgToLog(string msg);

private slots:
    void on_scanControlDevicesBtn_clicked();

    void on_connectDisconnectControlDeviceButton_clicked();

    void on_controlDeviceTableWidget_clicked(const QModelIndex &index);

    void on_selectedControlDeviceListWidget_doubleClicked(const QModelIndex &index);

    void on_clearCommandButton_clicked();

    void on_clearLogButton_clicked();

    void on_pauseLoggingButton_clicked();

    void on_sendCommandButton_clicked();

    void controlDevicesDataExchangeLogTimerUpdate();

    void controlDevicesTableRefreshTimerUpdate();

    void on_connectAllPushButton_clicked();

    void on_wdResetStopButton_clicked();

    void on_reScanControlDevicesButton_clicked();

private:
    Ui::MainWindow *ui;
    static MainWindow* pMainWindow;
    static bool instanceFlag;
    static const int logMaxLength = 5000;
    int logLength;
    QTimer *controlDevicesDataExchangeLogTimer;
    QTimer *controlDevicesTableRefreshTimer;
    string selectedControlDeviceId;
    bool allConnect;
};

#endif // MAINWINDOW_H
