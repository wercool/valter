#include "mainwindow.h"
#include <QApplication>
#include <QtDebug>

#include <signal.h>
#include <unistd.h>

#include <valter.h>

void catchUnixSignals(const std::vector<int>& quitSignals,
                      const std::vector<int>& ignoreSignals = std::vector<int>())
{

    auto handler = [](int sig) ->void {
        printf("\nquit the application (user request signal = %d).\n", sig);
        QCoreApplication::quit();
    };

    // all these signals will be ignored.
    for ( int sig : ignoreSignals )
        signal(sig, SIG_IGN);

    // each of these signals calls the handler (quits the QCoreApplication).
    for ( int sig : quitSignals )
        signal(sig, handler);
}

MainWindow *mainGUIWindow;
int main(int argc, char *argv[])
{
    int r = 0;
    try
    {
        if (argc > 1)
        {
            std::string iface = std::string(argv[1]);
            if (iface == "eth0" || iface == "wlan0")
            {
                TCPInterface::defaultNetworkInterface = string(argv[1]);
                qDebug("TCPInterface::defaultNetworkInterface: %s", TCPInterface::defaultNetworkInterface.c_str());
            }
        }

        QApplication application(argc, argv);
        application.setStyle("gtk+");

        mainGUIWindow = MainWindow::getInstance();
        mainGUIWindow->setWindowFlags(Qt::WindowStaysOnTopHint);
        mainGUIWindow->show();

        Valter::getInstance();

        Valter::log(Valter::format_string("Valter Exp Client V.%s", Valter::getVersion().c_str()));
        Valter::getInstance()->autoInitialization();

        catchUnixSignals({SIGQUIT, SIGINT, SIGTERM, SIGHUP});

        r = application.exec();

        Valter::getInstance()->prepareShutdown();
        Valter::getInstance()->closeAllControlDevicePorts();

        this_thread::sleep_for(std::chrono::milliseconds(500));

        qDebug("Finished");
    }
    catch (const exception &ex)
    {
        qDebug("Fatal exception: %s", ex.what());
    }

    return r;
}
