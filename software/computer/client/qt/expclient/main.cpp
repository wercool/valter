#include "mainwindow.h"
#include <QApplication>
#include <QtDebug>

#include <valter.h>

MainWindow *mainGUIWindow;
int main(int argc, char *argv[])
{
    int r = 0;
    try
    {
        QApplication application(argc, argv);
        application.setStyle("gtk+");

        mainGUIWindow = MainWindow::getInstance();
        mainGUIWindow->show();

        Valter::getInstance();
        Valter::log(Valter::format_string("Valter Exp Client V.%s", Valter::getVersion().c_str()));

        r = application.exec();

        Valter::getInstance()->closeAllControlDevicePorts();

        qDebug("Finished");
    }
    catch (const exception &ex)
    {
        qDebug("Fatal esception: %s", ex.what());
    }

    return r;
}
