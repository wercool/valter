#include "mainwindow.h"
#include <QApplication>
#include <QtDebug>

#include <valter.h>

MainWindow *mainGUIWindow;
int main(int argc, char *argv[])
{
    QApplication application(argc, argv);

    mainGUIWindow = MainWindow::getInstance();
    mainGUIWindow->show();

    Valter::getInstance();
    Valter::log(Valter::format_string("Valter Exp Client V.%s", Valter::getVersion().c_str()));

    int r = application.exec();

    Valter::getInstance()->closeAllControlDevicePorts();

    delete(mainGUIWindow);
    mainGUIWindow = NULL;
    delete(Valter::getInstance());

    qDebug("Finished");

    return r;
}
