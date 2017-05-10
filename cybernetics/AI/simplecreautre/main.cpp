#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication::setDesktopSettingsAware(false);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
