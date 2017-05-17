#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication::setDesktopSettingsAware(false);
    QApplication app(argc, argv);
    MainWindow w;
    w.show();

    return app.exec();
}
