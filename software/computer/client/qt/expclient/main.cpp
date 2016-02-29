#include "mainwindow.h"
#include <QApplication>
#include <QTextStream>

#include <valter.h>

int main(int argc, char *argv[])
{
    Valter* valter = Valter::getInstance();


    QApplication a(argc, argv);
    QTextStream out(stdout);
    MainWindow w;
    w.show();

    out << QObject::tr("Valter's model version: ") << valter->getVersion().c_str() << endl;
    valter->listControlDevices();

    return a.exec();
}
