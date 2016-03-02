#include "mainwindow.h"
#include <QApplication>
#include <QtDebug>

#include <valter.h>

int main(int argc, char *argv[])
{
    Valter::getInstance();

    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    qDebug("Valter Exp Client V%s", Valter::getVersion().c_str());

    int r = a.exec();

    qDebug("Finished");

    return r;
}
