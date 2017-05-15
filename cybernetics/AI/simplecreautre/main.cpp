#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    int r = 0;

    Q_INIT_RESOURCE(images);

    QApplication::setDesktopSettingsAware(false);
//    QApplication::setGraphicsSystem("raster");
    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);

    MainWindow w;
    w.show();

    r = app.exec();

    w.killColony();

    return r;
}
