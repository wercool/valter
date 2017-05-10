#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    Q_INIT_RESOURCE(images);

    QApplication::setDesktopSettingsAware(false);
//    QApplication::setGraphicsSystem("raster");
    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);

    MainWindow w;
    w.show();

    return app.exec();
}
