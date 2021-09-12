#ifndef LINK3ENDPOINTVIEWITEM_H
#define LINK3ENDPOINTVIEWITEM_H

#include <QGraphicsEllipseItem>
#include <QGraphicsSceneMouseEvent>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

#include <gui/platformmanipulatorandirbumperGUI.h>

class Link3EndPointViewItem : public QGraphicsEllipseItem
{
private:
    double endX, endY;

public:
    Link3EndPointViewItem();
    void setAngle(float g, bool manual);

    // QGraphicsItem interface
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
};

Link3EndPointViewItem::Link3EndPointViewItem()
{

}

void Link3EndPointViewItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    (void)event;
}

void Link3EndPointViewItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    this->setPos(event->scenePos().x() - 6, event->scenePos().y() - 6);

    double x1 = MainWindow::getInstance()->platfromManipulatorLink3Initial->line().x1();
    double y1 = MainWindow::getInstance()->platfromManipulatorLink3Initial->line().y1();
    double x2 = MainWindow::getInstance()->platfromManipulatorLink3Initial->line().x2();
    double y2 = MainWindow::getInstance()->platfromManipulatorLink3Initial->line().y2();

    double x3 = x1;
    double y3 = y1;
    double x4 = event->scenePos().x();
    double y4 = event->scenePos().y();

    //dir
    float dx = x2 - x1;
    float dy = y1 - y2;
    float norm = sqrt(dx * dx + dy * dy);
    dx = dx / norm;
    dy = dy / norm;
    x2 = x1 + dy;
    y2 = y1 + dx;

    double dx1 = x2-x1;
    double dy1 = y2-y1;
    double dx2 = x4-x3;
    double dy2 = y4-y3;

    double d  = dx1*dx2 + dy1*dy2;   // dot product of the 2 vectors
    double l = (dx1*dx1 + dy1*dy1)*(dx2*dx2+dy2*dy2); // product of the squared lengths

    double cos_g = d/sqrt(l);
    double g = acos(cos_g) - M_PI / 2;

    MainWindow::getInstance()->platfromManipulatorLink3Helper->setLine(x1, y1, x4, y4);

    setAngle(g, true);
}

void Link3EndPointViewItem::setAngle(float g, bool manual)
{
    double man_l3 = PlatformManipulatorAndIRBumper::man_l3;

    double x1 = MainWindow::getInstance()->platfromManipulatorLink3Initial->line().x1();
    double y1 = MainWindow::getInstance()->platfromManipulatorLink3Initial->line().y1();

    PlatformManipulatorAndIRBumper::man_g = g;

    double a = PlatformManipulatorAndIRBumper::man_a;
    double b = PlatformManipulatorAndIRBumper::man_b;

    //joint3
    double j1x  = x1 - man_l3 * sin(g + b - a);
    double j1y  = y1 - man_l3 * cos(g + b - a);

    endX = j1x;
    endY = j1y;
    if (!manual)
    {
        MainWindow::getInstance()->platfromManipulatorLink3Helper->setLine(x1, y1, endX, endY);
        this->setPos(endX - 6, endY - 6);
    }
    else
    {
        getAndSetCurrentABGangles();
    }

    MainWindow::getInstance()->platfromManipulatorLink3->setLine(x1, y1, j1x, j1y);

    MainWindow::getInstance()->gamma->setPlainText(Valter::format_string("γ:%.2f", PlatformManipulatorAndIRBumper::man_g * 180 / M_PI).c_str());
}

void Link3EndPointViewItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    (void)event;
    double x1 = MainWindow::getInstance()->platfromManipulatorLink3Initial->line().x1();
    double y1 = MainWindow::getInstance()->platfromManipulatorLink3Initial->line().y1();
    this->setPos(endX - 6, endY - 6);
    MainWindow::getInstance()->platfromManipulatorLink3Helper->setLine(x1, y1, endX, endY);
}

#endif // LINK3ENDPOINTVIEWITEM_H
