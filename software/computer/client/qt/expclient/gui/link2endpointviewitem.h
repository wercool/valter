#ifndef LINK2ENDPOINTVIEWITEM_H
#define LINK2ENDPOINTVIEWITEM_H

#include <QGraphicsEllipseItem>
#include <QGraphicsSceneMouseEvent>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

class Link2EndPointViewItem : public QGraphicsEllipseItem
{
private:
    double endX, endY;

public:
    Link2EndPointViewItem();

    // QGraphicsItem interface
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
};

Link2EndPointViewItem::Link2EndPointViewItem()
{

}

void Link2EndPointViewItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    (void)event;
}

void Link2EndPointViewItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    this->setPos(event->scenePos().x() - 6, event->scenePos().y() - 6);

    double man_l2 = PlatformManipulatorAndIRBumper::man_l2;
    double man_l3 = PlatformManipulatorAndIRBumper::man_l3;
    double man_l2_l3 = PlatformManipulatorAndIRBumper::man_l2_l3;

    double x1 = MainWindow::getInstance()->platfromManipulatorLink2Initial->line().x1();
    double y1 = MainWindow::getInstance()->platfromManipulatorLink2Initial->line().y1();
    double x2 = MainWindow::getInstance()->platfromManipulatorLink2Initial->line().x2();
    double y2 = MainWindow::getInstance()->platfromManipulatorLink2Initial->line().y2();

    double x3 = x1;
    double y3 = y1;
    double x4 = event->scenePos().x();
    double y4 = event->scenePos().y();

    double dx1 = x2-x1;
    double dy1 = y2-y1;
    double dx2 = x4-x3;
    double dy2 = y4-y3;

    double d  = dx1*dx2 + dy1*dy2;   // dot product of the 2 vectors
    double l = (dx1*dx1+dy1*dy1)*(dx2*dx2+dy2*dy2); // product of the squared lengths

    double b = acos(d/sqrt(l));

    //joint1
    double j1x  = x1 - man_l2 * sin(b - PlatformManipulatorAndIRBumper::man_a);
    double j1y  = y1 - man_l2 * cos(b - PlatformManipulatorAndIRBumper::man_a);


    //joint2console
    float dx = j1x - x1;
    float dy = y1 - j1y;
    float norm = sqrt(dx * dx + dy * dy);
    dx = dx / norm;
    dy = dy / norm;
    float j2x = j1x + dy * man_l2_l3;
    float j2y = j1y + dx * man_l2_l3;

    //joint3
    double a = PlatformManipulatorAndIRBumper::man_a;
    double g = PlatformManipulatorAndIRBumper::man_g;

    double j3x = j2x - man_l3 * sin(g + b - a);
    double j3y = j2y - man_l3 * cos(g + b - a);

    double j3xi = j2x - man_l3 * sin(b - a);
    double j3yi = j2y - man_l3 * cos(b - a);

    PlatformManipulatorAndIRBumper::man_b = b;

    MainWindow::getInstance()->platfromManipulatorLink2->setLine(x1, y1, j1x, j1y);
    MainWindow::getInstance()->platfromManipulatorLink2Helper->setLine(x1, y1, x4, y4);
    MainWindow::getInstance()->platfromManipulatorLink2Link3Console->setLine(j1x, j1y, j2x, j2y);
    MainWindow::getInstance()->platfromManipulatorLink3->setLine(j2x, j2y, j3x, j3y);
    MainWindow::getInstance()->platfromManipulatorLink3Initial->setLine(j2x, j2y, j3xi, j3yi);
    MainWindow::getInstance()->platfromManipulatorLink3Helper->setLine(j2x, j2y, j3x, j3y);

    MainWindow::getInstance()->link2link3Point->setPos(j2x - 6, j2y - 6);
    MainWindow::getInstance()->link2link3Point->setRect(0, 0, 12, 12);

    MainWindow::getInstance()->link3EndPoint->setPos(j3x - 6, j3y - 6);
    MainWindow::getInstance()->link3EndPoint->setRect(0, 0, 12, 12);

    MainWindow::getInstance()->gamma->setPos(j2x + 5, j2y - 15);


    MainWindow::getInstance()->beta->setPlainText(Valter::format_string("β:%.2f", PlatformManipulatorAndIRBumper::man_b * 180 / M_PI).c_str());

    endX = j1x;
    endY = j1y;
}

void Link2EndPointViewItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    (void)event;
    double x1 = MainWindow::getInstance()->platfromManipulatorLink2Initial->line().x1();
    double y1 = MainWindow::getInstance()->platfromManipulatorLink2Initial->line().y1();
    this->setPos(endX - 6, endY - 6);
    MainWindow::getInstance()->platfromManipulatorLink2Helper->setLine(x1, y1, endX, endY);
}

#endif // LINK2ENDPOINTVIEWITEM_H
