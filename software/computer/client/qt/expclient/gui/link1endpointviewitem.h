#ifndef LINK1ENDPOINTVIEWITEM_H
#define LINK1ENDPOINTVIEWITEM_H

#include <QGraphicsEllipseItem>
#include <QGraphicsSceneMouseEvent>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

#include <gui/platformmanipulatorandirbumperGUI.h>

class Link1EndPointViewItem : public QGraphicsEllipseItem
{
private:
    double endX, endY;

public:
    Link1EndPointViewItem();
    void setAngle(float a, bool manual);

    // QGraphicsItem interface
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
};

Link1EndPointViewItem::Link1EndPointViewItem()
{

}

void Link1EndPointViewItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    (void)event;
}

void Link1EndPointViewItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    this->setPos(event->scenePos().x() - 6, event->scenePos().y() - 6);

    double man_l1 = PlatformManipulatorAndIRBumper::man_l1;
    double rootX = PlatformManipulatorAndIRBumper::rootX;
    double rootY = PlatformManipulatorAndIRBumper::rootY;

    double x1 = rootX;
    double y1 = rootY;
    double x2 = rootX;
    double y2 = rootY + man_l1;

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

    double a = acos(d/sqrt(l));

    MainWindow::getInstance()->platfromManipulatorLink1Helper->setLine(x1, y1, x4, y4);

    setAngle(a, true);
}

void Link1EndPointViewItem::setAngle(float a, bool manual)
{
    double man_l1 = PlatformManipulatorAndIRBumper::man_l1;
    double man_l2 = PlatformManipulatorAndIRBumper::man_l2;
    double man_l3 = PlatformManipulatorAndIRBumper::man_l3;
    double man_l1_l2 = PlatformManipulatorAndIRBumper::man_l1_l2;
    double man_l2_l3 = PlatformManipulatorAndIRBumper::man_l2_l3;
    double rootX = PlatformManipulatorAndIRBumper::rootX;
    double rootY = PlatformManipulatorAndIRBumper::rootY;

    double x1 = rootX;
    double y1 = rootY;

    //joint1
    double j1x  = x1 - man_l1 * sin(a);
    double j1y  = y1 + man_l1 * cos(a);

    //join1console
    float dx = j1x - x1;
    float dy = y1 - j1y;
    float norm = sqrt(dx * dx + dy * dy);
    dx = dx / norm;
    dy = dy / norm;
    float j2x = j1x + dy * man_l1_l2;
    float j2y = j1y + dx * man_l1_l2;

    //joint2
    double b = PlatformManipulatorAndIRBumper::man_b;

    double j3x = j2x - man_l2 * sin(b - a);
    double j3y = j2y - man_l2 * cos(b - a);

    double j3xi = j2x - man_l2 * sin(- a);
    double j3yi = j2y - man_l2 * cos(- a);

    //join2console
    dx = j3x - j2x;
    dy = j2y - j3y;
    norm = sqrt(dx * dx + dy * dy);
    dx = dx / norm;
    dy = dy / norm;
    float j4x = j3x + dy * man_l2_l3;
    float j4y = j3y + dx * man_l2_l3;

    //joint3
    double g = PlatformManipulatorAndIRBumper::man_g;

    double j5x = j4x - man_l3 * sin(g + b - a);
    double j5y = j4y - man_l3 * cos(g + b - a);

    double j5xi = j4x - man_l3 * sin(b - a);
    double j5yi = j4y - man_l3 * cos(b - a);


    PlatformManipulatorAndIRBumper::man_a = a;

    endX = j1x;
    endY = j1y;

    if (!manual)
    {
        MainWindow::getInstance()->platfromManipulatorLink1Helper->setLine(x1, y1, endX, endY);
        this->setPos(endX - 6, endY - 6);
    }
    else
    {
        getAndSetCurrentABGangles();
    }

    MainWindow::getInstance()->valter3d->setLink1ZAngle(PlatformManipulatorAndIRBumper::man_a);

    MainWindow::getInstance()->platfromManipulatorLink1->setLine(x1, y1, j1x, j1y);
    MainWindow::getInstance()->platfromManipulatorLink1Link2Console->setLine(j1x, j1y, j2x, j2y);
    MainWindow::getInstance()->platfromManipulatorLink2->setLine(j2x, j2y, j3x, j3y);
    MainWindow::getInstance()->platfromManipulatorLink2Helper->setLine(j2x, j2y, j3x, j3y);
    MainWindow::getInstance()->platfromManipulatorLink2Initial->setLine(j2x, j2y, j3xi, j3yi);
    MainWindow::getInstance()->platfromManipulatorLink2Link3Console->setLine(j3x, j3y, j4x, j4y);
    MainWindow::getInstance()->platfromManipulatorLink3->setLine(j4x, j4y, j5x, j5y);
    MainWindow::getInstance()->platfromManipulatorLink3Helper->setLine(j4x, j4y, j5x, j5y);
    MainWindow::getInstance()->platfromManipulatorLink3Initial->setLine(j4x, j4y, j5xi, j5yi);

    MainWindow::getInstance()->link1link2Point->setPos(j2x - 6, j2y - 6);
    MainWindow::getInstance()->link1link2Point->setRect(0, 0, 12, 12);

    MainWindow::getInstance()->link2link3Point->setPos(j4x - 6, j4y - 6);
    MainWindow::getInstance()->link2link3Point->setRect(0, 0, 12, 12);

    MainWindow::getInstance()->link2EndPoint->setPos(j3x - 6, j3y - 6);
    MainWindow::getInstance()->link2EndPoint->setRect(0, 0, 12, 12);

    MainWindow::getInstance()->link3EndPoint->setPos(j5x - 6, j5y - 6);
    MainWindow::getInstance()->link3EndPoint->setRect(0, 0, 12, 12);

    MainWindow::getInstance()->beta->setPos(j2x - 20, j2y);
    MainWindow::getInstance()->gamma->setPos(j4x + 5, j4y - 15);

    MainWindow::getInstance()->alpha->setPlainText(Valter::format_string("α:%.2f", PlatformManipulatorAndIRBumper::man_a * 180 / M_PI).c_str());
}

void Link1EndPointViewItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    (void)event;
    double rootX = PlatformManipulatorAndIRBumper::rootX;
    double rootY = PlatformManipulatorAndIRBumper::rootY;
    this->setPos(endX - 6, endY - 6);
    MainWindow::getInstance()->platfromManipulatorLink1Helper->setLine(rootX, rootY, endX, endY);
}

#endif // LINK1ENDPOINTVIEWITEM_H
