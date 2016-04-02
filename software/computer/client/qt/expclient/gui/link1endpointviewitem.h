#ifndef LINK2ENDPOINTVIEWITEM_H
#define LINK2ENDPOINTVIEWITEM_H

#include <QGraphicsEllipseItem>
#include <QGraphicsSceneMouseEvent>
#include <mainwindow.h>

class Link1EndPointViewItem : public QGraphicsEllipseItem
{
private:
    double endX, endY;
public:
    Link1EndPointViewItem();

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


    double x1 = 220;
    double y1 = 110;
    double x2 = 220;
    double y2 = 230;

    double x3 = x1;
    double y3 = y1;
    double x4 = event->scenePos().x();
    double y4 = event->scenePos().y();


    double dx1 = x2-x1;
    double dy1 = y2-y1;
    double dx2 = x4-x3;
    double dy2 = y4-y3;

    double d = dx1*dx2 + dy1*dy2;   // dot product of the 2 vectors
    double l2 = (dx1*dx1+dy1*dy1)*(dx2*dx2+dy2*dy2); // product of the squared lengths

    double a = acos(d/sqrt(l2));

    //joint1
    double j1x  = x1 - 120 * sin(a);
    double j1y  = y1 + 120 * cos(a);

    //joint2
    float dx3 = j1x - x1;
    float dy3 = y1 - j1y;
    float norm = sqrt(dx3 * dx3 + dy3 * dy3);
    dx3 = dx3 / norm;
    dy3 = dy3 / norm;
    float j2x = j1x + dy3 * 30;
    float j2y = j1y + dx3 * 30;

    MainWindow::getInstance()->platfromManipulatorHelper->setLine(x1, y1, x4, y4);
    MainWindow::getInstance()->platfromManipulatorLink1->setLine(x1, y1, j1x, j1y);
    MainWindow::getInstance()->platfromManipulatorLink1Link2Console->setLine(j1x, j1y, j2x, j2y);
    MainWindow::getInstance()->link1link2Point->setRect(j2x - 6, j2y - 6, 12, 12);

    qDebug("a = %.3f", a * 180 / M_PI);

    endX = j1x;
    endY = j1y;

}

void Link1EndPointViewItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    (void)event;
    double x1 = 220;
    double y1 = 110;
    this->setPos(endX - 6, endY - 6);
    MainWindow::getInstance()->platfromManipulatorHelper->setLine(x1, y1, endX, endY);
}

#endif // LINK2ENDPOINTVIEWITEM_H
