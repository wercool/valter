#ifndef LINK2ENDPOINTVIEWITEM_H
#define LINK2ENDPOINTVIEWITEM_H

#include <QGraphicsEllipseItem>
#include <QGraphicsSceneMouseEvent>
#include <mainwindow.h>

class Link2EndPointViewItem : public QGraphicsEllipseItem
{
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
    this->setPos(QPointF(event->scenePos().x() - 6, event->scenePos().y() - 6));

    QPointF root(220, 110);

    float endEffectorX = this->scenePos().x() + 6;
    float endEffectorY = this->scenePos().y() + 6;

    float dirx = endEffectorX - root.x();
    float diry = endEffectorY - root.y();

    float len = sqrt(dirx * dirx + diry * diry);
    dirx = dirx / len;
    diry = diry / len;

    float segmentlength = 120;
    float poleVectorX, poleVectorY;
    float disc = segmentlength * segmentlength - len * len / 4;

    if(disc < 0)
    {
        poleVectorX = root.x() + dirx * segmentlength;
        poleVectorY = root.y() + diry * segmentlength;
        endEffectorX = root.x() + dirx * segmentlength * 2;
        endEffectorY = root.y() + diry * segmentlength * 2;
    }
    else
    {
        poleVectorX = root.x() + dirx * len / 2;
        poleVectorY = root.y() + diry * len / 2;
        disc = sqrt(disc);
        disc =- disc; // Make it a negative number
        poleVectorX -= diry * disc;
        poleVectorY += dirx * disc;
    }

    MainWindow::getInstance()->platfromManipulatorLink1->setLine(root.x(), root.y(), poleVectorX, poleVectorY);
    MainWindow::getInstance()->platfromManipulatorLink1Link2Console->setLine(poleVectorX, poleVectorY, poleVectorX - 30, poleVectorY);
    MainWindow::getInstance()->platfromManipulatorLink2->setLine(poleVectorX - 30, poleVectorY, endEffectorX, endEffectorY);
    MainWindow::getInstance()->link1link2Point->setRect(poleVectorX - 36, poleVectorY - 6, 12, 12);
}

void Link2EndPointViewItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    (void)event;
}

#endif // LINK2ENDPOINTVIEWITEM_H
