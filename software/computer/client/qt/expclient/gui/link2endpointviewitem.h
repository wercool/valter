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

    float rootX = 220;
    float rootY = 110;

    float endEffectorX = this->scenePos().x() + 6;
    float endEffectorY = this->scenePos().y() + 6;

    float dirx = endEffectorX - rootX;
    float diry = endEffectorY - rootY;

    float len = sqrt(dirx * dirx + diry * diry);
    dirx = dirx / len;
    diry = diry / len;

    float segmentlength = 120;
    float poleVectorX, poleVectorY;
    float disc = segmentlength * segmentlength - len * len / 4;

    poleVectorX = rootX + dirx * len / 2;
    poleVectorY = rootY + diry * len / 2;

    disc = sqrt(disc);
    disc =- disc; // Make it a negative number
    poleVectorX -= diry * disc;
    poleVectorY += dirx * disc;

    float dx = poleVectorX - rootX;
    float dy = rootY - poleVectorY;
    float norm = sqrt(dx * dx + dy * dy);
    dx = dx / norm;
    dy = dy / norm;
    float link2PoleX = poleVectorX + dy * 30;
    float link2PoleY = poleVectorY + dx * 30;

    MainWindow::getInstance()->platfromManipulatorLink1->setLine(rootX, rootY, poleVectorX, poleVectorY);

    MainWindow::getInstance()->platfromManipulatorLink1Link2Console->setLine(poleVectorX, poleVectorY, link2PoleX, link2PoleY);
    MainWindow::getInstance()->link1link2Point->setRect(link2PoleX - 6, link2PoleY - 6, 12, 12);

    MainWindow::getInstance()->platfromManipulatorHelper->setLine(poleVectorX, poleVectorY, endEffectorX, endEffectorY);

    MainWindow::getInstance()->platfromManipulatorLink2->setLine(link2PoleX, link2PoleY, endEffectorX, endEffectorY);
}

void Link2EndPointViewItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    (void)event;
}

#endif // LINK2ENDPOINTVIEWITEM_H
