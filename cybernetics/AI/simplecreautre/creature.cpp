#include "creature.h"

#include <QtWidgets>

// QGraphicsItem interface
Creature::Creature(double w, double h, const QColor &color)
{
    this->w = w;
    this->h = h;
    this->color = color;

    setTransformOriginPoint(QPoint(w / 2, h / 2));

    setFlags(ItemIsSelectable | ItemIsMovable);
    setAcceptHoverEvents(true);
}

void Creature::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);
    Q_UNUSED(painter);

    fillColor = (option->state & QStyle::State_Selected) ? color.dark(175) : color;
    if (option->state & QStyle::State_MouseOver)
    {
        fillColor = fillColor.light(150);
    }
}

QRectF Creature::boundingRect() const
{
    return QRectF(0, 0, w, h);
}

void Creature::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mousePressEvent(event);
    update();
}

void Creature::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    x = QGraphicsItem::x();
    y = QGraphicsItem::y();
    QGraphicsItem::mouseMoveEvent(event);
}

void Creature::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mouseReleaseEvent(event);
    update();
}

void Creature::lifeThreadProcess()
{
    // stub
}
