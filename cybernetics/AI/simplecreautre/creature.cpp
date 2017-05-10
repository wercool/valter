#include "creature.h"

#include <QtWidgets>

Creature::Creature(int x, int y, const QColor &color)
{
    this->x = x;
    this->y = y;
    this->color = color;

    setFlags(ItemIsSelectable);
    setAcceptHoverEvents(true);
}

void Creature::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);

    painter->setBrush(QBrush(color));
    painter->drawEllipse(x, y, 20, 20);
}

QRectF Creature::boundingRect() const
{
    return QRectF(0, 0, 40, 40);
}
