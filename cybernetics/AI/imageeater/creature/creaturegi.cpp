#include "creaturegi.h"

CreatureGI::CreatureGI(double rx, double ry, const QColor &color)
{
    this->rx = rx;
    this->ry = ry;
    this->color = color;
}

QRectF CreatureGI::boundingRect() const
{
    return QRectF(-rx, -ry, rx * 2 , ry * 2);
}

void CreatureGI::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);
    Q_UNUSED(painter);

    fillColor = (option->state & QStyle::State_Selected) ? color.darker(175) : color;

    if (option->state & QStyle::State_MouseOver)
    {
        fillColor = fillColor.light(150);
    }
}
