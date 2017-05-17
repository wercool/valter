#include "creature.h"

Creature::Creature(double rx, double ry, const QColor &color): CreatureGI(rx, ry, color)
{

}

void Creature::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    CreatureGI::paint(painter, option, widget);

    painter->setPen(QPen(Qt::black, 0.5));
    painter->setBrush(fillColor);
    painter->drawEllipse(QPointF(0.0, 0.0), rx, rx);
}
