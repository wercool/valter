#ifndef CREATURE_H
#define CREATURE_H

#include <creature/creaturegi.h>

class Creature : public CreatureGI
{
public:
    Creature(double rx, double ry, const QColor &color);
    // QGraphicsItem interface
public:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
};

#endif // CREATURE_H
