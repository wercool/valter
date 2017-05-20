#ifndef CREATUREB_H
#define CREATUREB_H


#include <utils/generic-utils.h>

#include <creature/creature.h>

class CreatureB : public Creature
{
public:
    CreatureB(double rx, double ry, const QColor &color);

    void initNN(int hiddenNeuronsNum, int outputNeuronsNum) override;

    // QGraphicsItem interface
public:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

protected:
    void lifeThreadProcess() override;

};

#endif // CREATUREB_H
