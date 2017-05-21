#ifndef CREASTUREC_H
#define CREASTUREC_H

#include <utils/generic-utils.h>

#include <creature/creature.h>

class CreatureC : public Creature
{
public:
    CreatureC(double rx, double ry, const QColor &color);

    void initNN(int hiddenNeuronsNum, int outputNeuronsNum) override;

    // QGraphicsItem interface
public:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

protected:
    void lifeThreadProcess() override;

private:
    double prevDistanceToTargetPoint = -1000.0;

};

#endif // CREASTUREC_H
