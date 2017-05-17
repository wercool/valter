#ifndef CREATUREA_H
#define CREATUREA_H

#include <utils/generic-utils.h>

#include <creature/creature.h>

class CreatureA : public Creature
{
public:
    CreatureA(double rx, double ry, const QColor &color);

    void initNN(int hiddenNeuronsNum, int outputNeuronsNum) override;

    // QGraphicsItem interface
public:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

protected:
    void lifeThreadProcess() override;

private:
    int dLifeTime = 10;
};

#endif // CREATUREA_H
