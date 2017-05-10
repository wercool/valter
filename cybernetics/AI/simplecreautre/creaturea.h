#ifndef CREATUREA_H
#define CREATUREA_H

#include <QPainter>
#include <QColor>
#include <QGraphicsItem>
#include <QStyleOptionGraphicsItem>

#include <creature.h>
#include <neuralnetwork.h>
#include <neuron.h>

class CreatureA : public Creature
{
public:
    CreatureA(int w, int h, const QColor &color);

public:
    // QGraphicsItem interface
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
};

#endif // CREATUREA_H
