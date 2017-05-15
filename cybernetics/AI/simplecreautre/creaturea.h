#ifndef CREATUREA_H
#define CREATUREA_H

#include <QPainter>
#include <QColor>
#include <QGraphicsItem>
#include <QStyleOptionGraphicsItem>

#include <creature.h>
#include <receptor.h>
#include <neuralnetwork.h>
#include <neuron.h>

class CreatureA : public Creature
{
public:
    CreatureA(double rx, double ry, const QColor &color, bool initNeuralNetwork = true);

public:
    // QGraphicsItem interface
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    vector<Receptor *> getReceptors() const;

private:
    vector<Receptor *> receptors;

    int hiddenNeuronsNum = 5;
    int outputNeuronsNum = 3;

    void lifeThreadProcess() override;
};

#endif // CREATUREA_H
