#ifndef CREATUREB_H
#define CREATUREB_H

#include <QPainter>
#include <QColor>
#include <QGraphicsItem>
#include <QStyleOptionGraphicsItem>

#include <creature.h>

class CreatureB : public Creature
{
public:
    CreatureB(double w, double h, const QColor &color);

public:
    // QGraphicsItem interface
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

private:
    void lifeThreadProcess() override;
};

#endif // CREATUREB_H
