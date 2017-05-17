#ifndef CREATUREGI_H
#define CREATUREGI_H

#include <QtWidgets>
#include <QPainter>
#include <QColor>
#include <QGraphicsItem>

class CreatureGI : public QGraphicsItem
{
public:
    CreatureGI(double rx, double ry, const QColor &color);

    // QGraphicsItem interface
public:
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

protected:
    double rx = 30.0;
    double ry = 30.0;
    QColor color;
    QColor fillColor;
};

#endif // CREATUREGI_H
