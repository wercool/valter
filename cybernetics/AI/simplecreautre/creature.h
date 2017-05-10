#ifndef CREATURE_H
#define CREATURE_H

#include <QColor>
#include <QGraphicsItem>

class Creature : public QGraphicsItem
{
public:
    Creature(int x, int y, const QColor &color);

private:
    int x;
    int y;
    QColor color;

    // QGraphicsItem interface
public:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const;
};

#endif // CREATURE_H
