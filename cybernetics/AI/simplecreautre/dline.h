#ifndef DLINE_H
#define DLINE_H

#include <QPainter>
#include <QColor>
#include <QGraphicsItem>

class DLine : public QGraphicsItem
{
public:
    DLine(const QColor &color);

    // QGraphicsItem interface
public:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const;

    double x1, x2, y1, y2 = 0.0;

private:
    QColor color;
};

#endif // DLINE_H
