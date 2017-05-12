#include "dline.h"

DLine::DLine(const QColor &color)
{
    setPos(QPointF(0.0, 0.0));
    this->color = color;
}

void DLine::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);
    Q_UNUSED(option);

    painter->setPen(QPen(this->color, 0.5));
    painter->drawLine(x1, y1, x2, y2);
}

QRectF DLine::boundingRect() const
{
    return QRectF(0, 0, 0, 0);
}
