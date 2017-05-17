#include "creaturegi.h"

CreatureGI::CreatureGI(double rx, double ry, const QColor &color)
{
    this->rx = rx;
    this->ry = ry;
    this->color = color;

    this->rxI = rx;
    this->ryI = ry;

    setTransformOriginPoint(QPoint(0, 0));

    setFlags(ItemIsSelectable | ItemIsMovable);
    setAcceptHoverEvents(true);
}

QRectF CreatureGI::boundingRect() const
{
    return QRectF(-rx, -ry, rx * 2 , ry * 2);
}

void CreatureGI::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);
    Q_UNUSED(painter);

    fillColor = (option->state & QStyle::State_Selected) ? color.darker(175) : color;

    if (option->state & QStyle::State_MouseOver)
    {
        fillColor = fillColor.light(150);
    }

//    painter->setPen(QPen(Qt::red, 0.5));
//    painter->setBrush(QBrush(Qt::transparent));
//    painter->drawRect(boundingRect());
}

double CreatureGI::getX() const
{
    return x;
}

int CreatureGI::getIntX() const
{
    return (int) x;
}

void CreatureGI::setX(double value)
{
    x = value;
}

double CreatureGI::getY() const
{
    return y;
}

int CreatureGI::getIntY() const
{
    return (int) y;
}

void CreatureGI::setY(double value)
{
    y = value;
}

double CreatureGI::getA() const
{
    return a;
}

void CreatureGI::setA(double value)
{
    a = value;
}

double CreatureGI::getRx() const
{
    return rx;
}

double CreatureGI::getRy() const
{
    return ry;
}

void CreatureGI::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mousePressEvent(event);
    update();
}

void CreatureGI::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    x = QGraphicsItem::x();
    y = QGraphicsItem::y();
    QGraphicsItem::mouseMoveEvent(event);
    update();
}

void CreatureGI::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mouseReleaseEvent(event);
    update();
}

