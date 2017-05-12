#include "creature.h"

#include <QtWidgets>

// QGraphicsItem interface
Creature::Creature(double rx, double ry, const QColor &color)
{
    this->rx = rx;
    this->ry = ry;
    this->color = color;

    setTransformOriginPoint(QPoint(0, 0));

    setFlags(ItemIsSelectable | ItemIsMovable);
    setAcceptHoverEvents(true);
}

void Creature::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);
    Q_UNUSED(painter);

    fillColor = (option->state & QStyle::State_Selected) ? color.darker(175) : color;

    if (option->state & QStyle::State_MouseOver)
    {
        fillColor = fillColor.light(150);
    }
}

QRectF Creature::boundingRect() const
{
    return QRectF(-rx, -ry, rx * 2 , ry * 2);
}

void Creature::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mousePressEvent(event);
    update();
}

void Creature::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    x = QGraphicsItem::x();
    y = QGraphicsItem::y();
    QGraphicsItem::mouseMoveEvent(event);
}

void Creature::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mouseReleaseEvent(event);
    update();
}

double Creature::getVitality() const
{
    return vitality;
}

void Creature::setVitality(double value)
{
    vitality = value;
}

int Creature::getDLifeTime() const
{
    return dLifeTime;
}

void Creature::setDLifeTime(int value)
{
    dLifeTime = value;
}

cv::Mat *Creature::getEnvMatMap() const
{
    return envMatMap;
}

void Creature::setEnvMatMap(cv::Mat *value)
{
    envMatMap = value;
}

bool Creature::getLifeSuspended() const
{
    return lifeSuspended;
}

void Creature::setLifeSuspended(bool value)
{
    lifeSuspended = value;
}

double Creature::getA() const
{
    return a;
}

void Creature::setA(double value)
{
    a = value;
}

double Creature::getY() const
{
    return y;
}

int Creature::getIntY() const
{
    return (int) y;
}

void Creature::setY(double value)
{
    y = value;
}

double Creature::getX() const
{
    return x;
}

int Creature::getIntX() const
{
    return (int) x;
}

void Creature::setX(double value)
{
    x = value;
}

void Creature::lifeThreadProcess()
{
    // stub
}
