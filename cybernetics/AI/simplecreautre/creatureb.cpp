#include "creatureb.h"

CreatureB::CreatureB(double w, double h, const QColor &color) : Creature(w, h, color)
{
    lifeThread = new std::thread(&CreatureB::lifeThreadProcess, this);
}


void CreatureB::lifeThreadProcess()
{
    while (true)
    {
        qDebug("BBBBBBB");
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


void CreatureB::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Creature::paint(painter, option, widget);

    painter->setPen(QPen(Qt::gray, 0.5));
    painter->setBrush(fillColor);
    painter->drawEllipse(0, 0, w, h);

    painter->setPen(QPen(Qt::red, 2.0));
    painter->drawLine(w/2, h + 1.0, w/2, h + 100.0);
}
