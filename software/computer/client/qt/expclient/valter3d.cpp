#include "valter3d.h"

Valter3d::Valter3d(QWidget *parent) : QWidget(parent)
{

}

void Valter3d::setValterTrunkRotationY(double angle)
{
    QMetaObject::invokeMethod((QObject*)this->valter3dView->rootObject(), "setValterTrunkRotationY", Q_ARG(QVariant, QVariant::fromValue(angle)));
}

void Valter3d::setValterBodyRotationZ(double angle)
{
    QMetaObject::invokeMethod((QObject*)this->valter3dView->rootObject(), "setValterBodyRotationZ", Q_ARG(QVariant, QVariant::fromValue(angle)));
}

void Valter3d::setLink1ZAngle(double angle)
{
    QMetaObject::invokeMethod((QObject*)this->valter3dView->rootObject(), "setLink1ZAngle", Q_ARG(QVariant, QVariant::fromValue(angle)));
}

void Valter3d::setLink2ZAngle(double angle)
{
    QMetaObject::invokeMethod((QObject*)this->valter3dView->rootObject(), "setLink2ZAngle", Q_ARG(QVariant, QVariant::fromValue(angle)));
}

void Valter3d::setManTiltZAngle(double angle)
{
    QMetaObject::invokeMethod((QObject*)this->valter3dView->rootObject(), "setManTiltZAngle", Q_ARG(QVariant, QVariant::fromValue(angle)));
}
