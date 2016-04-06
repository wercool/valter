#include "valter3d.h"

Valter3d::Valter3d(QWidget *parent) : QWidget(parent)
{

}

void Valter3d::setValterGroupRotationY(double angle)
{
    QMetaObject::invokeMethod((QObject*)this->valter3dView->rootObject(), "setValterGroupRotationY", Q_ARG(QVariant, QVariant::fromValue(angle)));
}
