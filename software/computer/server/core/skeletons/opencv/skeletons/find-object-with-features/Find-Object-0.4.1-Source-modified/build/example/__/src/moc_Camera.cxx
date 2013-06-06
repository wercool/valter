/****************************************************************************
** Meta object code from reading C++ file 'Camera.h'
**
** Created: Wed Jun 5 18:46:06 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/Camera.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Camera.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Camera[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      14,    8,    7,    7, 0x05,

 // slots: signature, parameters, type, tag, flags
      37,    7,    7,    7, 0x0a,
      55,    7,    7,    7, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Camera[] = {
    "Camera\0\0image\0imageReceived(cv::Mat)\0"
    "updateImageRate()\0takeImage()\0"
};

const QMetaObject Camera::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Camera,
      qt_meta_data_Camera, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Camera::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Camera::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Camera::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Camera))
        return static_cast<void*>(const_cast< Camera*>(this));
    return QObject::qt_metacast(_clname);
}

int Camera::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: imageReceived((*reinterpret_cast< const cv::Mat(*)>(_a[1]))); break;
        case 1: updateImageRate(); break;
        case 2: takeImage(); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void Camera::imageReceived(const cv::Mat & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
