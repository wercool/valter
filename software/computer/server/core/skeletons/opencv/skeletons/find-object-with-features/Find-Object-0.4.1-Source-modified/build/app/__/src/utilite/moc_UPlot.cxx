/****************************************************************************
** Meta object code from reading C++ file 'UPlot.h'
**
** Created: Wed Jun 5 18:44:57 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/utilite/UPlot.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'UPlot.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_UPlotCurve[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      43,   11,   11,   11, 0x0a,
      59,   51,   11,   11, 0x0a,
      86,   76,   11,   11, 0x0a,
     111,  107,   11,   11, 0x0a,
     133,  128,   11,   11, 0x0a,
     156,  154,   11,   11, 0x0a,
     176,  172,   11,   11, 0x0a,
     198,  154,   11,   11, 0x0a,
     216,  128,   11,   11, 0x0a,
     254,  248,   11,   11, 0x0a,
     298,  295,   11,   11, 0x0a,
     324,  295,   11,   11, 0x0a,
     348,  295,   11,   11, 0x0a,
     378,  295,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_UPlotCurve[] = {
    "UPlotCurve\0\0dataChanged(const UPlotCurve*)\0"
    "clear()\0visible\0setVisible(bool)\0"
    "increment\0setXIncrement(float)\0val\0"
    "setXStart(float)\0data\0addValue(UPlotItem*)\0"
    "y\0addValue(float)\0x,y\0addValue(float,float)\0"
    "addValue(QString)\0addValues(QVector<UPlotItem*>&)\0"
    "xs,ys\0addValues(QVector<float>,QVector<float>)\0"
    "ys\0addValues(QVector<float>)\0"
    "addValues(QVector<int>)\0"
    "addValues(std::vector<float>)\0"
    "addValues(std::vector<int>)\0"
};

const QMetaObject UPlotCurve::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_UPlotCurve,
      qt_meta_data_UPlotCurve, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlotCurve::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlotCurve::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlotCurve::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlotCurve))
        return static_cast<void*>(const_cast< UPlotCurve*>(this));
    return QObject::qt_metacast(_clname);
}

int UPlotCurve::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: dataChanged((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        case 1: clear(); break;
        case 2: setVisible((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: setXIncrement((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: setXStart((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: addValue((*reinterpret_cast< UPlotItem*(*)>(_a[1]))); break;
        case 6: addValue((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 7: addValue((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 8: addValue((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 9: addValues((*reinterpret_cast< QVector<UPlotItem*>(*)>(_a[1]))); break;
        case 10: addValues((*reinterpret_cast< const QVector<float>(*)>(_a[1])),(*reinterpret_cast< const QVector<float>(*)>(_a[2]))); break;
        case 11: addValues((*reinterpret_cast< const QVector<float>(*)>(_a[1]))); break;
        case 12: addValues((*reinterpret_cast< const QVector<int>(*)>(_a[1]))); break;
        case 13: addValues((*reinterpret_cast< const std::vector<float>(*)>(_a[1]))); break;
        case 14: addValues((*reinterpret_cast< const std::vector<int>(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void UPlotCurve::dataChanged(const UPlotCurve * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
static const uint qt_meta_data_UPlotCurveThreshold[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      31,   21,   20,   20, 0x0a,
      63,   51,   20,   20, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_UPlotCurveThreshold[] = {
    "UPlotCurveThreshold\0\0threshold\0"
    "setThreshold(float)\0orientation\0"
    "setOrientation(Qt::Orientation)\0"
};

const QMetaObject UPlotCurveThreshold::staticMetaObject = {
    { &UPlotCurve::staticMetaObject, qt_meta_stringdata_UPlotCurveThreshold,
      qt_meta_data_UPlotCurveThreshold, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlotCurveThreshold::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlotCurveThreshold::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlotCurveThreshold::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlotCurveThreshold))
        return static_cast<void*>(const_cast< UPlotCurveThreshold*>(this));
    return UPlotCurve::qt_metacast(_clname);
}

int UPlotCurveThreshold::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = UPlotCurve::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: setThreshold((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: setOrientation((*reinterpret_cast< Qt::Orientation(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}
static const uint qt_meta_data_UPlotLegendItem[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      17,   16,   16,   16, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_UPlotLegendItem[] = {
    "UPlotLegendItem\0\0legendItemRemoved(const UPlotCurve*)\0"
};

const QMetaObject UPlotLegendItem::staticMetaObject = {
    { &QPushButton::staticMetaObject, qt_meta_stringdata_UPlotLegendItem,
      qt_meta_data_UPlotLegendItem, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlotLegendItem::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlotLegendItem::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlotLegendItem::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlotLegendItem))
        return static_cast<void*>(const_cast< UPlotLegendItem*>(this));
    return QPushButton::qt_metacast(_clname);
}

int UPlotLegendItem::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QPushButton::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: legendItemRemoved((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void UPlotLegendItem::legendItemRemoved(const UPlotCurve * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
static const uint qt_meta_data_UPlotLegend[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      19,   13,   12,   12, 0x05,
      70,   56,   12,   12, 0x05,

 // slots: signature, parameters, type, tag, flags
     112,   13,   12,   12, 0x0a,
     148,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_UPlotLegend[] = {
    "UPlotLegend\0\0curve\0"
    "legendItemRemoved(const UPlotCurve*)\0"
    "curve,toggled\0legendItemToggled(const UPlotCurve*,bool)\0"
    "removeLegendItem(const UPlotCurve*)\0"
    "redirectToggled(bool)\0"
};

const QMetaObject UPlotLegend::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_UPlotLegend,
      qt_meta_data_UPlotLegend, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlotLegend::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlotLegend::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlotLegend::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlotLegend))
        return static_cast<void*>(const_cast< UPlotLegend*>(this));
    return QWidget::qt_metacast(_clname);
}

int UPlotLegend::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: legendItemRemoved((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        case 1: legendItemToggled((*reinterpret_cast< const UPlotCurve*(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: removeLegendItem((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        case 3: redirectToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void UPlotLegend::legendItemRemoved(const UPlotCurve * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void UPlotLegend::legendItemToggled(const UPlotCurve * _t1, bool _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
static const uint qt_meta_data_UOrientableLabel[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

static const char qt_meta_stringdata_UOrientableLabel[] = {
    "UOrientableLabel\0"
};

const QMetaObject UOrientableLabel::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_UOrientableLabel,
      qt_meta_data_UOrientableLabel, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UOrientableLabel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UOrientableLabel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UOrientableLabel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UOrientableLabel))
        return static_cast<void*>(const_cast< UOrientableLabel*>(this));
    return QLabel::qt_metacast(_clname);
}

int UOrientableLabel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
static const uint qt_meta_data_UPlot[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,    7,    6,    6, 0x0a,
      56,   44,    6,    6, 0x0a,
      90,    6,    6,    6, 0x0a,
     103,    6,    6,    6, 0x0a,
     115,    6,    6,    6, 0x08,
     131,    7,    6,    6, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_UPlot[] = {
    "UPlot\0\0curve\0removeCurve(const UPlotCurve*)\0"
    "curve,shown\0showCurve(const UPlotCurve*,bool)\0"
    "updateAxis()\0clearData()\0captureScreen()\0"
    "updateAxis(const UPlotCurve*)\0"
};

const QMetaObject UPlot::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_UPlot,
      qt_meta_data_UPlot, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlot::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlot::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlot::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlot))
        return static_cast<void*>(const_cast< UPlot*>(this));
    return QWidget::qt_metacast(_clname);
}

int UPlot::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: removeCurve((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        case 1: showCurve((*reinterpret_cast< const UPlotCurve*(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: updateAxis(); break;
        case 3: clearData(); break;
        case 4: captureScreen(); break;
        case 5: updateAxis((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
