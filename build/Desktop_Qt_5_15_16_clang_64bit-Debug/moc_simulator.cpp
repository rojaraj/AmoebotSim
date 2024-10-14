/****************************************************************************
** Meta object code from reading C++ file 'simulator.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.16)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../core/simulator.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'simulator.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.16. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Simulator_t {
    QByteArrayData data[24];
    char stringdata0[272];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Simulator_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Simulator_t qt_meta_stringdata_Simulator = {
    {
QT_MOC_LITERAL(0, 0, 9), // "Simulator"
QT_MOC_LITERAL(1, 10, 13), // "systemChanged"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 23), // "std::shared_ptr<System>"
QT_MOC_LITERAL(4, 49, 7), // "_system"
QT_MOC_LITERAL(5, 57, 19), // "stepDurationChanged"
QT_MOC_LITERAL(6, 77, 2), // "ms"
QT_MOC_LITERAL(7, 80, 14), // "saveScreenshot"
QT_MOC_LITERAL(8, 95, 8), // "filePath"
QT_MOC_LITERAL(9, 104, 7), // "started"
QT_MOC_LITERAL(10, 112, 7), // "stopped"
QT_MOC_LITERAL(11, 120, 5), // "start"
QT_MOC_LITERAL(12, 126, 4), // "stop"
QT_MOC_LITERAL(13, 131, 4), // "step"
QT_MOC_LITERAL(14, 136, 17), // "stepForParticleAt"
QT_MOC_LITERAL(15, 154, 4), // "Node"
QT_MOC_LITERAL(16, 159, 4), // "node"
QT_MOC_LITERAL(17, 164, 15), // "setStepDuration"
QT_MOC_LITERAL(18, 180, 19), // "runUntilTermination"
QT_MOC_LITERAL(19, 200, 12), // "numParticles"
QT_MOC_LITERAL(20, 213, 16), // "numImmoParticles"
QT_MOC_LITERAL(21, 230, 7), // "metrics"
QT_MOC_LITERAL(22, 238, 13), // "exportMetrics"
QT_MOC_LITERAL(23, 252, 19) // "saveScreenshotSetup"

    },
    "Simulator\0systemChanged\0\0"
    "std::shared_ptr<System>\0_system\0"
    "stepDurationChanged\0ms\0saveScreenshot\0"
    "filePath\0started\0stopped\0start\0stop\0"
    "step\0stepForParticleAt\0Node\0node\0"
    "setStepDuration\0runUntilTermination\0"
    "numParticles\0numImmoParticles\0metrics\0"
    "exportMetrics\0saveScreenshotSetup"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Simulator[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   94,    2, 0x06 /* Public */,
       5,    1,   97,    2, 0x06 /* Public */,
       7,    1,  100,    2, 0x06 /* Public */,
       9,    0,  103,    2, 0x06 /* Public */,
      10,    0,  104,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    0,  105,    2, 0x0a /* Public */,
      12,    0,  106,    2, 0x0a /* Public */,
      13,    0,  107,    2, 0x0a /* Public */,
      14,    1,  108,    2, 0x0a /* Public */,
      17,    1,  111,    2, 0x0a /* Public */,
      18,    0,  114,    2, 0x0a /* Public */,
      19,    0,  115,    2, 0x0a /* Public */,
      20,    0,  116,    2, 0x0a /* Public */,
      21,    0,  117,    2, 0x0a /* Public */,
      22,    0,  118,    2, 0x0a /* Public */,
      23,    1,  119,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void, QMetaType::QString,    8,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void,
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::QVariant,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    8,

       0        // eod
};

void Simulator::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Simulator *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->systemChanged((*reinterpret_cast< std::shared_ptr<System>(*)>(_a[1]))); break;
        case 1: _t->stepDurationChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->saveScreenshot((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->started(); break;
        case 4: _t->stopped(); break;
        case 5: _t->start(); break;
        case 6: _t->stop(); break;
        case 7: _t->step(); break;
        case 8: _t->stepForParticleAt((*reinterpret_cast< Node(*)>(_a[1]))); break;
        case 9: _t->setStepDuration((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->runUntilTermination(); break;
        case 11: { int _r = _t->numParticles();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 12: { int _r = _t->numImmoParticles();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 13: { QVariant _r = _t->metrics();
            if (_a[0]) *reinterpret_cast< QVariant*>(_a[0]) = std::move(_r); }  break;
        case 14: _t->exportMetrics(); break;
        case 15: _t->saveScreenshotSetup((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Simulator::*)(std::shared_ptr<System> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Simulator::systemChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Simulator::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Simulator::stepDurationChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Simulator::*)(const QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Simulator::saveScreenshot)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (Simulator::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Simulator::started)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (Simulator::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Simulator::stopped)) {
                *result = 4;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Simulator::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_Simulator.data,
    qt_meta_data_Simulator,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Simulator::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Simulator::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Simulator.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Simulator::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}

// SIGNAL 0
void Simulator::systemChanged(std::shared_ptr<System> _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Simulator::stepDurationChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Simulator::saveScreenshot(const QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Simulator::started()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void Simulator::stopped()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
