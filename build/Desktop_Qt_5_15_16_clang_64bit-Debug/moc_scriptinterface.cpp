/****************************************************************************
** Meta object code from reading C++ file 'scriptinterface.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.16)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../script/scriptinterface.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'scriptinterface.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.16. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ScriptInterface_t {
    QByteArrayData data[31];
    char stringdata0[291];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ScriptInterface_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ScriptInterface_t qt_meta_stringdata_ScriptInterface = {
    {
QT_MOC_LITERAL(0, 0, 15), // "ScriptInterface"
QT_MOC_LITERAL(1, 16, 3), // "log"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 3), // "msg"
QT_MOC_LITERAL(4, 25, 5), // "error"
QT_MOC_LITERAL(5, 31, 9), // "runScript"
QT_MOC_LITERAL(6, 41, 14), // "scriptFilePath"
QT_MOC_LITERAL(7, 56, 11), // "writeToFile"
QT_MOC_LITERAL(8, 68, 8), // "filePath"
QT_MOC_LITERAL(9, 77, 4), // "text"
QT_MOC_LITERAL(10, 82, 4), // "step"
QT_MOC_LITERAL(11, 87, 15), // "setStepDuration"
QT_MOC_LITERAL(12, 103, 2), // "ms"
QT_MOC_LITERAL(13, 106, 19), // "runUntilTermination"
QT_MOC_LITERAL(14, 126, 15), // "getNumParticles"
QT_MOC_LITERAL(15, 142, 19), // "getNumImmoParticles"
QT_MOC_LITERAL(16, 162, 13), // "exportMetrics"
QT_MOC_LITERAL(17, 176, 9), // "getMetric"
QT_MOC_LITERAL(18, 186, 4), // "name"
QT_MOC_LITERAL(19, 191, 7), // "history"
QT_MOC_LITERAL(20, 199, 13), // "setWindowSize"
QT_MOC_LITERAL(21, 213, 5), // "width"
QT_MOC_LITERAL(22, 219, 6), // "height"
QT_MOC_LITERAL(23, 226, 7), // "focusOn"
QT_MOC_LITERAL(24, 234, 1), // "x"
QT_MOC_LITERAL(25, 236, 1), // "y"
QT_MOC_LITERAL(26, 238, 7), // "setZoom"
QT_MOC_LITERAL(27, 246, 4), // "zoom"
QT_MOC_LITERAL(28, 251, 14), // "saveScreenshot"
QT_MOC_LITERAL(29, 266, 14), // "filmSimulation"
QT_MOC_LITERAL(30, 281, 9) // "stepLimit"

    },
    "ScriptInterface\0log\0\0msg\0error\0runScript\0"
    "scriptFilePath\0writeToFile\0filePath\0"
    "text\0step\0setStepDuration\0ms\0"
    "runUntilTermination\0getNumParticles\0"
    "getNumImmoParticles\0exportMetrics\0"
    "getMetric\0name\0history\0setWindowSize\0"
    "width\0height\0focusOn\0x\0y\0setZoom\0zoom\0"
    "saveScreenshot\0filmSimulation\0stepLimit"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ScriptInterface[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      20,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,  114,    2, 0x0a /* Public */,
       1,    1,  119,    2, 0x2a /* Public | MethodCloned */,
       5,    1,  122,    2, 0x0a /* Public */,
       7,    2,  125,    2, 0x0a /* Public */,
      10,    0,  130,    2, 0x0a /* Public */,
      11,    1,  131,    2, 0x0a /* Public */,
      13,    0,  134,    2, 0x0a /* Public */,
      14,    0,  135,    2, 0x0a /* Public */,
      15,    0,  136,    2, 0x0a /* Public */,
      16,    0,  137,    2, 0x0a /* Public */,
      17,    2,  138,    2, 0x0a /* Public */,
      17,    1,  143,    2, 0x2a /* Public | MethodCloned */,
      20,    2,  146,    2, 0x0a /* Public */,
      20,    1,  151,    2, 0x2a /* Public | MethodCloned */,
      20,    0,  154,    2, 0x2a /* Public | MethodCloned */,
      23,    2,  155,    2, 0x0a /* Public */,
      26,    1,  160,    2, 0x0a /* Public */,
      28,    1,  163,    2, 0x0a /* Public */,
      28,    0,  166,    2, 0x2a /* Public | MethodCloned */,
      29,    2,  167,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Bool,    3,    4,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    8,    9,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   12,
    QMetaType::Void,
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::Void,
    QMetaType::QVariant, QMetaType::QString, QMetaType::Bool,   18,   19,
    QMetaType::QVariant, QMetaType::QString,   18,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   21,   22,
    QMetaType::Void, QMetaType::Int,   21,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   24,   25,
    QMetaType::Void, QMetaType::Float,   27,
    QMetaType::Void, QMetaType::QString,    8,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    8,   30,

       0        // eod
};

void ScriptInterface::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ScriptInterface *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->log((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 1: _t->log((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->runScript((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->writeToFile((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 4: _t->step(); break;
        case 5: _t->setStepDuration((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 6: _t->runUntilTermination(); break;
        case 7: { int _r = _t->getNumParticles();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 8: { int _r = _t->getNumImmoParticles();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 9: _t->exportMetrics(); break;
        case 10: { QVariant _r = _t->getMetric((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< QVariant*>(_a[0]) = std::move(_r); }  break;
        case 11: { QVariant _r = _t->getMetric((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QVariant*>(_a[0]) = std::move(_r); }  break;
        case 12: _t->setWindowSize((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 13: _t->setWindowSize((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->setWindowSize(); break;
        case 15: _t->focusOn((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 16: _t->setZoom((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 17: _t->saveScreenshot((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 18: _t->saveScreenshot(); break;
        case 19: _t->filmSimulation((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< const int(*)>(_a[2]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ScriptInterface::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ScriptInterface.data,
    qt_meta_data_ScriptInterface,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ScriptInterface::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ScriptInterface::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ScriptInterface.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ScriptInterface::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 20)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 20;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 20)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 20;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
