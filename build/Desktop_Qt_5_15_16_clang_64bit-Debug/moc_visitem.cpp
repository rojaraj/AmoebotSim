/****************************************************************************
** Meta object code from reading C++ file 'visitem.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.16)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../ui/visitem.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'visitem.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.16. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_VisItem_t {
    QByteArrayData data[23];
    char stringdata0[238];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_VisItem_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_VisItem_t qt_meta_stringdata_VisItem = {
    {
QT_MOC_LITERAL(0, 0, 7), // "VisItem"
QT_MOC_LITERAL(1, 8, 17), // "stepForParticleAt"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 4), // "Node"
QT_MOC_LITERAL(4, 32, 4), // "node"
QT_MOC_LITERAL(5, 37, 15), // "inspectParticle"
QT_MOC_LITERAL(6, 53, 4), // "text"
QT_MOC_LITERAL(7, 58, 13), // "systemChanged"
QT_MOC_LITERAL(8, 72, 23), // "std::shared_ptr<System>"
QT_MOC_LITERAL(9, 96, 7), // "_system"
QT_MOC_LITERAL(10, 104, 19), // "focusOnCenterOfMass"
QT_MOC_LITERAL(11, 124, 13), // "setWindowSize"
QT_MOC_LITERAL(12, 138, 5), // "width"
QT_MOC_LITERAL(13, 144, 6), // "height"
QT_MOC_LITERAL(14, 151, 7), // "focusOn"
QT_MOC_LITERAL(15, 159, 7), // "setZoom"
QT_MOC_LITERAL(16, 167, 4), // "zoom"
QT_MOC_LITERAL(17, 172, 14), // "saveScreenshot"
QT_MOC_LITERAL(18, 187, 8), // "filePath"
QT_MOC_LITERAL(19, 196, 10), // "initialize"
QT_MOC_LITERAL(20, 207, 5), // "paint"
QT_MOC_LITERAL(21, 213, 12), // "deinitialize"
QT_MOC_LITERAL(22, 226, 11) // "sizeChanged"

    },
    "VisItem\0stepForParticleAt\0\0Node\0node\0"
    "inspectParticle\0text\0systemChanged\0"
    "std::shared_ptr<System>\0_system\0"
    "focusOnCenterOfMass\0setWindowSize\0"
    "width\0height\0focusOn\0setZoom\0zoom\0"
    "saveScreenshot\0filePath\0initialize\0"
    "paint\0deinitialize\0sizeChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_VisItem[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   74,    2, 0x06 /* Public */,
       5,    1,   77,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   80,    2, 0x0a /* Public */,
      10,    0,   83,    2, 0x0a /* Public */,
      11,    2,   84,    2, 0x0a /* Public */,
      14,    1,   89,    2, 0x0a /* Public */,
      15,    1,   92,    2, 0x0a /* Public */,
      17,    1,   95,    2, 0x0a /* Public */,
      19,    0,   98,    2, 0x09 /* Protected */,
      20,    0,   99,    2, 0x09 /* Protected */,
      21,    0,  100,    2, 0x09 /* Protected */,
      22,    2,  101,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,    6,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   12,   13,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::Double,   16,
    QMetaType::Void, QMetaType::QString,   18,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   12,   13,

       0        // eod
};

void VisItem::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<VisItem *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->stepForParticleAt((*reinterpret_cast< Node(*)>(_a[1]))); break;
        case 1: _t->inspectParticle((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->systemChanged((*reinterpret_cast< std::shared_ptr<System>(*)>(_a[1]))); break;
        case 3: _t->focusOnCenterOfMass(); break;
        case 4: _t->setWindowSize((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 5: _t->focusOn((*reinterpret_cast< Node(*)>(_a[1]))); break;
        case 6: _t->setZoom((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->saveScreenshot((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->initialize(); break;
        case 9: _t->paint(); break;
        case 10: _t->deinitialize(); break;
        case 11: _t->sizeChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (VisItem::*)(Node );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VisItem::stepForParticleAt)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (VisItem::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VisItem::inspectParticle)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject VisItem::staticMetaObject = { {
    QMetaObject::SuperData::link<GLItem::staticMetaObject>(),
    qt_meta_stringdata_VisItem.data,
    qt_meta_data_VisItem,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *VisItem::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *VisItem::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_VisItem.stringdata0))
        return static_cast<void*>(this);
    return GLItem::qt_metacast(_clname);
}

int VisItem::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GLItem::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void VisItem::stepForParticleAt(Node _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void VisItem::inspectParticle(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
