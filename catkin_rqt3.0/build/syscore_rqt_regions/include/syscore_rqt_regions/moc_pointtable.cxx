/****************************************************************************
** Meta object code from reading C++ file 'pointtable.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/syscore_rqt_regions/include/syscore_rqt_regions/pointtable.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pointtable.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PointTable[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,
      27,   11,   11,   11, 0x05,
      48,   11,   11,   11, 0x05,
      66,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      94,   11,   11,   11, 0x0a,
     117,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PointTable[] = {
    "PointTable\0\0del_point(int)\0"
    "painter_Mark(QPoint)\0insert_point(int)\0"
    "request_enable_replaceBtn()\0"
    "slot_menu_del_action()\0slot_menu_insert_action()\0"
};

void PointTable::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PointTable *_t = static_cast<PointTable *>(_o);
        switch (_id) {
        case 0: _t->del_point((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->painter_Mark((*reinterpret_cast< QPoint(*)>(_a[1]))); break;
        case 2: _t->insert_point((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->request_enable_replaceBtn(); break;
        case 4: _t->slot_menu_del_action(); break;
        case 5: _t->slot_menu_insert_action(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PointTable::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PointTable::staticMetaObject = {
    { &QTableWidget::staticMetaObject, qt_meta_stringdata_PointTable,
      qt_meta_data_PointTable, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PointTable::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PointTable::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PointTable::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PointTable))
        return static_cast<void*>(const_cast< PointTable*>(this));
    return QTableWidget::qt_metacast(_clname);
}

int PointTable::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTableWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void PointTable::del_point(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PointTable::painter_Mark(QPoint _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void PointTable::insert_point(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void PointTable::request_enable_replaceBtn()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}
QT_END_MOC_NAMESPACE
