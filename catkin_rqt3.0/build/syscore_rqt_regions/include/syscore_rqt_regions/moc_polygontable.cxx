/****************************************************************************
** Meta object code from reading C++ file 'polygontable.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/syscore_rqt_regions/include/syscore_rqt_regions/polygontable.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'polygontable.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PolygonTable[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   14,   13,   13, 0x05,
      55,   13,   13,   13, 0x05,
      72,   13,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
      92,   13,   13,   13, 0x0a,
     115,   13,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PolygonTable[] = {
    "PolygonTable\0\0,\0painter_MarkPolygon(QPolygon,QPolygon)\0"
    "request_Update()\0request_EditTable()\0"
    "slot_menu_del_action()\0slot_menu_edit_action()\0"
};

void PolygonTable::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PolygonTable *_t = static_cast<PolygonTable *>(_o);
        switch (_id) {
        case 0: _t->painter_MarkPolygon((*reinterpret_cast< QPolygon(*)>(_a[1])),(*reinterpret_cast< QPolygon(*)>(_a[2]))); break;
        case 1: _t->request_Update(); break;
        case 2: _t->request_EditTable(); break;
        case 3: _t->slot_menu_del_action(); break;
        case 4: _t->slot_menu_edit_action(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PolygonTable::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PolygonTable::staticMetaObject = {
    { &QTableWidget::staticMetaObject, qt_meta_stringdata_PolygonTable,
      qt_meta_data_PolygonTable, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PolygonTable::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PolygonTable::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PolygonTable::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PolygonTable))
        return static_cast<void*>(const_cast< PolygonTable*>(this));
    return QTableWidget::qt_metacast(_clname);
}

int PolygonTable::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTableWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void PolygonTable::painter_MarkPolygon(QPolygon _t1, QPolygon _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PolygonTable::request_Update()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void PolygonTable::request_EditTable()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
