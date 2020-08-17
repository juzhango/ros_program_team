/****************************************************************************
** Meta object code from reading C++ file 'scenetable.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/robot_rqt/include/robot_rqt/scenetable.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'scenetable.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SceneTable[] = {

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
      15,   12,   11,   11, 0x05,
     100,   11,   11,   11, 0x05,
     116,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
     134,   11,   11,   11, 0x0a,
     158,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SceneTable[] = {
    "SceneTable\0\0,,\0"
    "launchUpdateTable(std::vector<QPolygon>,std::vector<QPolygon>,std::vec"
    "tor<QPolygon>)\0"
    "updatePainter()\0menu_edit_scene()\0"
    "slot_menu_edit_action()\0slot_menu_del_action()\0"
};

void SceneTable::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SceneTable *_t = static_cast<SceneTable *>(_o);
        switch (_id) {
        case 0: _t->launchUpdateTable((*reinterpret_cast< std::vector<QPolygon>(*)>(_a[1])),(*reinterpret_cast< std::vector<QPolygon>(*)>(_a[2])),(*reinterpret_cast< std::vector<QPolygon>(*)>(_a[3]))); break;
        case 1: _t->updatePainter(); break;
        case 2: _t->menu_edit_scene(); break;
        case 3: _t->slot_menu_edit_action(); break;
        case 4: _t->slot_menu_del_action(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SceneTable::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SceneTable::staticMetaObject = {
    { &QTableWidget::staticMetaObject, qt_meta_stringdata_SceneTable,
      qt_meta_data_SceneTable, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SceneTable::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SceneTable::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SceneTable::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SceneTable))
        return static_cast<void*>(const_cast< SceneTable*>(this));
    return QTableWidget::qt_metacast(_clname);
}

int SceneTable::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void SceneTable::launchUpdateTable(std::vector<QPolygon> _t1, std::vector<QPolygon> _t2, std::vector<QPolygon> _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SceneTable::updatePainter()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void SceneTable::menu_edit_scene()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
