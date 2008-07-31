/****************************************************************************
** Meta object code from reading C++ file 'SphericalRobotGUI.h'
**
** Created: Wed Jul 30 08:21:42 2008
**      by: The Qt Meta Object Compiler version 59 (Qt 4.3.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SphericalRobotGUI.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SphericalRobotGUI.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 59
#error "This file was generated using the moc from 4.3.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

static const uint qt_meta_data_SphericalRobotGUI[] = {

 // content:
       1,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   10, // methods
       0,    0, // properties
       0,    0, // enums/sets

 // signals: signature, parameters, type, tag, flags
      19,   18,   18,   18, 0x05,

 // slots: signature, parameters, type, tag, flags
      37,   18,   18,   18, 0x0a,
      57,   18,   18,   18, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SphericalRobotGUI[] = {
    "SphericalRobotGUI\0\0valueChanged(int)\0"
    "setArrowParamX(int)\0setArrowParamY(int)\0"
};

const QMetaObject SphericalRobotGUI::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_SphericalRobotGUI,
      qt_meta_data_SphericalRobotGUI, 0 }
};

const QMetaObject *SphericalRobotGUI::metaObject() const
{
    return &staticMetaObject;
}

void *SphericalRobotGUI::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SphericalRobotGUI))
	return static_cast<void*>(const_cast< SphericalRobotGUI*>(this));
    return QWidget::qt_metacast(_clname);
}

int SphericalRobotGUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: setArrowParamX((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: setArrowParamY((*reinterpret_cast< int(*)>(_a[1]))); break;
        }
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void SphericalRobotGUI::valueChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
