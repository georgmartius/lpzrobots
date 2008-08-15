/****************************************************************************
** Meta object code from reading C++ file 'SRIRSensorWidget.h'
**
** Created: Thu Aug 14 13:10:53 2008
**      by: The Qt Meta Object Compiler version 59 (Qt 4.3.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SRIRSensorWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SRIRSensorWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 59
#error "This file was generated using the moc from 4.3.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

static const uint qt_meta_data_SRIRSensorWidget[] = {

 // content:
       1,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets

       0        // eod
};

static const char qt_meta_stringdata_SRIRSensorWidget[] = {
    "SRIRSensorWidget\0"
};

const QMetaObject SRIRSensorWidget::staticMetaObject = {
    { &SphericalRobotSubWidget::staticMetaObject, qt_meta_stringdata_SRIRSensorWidget,
      qt_meta_data_SRIRSensorWidget, 0 }
};

const QMetaObject *SRIRSensorWidget::metaObject() const
{
    return &staticMetaObject;
}

void *SRIRSensorWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SRIRSensorWidget))
	return static_cast<void*>(const_cast< SRIRSensorWidget*>(this));
    return SphericalRobotSubWidget::qt_metacast(_clname);
}

int SRIRSensorWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = SphericalRobotSubWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
