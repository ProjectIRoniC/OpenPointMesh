/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../include/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   12,   11,   11, 0x05,
      41,   11,   11,   11, 0x05,
      52,   11,   11,   11, 0x05,
      74,   11,   11,   11, 0x05,
     101,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
     130,   11,   11,   11, 0x08,
     168,  149,   11,   11, 0x08,
     182,   11,   11,   11, 0x08,
     209,   11,   11,   11, 0x08,
     238,   11,   11,   11, 0x08,
     249,   11,   11,   11, 0x08,
     260,   11,   11,   11, 0x08,
     282,   11,   11,   11, 0x08,
     299,   11,   11,   11, 0x08,
     320,   11,   11,   11, 0x08,
     340,   11,   11,   11, 0x08,
     360,   11,   11,   11, 0x08,
     380,   11,   11,   11, 0x08,
     392,   11,   11,   11, 0x08,
     407,   11,   11,   11, 0x08,
     438,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0msg\0appendToConsole(QString)\0"
    "start(int)\0oniToPCDFinished(int)\0"
    "cloudStitcherFinished(int)\0"
    "meshConstructorFinished(int)\0"
    "on_Start_clicked()\0controllerConstant\0"
    "nextStep(int)\0on_Browse_output_clicked()\0"
    "ensureCursorVisible(QString)\0openSlot()\0"
    "exitSlot()\0sampleFrameRateSlot()\0"
    "omitFramesSlot()\0filterAccuracySlot()\0"
    "meshOutputOBJSlot()\0meshOutputPLYSlot()\0"
    "meshOutputVTKSlot()\0aboutSlot()\0"
    "viewWikiSlot()\0on_oni_browse_button_clicked()\0"
    "onAccuracyControlDialogClose()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->appendToConsole((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->start((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->oniToPCDFinished((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->cloudStitcherFinished((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->meshConstructorFinished((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->on_Start_clicked(); break;
        case 6: _t->nextStep((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 7: _t->on_Browse_output_clicked(); break;
        case 8: _t->ensureCursorVisible((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 9: _t->openSlot(); break;
        case 10: _t->exitSlot(); break;
        case 11: _t->sampleFrameRateSlot(); break;
        case 12: _t->omitFramesSlot(); break;
        case 13: _t->filterAccuracySlot(); break;
        case 14: _t->meshOutputOBJSlot(); break;
        case 15: _t->meshOutputPLYSlot(); break;
        case 16: _t->meshOutputVTKSlot(); break;
        case 17: _t->aboutSlot(); break;
        case 18: _t->viewWikiSlot(); break;
        case 19: _t->on_oni_browse_button_clicked(); break;
        case 20: _t->onAccuracyControlDialogClose(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::appendToConsole(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MainWindow::start(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MainWindow::oniToPCDFinished(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MainWindow::cloudStitcherFinished(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void MainWindow::meshConstructorFinished(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
