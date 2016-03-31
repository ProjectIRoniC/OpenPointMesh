#include "qtestsuite.h"

#include <QtTest/QtTest>
#include "../../include/mainwindow.h"

#ifndef TST_MAINWINDOW_H
#define TST_MAINWINDOW_H

class tst_MainWindow: public QTestSuite
{
     Q_OBJECT
private slots:
    /* Simulate a single run through of the program. */
    void tst_SingleFileRuneThrough();
};

#endif
