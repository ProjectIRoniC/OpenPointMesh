#include "qtestsuite.h"

#include <QtTest/QtTest>

#ifndef MYTESTSUITE1_H
#define MYTESTSUITE1_H

class MyTestSuite1: public QTestSuite
{
     Q_OBJECT
private slots:
    void aTestFunction();
    void anotherTestFunction();
};

#endif
