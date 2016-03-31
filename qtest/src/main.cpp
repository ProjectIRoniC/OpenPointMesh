/*
    This is the main function that will run all the tests that inherit from QTestSuite. 
*/

#include "../include/mainwindow.h"
#include <vector>
#include <QtTest/QtTest>
#include <QApplication>

#include "qtestsuite.h"
#include "sharedData.h"
#include <iostream>

extern SharedData sharedvar;

int main(int argc, char*argv[])
{
    int failedSuitesCount = 0;
    std::vector<QObject*>::iterator iSuite;
    sharedvar.setArgs(argc, argv); // needed to create QApplications inside test classes
    for (iSuite = QTestSuite::m_suites.begin(); iSuite != QTestSuite::m_suites.end(); iSuite++)
    {
        int result = QTest::qExec(*iSuite);
        if (result != 0)
        {
            failedSuitesCount++;
        }
    }
    return failedSuitesCount;
}


