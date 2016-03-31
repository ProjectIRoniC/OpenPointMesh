/*
 * File:  tst_MainWindow.h
 * Usage: Only private slot functions will be run as tests. So if you want to add additional tests
 *        you can add more functions.
 */

#include "qtestsuite.h"
#include <QtTest/QtTest>
#include "../../include/mainwindow.h"
#include <string>

#ifndef TST_MAINWINDOW_H
#define TST_MAINWINDOW_H

class tst_MainWindow: public QTestSuite
{
     Q_OBJECT
private:
    /* Declaration inside .cpp file. Should contain absolute path to oni file*/
    static const std::string oni_file_path;


private slots:
    /* Simulate a single run through of the program.
       Pre: oni_file_path contains the absolute path to the file you want to test
       Post: */
    void tst_SingleFileRunThrough();
    
};

#endif
