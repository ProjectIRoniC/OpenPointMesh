/*
 * File:  tst_MainWindow.h
 * Usage: Only private slot functions will be run as tests. So if you want to add additional tests
 *        you can add more functions.
 */

#include "qtestsuite.h"
#include <QtTest/QtTest>
#include "mainwindow.h"
#include <string>
#include <QtGui>
#include <QMainWindow>
#include <QFileDialog>
#include "../build/ui_mainwindow.h"



#ifndef TST_MAINWINDOW_H
#define TST_MAINWINDOW_H

class MainWindow;
namespace Ui {
class MainWindow;
}

class tst_MainWindow: public QTestSuite
{
     Q_OBJECT
private:
    /* Declaration inside .cpp file. Should contain absolute path to oni file*/
    static const std::string oni_file_path;
    MainWindow *w;

private slots:
    /* Simulate a single run through of the program.
       Pre: oni_file_path contains the absolute path to the file you want to test
       Post: */
    void tst_SingleFileRunThrough();

    /*
     * Simulate a press to file menu bar
     */
    void tst_OpenOniFile();

    void initTestCase();

    void cleanupTestCase();

};

#endif
