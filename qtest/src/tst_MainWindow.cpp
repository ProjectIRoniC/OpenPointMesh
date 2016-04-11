/*
 * File:  tst_MainWindow.cpp
 * Usage: Run executable inside build/build_output folder and all tests will be run
 */
#include "tst_MainWindow.h"
#include <iostream>
#include <fstream>
#include "../include/sharedData.h"
#include "mainwindow.h"
#include <QString>


extern SharedData sharedvar;

namespace Ui {
class MainWindow;
}

const std::string tst_MainWindow::oni_file_path = "/home/paul/Documents/2016Volcanobot/OpenPointMesh/qtest/onifiles/overlappingParallelC2of2C.oni";

void tst_MainWindow::tst_SingleFileRunThrough() {

    QApplication a(sharedvar.argc, sharedvar.argv);
    MainWindow w;
    w.show();
    QCoreApplication::setAttribute(Qt::AA_DontUseNativeMenuBar);
    bool isshown = QTest::qWaitForWindowShown(&w);
    if(isshown == false )
        QFAIL("could not show window\n");
    QTest::qWait(10000);
    return;
    // update the text browser so testing will work
    w.findChild<QTextBrowser *> ("textBrowser")->setPlainText(QString::fromStdString(oni_file_path) );
    // click browse
    QTest::mouseClick(w.findChild<QPushButton *>("oni_browse_button"), Qt::LeftButton);
    //  update outputfolderanme
    w.outputFolderName = QString("/home/paul/Documents/res");
    //  click "select output folder" <Browse_output>
    QTest::mouseClick(w.findChild<QPushButton *>("Browse_output"), Qt::LeftButton);

    // option to run test quickly or slowly
    if(sharedvar.RUN_WITH_ONI_FILE){
        // Press start
        QTest::mouseClick(w.findChild<QPushButton *>("Start"), Qt::LeftButton);
        // wait until complete. wait 15 minutes max
        int MAXTIME = 600;
        for(int i = 0; i < MAXTIME; ++i) {
            if(w.taskThread == NULL)
                break;
            else if(i == MAXTIME - 1)
                QFAIL("File did not complete in time. May have stalled or file was too large for give time. Consider changing MAXTIME\n");
            else
                QTest::qWait(1000);
        }
    }

    QTest::mouseClick(w.findChild<QMenu *>("fileMenu"), Qt::LeftButton);
    //QTest::keyClick(w.findChild<QMenu *>("fileMenu"), Qt::Key_Down);
    QTest::qWait(3000);
    QTest::mouseClick(w.findChild<QWidget *>("exitAct"), Qt::LeftButton);
    //QTest::keyClick(w.findChild<QMenu *>("fileMenu"), Qt::Key_Down);
    QTest::qWait(3000);
    //QTest::keyClick(w.findChild<QMenu *>("fileMenu"), Qt::Key_Enter);
    QTest::qWait(3000);
    //QTest::mouseClick(w.findChild<QWidget *>("exitAct"), Qt::LeftButton);
    //QApplication::quit();
    //QCoreApplication::quit();
    //qApp->exit();
    //a.exit();

    return;
}

void tst_MainWindow::tst_OpenOniFile()
{

    return;
    // Start QApplication. Without this MainWindow cannot open.
    QApplication a(sharedvar.argc, sharedvar.argv);
    // Start current GUI initializeation
    MainWindow w;
    // Make visible to users
    w.show();
    // Wait for GUI to Show
    bool isshown = QTest::qWaitForWindowShown(&w);
    if(isshown == false )
        QFAIL("could not show window\n");
    //w.findChild<QAction *> ("exitAct")->isEmpty();
    //QTest::mouseClick(w.findChild<QAction *>("exitAct"), Qt::LeftButton);
    //QMenu *menu = getMenu(w, "fileMenu");
    //QTest::mouseClick(w.findChild<QMenu *>("fileMenu"), Qt::LeftButton);
    //QTest::qWait(20000);
    /* example
    QMenu *menu = getMenu(mainWindow, menuName);
    if (menu != nullptr) {
        QTest::keyClick(mainWindow, menu->title().at(1).toLatin1(), Qt::AltModifier);
    }


    QList<QAction *> actions = menu->actions();
    foreach (QAction *action, actions) {
        if (action->objectName() == actionName) {
            QTest::keyClick(menu, Qt::Key_Enter);
            break;
        }
        QTest::qWait(1000);
        QTest::keyClick(menu, Qt::Key_Down);
   / end of example*/

}




static tst_MainWindow instance;  //This is where this particular test is instantiated, and thus added to the static list of test suites


