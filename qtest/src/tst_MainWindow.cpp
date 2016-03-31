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
    bool isshown = QTest::qWaitForWindowShown(&w);
    if(isshown == false )
        QFAIL("could not show window\n");
    // update the text browser so testing will work
    w.findChild<QTextBrowser *> ("textBrowser")->setPlainText(QString::fromStdString(oni_file_path) );
    // click browse
    QTest::mouseClick(w.findChild<QPushButton *>("oni_browse_button"), Qt::LeftButton);
    //  update outputfolderanme
    w.outputFolderName = QString("/home/paul/Documents/res");
    //  click "select output folder" <Browse_output>
    QTest::mouseClick(w.findChild<QPushButton *>("Browse_output"), Qt::LeftButton);
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

    return;
}




static tst_MainWindow instance;  //This is where this particular test is instantiated, and thus added to the static list of test suites


