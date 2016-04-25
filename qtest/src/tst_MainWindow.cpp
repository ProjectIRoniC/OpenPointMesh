/*
 * File:  tst_MainWindow.cpp
 * Usage: Run executable inside build/build_output folder and all tests will be run
 */
#include "tst_MainWindow.h"
#include <iostream>
#include <fstream>
#include "../include/sharedData.h"
#include <QString>
#include "../include/debugger.h"
#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations



extern SharedData sharedvar;

QApplication a(sharedvar.argc, sharedvar.argv);
const std::string tst_MainWindow::oni_file_path = "../onifiles/overlappingParallelC2of2C.oni";
const std::string tst_MainWindow::oni_file_path2 = "../onifiles/overlappingParallelC1of2C.oni";

std::string tst_MainWindow::getAbsolutePath(const std::string _path){
    boost::filesystem::path p(_path);
    boost::filesystem::path full_p = boost::filesystem::complete(p); // absolutepath
    return full_p.string();

}

void tst_MainWindow::tst_SingleFileRunThrough() {

    boost::filesystem::path p("outputFolderForOniFiles");
    boost::filesystem::path full_p = boost::filesystem::complete(p); // absolutepath

    QString outputFolderName = QString::fromStdString(full_p.string());
    bool isshown = QTest::qWaitForWindowShown(w);
    if(isshown == false )
        QFAIL("could not show window\n");
    // update the text browser so testing will work
    w->setTextBrowser(QString::fromStdString(getAbsolutePath(oni_file_path)));
    QTest::mouseClick(w->findChild<QPushButton *>("oni_browse_button"), Qt::LeftButton);
    w->setOutputFolderName(outputFolderName);
    QTest::mouseClick(w->findChild<QPushButton *>("Browse_output"), Qt::LeftButton);
    if(debugger::RUN_WITH_ONI_FILE){
        // Press start
        QTest::mouseClick(w->findChild<QPushButton *>("Start"), Qt::LeftButton);
        // wait until complete. wait 15 minutes max
        int MAXTIME = 600;
        for(int i = 0; i < MAXTIME; ++i) {
            if(w->hasStartedWorkingOnFile() == false)
                break;
            else if(i == MAXTIME - 1)
                QFAIL("File did not complete in time. May have stalled or file was too large for give time. Consider changing MAXTIME\n");
            else
                QTest::qWait(1000);
        }
    }
    return;
}

void tst_MainWindow::tst_MultipleFileRunThrough()
{
    boost::filesystem::path p("outputFolderForOniFiles");
    boost::filesystem::path full_p = boost::filesystem::complete(p); // absolutepath
    QString outputFolderName = QString::fromStdString(full_p.string());

    bool isshown = QTest::qWaitForWindowShown(w);
    if(isshown == false )
        QFAIL("could not show window\n");
    // update the text browser so testing will work
    w->setTextBrowser(QString::fromStdString(getAbsolutePath(oni_file_path) + getAbsolutePath(oni_file_path2)));
    QTest::mouseClick(w->findChild<QPushButton *>("oni_browse_button"), Qt::LeftButton);
    w->setOutputFolderName(outputFolderName);
    QTest::mouseClick(w->findChild<QPushButton *>("Browse_output"), Qt::LeftButton);
    if(debugger::RUN_WITH_ONI_FILE){
        // Press start
        QTest::mouseClick(w->findChild<QPushButton *>("Start"), Qt::LeftButton);
        // wait until complete. wait 15 minutes max
        int MAXTIME = 600;
        for(int i = 0; i < MAXTIME; ++i) {
            if(w->hasStartedWorkingOnFile() == false)
                break;
            else if(i == MAXTIME - 1)
                QFAIL("File did not complete in time. May have stalled or file was too large for give time. Consider changing MAXTIME\n");
            else
                QTest::qWait(1000);
        }
    }
    return;
}

void tst_MainWindow::init()
{
    /* This will run before every function */
}

void tst_MainWindow::cleanup()
{
    /* This will run after every function*/
}

void tst_MainWindow::initTestCase()
{
    QCoreApplication::setAttribute(Qt::AA_DontUseNativeMenuBar);
    w = new MainWindow();
    w->show();
    bool isshown = QTest::qWaitForWindowShown(w);
    if(isshown == false )
        QFAIL("could not show window\n");
}

void tst_MainWindow::cleanupTestCase()
{
}

void tst_MainWindow::tst_ExitGUI()
{
    QVERIFY2(w->isVisible() == true,"GUI Window is not visible\n" );
    w->findChild<QAction *>("exitAct")->activate(QAction::Trigger);
    QVERIFY2(w->isVisible() == false,"GUI Window is not visible\n" );

    return;
}

void tst_MainWindow::tst_Open()
{
    QVERIFY( false == QObject::connect( w->findChild<QAction *>("openAct"), SIGNAL(triggered()), w, SLOT(openSlot()), Qt::UniqueConnection) );
}

void tst_MainWindow::tst_OmitFrames()
{
    QVERIFY( false == QObject::connect( w->findChild<QAction *>("omitFramesAct"), SIGNAL(triggered()), w, SLOT(omitFramesSlot()), Qt::UniqueConnection) );
}
void tst_MainWindow::tst_OniSampleRate()
{
    QVERIFY( false == QObject::connect( w->findChild<QAction *>("sampleRateAct"), SIGNAL(triggered()), w, SLOT(sampleFrameRateSlot()), Qt::UniqueConnection) );
}

void tst_MainWindow::tst_FilterAccuracy()
{
    QVERIFY( false == QObject::connect( w->findChild<QAction *>("filterAccuracyAct"), SIGNAL(triggered()), w, SLOT(filterAccuracySlot()), Qt::UniqueConnection) );
}

void tst_MainWindow::tst_MeshOutputFileType()
{
    QVERIFY( false == QObject::connect( w->findChild<QAction *>("meshOutputOBJAct"), SIGNAL(triggered()), w, SLOT(meshOutputPLYSlot()), Qt::UniqueConnection) );
    QVERIFY( false == QObject::connect( w->findChild<QAction *>("meshOutputOBJAct"), SIGNAL(triggered()), w, SLOT(meshOutputOBJSlot()), Qt::UniqueConnection) );
    QVERIFY( false == QObject::connect( w->findChild<QAction *>("meshOutputOBJAct"), SIGNAL(triggered()), w, SLOT(meshOutputVTKSlot()), Qt::UniqueConnection) );
}

void tst_MainWindow::tst_About()
{
    QVERIFY( false == QObject::connect( w->findChild<QAction *>("aboutAct"), SIGNAL(triggered()), w, SLOT(aboutSlot()), Qt::UniqueConnection) );
}

void tst_MainWindow::tst_Wiki()
{
    QVERIFY( false == QObject::connect( w->findChild<QAction *>("viewWikiAct"), SIGNAL(triggered()), w, SLOT(viewWikiSlot()), Qt::UniqueConnection) );
}


static tst_MainWindow instance;  //This is where this particular test is instantiated, and thus added to the static list of test suites


