/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.cpp
Description:	Reads an oni file recorded using the Openni2 or Openni library and outputs point clouds (pcd files)
*/

#include "../include/mainwindow.h"
#include <QTextStream>

// ### from other main

#include <fstream>
#include <vector>
#include <QtTest/QtTest>
#include <QApplication>

// ###

#include "qtestsuite.h"
#include "sharedData.h"
#include <iostream>

extern SharedData sharedvar;

int main(int argc, char*argv[])
{
/*
QApplication a(argc, argv);
    MainWindow w;

    w.show();
bool isshown = QTest::qWaitForWindowShown(&w);
if(isshown == false )
    return 0;
    //int rtn = a.exec();

QTest::qWait(2000);

return 0;
return 0;
*/
    int failedSuitesCount = 0;
    std::vector<QObject*>::iterator iSuite;
    sharedvar.setArgs(argc, argv);
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




/*
int main(int argc, char* argv[])
{

//int argc = 1;
//char *argv[1];
//argv[0] = "/home/paul/Documents/2016Volcanobot/qtestexample/build/build_output/OpenPointMeshTests";
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
bool isshown = QTest::qWaitForWindowShown(&w);
if(isshown == false )
    return -1;
    //int rtn = a.exec();

QTest::qWait(2000);
QWidget* btn  = w.getQPushButton();
QTest::mouseClick(btn, Qt::LeftButton);


std::ofstream myfile;
  myfile.open ("example.txt");
  myfile << "Writing this to a file.\n";
  
myfile << "argc = " << argc << "\nargv = " << argv[0] << "\n";
myfile.close();


return 0;

}
*/

