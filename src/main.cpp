/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.cpp
Description:	Reads an oni file recorded using the Openni2 or Openni library and outputs point clouds (pcd files)
*/

#include <QApplication>
#include <iostream>
#include "../include/mainwindow.h"

/***************************************************************
here for execution of code in standalone, will be removed once 
integrated in project
***************************************************************/
int main( int argc, char* argv[] )
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}

