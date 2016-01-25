/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.cpp
Description:	Reads an oni file recorded using the Openni2 or Openni library and outputs point clouds (pcd files)
*/

#include "../include/oni-to-pcd.h"
#include "../include/errorMsgHandler.h"
#include "../include/mainwindow.h"
#include <QApplication>
#include <iostream>
#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/spsc_queue.hpp>

#include <boost/thread/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/lockfree/policies.hpp>


/***************************************************************
here for execution of code in standalone, will be removed once 
integrated in project
***************************************************************/
int main (int argc, char* argv[]) {

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();

}

