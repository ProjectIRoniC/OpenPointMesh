
#include "../include/mytestsuite.h"
#include "../include/mainwindow.h"
#include <iostream>
#include <fstream>

void MyTestSuite1::aTestFunction()
{
    QString str = "Hello";
    QVERIFY(str.toUpper() == "HELLO");
}

void MyTestSuite1::anotherTestFunction()
{

    //QApplication a(argc, argv);
/*
    MainWindow w;
    w.show();
bool isshown = QTest::qWaitForWindowShown(&w);
if(isshown == false )
    return;
    //int rtn = a.exec();

QTest::qWait(2000);

std::ofstream myfile;
  myfile.open ("example.txt");
  myfile << "Writing this to a file.\n";
  
//myfile << "argc = " << argc << "\nargv = " << argv[0] << "\n";
myfile.close();


return;
*/

    QString str = "Goodbye";
    QVERIFY(str.toUpper() == "GOODBYE");
}

static MyTestSuite1 instance;  //This is where this particular test is instantiated, and thus added to the static list of test suites


