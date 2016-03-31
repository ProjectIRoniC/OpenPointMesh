#include "../include/sharedData.h"
#include <iostream>
#include <fstream>

int SharedData::argc = 0;
char** SharedData::argv = 0;

SharedData::SharedData() {
    std::ofstream myfile;
  myfile.open ("example.txt");
  myfile << "Writing this to------ a file.\n";
  
//myfile << "argc = " << argc << "\nargv = " << argv[0] << "\n";
myfile.close();

}

void SharedData::func() {
std::ofstream myfile;
  myfile.open ("example.txt");
  myfile << "Writing this to------ a file.\n";
myfile<< argc << ":argc\n";
  
//myfile << "argc = " << argc << "\nargv = " << argv[0] << "\n";
myfile.close();
return;
}

void SharedData::setArgs(int _argc, char** _argv)
{
    argc = _argc;
    argv = _argv;
}


static SharedData sharedvar; // Should be global variable
