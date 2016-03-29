#ifndef SHAREDDATA_H
#define SHAREDDATA_H


class SharedData {

public:
    SharedData();
    static void func();
    static void setArgs(int, char**);


// variables
static int argc;
static char** argv;
};

#endif
