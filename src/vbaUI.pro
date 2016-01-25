#-------------------------------------------------
#
# Project created by QtCreator 2015-10-08T15:45:14
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = vbaUI
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    ../build/__/include/moc_CloudStitcher.cxx \
    ../build/__/include/moc_errorMsgHandler.cxx \
    ../build/__/include/moc_mainwindow.cxx \
    ../build/__/include/moc_oni-to-pcd.cxx \
    ../build/__/include/moc_PCDRegistration.cxx \
    ../build/CMakeFiles/3.4.0-rc2/CompilerIdCXX/CMakeCXXCompilerId.cpp \
    ../build/CMakeFiles/feature_tests.cxx \
    cloud_registration.cpp \
    CloudStitcher.cpp \
    errorMsgHandler.cpp \
    oni-to-pcd.cpp \
    PCDRegistration.cpp \
    ../build/CMakeFiles/3.4.0-rc2/CompilerIdC/CMakeCCompilerId.c \
    ../build/CMakeFiles/feature_tests.c \
    MeshConstructor.cpp

HEADERS  += mainwindow.h \
    ../include/mainwindow.h \
    ../build/ui_mainwindow.h \
    ../include/CloudStitcher.h \
    ../include/errorMsgHandler.h \
    ../include/oni-to-pcd.h \
    ../include/PCDRegistration.h \
    ../include/MeshConstructor.h

FORMS    += mainwindow.ui
