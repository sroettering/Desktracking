#-------------------------------------------------
#
# Project created by QtCreator 2014-11-11T14:00:00
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = desktracking
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp \
    KinectFrameReceiver.cpp

HEADERS  += pclviewer.h

FORMS    += pclviewer.ui
