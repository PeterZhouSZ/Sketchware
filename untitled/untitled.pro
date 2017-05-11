#-------------------------------------------------
#
# Project created by QtCreator 2016-12-19T15:55:24
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = untitled
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    menuitemmodel.cpp \
    qtmenufactory.cpp \
    qtmenu.cpp \
    qtmenuitem.cpp \
    qtmenubar.cpp \
    qtmenuassemblehelper.cpp \
    menuassemblecontroller.cpp \
    mainwindowmanager.cpp \
    test.cpp \
    QTSliderFactory.cpp \
    QTSliderItemCheckbox.cpp \
    QTSliderItemDigital.cpp \
    QTSliderItemOptions.cpp \
    qtsliderframe.cpp \
    OpenGLDisplayWidget.cpp

HEADERS  += mainwindow.h \
    menuabstractfactory.h \
    menuitemmodel.h \
    qtmenufactory.h \
    menuitem.h \
    menubar.h \
    menu.h \
    qtmenu.h \
    qtmenuitem.h \
    qtmenubar.h \
    menuassemblehelper.h \
    qtmenuassemblehelper.h \
    menuassemblecontroller.h \
    mainwindowmanager.h \
    test.h \
    QTSliderFactory.h \
    QTSliderItemCheckbox.h \
    QTSliderItemDigital.h \
    QTSliderItemOptions.h \
    SliderAssembleController.h \
    SliderCellModel.h \
    SliderFactory.h \
    qtsliderframe.h \
    OpenGLDisplayWidget.h

FORMS    += mainwindow.ui
