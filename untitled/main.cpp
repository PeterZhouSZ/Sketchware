#include "mainwindow.h"
#include <GL\glew.h>
#include <QApplication>
#include <qdockwidget.h>
#include "menuassemblecontroller.h"
#include "qtmenu.h"
#include "qtmenubar.h"
#include "qtmenuitem.h"
#include "test.h"
#include "SliderAssembleController.h"
#include "qtsliderframe.h"
#include "OpenGLDisplayWidget.h"
#include <iostream>

int main(int argc, char *argv[])
{
	std::setbuf(stdout, NULL);

    QApplication a(argc, argv);
	MainWindow& w = MainWindow::getInstance();

    //Dock
	sketcherUI::QTSliderFrame * dock = sketcherUI::SliderAssembleController<sketcherUI::QTSliderFrame, QWidget>().loadSlider();
	w.addDockWidget(Qt::LeftDockWidgetArea, dock);

    sketcherUI::MenuAssembleController<sketcherUI::QTMenuItem, sketcherUI::QTMenu, sketcherUI::QTMenuBar>  controller;// = new sketcherUI::MenuAssembleController<class QTMenuItem, class QTMenu, class QTMenuBar>();
    w.setMenuBar(controller.loadMenu());

	w.setCentralWidget(&OpenGLDisplayWidget::getInstance());

    w.show();

    return a.exec();
}
