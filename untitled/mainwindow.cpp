#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtDebug>

MainWindow* MainWindow::mw_Instance = NULL;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    qDebug() << "MainWindow created\n";
}

MainWindow::~MainWindow()
{
    delete ui;
}

MainWindow & MainWindow::getInstance(){
	if (mw_Instance == NULL)
		mw_Instance = new MainWindow;
	return *mw_Instance;
}