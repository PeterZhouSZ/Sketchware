#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
	static MainWindow & getInstance();
    



private:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();
    Ui::MainWindow *ui;
	static MainWindow * mw_Instance;
};

#endif // MAINWINDOW_H
