#pragma once


#include <GL\glew.h>
#include <QPoint>
//#include <QtOpenGL\qgl.h>
#include <qopenglwidget.h>
#include <QOpenGLFunctions>
#include <QInputEvent>
#include "state\Button.hh"







class OpenGLDisplayWidget : public QOpenGLWidget, public QOpenGLFunctions
{
	Q_OBJECT

public:
	static OpenGLDisplayWidget & getInstance();

	

	Button buttonFromQtButton(Qt::MouseButton button);
	Button buttonFromQtButtons(Qt::MouseButtons buttons);
	bool isShiftPressed(QInputEvent * event) { return (event->modifiers() & Qt::ShiftModifier) != 0; }
	bool isCtrlPressed(QInputEvent * event) { return (event->modifiers() & Qt::ControlModifier) != 0; }
	bool isAltPressed(QInputEvent * event) {return  (event->modifiers() & Qt::AltModifier) != 0; }

protected:
	
	//初始化
	virtual void initializeGL();

	//绘制函数
	virtual void paintGL();
	void setProjectionMatrix();
	void setModelviewMatrix();


	//更改Widget大小时会用到
	virtual void resizeGL(int width, int height);

	//鼠标事件
	void mousePressEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
	void mouseMoveEvent(QMouseEvent* event);
	void wheelEvent(QWheelEvent* event);
	void keyPressEvent(QKeyEvent * event);
	void keyReleaseEvent(QKeyEvent* event);

	void OpenGLDisplayWidget::mapCursorPos(QPoint & pos);


private:
	OpenGLDisplayWidget(QWidget* pParent = Q_NULLPTR) : QOpenGLWidget(pParent) {
		setFocusPolicy(Qt::StrongFocus);
		setMouseTracking(true);
	}
	~OpenGLDisplayWidget();

	static OpenGLDisplayWidget * glDisplayInstance;

};
