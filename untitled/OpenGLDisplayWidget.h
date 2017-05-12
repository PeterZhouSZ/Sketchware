#pragma once


#include <GL\glew.h>
#include <QPoint>
//#include <QtOpenGL\qgl.h>
#include <qopenglwidget.h>
#include <QOpenGLFunctions>
#include <QInputEvent>


#include "SketchRetopo.hh"




class OpenGLDisplayWidget : public QOpenGLWidget, protected QOpenGLFunctions
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
	
	//��ʼ��
	virtual void initializeGL();

	//���ƺ���
	virtual void paintGL();
	void setProjectionMatrix();
	void setModelviewMatrix();


	//����Widget��Сʱ���õ�
	virtual void resizeGL(int width, int height);

	//����¼�
	void mousePressEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
	void mouseMoveEvent(QMouseEvent* event);
	void wheelEvent(QWheelEvent* event);
	void keyPressEvent(QKeyEvent * event);
	void keyReleaseEvent(QKeyEvent* event);

	void OpenGLDisplayWidget::mapCursorPos(QPoint & pos);


private:
	OpenGLDisplayWidget(QWidget* pParent = Q_NULLPTR) : QOpenGLWidget(pParent), _core(SketchRetopo::get_instance()) {
		setFocusPolicy(Qt::StrongFocus);
		setMouseTracking(true);
	}
	~OpenGLDisplayWidget();

	static OpenGLDisplayWidget * glDisplayInstance;
	SketchRetopo &_core;

};
