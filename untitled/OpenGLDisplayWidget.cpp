#include "OpenGLDisplayWidget.h"
#include <QKeyEvent>
#include <QMouseEvent>
#include <qkeyeventtransition.h>
#include <cmath>
#include <cstdlib>
#include <../kt84/graphics/graphics_util.hh>
//#include <kt84/zenity_util.hh>
#include <../kt84/tinyfd_util.hh>
#include <boost/algorithm/string.hpp>
#include "SketchRetopo\SketchRetopo.hh"
#include <qdebug.h>

using namespace std;
using namespace kt84;
using namespace kt84::graphics_util;

bool isNumberQtKey(int key);
bool isLetterQtKey(int key);
unsigned char qtKeyToChar(int key);
bool isDirectionQtKey(int key);
bool isLegalQtKey(int key);

OpenGLDisplayWidget * OpenGLDisplayWidget::glDisplayInstance = NULL;

OpenGLDisplayWidget & OpenGLDisplayWidget::getInstance() {
	if (glDisplayInstance == NULL)
	{
		glDisplayInstance = new OpenGLDisplayWidget;
	}

	return *glDisplayInstance;
}

OpenGLDisplayWidget::~OpenGLDisplayWidget()
{

}


void OpenGLDisplayWidget::resizeGL(int width, int height)
{
	_core.camera_free.reshape(width, height);
	_core.camera_upright.reshape(width, height);
	glViewport(0, 0, width, height);
	if(!_core.basemesh.faces_empty())_core.contour2D->generateContour();

	//setProjectionMatrix();
	//setModelviewMatrix();
}

void OpenGLDisplayWidget::initializeGL() 
{
	initializeOpenGLFunctions();
	_core.init();
}


void OpenGLDisplayWidget::paintGL()
{
	lock_guard<mutex> lock(_core.learnquad.mtx_display);
	/*stringstream window_title;
	window_title << "SketcherWare";
	if (!_core.configTemp.autoSave.filename.empty())
		window_title << " - " << _core.configTemp.autoSave.filename;
	if (_core.configTemp.autoSave.unsaved)
		window_title << "*";
	glfwSetWindowTitle(_core.glfw_control.window, window_title.str().c_str());*/

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	// background color ramp ====================================================================
	glMatrixMode(GL_PROJECTION);    glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);    glLoadIdentity();
	glDepthMask(GL_FALSE);
	glBegin(GL_QUADS);
	glColor3f(_core.configRender.color.background_bottom);    glVertex2d(-1, -1);    glVertex2d(1, -1);    // bottom
	glColor3f(_core.configRender.color.background_top);    glVertex2d(1, 1);    glVertex2d(-1, 1);    // top
	glEnd();
	glDepthMask(GL_TRUE);
	// ==================================================================== background color ramp

	setProjectionMatrix();

	setModelviewMatrix();

	_core.display_pre();

	if (_core.configRender.mode == ConfigRender::Mode::DEFAULT)
		_core.state->display();

	_core.display_post();
	glPopAttrib();
	//TwDraw();
}

void OpenGLDisplayWidget::setProjectionMatrix() {
	// set projection matrix
	double zNear = _core.camera->center_to_eye().norm() * 0.1;
	double zFar = zNear * 10 + _core.basemesh.boundingBox_diagonal_norm() * 10;
	double aspect_ratio = _core.camera->width / static_cast<double>(_core.camera->height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (_core.configRender.use_ortho) {
		double ortho_y = zNear * 2.5;
		double ortho_x = ortho_y * aspect_ratio;
		glOrtho(-ortho_x, ortho_x, -ortho_y, ortho_y, zNear, zFar);
	}
	else {
		gluPerspective(40, aspect_ratio, zNear, zFar);
	}
}

void OpenGLDisplayWidget::setModelviewMatrix() {
	// set modelview matrix
	auto eye = _core.camera->get_eye();
	auto center = _core.camera->center;
	auto up = _core.camera->get_up();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eye, center, up);
}

void OpenGLDisplayWidget::mapCursorPos(QPoint& pos) {
	pos.setY(_core.camera->height - pos.y());
}

void OpenGLDisplayWidget::mousePressEvent(QMouseEvent* event) {
	QPoint pos = event->pos();
	mapCursorPos(pos);
	if (!_core.common_mouse_down(pos.x(), pos.y(), buttonFromQtButtons(event->buttons()), isShiftPressed(event), isCtrlPressed(event), isAltPressed(event))) {
		_core.state->mouse_down(pos.x(), pos.y(), buttonFromQtButtons(event->buttons()), isShiftPressed(event), isCtrlPressed(event), isAltPressed(event));
	}
	qDebug() << event->pos();
}

void OpenGLDisplayWidget::mouseMoveEvent(QMouseEvent* event) {
	QPoint pos = event->pos();
	mapCursorPos(pos);
	if (!_core.common_mouse_move(pos.x(), pos.y())) {
		_core.state->mouse_move(pos.x(), pos.y());
	}
	qDebug() << event->pos();
	update();
}

void OpenGLDisplayWidget::mouseReleaseEvent(QMouseEvent* event) {
	QPoint pos = event->pos();
	mapCursorPos(pos);
	if (!_core.common_mouse_up(pos.x(), pos.y(), buttonFromQtButton(event->button()), isShiftPressed(event), isCtrlPressed(event), isAltPressed(event))) {
		_core.state->mouse_up(pos.x(), pos.y(), buttonFromQtButton(event->button()), isShiftPressed(event), isCtrlPressed(event), isAltPressed(event));
	}
	update();

	qDebug() << event->pos();
}

void OpenGLDisplayWidget::wheelEvent(QWheelEvent* event) {
}

void OpenGLDisplayWidget::keyPressEvent(QKeyEvent * event) {
	if (isLegalQtKey(event->key())) {
		unsigned char key = qtKeyToChar(event->key());
		if (key) {
			if (!_core.common_keyboard(key, isShiftPressed(event), isCtrlPressed(event), isAltPressed(event))) {
				_core.state->keyboard(key, isShiftPressed(event), isCtrlPressed(event), isAltPressed(event));
			}
		}
	}
}

void OpenGLDisplayWidget::keyReleaseEvent(QKeyEvent* event) {
	if (isLegalQtKey(event->key())) {
		unsigned char key = qtKeyToChar(event->key());
		if (key) {
			_core.common_keyboardup(key, isShiftPressed(event), isCtrlPressed(event), isAltPressed(event));
		}
	}
}

Button OpenGLDisplayWidget::buttonFromQtButtons(Qt::MouseButtons buttons) {
	if (buttons & Qt::LeftButton) return Button::LEFT;
	if (buttons & Qt::RightButton) return Button::RIGHT;
	return Button::MIDDLE;
}

Button OpenGLDisplayWidget::buttonFromQtButton(Qt::MouseButton button) {
	if (button == Qt::LeftButton) return Button::LEFT;
	if (button == Qt::RightButton) return Button::RIGHT;
	return Button::MIDDLE;
}

bool isNumberQtKey(int key) {
	return key >= Qt::Key_0 && key <= Qt::Key_9;
}

bool isLetterQtKey(int key) {
	return key <= Qt::Key_Z && key >= Qt::Key_A;
}

bool isDirectionQtKey(int key) {
	return key >= Qt::Key_Left && key <= Qt::Key_Down;
}

bool isLegalQtKey(int key) {
	return isNumberQtKey(key) || isLetterQtKey(key) ||  key == Qt::Key_Space;
}

unsigned char qtKeyToChar(int key) {
	if (isNumberQtKey(key)) {
		return '0' + key - Qt::Key_0;
	}
	else if (isLetterQtKey(key)) {
		return 'a' + key - Qt::Key_A;
	}
	else if (key == Qt::Key_Space) {
		return ' ';
	}

	return 0;
}


