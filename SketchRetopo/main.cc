#include <cstdlib>
#include <kt84/graphics/graphics_util.hh>
//#include <kt84/zenity_util.hh>
#include <kt84/tinyfd_util.hh>
#include "SketchRetopo.hh"
#include <boost/algorithm/string.hpp>
using namespace std;
using namespace kt84;
using namespace kt84::graphics_util;

// google-breakpad ==================================================================
//#define USE_GOOGLE_BREAKPAD

#ifdef USE_GOOGLE_BREAKPAD
#   ifdef WIN32

#include "client/windows/handler/exception_handler.h"
#include <csignal>

bool google_breakpad_callback(
    const wchar_t *dump_path,
    const wchar_t *id,
    void *context,
    EXCEPTION_POINTERS *exinfo,
    MDRawAssertionInfo *assertion,
    bool succeeded)
{
    if (succeeded) {
        printf("dump guid is %ws\n", id);
    } else {
        printf("dump failed\n");
    }
    fflush(stdout);
    
    return succeeded;
}

void __cdecl sigabrt_handler(int) { *static_cast<char*>(0) = 1; }
void __cdecl sigint_handler (int) { *static_cast<char*>(0) = 1; }

#   endif
#endif
// ================================================================== google-breakpad

namespace {
    auto& core = SketchRetopo::get_instance();
}

void display() {
    lock_guard<mutex> lock(core.learnquad.mtx_display);
    stringstream window_title;
    window_title << "SketchRetopo";
    if (!core.configTemp.autoSave.filename.empty())
        window_title << " - " << core.configTemp.autoSave.filename;
    if (core.configTemp.autoSave.unsaved)
        window_title << "*";
    glfwSetWindowTitle(core.glfw_control.window, window_title.str().c_str());
    
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    
    // background color ramp ====================================================================
    glMatrixMode(GL_PROJECTION);    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW );    glLoadIdentity();
    glDepthMask(GL_FALSE);
    glBegin(GL_QUADS);
    glColor3f(core.configRender.color.background_bottom);    glVertex2d(-1, -1);    glVertex2d( 1, -1);    // bottom
    glColor3f(core.configRender.color.background_top   );    glVertex2d( 1,  1);    glVertex2d(-1,  1);    // top
    glEnd();
    glDepthMask(GL_TRUE);
    // ==================================================================== background color ramp
    
    // set projection matrix
    double zNear = core.camera->center_to_eye().norm() * 0.1;
    double zFar  = zNear * 10 + core.basemesh.boundingBox_diagonal_norm() * 10;
    double aspect_ratio = core.camera->width / static_cast<double>(core.camera->height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (core.configRender.use_ortho) {
        double ortho_y = zNear * 2.5;
        double ortho_x  = ortho_y * aspect_ratio;
        glOrtho(-ortho_x, ortho_x, -ortho_y, ortho_y, zNear, zFar);
    } else {
        gluPerspective(40, aspect_ratio, zNear, zFar);
    }
    
    // set modelview matrix
    auto eye    = core.camera->get_eye();
    auto center = core.camera->center;
    auto up     = core.camera->get_up();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye, center, up);
    
    core.display_pre();
    
    if (core.configRender.mode == ConfigRender::Mode::DEFAULT)
        core.state->display();
    
    core.display_post();
    glPopAttrib();
    TwDraw();
}
int getDevicePixelRatio(GLFWwindow* window) {
  int wWidth, wHeight;
  glfwGetWindowSize(window, &wWidth, &wHeight);
  int fbWidth, fbHeight;
  glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
  return fbWidth/wWidth;
}
void mapCursorPos(GLFWwindow* window, int &x, int &y) {
    int wWidth, wHeight;
    glfwGetWindowSize(window, &wWidth, &wHeight);
    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    x = x * fbWidth  / wWidth;
    y = y * fbHeight / wHeight;
}
namespace callback {
    void framebuffersize(GLFWwindow* window, int width, int height) {
        core.camera_free.reshape(width, height);
        core.camera_upright.reshape(width, height);
        glViewport(0, 0, width, height);
        TwWindowSize(width, height);
    }
    void key(GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (glfw_util::TwEventKeyGLFW3(key, scancode, action, mods))
            return;
        if (key > 255)
            // ignore non-printable characters
            return;
        auto actionFlag = glfw_util::parseAction(action);
        auto modFlag = glfw_util::parseMods(mods);
        if ('A' <= key && key <= 'Z' && !modFlag.shift)
            key += 'a' - 'A';
        if (actionFlag.press || actionFlag.repeat) {
            if (!core.common_keyboard(key, modFlag.shift, modFlag.ctrl, modFlag.alt))
                core.state->keyboard(key, modFlag.shift, modFlag.ctrl, modFlag.alt);
        }
        else if (actionFlag.release) {
            core.common_keyboardup(key, modFlag.shift, modFlag.ctrl, modFlag.alt);
        }
    }
    void mousebutton(GLFWwindow* window, int button, int action, int mods) {
        if (TwEventMouseButtonGLFW(button, action))
            return;

        auto p = glfw_util::getCursorPos(window);
        mapCursorPos(window, p.x, p.y);
        p.y = core.camera->height - p.y;

        auto buttonFlag = glfw_util::parseButton(button);
        Button button_ =
            buttonFlag.left ? Button::LEFT :
            buttonFlag.middle ? Button::MIDDLE : Button::RIGHT;

        auto modFlag = glfw_util::parseMods(mods);

        auto actionFlag = glfw_util::parseAction(action);
        if (actionFlag.press) {
            if (!core.common_mouse_down(p.x, p.y, button_, modFlag.shift, modFlag.ctrl, modFlag.alt))
                core.state->mouse_down(p.x, p.y, button_, modFlag.shift, modFlag.ctrl, modFlag.alt);
        }
        else {
            if (!core.common_mouse_up(p.x, p.y, button_, modFlag.shift, modFlag.ctrl, modFlag.alt))
                core.state->mouse_up(p.x, p.y, button_, modFlag.shift, modFlag.ctrl, modFlag.alt);
        }
    }
    void cursorpos(GLFWwindow* window, double dx, double dy) {
        int x = static_cast<int>(dx);
        int y = static_cast<int>(dy);
        mapCursorPos(window, x, y);
        if (TwEventMousePosGLFW(x, y))
            return;
        y = core.camera->height - y;

        if (!core.common_mouse_move(x, y))
            core.state->mouse_move(x, y);
    }
    void drop(GLFWwindow* window, int count, const char** names) {
        if (count != 1)
            return;
        if (core.configTemp.autoSave.unsaved && !tinyfd_messageBox("SketchRetopo", "Current data is not saved yet. Continue?", "yesno", "question", 0))
            return;
        string fname = names[0];
        auto fname_ext = fname.substr(fname.size() - 4, 4);
        boost::algorithm::to_lower(fname_ext);
        if (fname_ext == ".obj" || fname_ext == ".off" || fname_ext == ".ply" || fname_ext == ".stl")
            core.import_basemesh(fname);
        else if (fname_ext == ".xml")
            core.xml_load(fname);
        glfwPostEmptyEvent();
    }
}
int main(int argc, char* argv[]) {
    // google-breakpad
#ifdef USE_GOOGLE_BREAKPAD
#   ifdef WIN32
    new google_breakpad::ExceptionHandler(
        L".", NULL, google_breakpad_callback, NULL, google_breakpad::ExceptionHandler::HANDLER_ALL);
    signal(SIGABRT, sigabrt_handler);
    signal(SIGINT , sigint_handler );
#   endif
#endif
    
    glfwWindowHint(GLFW_SAMPLES, 4);

    glfw_util::InitConfig config;
    config.callback.framebuffersize = callback::framebuffersize;
    config.callback.key             = callback::key            ;
    config.callback.mousebutton     = callback::mousebutton    ;
    config.callback.cursorpos       = callback::cursorpos      ;
    config.callback.drop            = callback::drop           ;
    core.glfw_control = glfw_util::init(config);
    
    glewInit();

    // set fontscaling for anttweakbar to conform with high dpi displays (IMPORTANT: TwDefine must be called before TwInit!)
    std::stringstream twStream;
    twStream << " GLOBAL fontscaling=" << getDevicePixelRatio(core.glfw_control.window) << ' ';
    TwDefine(twStream.str().c_str());
    // since the fontscaling is set globally only once, moving the window to another display with different pixel ratio
    // could break the bar functionality - TO CHECK - possible overcome: re-init the bar after a display change

    TwInit(TW_OPENGL, nullptr);
    core.init();
    if (argc == 2) {
        // process command line argument
        string fname = argv[1];
        auto fname_ext = fname.substr(fname.size() - 4, 4);
        boost::algorithm::to_lower(fname_ext);
        if (fname_ext == ".obj" || fname_ext == ".off" || fname_ext == ".ply" || fname_ext == ".stl")
            core.import_basemesh(fname);
        else if (fname_ext == ".xml")
            core.xml_load(fname);
    }
    
    core.glfw_control.idle_func[0] = display;
    glfw_util::start_loop(core.glfw_control);
}
