#pragma once

#include "Button.hh"
#include <string>
#include <Eigen/Core>

struct State {
    std::string     state_name;
    
    State() {}
    State(const std::string& state_name_)
        : state_name (state_name_ )
    {}
    virtual ~State() {};
    
    // virtual functions (with empty default)
    virtual bool show_brushSize_snap() { return true; }
    virtual void enter_state() {}                // called when entering to this state
    virtual void exit_state() {}                // called when exiting from this state
    virtual void display() {}
    virtual void mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {}
    virtual void mouse_up  (int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {}
    virtual void mouse_move(int mouse_x, int mouse_y) {}
    virtual void keyboard(unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {}
    enum class EditLevel {
        CURVE_NETWORK,
        QUAD_MESH
    };
    virtual EditLevel get_editLevel() const = 0;
};
