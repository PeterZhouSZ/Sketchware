#pragma once
#include "State.hh"
#include "../curvenetwork/decl.hh"
#include <vector>

struct StateDeformCurve : public State {
    enum class Mode {
        None,
        DragCorner,
        DragSide,
        OverSketch,
    } mode;
    curvenetwork::Vertex*                 selected_vertex;
    Eigen::Vector3d                       prev_pos;
    
    StateDeformCurve();
    void enter_state() override;
    void display() override;
    void mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    void mouse_up  (int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    void mouse_move(int mouse_x, int mouse_y) override;
    EditLevel get_editLevel() const override { return EditLevel::CURVE_NETWORK; }
};
