#pragma once
#include "State.hh"
#include <kt84/geometry/Polyline_PointNormal.hh>

struct StateEdgeLoop : public State {
    kt84::Polyline_PointNormal edgeLoop_preview;
    
    StateEdgeLoop();
    void enter_state() override;
    void display() override;
    void mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    void mouse_move(int mouse_x, int mouse_y) override;
    EditLevel get_editLevel() const override { return EditLevel::QUAD_MESH; }
};
