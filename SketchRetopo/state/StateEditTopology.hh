#pragma once

#include "State.hh"
#include "../curvenetwork/decl.hh"
#include "../patchgen/PatchParam.hh"

struct StateEditTopology : public State {
    curvenetwork::Patch* current_patch;
    patchgen::PatchParam param;
    Eigen::Vector3d      p_prev;
    double               average_edge_length;
    int selected_variable;
    
    StateEditTopology();
    void enter_state() override;
    void display() override;
    void mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed_, bool ctrl_pressed, bool alt_pressed) override;
    void mouse_move(int mouse_x, int mouse_y) override;
    void keyboard(unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    EditLevel get_editLevel() const override { return EditLevel::QUAD_MESH; }
};
