#pragma once
#include "State.hh"

struct StateSketch : public State {
    StateSketch();
    void enter_state() override;
    void display() override;
    void mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    void mouse_up  (int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    void keyboard(unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    EditLevel get_editLevel() const override { return EditLevel::CURVE_NETWORK; }
};

