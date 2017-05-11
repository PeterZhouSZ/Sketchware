#pragma once
#include "State.hh"
#include <boost/optional.hpp>
#include <kt84/geometry/PointNormal.hh>

struct StateEditCorner : public State {
    boost::optional<kt84::PointNormal> pn_prev;
    
    StateEditCorner();
    void enter_state() override;
    void display() override;
    void mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    void mouse_up  (int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    EditLevel get_editLevel() const override { return EditLevel::CURVE_NETWORK; }
};
