#include "../SketchRetopo.hh"
#include <kt84/graphics/graphics_util.hh>
#include <kt84/MinSelector.hh>
#include <kt84/util.hh>
#include "../curvenetwork/Circulator.hh"
#include "../curvenetwork/helper.hh"
using namespace std;
using namespace Eigen;
using namespace kt84;
using namespace kt84::graphics_util;
using namespace curvenetwork;

namespace {
    auto& core = SketchRetopo::get_instance();
}

StateEditCorner::StateEditCorner()
    : State("edit corner")
{}
void StateEditCorner::enter_state() {
    pn_prev = boost::none;
}
void StateEditCorner::display() {
    if (!pn_prev || !core.common_pn_mouse)
        return;
    
    glDisable(GL_DEPTH_TEST);
    glColor3f(core.configRender.color.active);
    glLineWidth(5);
    glBegin(GL_LINES);
    glVertex3dv(&(*pn_prev      )[0]);
    glVertex3dv(&(*core.common_pn_mouse)[0]);
    glEnd();
}
void StateEditCorner::mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    if (!core.common_pn_mouse)
        return;
    
    if (ctrl_pressed) {
        // flip corner/non-corner for valence 2 vertices
        MinSelector<Vertex*> v_corner;
        MinSelector<Vertex*> v_side  ;
        for (auto& v : core.curvenetwork.vertices) {
            if (v.is_deleted || v.valence() != 2)
                continue;
            
            double dist = pn_norm(*core.common_pn_mouse - v.pn);
            if (core.configTemp.snapSize() < dist)
                continue;
            
            (v.is_corner() ? v_corner : v_side).update(dist, &v);
        }
        
        if (!v_corner.value && !v_side.value)
            return;
        
        core.memento_store();
        
        if (v_corner.value)
            core.curvenetwork.flip_corner_valance2(v_corner.value);
        else
            core.curvenetwork.flip_corner_valance2(v_side.value);
        
        core.generate_patches();
        return;
    }
    
    pn_prev = *core.common_pn_mouse;
}
void StateEditCorner::mouse_up  (int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    if (!pn_prev || !core.common_pn_mouse) {
        enter_state();
        return;
    }
    
    MinSelector<Vertex*> v_min;
    for (auto& v : core.curvenetwork.vertices) {
        if (v.is_deleted || !v.is_corner())
            continue;
        
        double dist = pn_norm(*core.common_pn_mouse - v.pn);
        v_min.update(dist, &v);
    }
    
    if (v_min.score > core.configTemp.snapSize()) {
        enter_state();
        return;
    }
    
    int num_corners = 0;
    for (VIHIter h(v_min.value); h; ++h)
        num_corners += h->is_corner ? 1 : 0;
    
    Vector3d n = v_min.value->pn.tail(3);
    Vector3d d = (*pn_prev - *core.common_pn_mouse).head(3).normalized();
    
    Halfedge* h_selected = nullptr;
    for (VIHIter h(v_min.value); h; ++h) {
        Vector3d d0 = -h      ->toVector3d().normalized();
        Vector3d d1 =  h->next->toVector3d().normalized();
        double angle  = util::acos_clamped(d1.dot(d ));
        double angle0 = util::acos_clamped(d1.dot(d0));
        if (d1.cross(d ).dot(n) < 0) angle  = 2 * util::pi() - angle ;
        if (d1.cross(d0).dot(n) < 0) angle0 = 2 * util::pi() - angle0;
        if (angle < angle0) {
            h_selected = &*h;
            break;
        }
    }
    if (!h_selected) {
        // this should not happen!
        enter_state();
        return;
    }
    
    if (num_corners == 1 && h_selected->is_corner) {
        // corner vertex should have at least one halfedge with corner flag!
        enter_state();
        return;
    }
    
    core.memento_store();
    
    // flip corner flag
    util::flip_bool(h_selected->is_corner);
    auto patch = h_selected->patch();
    if (patch) {
        delete_patch(patch);
        core.generate_patch(h_selected->halfchain);
    }
    core.curvenetwork.invalidate_displist();
    
    enter_state();
}
