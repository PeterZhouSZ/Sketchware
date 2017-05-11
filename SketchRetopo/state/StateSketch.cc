#include "../SketchRetopo.hh"
#include <kt84/container_util.hh>
#include <kt84/MinSelector.hh>
#include <kt84/graphics/graphics_util.hh>
#include <kt84/vector_cast.hh>
#include <kt84/ScopeExit.hh>
#include "../curvenetwork/helper.hh"
#include "../curvenetwork/Circulator.hh"
using namespace std;
using namespace Eigen;
using namespace kt84;
using namespace kt84::container_util;
using namespace kt84::graphics_util;
using namespace curvenetwork;

namespace {
    auto& core = SketchRetopo::get_instance();
}

StateSketch::StateSketch()
    : State("basic sketching")
{}

void StateSketch::enter_state() {
    core.common_sketch_curve.clear();
}

void StateSketch::display() {
	
    // current curve
    glBegin(GL_LINE_STRIP);
    glColor3f(core.configRender.color.active);
    for (auto& pn : core.common_sketch_curve)
        glVertex3dv(&pn[0]);
    glEnd();
}

void StateSketch::mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    if (!core.common_pn_mouse) return;
    Vector3d mouse_pos = core.common_pn_mouse->head(3);
    
    if (core.common_key_pressed['z'])       // fix weird problem around clicked area
    {
        // collect halfedges crossing the snapping sphere outward
        vector<pair<double, Halfedge*>> crossing_halfedges;       // pair<angle, halfedge> (angle is used later for sorting)
        vector<Halfedge*> halfedges_to_delete;
        for (auto& h : core.curvenetwork.halfedges) {
            double dist0 = pn_norm(*core.common_pn_mouse - h.from_vertex()->pn);
            double dist1 = pn_norm(*core.common_pn_mouse - h.vertex       ->pn);
            double dist_threshold = core.configTemp.snapSize();
            if (dist0 < dist_threshold && dist_threshold < dist1)
                crossing_halfedges.push_back(make_pair(0.0, &h));
            // erase halfedges that are completely inside snapping radius
            if (dist0 < dist_threshold && dist1 < dist_threshold)
                halfedges_to_delete.push_back(&h);
        }
    
        if (crossing_halfedges.empty()) return;
        core.memento_store();
        
        for (auto h : halfedges_to_delete)
            delete_halfedge(h);
    
        // make new vertex at mouse pos
        auto v_center = core.curvenetwork.new_vertex();
        v_center->pn = *core.common_pn_mouse;
    
        // sort halfedge according to angle with respect to the center normal and first halfedge
        Vector3d d0 = (crossing_halfedges[0].second->from_vertex()->pn - v_center->pn).head(3);
        Vector3d n = v_center->pn.tail(3);
        Vector3d d1 = n.cross(d0);
        for (auto& p : crossing_halfedges) {
            Vector3d d = (p.second->from_vertex()->pn - v_center->pn).head(3);
            double angle = atan2(d1.dot(d), d0.dot(d));
            if (angle < 0) angle += 2 * util::pi();
            p.first = angle;
        }
        boost::range::sort(crossing_halfedges);
    
        // make new halfedges
        vector<Halfedge*> new_halfedges;
        for (auto& p : crossing_halfedges) {
            auto h0 = core.curvenetwork.new_halfedge();
            auto h1 = core.curvenetwork.new_halfedge();
            new_halfedges.push_back(h0);
            set_vertex_halfedge(v_center, p.second->from_vertex(), h0, h1);
            set_next_prev(h0, p.second);
            set_next_prev(p.second->opposite, h1);
            h1->is_corner = true;
        }
    
        // set next/prev between new halfedges
        for (int i = 0; i < crossing_halfedges.size(); ++i) {
            auto h0 = new_halfedges[i];
            auto h1 = at_mod(new_halfedges, i + 1);
            set_next_prev(h1->opposite, h0);
        }
    
        core.curvenetwork.garbage_collect();
        core.curvenetwork.generate_halfchains();
        core.generate_patches();
        core.curvenetwork.invalidate_displist();
    } else if (core.common_key_pressed['v']) // manually delete patch
    {
        MinSelector<Patch*> patch_selector;
        for (auto& patch : core.curvenetwork.patches) {
            // bounding box test
            AlignedBox3d bbox;
            for (auto v : patch.vertices())
                bbox.extend(patch.data(v).pn.head(3));
            Vector3d snap_margin = Vector3d::Constant(core.configTemp.snapSize());
            bbox.extend(bbox.max() + snap_margin);
            bbox.extend(bbox.min() - snap_margin);
            if (!bbox.contains(mouse_pos))
                continue;
            core.memento_store();
            patch_selector.update(patch.distance(mouse_pos), &patch);
        }
        if (patch_selector.score > core.configTemp.snapSize())
            return;
        delete_patch(patch_selector.value);
        core.curvenetwork.garbage_collect();
    } else if (core.common_key_pressed['b'] || core.common_key_pressed['n']) // generate curve along basemesh boundary starting from the clicked basemesh edge
    {
        MinSelector<BaseMesh::HHandle> h_start;
        for (auto v : core.basemesh.vertices()) {
            if (!core.basemesh.is_boundary(v))
                continue;
            h_start.update((mouse_pos - vector_cast<3, Vector3d>(core.basemesh.point(v))).norm(), core.basemesh.halfedge_handle(v));
        }
        if (h_start.score > core.configTemp.snapSize())
            return;
        core.trace_basemesh_boundary(h_start.value, core.common_key_pressed['b']);
    }
}
void StateSketch::mouse_up(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    auto scope_guard = kt84::make_ScopeExit([&](){ enter_state(); });
    
    if (core.common_sketch_curve.size() < 3)
        return;
    
    core.memento_store();
    
    if (core.common_key_pressed['c'])           // erase parts of curve network crossing with scribble
    {
        core.curvenetwork.erase_curve(core.common_sketch_curve);
    }
    else if (core.common_key_pressed['x'])
        /*
            manually generate patch if the sketched stroke starts from an edgechain and the 
            stroke's ending point belongs to its left side halfchain which forms a loop of halfchains
        */
    {
        // select non-corner vertex closest to mouse position |
        //----------------------------------------------------+
        MinSelector<Vertex*> vertex_selector;
        for (auto& v : core.curvenetwork.vertices) {
            if (v.is_deleted || v.is_corner()) continue;
            vertex_selector.update(pn_norm(v.pn - core.common_sketch_curve.front()), &v);
        }
        if (vertex_selector.score > core.configTemp.snapSize())
            return;
        auto selected_vertex = vertex_selector.value;
        // select halfchain on the side belonging to the stroke's ending point |
        //---------------------------------------------------------------------+
        Vector3d n = selected_vertex->pn.tail(3);
        Vector3d d1 = selected_vertex->halfedge->toVector3d().normalized();
        Vector3d d2 = (core.common_sketch_curve.back() - selected_vertex->pn).head(3).normalized();
        double is_left = n.cross(d1).dot(d2);
        Halfchain* c = (is_left > 0 ? selected_vertex->halfedge : selected_vertex->halfedge->opposite)->halfchain;
        // partially copied from SketchRetopo::generate_patches() |
        // -------------------------------------------------------+
        if (c->patch)
            // this halfchain already has a patch
            return;
        if (!Patch::is_valid_for_patch(c))
            return;
        core.generate_patch(c);
        core.curvenetwork.garbage_collect();

    } else {
        if (core.common_sketch_curve.is_loop)
            core.curvenetwork.add_curve_loop(core.common_sketch_curve);
        else
            core.add_curve_open_delegate(core.common_sketch_curve, !ctrl_pressed);
    }
    
    core.generate_patches();
}
void StateSketch::keyboard(unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    if (key == 'g') {
        // erase all open-ended curves
        vector<Edgechain*> erased_curves;
        for (auto& v : core.curvenetwork.vertices) {
            if (v.is_openend())
                erased_curves.push_back(v.halfedge->halfchain->edgechain);
        }
        
        container_util::remove_duplicate(erased_curves);
        if (erased_curves.empty())
            return;
        
        core.memento_store();
        
        vector<Halfedge*> erased_halfedges;
        for (auto e : erased_curves)
            erased_halfedges.push_back(e->halfchain[0]->halfedge_front);
        
        for (auto h : erased_halfedges)
            core.curvenetwork.erase_curve_sub(h);
        
        core.generate_patches();
    }
}
