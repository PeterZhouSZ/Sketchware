#include "SketchRetopo.hh"
#include "curvenetwork/Circulator.hh"
#include <kt84/glfw_util.hh>
#include <kt84/container_util.hh>
#include <kt84/MinSelector.hh>
#include <kt84/tinyfd_util.hh>
#include "helper.hh"
#include <qdebug>
using namespace std;
using namespace Eigen;
using namespace kt84;

#define POINT_DEBUG_INFO

bool SketchRetopo::common_mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    if (button == Button::LEFT  ) common_button_left_pressed   = true;
    if (button == Button::RIGHT ) common_button_right_pressed  = true;
    if (button == Button::MIDDLE) common_button_middle_pressed = true;
    common_shift_pressed = shift_pressed;
    common_ctrl_pressed  = ctrl_pressed;
    common_alt_pressed   = alt_pressed;
    
    if (alt_pressed) {
        // changing view
        camera->mouse_down(mouse_x, mouse_y, 
            button == Button::LEFT   ? (ctrl_pressed ? Camera::DragMode::PAN : Camera::DragMode::ROTATE) :
            button == Button::MIDDLE ? Camera::DragMode::PAN    : Camera::DragMode::ZOOM);
        return true;
    }
    if (button == Button::RIGHT && shift_pressed && ctrl_pressed ) {
        configTemp.segmentSize.adjust_init(mouse_x);
        return true;
    }
    if (button == Button::RIGHT && shift_pressed) {
        configTemp.snapSize.adjust_init(mouse_x);
        return true;
    }
    if (button == Button::RIGHT && ctrl_pressed) {
        configTemp.quadSize.adjust_init(mouse_x);
        return true;
    }
    
    if (button == Button::LEFT && shift_pressed                 && state->get_editLevel() == State::EditLevel::CURVE_NETWORK) {
        common_dragMode = CommonDragMode::Smooth;
        memento_store();
        return true;
    }
    
    if (button == Button::LEFT && common_pn_mouse &&
        (state == &stateSketch     ||
         state == &stateCylinder   ||
         state == &stateDeformCurve))
    {
        common_sketch_curve.push_back(*common_pn_mouse);
    }
    
    return false;
}
bool SketchRetopo::common_mouse_up(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {

    if (button == Button::LEFT  ) common_button_left_pressed   = false;
    if (button == Button::RIGHT ) common_button_right_pressed  = false;
    if (button == Button::MIDDLE) common_button_middle_pressed = false;
    common_shift_pressed = shift_pressed;
    common_ctrl_pressed  = ctrl_pressed;
    common_alt_pressed   = alt_pressed;
    
	if (camera->drag_mode != Camera::DragMode::NONE) { // changing view

		if (camera->drag_mode == Camera::DragMode::ROTATE && shift_pressed) { camera->snap_to_canonical(); }// snap to closest canonical view
        if (camera->drag_mode == Camera::DragMode::PAN && configRender.auto_camera_center) {
            // set camera center to surface point where screen center is projected onto (as in Pixologic Sculptris)
            auto hit = intersect(camera->width / 2, camera->height / 2);
            if (hit) camera->update_center(intersect_convert(hit)->head(3));
        }
        camera->mouse_up();
		contour2D->generateContour();
        return true;
    }
    
    if (configTemp.snapSize   .is_adjusting()) { configTemp.snapSize   .adjust_done(); return true; }
    if (configTemp.quadSize   .is_adjusting()) { configTemp.quadSize   .adjust_done(); curvenetwork::Patch::quadSize = configTemp.quadSize(); return true; }
    if (configTemp.segmentSize.is_adjusting()) { configTemp.segmentSize.adjust_done(); return true; }
    
    if (common_dragMode == CommonDragMode::Smooth) {
		auto& affected_patches = collect_affected_patches();
        
        // update patch vertex positions
        container_util::remove_duplicate(affected_patches);
        for (auto patch : affected_patches) {
            if (patch->is_deleted) continue;
            patch->set_boundary_pn();
            compute_patch_interior_pn(patch);
        }
        curvenetwork.invalidate_displist();
        
        common_affected_edgechains.clear();
        common_dragMode = CommonDragMode::None;
        return true;
    }
    
    if (!common_sketch_curve.empty()) {
        // loop check, resampling, default smoothing, projection
        common_sketch_curve.is_loop = pn_norm(common_sketch_curve.back() - common_sketch_curve.front()) < configTemp.snapSize();
        common_sketch_curve.resample_by_length(configTemp.segmentSize());
        common_sketch_curve.smooth(2, 0.5, 0.5);
        project(common_sketch_curve);
    }
    
    return false;
}
bool SketchRetopo::common_mouse_move(int mouse_x, int mouse_y) {
    common_mouse_pos_prev = common_mouse_pos;
    common_mouse_pos << mouse_x, mouse_y;
    
    if (camera->drag_mode != Camera::DragMode::NONE) {
        // changing view
        camera->mouse_move(mouse_x, mouse_y);
        return true;
    }
    
    // changing real-valued parameters
    if (configTemp.snapSize.is_adjusting()) {
        configTemp.snapSize.adjust_move(mouse_x);
        // snap size shouldn't be larger than brush size.
        if (state == &stateMoveVertex) configTemp.snapSize.value = min<double>(configTemp.snapSize.value, configTemp.brushSize_moveVertex.value * 0.9);
        return true;
    }
    if (configTemp.brushSize_moveVertex.is_adjusting()) {
        configTemp.brushSize_moveVertex.adjust_move(mouse_x);
        configTemp.brushSize_moveVertex.value = max<double>(configTemp.brushSize_moveVertex.value, configTemp.snapSize.value * 0.9);
        return true;
    }
    if (configTemp.quadSize   .is_adjusting()) { configTemp.quadSize   .adjust_move(mouse_x); return true; }
    if (configTemp.segmentSize.is_adjusting()) { configTemp.segmentSize.adjust_move(mouse_x); return true; }
    
    // compute intersection
    common_pn_mouse = intersect_convert(intersect(mouse_x, mouse_y));
#ifdef POINT_DEBUG_INFO
	if (common_pn_mouse) {
		cout << "pn: (" << common_pn_mouse->x() << ", " << common_pn_mouse->y() << ", " << common_pn_mouse->z() << ")" << endl;
		qDebug() << "pn: (" << common_pn_mouse->x() << ", " << common_pn_mouse->y() << ", " << common_pn_mouse->z() << ")";
	}
#endif

    if (!common_pn_mouse)
        return true;
    
    project(*common_pn_mouse); //This needs to be rewritten.

#ifdef POINT_DEBUG_INFO
	if (common_pn_mouse) {
		qDebug() << "pn: (" << common_pn_mouse->x() << ", " << common_pn_mouse->y() << ", " << common_pn_mouse->z() << ")";
	}
#endif
    // common curve editing tool: smoothing
    if (common_dragMode == CommonDragMode::Smooth) {
		auto& v_closest = look_for_closest_non_corner_vertex();
        if (v_closest.score < configTemp.snapSize()) {
			perform_smoothing_on_the_corresponding_edgechain(v_closest);
        }
        return true;
    }
    
    if (common_pn_mouse && !common_sketch_curve.empty())
        common_sketch_curve.push_back(*common_pn_mouse);
    
    return false;
}
namespace {
void turntable_idle() {
    auto& core = SketchRetopo::get_instance();
    if (core.camera != &core.camera_upright) {
        auto eye    = core.camera->get_eye();
        auto center = core.camera->center;
        auto up     = core.camera->get_up();
        core.camera = &core.camera_upright;
        core.camera->init(eye, center, up);
        core.camera->update_center(center);
    }
    core.camera_upright.theta += core.configRender.turntable_speed;
}
}

bool SketchRetopo::common_keyboard(unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    common_key_pressed[key] = true;
    if (ctrl_pressed) {
        //const char *xml_types[1] = {"*.xml"};
        //const char *mesh_types[4] = {"*.obj","*.off","*.ply","*.stl"};
        string fname;
        switch (key) {
        case 'y':
            memento_redo();
            return true;
        case 'z':
            memento_undo();
            return true;
        case 'b':
            configRender.mode = configRender.mode == ConfigRender::Mode::BASEMESH_ONLY ? ConfigRender::Mode::DEFAULT : ConfigRender::Mode::BASEMESH_ONLY;
            return true;
        case 'o':        // load xml
            // if the change is not saved yet, confirm
            if (configTemp.autoSave.unsaved && !tinyfd_messageBox("SketchRetopo", "Current data is not saved yet. Continue?", "yesno", "question", 0))
                return true;
            if (!(fname = tinyfd_util::openFileDialog("SketchRetopo - load xml", "", {"*.xml"}, false)).empty())
                if (!xml_load(fname))
                    tinyfd_messageBox("SketchRetopo - load xml", "Error occurred!", "ok", "error", 1);
            return true;
        case 's':        // save xml
            // don't save if there's no change
            if (!configTemp.autoSave.unsaved)
                return true;
            if (configTemp.autoSave.filename.empty()) {
                if (!(fname = tinyfd_util::saveFileDialog("SketchRetopo - save xml", "retopo.xml", {"*.xml"})).empty())
                    if (!xml_save(fname))
                        tinyfd_messageBox("SketchRetopo - save xml", "Error occurred!", "ok", "error", 1);
                    return true;
            } else {
                configTemp.autoSave.unsaved = false;
                configTemp.autoSave.last_time = clock();
                // update ausoSave.filename if it has a specific form of <fname_core>.<fname_num>.xml
                string fname = configTemp.autoSave.filename;
                string fname_core;
                int fname_num;
                if (decompose_xml_fname(fname, fname_core, fname_num)) {
                    stringstream ss;
                    ss << fname_core << '.' << (fname_num + 1) << ".xml";
                    fname = ss.str();
                }
                xml_save(fname);
            }
            return true;
        case 'i':         // import obj
            // if the change is not saved yet, confirm
            if (configTemp.autoSave.unsaved && !tinyfd_messageBox("SketchRetopo", "Current data is not saved yet. Continue?", "yesno", "question", 0))
                return true;
            if (!(fname = tinyfd_util::openFileDialog("SketchRetopo - import base mesh", "", {"*.obj","*.off","*.ply","*.stl"}, false)).empty())
                if (!import_basemesh(fname))
                    tinyfd_messageBox("SketchRetopo - import base mesh", "Error occurred!", "ok", "error", 1);
            return true;
        case 'e':         // export obj
            if (!(fname = tinyfd_util::saveFileDialog("SketchRetopo - export retopo mesh", "retopo.obj", {"*.obj","*.off","*.ply","*.stl"})).empty())
                if (!export_retopomesh(fname))
                    tinyfd_messageBox("SketchRetopo - export retopo mesh", "Error occurred!", "ok", "error", 1);
            return true;
        }
        return false;
    }
    switch (key) {
        case 'q':
        case 'w':
        case 'e':
        case 'r':
        case 't':
        case 'a':
        case 's':
        case 'd':
        case 'f':
            // mode change
            state_set(
                key == 'q' ? EnumState::Sketch       :
                key == 'w' ? EnumState::DeformCurve  :
                key == 'e' ? EnumState::Laser        :
                key == 'r' ? EnumState::Cylinder     :
                key == 't' ? EnumState::EditCorner   :
                key == 'a' ? EnumState::EditTopology :
                key == 's' ? EnumState::QueryDB      :
                key == 'd' ? EnumState::MoveVertex   :
                key == 'f' ? EnumState::EdgeLoop     :
                EnumState::Sketch);
            return true;
        case ' ':
            configRender.mode = configRender.mode == ConfigRender::Mode::QUADMESH_ONLY ? ConfigRender::Mode::DEFAULT : ConfigRender::Mode::QUADMESH_ONLY;
            return true;
        case '0':       // reset camera center
            camera->center = basemesh.boundingBox.center();
            return true;
        case '8':
            // easy camera save/load
            {
               /* stringstream ss;
                auto eye    = camera->get_eye();
                auto center = camera->center;
                auto up     = camera->get_up();
                auto window = glfwGetCurrentContext();
                auto windowPos = glfw_util::getWindowPos(window);
                auto windowSize = glfw_util::getWindowSize(window);
                ss <<
                    eye   [0] << " " <<
                    eye   [1] << " " <<
                    eye   [2] << " " <<
                    center[0] << " " <<
                    center[1] << " " <<
                    center[2] << " " <<
                    up    [0] << " " <<
                    up    [1] << " " <<
                    up    [2] << " " <<
                    windowPos.x << " " <<
                    windowPos.y << " " <<
                    windowSize.width << " " <<
                    windowSize.height;
                auto str = ss.str();
                if ((str = tinyfd_inputBox("SketchRetopo", "camera parameter", str.c_str())).empty())
                    return true;
                ss.str(str);
                ss >>
                    eye   [0] >>
                    eye   [1] >>
                    eye   [2] >>
                    center[0] >>
                    center[1] >>
                    center[2] >>
                    up    [0] >>
                    up    [1] >>
                    up    [2] >>
                    windowPos.x >>
                    windowPos.y >>
                    windowSize.width >>
                    windowSize.height;
                camera->init(eye, center, up);
                glfw_util::setWindowPos(window, windowPos);
                glfw_util::setWindowSize(window, windowSize);*/
            }
            return true;
        case '7':
            // stats report
            cout << "# tri: " << basemesh.n_faces() << endl;
            {
                int n_quad = 0;
                int n_patch_tri = 0;
                int n_patch_rect = 0;
                int n_patch_pent = 0;
                for (auto& p : curvenetwork.patches) {
                    n_quad += p.n_faces();
                    int num_corners = p.num_corners();
                    ++(num_corners == 3 ? n_patch_tri : num_corners == 4 ? n_patch_rect : n_patch_pent);
                }
                cout << "# quad: " << n_quad << endl;
                cout << "# patch: " << n_patch_tri << "/" << n_patch_rect << "/" << n_patch_pent << endl;
            }
            return true;
        case '6':
            // turntable mode
            util::flip_bool(configRender.turntable_mode);
            glfw_control.poll_events = configRender.turntable_mode;
            if (configRender.turntable_mode)
                glfw_control.idle_func[1] = turntable_idle;
            else
                glfw_control.idle_func[1] = {};
            return true;
    }
    return false;
}
bool SketchRetopo::common_keyboardup(unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    common_key_pressed[key] = false;
    
    return false;
}

MinSelector<curvenetwork::Vertex*> & SketchRetopo::look_for_closest_non_corner_vertex() {
	MinSelector<curvenetwork::Vertex*> v_closest;
	for (auto& v : curvenetwork.vertices) {
		if (v.is_corner() || v.is_openend())
			continue;

		double dist = pn_norm(v.pn - *common_pn_mouse);
		v_closest.update(dist, &v);
	}

	return v_closest;
}
void SketchRetopo::perform_smoothing_on_the_corresponding_edgechain(MinSelector<curvenetwork::Vertex*> & v_closest) {
	auto c = v_closest.value->halfedge->halfchain;
	auto polyline = c->toPolyline();
	bool is_loop = polyline.is_loop;
	int  n = polyline.size();

	if (!is_loop && n < 5)
		return;

	polyline.smooth(25);
	auto h = c->halfedge_front;
	for (int i = 0; i < n; ++i) {
		auto& pn = h->from_vertex()->pn;
		pn = polyline[i];
		project(pn); //The projection should be removed, but I don't know if it's just that simple yet.

		h = h->next;
		if (!h)
			break;
	}

	common_affected_edgechains.push_back(c->edgechain);
}

vector<curvenetwork::Patch*> & SketchRetopo::collect_affected_patches() {
	vector<curvenetwork::Patch*> affected_patches;
	container_util::remove_duplicate(common_affected_edgechains);
	for (auto e : common_affected_edgechains) {
		for (int i = 0; i < 2; ++i) {
			auto patch = e->halfchain[i]->patch;
			if (patch)
				affected_patches.push_back(patch);
		}
	}
	return affected_patches;
}