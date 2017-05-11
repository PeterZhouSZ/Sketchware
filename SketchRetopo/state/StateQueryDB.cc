#include "../SketchRetopo.hh"
#include "../helper.hh"
#include "../curvenetwork/helper.hh"
#include "../learnquad/convert_patch.hh"
#include <patchgen/decl.hh>
#include <kt84/MinSelector.hh>
#include <kt84/vector_cast.hh>
#include <kt84/graphics/graphics_util.hh>
#include <kt84/glut_clone/font.hh>
#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>
using namespace std;
using namespace Eigen;
using namespace kt84;
using namespace kt84::graphics_util;
using namespace vcg::tri::pl;

namespace learnquad {
    template <class Archive>
    void Point5d::serialize(Archive& archive) {
        archive(CEREAL_NVP(x));
        archive(CEREAL_NVP(y));
        archive(CEREAL_NVP(z));
        archive(CEREAL_NVP(u));
        archive(CEREAL_NVP(v));
    }
    template <class XYZ, class UV>
    Point5d Point5d::make(XYZ&& xyz, UV&& uv) { return { xyz[0], xyz[1], xyz[2], uv[0], uv[1] }; }
    void QueryData::send_basemesh(PatchRetopologizer<PolyMesh, TMesh>& retopologizer) {
        TMesh &tMesh = retopologizer.getTriMesh();
        tMesh.Clear();
        auto vIt = vcg::tri::Allocator<TMesh>::AddVertices(tMesh, basemesh_vertices_xyz.size());
        for (int ind = 0; ind < basemesh_vertices_xyz.size(); ++ind, ++vIt) {
            vIt->P()     = basemesh_vertices_xyz   [ind];
            vIt->N()     = basemesh_vertices_normal[ind];
            vIt->T().P() = basemesh_vertices_uv    [ind];
        }
        auto fIt = vcg::tri::Allocator<TMesh>::AddFaces(tMesh, basemesh_faces.size());
        for (int ind = 0; ind < basemesh_faces.size(); ++ind, ++fIt)
            for (int ind2 = 0; ind2 < 3; ++ind2)
                fIt->V(ind2) = &tMesh.vert[basemesh_faces[ind][ind2]];
    }
    void QueryData::send_boundary(PatchRetopologizer<PolyMesh, TMesh>& retopologizer) {
        retopologizer.getNumberOfCorners() = num_subdiv.size();
        retopologizer.getBoundaryLen() = num_subdiv;
        retopologizer.getBoundary().resize(retopologizer.getBoundaryLen().size());
        retopologizer.getBoundaryUV().resize(retopologizer.getBoundaryLen().size());
        int offset = 0;
        size_t num_subdiv_total = boundary_xyz.size();
        for (size_t s = 0; s < retopologizer.getBoundaryLen().size(); ++s) {
            retopologizer.getBoundary()[s].resize(num_subdiv[s] + 1);
            retopologizer.getBoundaryUV()[s].resize(num_subdiv[s] + 1);
            for (num_type i = 0; i <= num_subdiv[s]; ++i) {
                retopologizer.getBoundary()[s][i] = boundary_xyz[(offset + i) % num_subdiv_total];
                retopologizer.getBoundaryUV()[s][i] = boundary_uv[(offset + i) % num_subdiv_total];
            }
            offset += num_subdiv[s];
        }
    }
    void QueryData::send_strokes(PatchRetopologizer<PolyMesh, TMesh>& retopologizer) {
        retopologizer.getFlowMap().clear();
        retopologizer.getFlowMapUV().clear();
        for (auto& stroke : strokes) {
            PatchRetopologizer<PolyMesh, TMesh>::PointsToFlow(stroke.points_xyz, stroke.is_loop, retopologizer.getBoundary(), retopologizer.getFlowMap(), 200);
            PatchRetopologizer<PolyMesh, TMesh>::PointsToFlow(stroke.points_uv, stroke.is_loop, retopologizer.getBoundaryUV(), retopologizer.getFlowMapUV(), 200);
        }
    }
}

namespace {
    SketchRetopo& core = SketchRetopo::get_instance();
}

void StateQueryDB::update_patch() {
    lock_guard<mutex> lock(core.learnquad.mtx_display);
    if (!convert_patch(core.learnquad.lookup, lookup_index, *selected_patch))
        return;
    if (!dbg_dont_project) {
        for (auto v : selected_patch->vertices())
            core.project(selected_patch->data(v).pn); //The projection should be removed, but I don't know if it's just that simple yet.
    }
}

StateQueryDB::StateQueryDB()
    : State("query database")
{}
void StateQueryDB::enter_state() {
    core.learnquad.lookup.stop();
    lock_guard<mutex> guard(mtx_status);
    core.learnquad.notifiable->func = [&](
        MT_PatchPriorityLookupNotifiable::NotificationType notificationType,
        count_type nTemplatePatches,
        count_type nPatches,
        std::chrono::nanoseconds duration)
    {
        lock_guard<mutex> lock(mtx_status);
        if (status == Status::idle)     // operation cancelled
            return;
        using E = decltype(notificationType);
        status = notificationType == E::COMPLETE ? Status::complete : notificationType == E::FLUSHING ? Status::flushing : Status::progress;
        if (notificationType != E::COMPLETE && duration - elapsed_time < std::chrono::milliseconds(200))
          return;
        elapsed_time = duration;
        update_status_msg();
        if (!freeze)
            update_patch();
        glfwPostEmptyEvent();
    };
    elapsed_time = std::chrono::nanoseconds::zero();
    querydata = {};
    lookup_index = -1;
    selected_patch = nullptr;
    current_stroke = {};
    status = Status::idle;
    update_status_msg();
}
void StateQueryDB::exit_state() {
    core.learnquad.lookup.stop();
}
void StateQueryDB::display() {
    if (selected_patch) {
        glEnable(GL_STENCIL_TEST);
        glDepthMask(GL_FALSE);
        glStencilFunc(GL_GREATER, ++core.stencil_index, ~0);
        // draw current patch
        glColor4f(core.configRender.color.active, core.configRender.color.patch_alpha);
        selected_patch->render_phong_fill();
        glStencilFunc(GL_GREATER, ++core.stencil_index, ~0);
        // draw already drawn strokes
        glColor3d(core.configRender.color.active);
        for (auto& stroke : querydata.strokes) {
            glBegin(stroke.is_loop ? GL_LINE_LOOP : GL_LINE_STRIP);
            for (auto& p : stroke.points_xyz)
                glVertex3d(p);
            glEnd();
        }
        // draw stroke currently being drawn
        glBegin(GL_LINE_STRIP);
        for (auto& pn : current_stroke)
            glVertex3d(pn);
        glEnd();
        glDisable(GL_STENCIL_TEST);
        glDepthMask(GL_TRUE);
    }
    
    // display current lookup status in 2D |
    //-------------------------------------+
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);    glPushMatrix();     glLoadIdentity();       gluOrtho2D(0, core.camera->width, 0, core.camera->height);
    glMatrixMode(GL_MODELVIEW );    glPushMatrix();     glLoadIdentity();
    glColor3d(1, 1, 1);
    draw_bitmap_string(GLUT_BITMAP_HELVETICA_18, status_msg, core.camera->width / 2, core.camera->height - 20, 1);
    glMatrixMode(GL_PROJECTION);    glPopMatrix();
    glMatrixMode(GL_MODELVIEW );    glPopMatrix();
}
void StateQueryDB::mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    if (!core.common_pn_mouse) {
        enter_state();
        return;
    }
    Vector3d mouse_pos = core.common_pn_mouse->head(3);
    
    // find patch under mouse cursor
    MinSelector<curvenetwork::Patch*> patch_selector;
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
        patch_selector.update(patch.distance(mouse_pos), &patch);
    }
    if (patch_selector.score > core.configTemp.snapSize()) {
        // clicked elsewhere
        enter_state();
        return;
    }
    
    if (patch_selector.value != selected_patch) {
        // selected a new patch --> parameterize the patch and set query data
        core.learnquad.lookup.stop();
        selected_patch = patch_selector.value;
        querydata = {};
        
        // QueryData: num_subdiv
        int num_sides = selected_patch->num_corners();
        auto num_subdiv = selected_patch->num_subdiv();
        int num_patch_boundary_vertices = num_subdiv.sum();
        querydata.num_subdiv = vector_cast<vector<num_type>>(num_sides, num_subdiv);
        
        // parameterize part of basemesh under the current patch
        const auto p_corner0 = selected_patch->patch_corner(0);
        auto p_h = selected_patch->halfedge_handle(p_corner0);
        for (int i = 0; i < num_sides; ++i) {
            for (int j = 0; j < querydata.num_subdiv[i]; ++j, p_h = selected_patch->prev_halfedge_handle(p_h)) {
                // assign UVs to patch boundary vertices (needed for parameterization)
                double t = i + j / static_cast<double>(querydata.num_subdiv[i]);
                selected_patch->data(selected_patch->from_vertex_handle(p_h)).harmonic_uv << patchgen::get_boundary_geometry(num_sides, t);
            }
        }
        BaseMesh::FHandle submesh_seed = core.basemesh.face_handle(core.intersect(*core.common_pn_mouse).id0);
        if (!submesh_seed.is_valid()) {
            enter_state();
            return;
        }
        vector<BaseMesh::FHandle> submesh_faces = core.parameterize_basemesh_under_patch(selected_patch, submesh_seed);
    
        // QueryData: basemesh vertices & faces
        map<BaseMesh::VHandle, int> vtxmap;
        for (auto b_f : submesh_faces)
            for (auto b_v : core.basemesh.fv_range(b_f))
                if (vtxmap.find(b_v) == vtxmap.end())
                    vtxmap.insert({b_v, vtxmap.size()});
        querydata.basemesh_vertices_xyz   .resize(vtxmap.size());
        querydata.basemesh_vertices_normal.resize(vtxmap.size());
        querydata.basemesh_vertices_uv    .resize(vtxmap.size());
        for (auto& p : vtxmap) {
            querydata.basemesh_vertices_xyz   [p.second] = vector_cast<3, vcg::Point3d>(core.basemesh.point(p.first));
            querydata.basemesh_vertices_normal[p.second] = vector_cast<3, vcg::Point3d>(core.basemesh.normal(p.first));
            querydata.basemesh_vertices_uv    [p.second] = vector_cast<2, vcg::Point2d>(core.basemesh.data(p.first).harmonic_uv);
        }
        querydata.basemesh_faces.reserve(submesh_faces.size());
        for (auto b_f : submesh_faces) {
            vector<int> fv(3);
            int i = 0;
            for (auto b_v : core.basemesh.fv_range(b_f))
                fv[i++] = vtxmap[b_v];
            querydata.basemesh_faces.push_back(fv);
        }
    
        // QueryData: patch boundary vertices
        querydata.boundary_xyz.reserve(num_patch_boundary_vertices);
        querydata.boundary_uv .reserve(num_patch_boundary_vertices);
        p_h = selected_patch->halfedge_handle(p_corner0);
        for (int i = 0; i < num_patch_boundary_vertices; ++i) {
            auto p_v = selected_patch->from_vertex_handle(p_h);
            querydata.boundary_xyz.push_back(vector_cast<3, vcg::Point3d>(selected_patch->data(p_v).pn.head(3)));
            Vector3d uv;
            uv << selected_patch->data(p_v).harmonic_uv, 0;
            querydata.boundary_uv .push_back(vector_cast<3, vcg::Point3d>(uv));
            p_h = selected_patch->prev_halfedge_handle(p_h);
        }

        // format into Giorgio's API
        querydata.send_basemesh(*core.learnquad.retopologizer);
        querydata.send_boundary(*core.learnquad.retopologizer);
    
    } else {
        if (!shift_pressed)
            querydata.strokes.clear();
        
        // add stroke
        auto f = BaseMesh::FHandle(core.intersect(*core.common_pn_mouse).id0);
        if (!core.basemesh.data(f).floodfill_flag)
            return;
        current_stroke.push_back(*core.common_pn_mouse);
    }
}
void StateQueryDB::mouse_up  (int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    if (!current_stroke.empty()) {
        // add current stroke to query
        current_stroke.set_loop_by_distance(core.configTemp.snapSize());
        current_stroke.smooth(30);
        current_stroke.resample_by_length(core.configTemp.segmentSize());
        if (current_stroke.length() < core.configTemp.segmentSize()) {
            current_stroke.clear();
            if (querydata.strokes.empty())
                core.learnquad.lookup.stop();
            return;
        }
        querydata.strokes.push_back({});
        querydata.strokes.back().is_loop = current_stroke.is_loop;
        for (auto& pn : current_stroke) {
            auto hit = core.intersect(pn);
            auto f = BaseMesh::FHandle(hit.id0);
            if (!core.basemesh.data(f).floodfill_flag)
                continue;
            auto fv = core.basemesh.util_fv_vec(f);
            Vector2d uv = {0, 0};
            uv += (1 - hit.u - hit.v) * core.basemesh.data(fv[0]).harmonic_uv;
            uv += hit.u * core.basemesh.data(fv[1]).harmonic_uv;
            uv += hit.v * core.basemesh.data(fv[2]).harmonic_uv;
            querydata.strokes.back().points_xyz.push_back(vector_cast<3, vcg::Point3d>(pn));
            querydata.strokes.back().points_uv .push_back({uv.x(), uv.y(), 0});
        }
        current_stroke.clear();
        // perform loookup
        if (dbg_export) {
            using learnquad::Point5d;
            // export to XML
            ofstream fout("patchretopo_data.xml");
            cereal::XMLOutputArchive oarchive(fout);
            int nv = querydata.basemesh_vertices_xyz.size();
            vector<Point5d> basemesh_vertices(nv);
            for (int i = 0; i < nv; ++i)
                basemesh_vertices[i] = Point5d::make(querydata.basemesh_vertices_xyz[i], querydata.basemesh_vertices_uv[i]);
            oarchive(cereal::make_nvp("basemesh_vertices", basemesh_vertices));
            oarchive(cereal::make_nvp("basemesh_faces", querydata.basemesh_faces));
            oarchive(cereal::make_nvp("num_subdiv", querydata.num_subdiv));
            int nb = querydata.boundary_xyz.size();
            vector<Point5d> boundary_vertices(nb);
            for (int i = 0; i < nb; ++i)
                boundary_vertices[i] = Point5d::make(querydata.boundary_xyz[i], querydata.boundary_uv[i]);
            oarchive(cereal::make_nvp("boundary_vertices", boundary_vertices));
            vector<pair<bool, vector<Point5d>>> strokes;
            for (auto& s : querydata.strokes) {
                int n = s.points_xyz.size();
                vector<Point5d> points(n);
                for (int i = 0; i < n; ++i)
                    points[i] = Point5d::make(s.points_xyz[i], s.points_uv[i]);
                strokes.push_back(make_pair(s.is_loop, points));
            }
            oarchive(cereal::make_nvp("strokes", strokes));
        }
        core.memento_store();
        core.learnquad.lookup.stop();
        querydata.send_strokes(*core.learnquad.retopologizer);
        status = Status::started;
        update_status_msg();
        elapsed_time = std::chrono::nanoseconds::zero();
        core.learnquad.lookup.findPatchesFromBoundaryLength(
            querydata.num_subdiv.size(),
            querydata.num_subdiv,
            core.learnquad.retopologizer->getFlowMapUV(), core.learnquad.retopologizer);
        
        lookup_index = 0;
    }
}
void StateQueryDB::mouse_move(int mouse_x, int mouse_y) {
    if (core.common_pn_mouse && !current_stroke.empty()) {
        auto f = BaseMesh::FHandle(core.intersect(*core.common_pn_mouse).id0);
        if (!core.basemesh.data(f).floodfill_flag)
            return;
        current_stroke.push_back(*core.common_pn_mouse);
    }
}
void StateQueryDB::keyboard(unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) {
    switch (key) {
    case 'z':
    case 'x':
        lookup_index += (key == 'z' ? -1 : 1);
        lookup_index = util::clamp<int>(lookup_index, 0, core.learnquad.lookup.numberOfPatches() - 1);
        if (lookup_index >= 0) {
            update_patch();
            update_status_msg();
        }
        break;
    }
}

void StateQueryDB::update_status_msg() {
    if (status == Status::idle) {
        status_msg = "";
    } else if (status == Status::flushing) {
        status_msg = string_util::cat("flushing... ") + core.learnquad.lookup.numberOfPatches();
    } else {
        status_msg = string_util::cat("lookup ") +
            + (status == Status::started  ? "started..." : 
               status == Status::progress ? "in progress..." : 
               status == Status::complete ? "complete!"      : "canceled.") + '\n';
        auto num_patches = core.learnquad.lookup.numberOfPatches();
        if (num_patches == 0) {
            status_msg += "(no patches found yet)";
        } else {
            status_msg += string_util::cat("patch index: ") + lookup_index + " / " + (num_patches - 1);
        }
    }
}
