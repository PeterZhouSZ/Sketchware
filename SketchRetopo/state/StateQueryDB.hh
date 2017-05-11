#pragma once
#include "State.hh"
#include <LearningQuadrangulationsCore/learning_quadrangulations.h>
#include "../curvenetwork/decl.hh"
#include <kt84/geometry/Polyline_PointNormal.hh>

namespace learnquad {
    struct Point5d {
        double x, y, z, u, v;
        template <class Archive>
        void serialize(Archive& archive);
        template <class XYZ, class UV>
        static Point5d make(XYZ&& xyz, UV&& uv);
    };
    struct QueryData {
        std::vector<vcg::Point3d> basemesh_vertices_xyz;
        std::vector<vcg::Point3d> basemesh_vertices_normal;
        std::vector<vcg::Point2d> basemesh_vertices_uv;
        std::vector<vector<int>>  basemesh_faces;
        std::vector<vcg::tri::pl::num_type> num_subdiv;
        std::vector<vcg::Point3d> boundary_xyz;
        std::vector<vcg::Point3d> boundary_uv;
        struct Stroke {
            std::deque<vcg::Point3d> points_xyz;
            std::deque<vcg::Point3d> points_uv;
            bool is_loop;
        };
        vector<Stroke> strokes;
        // send data to Giorgio's interface
        void send_basemesh(vcg::tri::pl::PatchRetopologizer<vcg::tri::pl::PolyMesh, vcg::tri::pl::TMesh>& retopologizer);
        void send_boundary(vcg::tri::pl::PatchRetopologizer<vcg::tri::pl::PolyMesh, vcg::tri::pl::TMesh>& retopologizer);
        void send_strokes (vcg::tri::pl::PatchRetopologizer<vcg::tri::pl::PolyMesh, vcg::tri::pl::TMesh>& retopologizer);
    };
}

struct StateQueryDB : public State {
    learnquad::QueryData querydata;
    int lookup_index = -1;
    bool freeze = false;            // set true to freeze currently shown result
    curvenetwork::Patch* selected_patch = nullptr;
    kt84::Polyline_PointNormal current_stroke;
    enum struct Status {
        idle = 0,
        started,
        progress,
        complete,
        flushing
    } status;
    std::mutex mtx_status;
    std::chrono::nanoseconds elapsed_time;
    std::string status_msg;
    void update_status_msg();
    bool dbg_export = false;
    bool dbg_dont_project = false;
    
    StateQueryDB();
    void update_patch();

    // virtual functions override
    bool show_brushSize_snap() override { return true; }
    void enter_state() override;
    void exit_state() override;
    void display() override;
    void mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    void mouse_up  (int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    void mouse_move(int mouse_x, int mouse_y) override;
    void keyboard(unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed) override;
    EditLevel get_editLevel() const override { return EditLevel::QUAD_MESH; }
};
