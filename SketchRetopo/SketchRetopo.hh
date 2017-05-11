#pragma once

// external libraries
#include <rtcore/common/accel.h>
#include <rtcore/common/intersector.h>
#include <geodesic/geodesic_algorithm_exact.h>

// kt84
#include <kt84/tw_util.hh>
#include <kt84/glfw_util.hh>
#include <kt84/geometry/CameraFree.hh>
#include <kt84/geometry/CameraUpright.hh>
#include <kt84/geometry/PolylineT.hh>
#include <kt84/graphics/ProgramObject.hh>
#include <kt84/graphics/TextureObjectT.hh>

// core data structure
#include "BaseMesh.hh"
#include "curvenetwork/Core.hh"
#include "curvenetwork/Vertex.hh"
#include "curvenetwork/Halfchain.hh"
#include "curvenetwork/Edgechain.hh"
#include "curvenetwork/Patch.hh"
#include <kt84/Memento.hh>

// config
#include "ConfigSaved.hh"
#include "ConfigTemp.hh"
#include "ConfigRender.hh"

#include "state/StateSketch.hh"            // edit at curve network level
#include "state/StateLaser.hh"
#include "state/StateCylinder.hh"
#include "state/StateDeformCurve.hh"
#include "state/StateEditCorner.hh"
#include "state/StateEditTopology.hh"        // edit at quad mesh level
#include "state/StateEdgeLoop.hh"
#include "state/StateMoveVertex.hh"
#include "state/StateQueryDB.hh"

// learning quadrangulations
#include <LearningQuadrangulationsCore/learning_quadrangulations.h>
#include "learnquad/Notifiable.hh"

//
#include "untitled\Contour2D.h"
#include "kt84\MinSelector.hh"

struct SketchRetopo {
    static SketchRetopo& get_instance() {
        static SketchRetopo instance;
        return instance;
    }
    
    // ctor for initializing primitive member data
    SketchRetopo();
    
    // top level init (executed only once)
    void init();
    
    // core data structure
    BaseMesh     basemesh;
    curvenetwork::Core curvenetwork;
    
    // patch generation is implemented here (instead of inside CurveNetwork) because patch interior vertices need to be projected onto basemesh
    bool is_loop_ccw(const kt84::Polyline_PointNormal& loop) const;
    curvenetwork::Patch* generate_patch(curvenetwork::Halfchain* halfchain);
    void generate_patches();
    
    // delegates to curvenetwork::Core::add_curve_open(), takes care of snapping endpoints and projection
    void add_curve_open_delegate(kt84::Polyline_PointNormal& curve, bool corner_at_openend = true);
    
    // compute interior vertices position/normal by Laplacian smoothing
    void compute_patch_interior_pn(curvenetwork::Patch* patch);
    
    // compute interior vertices position/normal by harmonic parameterization. The user must specify (click) one base mesh face as a seed to extract the mesh subset using floodfill.
    std::vector<BaseMesh::FHandle> parameterize_basemesh_under_patch(curvenetwork::Patch* patch, BaseMesh::FHandle submesh_seed);
    void compute_patch_interior_pn_harmonic(curvenetwork::Patch* patch, BaseMesh::FHandle submesh_seed);
    
    // cylinder sketching
    void add_cylinder_curve(kt84::Polyline_PointNormal cylinder_curve);     // by-value argument is intentional here...
    
    // extract a curve from cutting plane (laser sketching)
    kt84::Polyline_PointNormal trace_on_plane(const kt84::PointNormal& pn_start, const Eigen::Vector3d& plane_normal, double dist_max);
    
    // edge loop insertion
    void                       edgeLoop_insert (curvenetwork::Patch* patch_start, curvenetwork::Patch::HHandle h_start, double t);
    kt84::Polyline_PointNormal edgeLoop_preview(curvenetwork::Patch* patch_start, curvenetwork::Patch::HHandle h_start, double t) const;
    
    // walking over edge loop, used by EditSubdiv mode
    std::vector<std::pair<curvenetwork::Patch*, curvenetwork::Patch::HHandle>> edgeLoop_walk(curvenetwork::Patch* patch_start, curvenetwork::Patch::HHandle h_start) const;
    
    // vertex moving/smoothing
    void vertices_move  (const kt84::PointNormal& pn_center, const Eigen::Vector3d& center_offset, double radius);
    void vertices_smooth(const kt84::PointNormal& pn_center, double radius);
    auto vertices_smooth_sub(curvenetwork::Patch* patch, curvenetwork::Patch::VHandle patch_v, std::vector<curvenetwork::Edgechain*>& affected_edgechains) -> boost::optional<kt84::PointNormal> const;
    void vertices_smooth_global();
    void set_halfchain_pn_from_patch(curvenetwork::Halfchain* c);         // inverse of Patch::set_boundary_pn(): set PointNormal of its surrounding curve network vertices
    
    // snap curvenetwork::Vertex and curvenetwork::Patch::Vertex to symmetry (x=0) plane based on segment size threshold
    void snap_symmetry();
    
    // generate edgechains along basemesh boundary (taking symmetry flag into account); keep_exact: keep exactly the same chain of edges in the basemesh (i.e., no inward shifting, no resampling)
    void trace_basemesh_boundary(BaseMesh::HHandle h_start, bool keep_exact);
    
    // geodesic
    geodesic::Mesh                   geodesic_mesh;
    geodesic::GeodesicAlgorithmExact geodesic_algorithm;
    kt84::Polyline_PointNormal       geodesic_compute(const kt84::PointNormal& pn0, const kt84::PointNormal& pn1);
    void                             geodesic_init();
    
    // camera
    kt84::Camera*       camera;
    kt84::CameraFree    camera_free;
    kt84::CameraUpright camera_upright;
    
    // config data
    ConfigRender configRender;
    ConfigSaved  configSaved;
    ConfigTemp   configTemp;
    
    // Intel Embree ray tracer used for projection
    embree::Ref<embree::Accel      > embree_accel;
    embree::Ref<embree::Intersector> embree_intersector;
    void                             embree_init();
    
    // interection/projection
    boost::optional<kt84::PointNormal> intersect_convert(const embree::Hit& hit) const;
    embree::Hit                        intersect(const kt84::PointNormal& pn_in) const;
    embree::Hit                        intersect(int mouse_x, int mouse_y) const;
    void project(kt84::PointNormal& pn) const;
    void project(kt84::Polyline_PointNormal& polyline) const { /*for (auto& pn : polyline) project(pn);*/ }
    void reproject_all();       // projects all vertices again (i.e., curvenetwork::Vertex::pn, curvenetwork::Patch::Vertex::pn)
	float read_depth(int x, int y)const;
    
    // learning quadrangulation
    struct LearnQuad {
        std::shared_ptr<vcg::tri::pl::PatchDBServer<vcg::tri::pl::PolyMesh>> dbserver;
        vcg::tri::pl::MT_PatchPriorityLookup<vcg::tri::pl::PolyMesh> lookup;
        std::shared_ptr<Notifiable> notifiable;
        std::mutex mtx_display;
        std::shared_ptr<vcg::tri::pl::PatchRetopologizer<vcg::tri::pl::PolyMesh,vcg::tri::pl::TMesh>> retopologizer;
    } learnquad;
    
    // common behavior. if true is returned, event is not passed to the current state.
    enum class CommonDragMode {
        None,
        Smooth,
    }                                     common_dragMode;
    std::vector<curvenetwork::Edgechain*> common_affected_edgechains;
    boost::optional<kt84::PointNormal>    common_pn_mouse;
    Eigen::Vector2i                       common_mouse_pos;
    Eigen::Vector2i                       common_mouse_pos_prev;
    kt84::Polyline_PointNormal            common_sketch_curve;
    bool                                  common_key_pressed[256];
    bool                                  common_button_left_pressed;
    bool                                  common_button_right_pressed;
    bool                                  common_button_middle_pressed;
    bool                                  common_shift_pressed;             // to know modifier key state during motion event
    bool                                  common_ctrl_pressed;
    bool                                  common_alt_pressed;
    void display_pre ();
    void display_post();
    bool common_mouse_down(int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed);
    bool common_mouse_up  (int mouse_x, int mouse_y, Button button, bool shift_pressed, bool ctrl_pressed, bool alt_pressed);
    bool common_mouse_move(int mouse_x, int mouse_y);
    bool common_keyboard  (unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed);
    bool common_keyboardup(unsigned char key, bool shift_pressed, bool ctrl_pressed, bool alt_pressed);
    
    void render_default();
    void render_quadmesh_only();
    void render_basemesh_only();
	void render_basemesh();
	void render_patches();
	void render_patches_fill();
	void render_patches_line();
	void render_coordinateAxis();
	void render_boundingBox();
    
    // state
    State* state;
    StateSketch       stateSketch      ;
    StateDeformCurve  stateDeformCurve ;
    StateLaser        stateLaser       ;
    StateCylinder     stateCylinder    ;
    StateEditCorner   stateEditCorner  ;
    StateEditTopology stateEditTopology;
    StateQueryDB      stateQueryDB;
    StateMoveVertex   stateMoveVertex  ;
    StateEdgeLoop     stateEdgeLoop    ;
    enum class EnumState {
        Sketch      = 0,
        DeformCurve ,
        Laser       ,
        Cylinder    ,
        EditCorner  ,
        EditTopology,
        QueryDB     ,
        MoveVertex  ,
        EdgeLoop    ,
    };
    void      state_set(EnumState enumState);
    EnumState state_get() const;
    
    // undo-redo with "Memento" design pattern
    kt84::Memento<curvenetwork::Core> memento;
    void memento_store();
    void memento_undo();
    void memento_redo();
    
    // shaders
    kt84::ProgramObject program_basemesh;
    kt84::ProgramObject program_basic;
    kt84::ProgramObject program_corner;
    void                program_init();
    int stencil_index;
    
    // material texture png file (encoded using base64)
    kt84::TextureObject texture_overlay;
    std::string         material_texture_png_base64;
    kt84::TextureObject material_texture;
    bool                material_texture_load(const std::string& fname);
    bool                texture_overlay_load(const std::string& fname);
    void                material_texture_update();
    
    // AntTweakBar
    TwBar* bar_main;
    void   bar_init();

    // GLFW
    kt84::glfw_util::LoopControl glfw_control;
    
    // file IO
    bool import_basemesh(const std::string& fname);
    bool export_basemesh(std::string fname) const;
    bool export_retopomesh(std::string fname) const;
    bool xml_load(const std::string& fname);
    bool xml_save(std::string fname);

	// Mesh contour
	SketcherCore::Contour2D * contour2D;

private:
	//Event
	kt84::MinSelector<curvenetwork::Vertex*> & look_for_closest_non_corner_vertex();
	void perform_smoothing_on_the_corresponding_edgechain(kt84::MinSelector<curvenetwork::Vertex*> & v_closest);
	vector<curvenetwork::Patch*> & collect_affected_patches();
};

