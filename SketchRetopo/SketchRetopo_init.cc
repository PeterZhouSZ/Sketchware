#include <GL\glew.h>
#include "SketchRetopo.hh"
#include <algorithm>
#include <kt84/graphics/phong_tessellation.hh>
#include <kt84/Clock.hh>
#include <kt84/tinyfd_util.hh>
#include "helper.hh"

using namespace std;
using namespace Eigen;
using namespace kt84;
using namespace vcg::tri::pl;

namespace {
    auto& core = SketchRetopo::get_instance();
}

BaseMesh::BaseMesh()
    : has_texture(false)
{
    request_face_normals();
    request_vertex_normals();
    request_halfedge_texcoords2D();
}

ConfigRender::Color::Color()
    : background_bottom(0.25, 0.25, 0.25)
    , background_top(0, 0, 0)
    , active(1, 0.2, 0.1)
    , patch(0.7, 0.7, 0.7)
    , patch_alpha(0.2f)
{}

ConfigRender::ConfigRender()
    : mode(Mode::DEFAULT)
    , always_show_quads(true)
    , basemesh_render_line(false)
    , show_axis(false)
    , show_bbox(false)
    , use_ortho  (false)
    , show_expmap (false)
    , overlay_alpha(0.0)
    , overlay_v_flipped(true)
    , auto_camera_center(true)
    , corner_radius(0.005)
    , edge_width_interior(1)
    , edge_width_boundary(2)
    , singularity_radius(10)
    , patch_color_alpha(0.5)
    , light_pos(50, 100, 200)
    , quadmesh_only_show_auxiliary(false)
    , quadmesh_only_show_left_side(false)
    , turntable_mode(false)
    , turntable_speed(0.02)
{
    patch_color_rgb[0] << 0, 0, 0;
    patch_color_rgb[1] << 0, 0, 0;
    patch_color_rgb[2] << 0.2, 0.1, 0.9;
    patch_color_rgb[3] << 0.5, 0.5, 0.7;
    patch_color_rgb[4] << 0.7, 0.7, 0.7;
    patch_color_rgb[5] << 0.7, 0.6, 0.5;
    patch_color_rgb[6] << 0.2, 0.8, 0.3;
}

ConfigSaved::ConfigSaved()
    : projOffset(0)
    , symmetric(false)
{}

const double ConfigTemp::default_quadSize_ratio    = 0.03;
const double ConfigTemp::default_segmentSize_ratio = 0.002;

ConfigTemp::ConfigTemp()
    : snapSize            (0.02, util::dbl_max(), 0, true , 0.001)
    , brushSize_moveVertex(0.06, util::dbl_max(), 0, true , 0.001)
    , quadSize            (0   , util::dbl_max(), 0, false, 0    )       // these are set when loading basemesh
    , segmentSize         (0   , util::dbl_max(), 0, false, 0    )
    , cylinder_num_div(4)
    , loop_threshold_normal(0.3)
    , loop_threshold_area  (0.3)
{
}

AutoSave::AutoSave()
    : enabled (true)
    , interval(30)
    , unsaved(false)
    , last_time()
{}

SketchRetopo::SketchRetopo()
    : camera(&camera_free)
    , common_dragMode(CommonDragMode::None)
    , state(0)
    , bar_main(0)
    , stencil_index()
    , memento(150)
{
    fill(common_key_pressed, common_key_pressed + 256, false);
    state = &stateSketch;
}

void SketchRetopo::init() {
    //-----------------+
    // OpenGL settings |
    //-----------------+

	glewInit();
    glEnable(GL_DEPTH_TEST);
    // polygon offset
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0, 1.0);
    // color material (diffuse)
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);	
    // stencil op
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    // blend enabled
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // anti-aliasing. somehow not working?
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);    
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    
    // init material texture
    texture_overlay.init();
    texture_overlay.bind();
    texture_overlay.set_default_param();
    material_texture.init();
    material_texture.bind();
    material_texture.set_default_param();
    string base64_stringified[] = {
        #include "../../resource/material_default.base64.txt.stringify.txt"
    };
    material_texture_png_base64 = de_stringify(base64_stringified, sizeof(base64_stringified));
    material_texture_update();
    
    //bar_init();
    
    program_init();
    
    memento.init(150);
    
    state->enter_state();
    camera_free.auto_flip_y = camera_upright.auto_flip_y = false;
    
    // init LearningQuadrangulation data
    learnquad.dbserver = make_shared<PatchDBServer<PolyMesh>>();
    string db_filename;
#ifdef WIN32
    db_filename = string_util::cat(getenv("USERPROFILE")) + "\\patches.db";
#else
    db_filename = string_util::cat(getenv("HOME")) + "/patches.db";
#endif
    learnquad.dbserver->setFilename(db_filename);
    learnquad.dbserver->initialize();
    cout << "Loading DB..." << flush;
    {
        ClkSimple clk;
        learnquad.dbserver->load();
    }
    cout << "done. " << learnquad.dbserver->numberOfCachedPatches() << " templates loaded." << endl;
//#ifdef WIN32
//    _pclose(_popen("play C:\\Windows\\Media\\notify.wav", "r"));
//#endif
    auto& lookup = learnquad.lookup;
    lookup.setPatchDBServer(learnquad.dbserver);
    lookup.setWaitTime(chrono::milliseconds(100));
    lookup.setMaxSingularities(16);
    lookup.setMaxConcavities(0);
    lookup.setMinValence(3);
    lookup.setMaxValence(5);
    lookup.setMinSizeThresholdILP(1);
    learnquad.notifiable = make_shared<Notifiable>();
    lookup.registerNotifiable(dynamic_pointer_cast<MT_PatchPriorityLookupNotifiable>(learnquad.notifiable));
    using E = PatchCompare<PolyMesh>::CompareCriteria;
    lookup.setSortCriteria({E::FLOW_DISTANCE, E::SINGULARITYNUM, E::SINGULARITYTYPES, E::QUADS_QUALITY});
    learnquad.retopologizer = make_shared<PatchRetopologizer<PolyMesh, TMesh>>();
}

void SketchRetopo::geodesic_init() {
    vector<double> points;
    points.reserve(basemesh.n_vertices() * 3);
    for (auto v : basemesh.vertices()) {
        auto p = basemesh.point(v);
        points.push_back(p[0]);
        points.push_back(p[1]);
        points.push_back(p[2]);
    }
    
    vector<unsigned> faces;
    faces.reserve(basemesh.n_faces() * 3);
    for (auto f : basemesh.faces()) {
        for (auto v : basemesh.fv_range(f))
            faces.push_back(v.idx());
    }
    
    geodesic_mesh.initialize_mesh_data(points, faces);
    geodesic_algorithm = geodesic::GeodesicAlgorithmExact(&geodesic_mesh);
}

void SketchRetopo::embree_init() {
    // triangles
    auto triangles = static_cast<embree::BuildTriangle*>(embree::rtcMalloc(basemesh.n_faces() * sizeof(embree::BuildTriangle)));
    for (auto f : basemesh.faces()) {
        auto v = basemesh.fv_iter(f);
        
        int f_idx = f.idx();
        int v0_idx = v->idx();
        int v1_idx = (++v)->idx();
        int v2_idx = (++v)->idx();
        
        triangles[f_idx] = embree::BuildTriangle(v0_idx, v1_idx, v2_idx, f_idx);
    }
    
    // vertices
    auto vertices  = static_cast<embree::BuildVertex*>(embree::rtcMalloc(basemesh.n_vertices() * sizeof(embree::BuildVertex)));
    for (auto v : basemesh.vertices()) {
        OpenMesh::Vec3f p(basemesh.point(v));
        vertices[v.idx()] = embree::BuildVertex(p[0], p[1], p[2]);
    }
    
    // build
    embree_accel = embree::rtcCreateAccel("default", "default", triangles, basemesh.n_faces(), vertices, basemesh.n_vertices());
    embree_intersector = embree_accel->queryInterface<embree::Intersector>();
    
    // warm up?
    cout << "waiting for embree to warm up\n";
    auto clk = clock();
    while (clock() - clk < CLOCKS_PER_SEC);
}

void SketchRetopo::program_init() {
    // basemesh
    {
        string src_frag_stringified[] = {
            #include "../../resource/basemesh.frag.stringify.txt"
        };
        string src_vert_stringified[] = {
            #include "../../resource/basemesh.vert.stringify.txt"
        };
        auto src_frag = de_stringify(src_frag_stringified, sizeof(src_frag_stringified));
        auto src_vert = de_stringify(src_vert_stringified, sizeof(src_vert_stringified));
        
        ShaderObject frag;                                      ShaderObject vert;
        frag.init(ShaderObject::Type::FRAGMENT_SHADER);         vert.init(ShaderObject::Type::VERTEX_SHADER);
        frag.set_source(src_frag);                              vert.set_source(src_vert);
        frag.compile();                                         vert.compile();
        
        program_basemesh.init();
        program_basemesh.attach(frag);
        program_basemesh.attach(vert);
        program_basemesh.link();
        program_basemesh.enable();
        program_basemesh.set_uniform_1<int>("material_texture", 0);
        program_basemesh.set_uniform_1<int>("texture_overlay", 1);
        program_basemesh.disable();
    }
    
    // basic
    {
        string src_frag_stringified[] = {
            #include "../../resource/basic.frag.stringify.txt"
        };
        string src_vert_stringified[] = {
            #include "../../resource/basic.vert.stringify.txt"
        };
        auto src_frag = de_stringify(src_frag_stringified, sizeof(src_frag_stringified));
        auto src_vert = de_stringify(src_vert_stringified, sizeof(src_vert_stringified));
        
        ShaderObject frag;                                      ShaderObject vert;
        frag.init(ShaderObject::Type::FRAGMENT_SHADER);         vert.init(ShaderObject::Type::VERTEX_SHADER);
        frag.set_source(src_frag);                              vert.set_source(src_vert);
        frag.compile();                                         vert.compile();
        
        program_basic.init();
        program_basic.attach(frag);
        program_basic.attach(vert);
        program_basic.link();
    }
    
    // corner
    {
        string src_frag_stringified[] = {
            #include "../../resource/corner.frag.stringify.txt"
        };
        string src_vert_stringified[] = {
            #include "../../resource/corner.vert.stringify.txt"
        };
        auto src_frag = de_stringify(src_frag_stringified, sizeof(src_frag_stringified));
        auto src_vert = de_stringify(src_vert_stringified, sizeof(src_vert_stringified));
        
        ShaderObject frag;                                      ShaderObject vert;
        frag.init(ShaderObject::Type::FRAGMENT_SHADER);         vert.init(ShaderObject::Type::VERTEX_SHADER);
        frag.set_source(src_frag);                              vert.set_source(src_vert);
        frag.compile();                                         vert.compile();
        
        program_corner.init();
        program_corner.attach(frag);
        program_corner.attach(vert);
        program_corner.link();
        program_corner.enable();
        program_corner.set_uniform_1("uni_symmetric_mirror", 0);
        program_corner.disable();
    }
}

void SketchRetopo::bar_init() {
    bar_main = TwNewBar("SketchRetopo");
    TwDefine("GLOBAL fontsize=2");
//    TwDefine("SketchRetopo size='300 830' valueswidth=120 alpha=200 text=light");
    std::stringstream twStream;
    twStream << "SketchRetopo size='" << camera_free.width/6 << ' ' << camera_free.height*4/5 << "'";
    twStream << "valueswidth=" << camera_free.width/12 << " text=light";
    TwDefine(twStream.str().c_str());
    
    // file |
    //------+
    tw_util::AddButton(bar_main, "import_basemesh",
        [&](){
            if (configTemp.autoSave.unsaved && !tinyfd_messageBox("SketchRetopo", "Current data is not saved yet. Continue?", "yesno", "question", 0))
                return;
            string fname = tinyfd_util::openFileDialog("SketchRetopo - import base mesh", "", {"*.obj","*.off","*.ply","*.stl"}, false);
            if (!fname.empty()) {
                if (!import_basemesh(fname))
                    tinyfd_messageBox("SketchRetopo - import base mesh", "Error occurred!", "ok", "error", 1);
            }
        }, "group=file label='import base mesh'"  );
    tw_util::AddButton(bar_main, "export_basemesh",
        [&](){
            string fname = tinyfd_util::saveFileDialog("SketchRetopo - export base mesh", "base.obj", {"*.obj","*.off","*.ply","*.stl"});
            if (!fname.empty()) {
                if (!export_basemesh(fname))
                    tinyfd_messageBox("SketchRetopo - export base mesh", "Error occurred!", "ok", "error", 1);
            }
        }, "group=file label='export base mesh'"  );
    tw_util::AddButton(bar_main, "export_retopomesh",
        [&](){
            string fname = tinyfd_util::saveFileDialog("SketchRetopo - export retopo mesh", "retopo.obj", {"*.obj","*.off","*.ply","*.stl"});
            if (!fname.empty()) {
                if (!export_retopomesh(fname))
                    tinyfd_messageBox("SketchRetopo - export retopo mesh", "Error occurred!", "ok", "error", 1);
            }
        }, "group=file label='export retopo mesh'");
    
    TwAddSeparator(bar_main, NULL, " group='file'");
    tw_util::AddButton(bar_main, "xml_save",
        [&](){
            string fname = tinyfd_util::saveFileDialog("SketchRetopo - save xml", "retopo.xml", {"*.xml"});
            if (!fname.empty()) {
                if (!xml_save(fname))
                    tinyfd_messageBox("SketchRetopo - save xml", "Error occurred!", "ok", "error", 1);
            }
        }, "group=file label='save xml'");
    tw_util::AddButton(bar_main, "xml_load",
        [&](){
            if (configTemp.autoSave.unsaved && !tinyfd_messageBox("SketchRetopo", "Current data is not saved yet. Continue?", "yesno", "question", 0))
                return;
            string fname = tinyfd_util::openFileDialog("SketchRetopo - load xml", "", {"*.xml"}, false);
            if (!fname.empty()) {
                if (!xml_load(fname))
                    tinyfd_messageBox("SketchRetopo - load xml", "Error occurred!", "ok", "error", 1);
            }
        }, "group=file label='load xml'");
    TwAddVarRW(bar_main, "autosave_enabled" , TW_TYPE_BOOLCPP, &configTemp.autoSave.enabled , "group=file label='autosave'");
    TwAddVarRW(bar_main, "autosave_interval", TW_TYPE_INT32  , &configTemp.autoSave.interval, "group=file label='autosave intrv' min=1");
    
    TwAddSeparator(bar_main, NULL, " group='file'");
    tw_util::AddButton(bar_main, "load_texture",
        [&](){
            string fname = tinyfd_util::openFileDialog("SketchRetopo - load texture", "", {"*.png"}, false);
            if (!fname.empty()) {
                if (!texture_overlay_load(fname))
                    tinyfd_messageBox("SketchRetopo - load texture", "Error occurred!", "ok", "error", 1);
            }
        }, "group=file");
    tw_util::AddButton(bar_main, "load_material",
        [&](){
            string fname = tinyfd_util::openFileDialog("SketchRetopo - load material", "", {"*.png"}, false);
            if (!fname.empty()) {
                if (!material_texture_load(fname))
                    tinyfd_messageBox("SketchRetopo - load material", "Error occurred!", "ok", "error", 1);
            }
        }, "group=file");
    // command |
    //---------+
    tw_util::AddButton(bar_main, "clear", [&](){ memento_store(); curvenetwork.clear(); }, "group=command");
    tw_util::AddVarCB<bool>(bar_main, "symmetric", [&](const bool& value){
        memento_store();
        configSaved.symmetric = value;
    }, [&](bool& value) {
        value = configSaved.symmetric;
    }, "group=command");
    tw_util::AddButton(bar_main, "snap_symmetry", [&](){ memento_store(); snap_symmetry(); }, "group=command");
    tw_util::AddVarCB_default<double>(bar_main, "projOffset", configSaved.projOffset, [&](){ memento_store(); reproject_all(); }, "group=command min=0 precision=8");
    // query_db |
    //----------+
    auto& lookup = learnquad.lookup;
    tw_util::AddVarCB<int>(bar_main, "index", [&](const int& value) {
        if (state == &stateQueryDB && value < lookup.numberOfPatches()) {
            stateQueryDB.lookup_index = value;
            stateQueryDB.update_patch();
        }
    }, [&](int& value) {
        value = state == &stateQueryDB ? stateQueryDB.lookup_index : -1;
    }, "group=query_db min=0");
    TwAddVarRW(bar_main, "freeze", TW_TYPE_BOOLCPP, &stateQueryDB.freeze, "group=query_db");
    tw_util::AddVarCB<int>(bar_main, "max_numsing", [&](const int& value){ lookup.setMaxSingularities(value); }, [&](int& value){ value = lookup.getMaxSingularities(); }, "group=query_db min=0");
    tw_util::AddVarCB<int>(bar_main, "max_concave", [&](const int& value){ lookup.setMaxConcavities(value); }, [&](int& value){ value = lookup.getMaxConcavities(); }, "group=query_db min=0");
    tw_util::AddVarCB<int>(bar_main, "min_valence", [&](const int& value){ lookup.setMinValence(value); }, [&](int& value){ value = lookup.getMinValence(); }, "group=query_db min=2 max=3");
    tw_util::AddVarCB<int>(bar_main, "max_valence", [&](const int& value){ lookup.setMaxValence(value); }, [&](int& value){ value = lookup.getMaxValence(); }, "group=query_db min=5");
    tw_util::AddVarCB<bool>(bar_main, "ilp_flow", [&](const bool& value) { lookup.setMinSizeThresholdILP(value ? 1 : numeric_limits<int>::max()); }, [&](bool& value) { value = lookup.getMinSizeThresholdILP() == 1; }, "group=query_db");
    tw_util::AddVarCB<int>(bar_main, "ilp_flow_tlrnc", [&](const int& value) { lookup.setFlowConstraintsILPtolerance(value); }, [&](int& value) { value = lookup.getFlowConstraintsILPtolerance(); }, "group=query_db min=0");
    tw_util::AddVarCB<double>(bar_main, "quality_th", [&](const double& value){ learnquad.retopologizer->setQualityThreshold(value); }, [&](double& value){ value = learnquad.retopologizer->getQualityThreshold(); }, "group=query_db min=0 max=1 step=0.01");
    using E = PatchCompare<PolyMesh>::CompareCriteria;
    tw_util::AddVarCB<bool>(bar_main, "by_flow"   , [&](const bool& value) { if (value) lookup.setSortCriteria({E::FLOW_DISTANCE, E::SINGULARITYNUM, E::SINGULARITYTYPES, E::QUADS_QUALITY}); }, [&](bool& value) { value = lookup.getSortCriteria().front() == E::FLOW_DISTANCE ; }, "group=query_db");
    tw_util::AddVarCB<bool>(bar_main, "by_quality", [&](const bool& value) { if (value) lookup.setSortCriteria({E::QUADS_QUALITY, E::SINGULARITYNUM, E::SINGULARITYTYPES, E::FLOW_DISTANCE}); }, [&](bool& value) { value = lookup.getSortCriteria().front() == E::QUADS_QUALITY ; }, "group=query_db");
    tw_util::AddVarCB<bool>(bar_main, "by_numsing", [&](const bool& value) { if (value) lookup.setSortCriteria({E::SINGULARITYNUM, E::SINGULARITYTYPES, E::FLOW_DISTANCE, E::QUADS_QUALITY}); }, [&](bool& value) { value = lookup.getSortCriteria().front() == E::SINGULARITYNUM; }, "group=query_db");
    tw_util::AddVarCB<double>(bar_main, "laplace_w", [&](const double& value) { learnquad.retopologizer->setLaplacianWeight(value); }, [&](double& value) { value = learnquad.retopologizer->getLaplacianWeight(); }, "group=query_db min=0 max=1 step=0.01");
    tw_util::AddButton(bar_main, "stop", [&](){
        lookup.stop();
        lock_guard<mutex> guard(stateQueryDB.mtx_status);
        stateQueryDB.update_status_msg();
    }, "group=query_db");
    tw_util::AddButton(bar_main, "restart", [&](){
        stateQueryDB.status = StateQueryDB::Status::started;
        stateQueryDB.update_status_msg();
        lookup.stop();
        lock_guard<mutex> guard(stateQueryDB.mtx_status);
        stateQueryDB.elapsed_time = std::chrono::nanoseconds::zero();
        lookup.findPatchesFromBoundaryLength(
            stateQueryDB.querydata.num_subdiv.size(),
            stateQueryDB.querydata.num_subdiv,
            learnquad.retopologizer->getFlowMapUV(), learnquad.retopologizer);
        stateQueryDB.lookup_index = 0;
    }, "group=query_db");
    tw_util::AddButton(bar_main, "delete_flow", [&](){
        if (!stateQueryDB.querydata.strokes.empty()) {
            lookup.stop();
            lock_guard<mutex> guard(stateQueryDB.mtx_status);
            stateQueryDB.querydata.strokes.pop_back();
            stateQueryDB.elapsed_time = std::chrono::nanoseconds::zero();
            stateQueryDB.status = StateQueryDB::Status::idle;
            stateQueryDB.querydata.send_strokes(*core.learnquad.retopologizer);
        }
    }, "group=query_db");
    TwAddVarRW(bar_main, "dbg_export", TW_TYPE_BOOLCPP, &stateQueryDB.dbg_export, "group=query_db");
//    TwAddVarRW(bar_main, "dbg_dont_project", TW_TYPE_BOOLCPP, &stateQueryDB.dbg_dont_project, "group=query_db");
    // config (temp) |
    //---------------+
    tw_util::AddVarCB<int>(bar_main, "undo_buffer_size",
        [&](const int& value){
            memento.init(value);
        }, [&](int& value) {
            value = memento.buffer_size();
        }, "group=config label='undo buf size' min=10 max=1000");
    TwAddVarRW(bar_main, "cylinder_num_div", TW_TYPE_INT32  , &configTemp.cylinder_num_div, "group=config label='cylndr num div' min=1");
    TwAddVarRW(bar_main, "use_even_num_subdiv", TW_TYPE_BOOLCPP, &curvenetwork::Patch::use_even_num_subdiv, "group=config label='use even subdiv'");
    TwAddVarRW(bar_main, "prefer_rect_proc3", TW_TYPE_BOOLCPP, &curvenetwork::Patch::prefer_rect_proc3, "group=config label='prefer proc3'");
    TwAddVarRW(bar_main, "loop_threshold_area"  , TW_TYPE_DOUBLE, &configTemp.loop_threshold_area  , "group=config label='loop thr area' min=0 max=1 step=0.01");
    TwAddVarRW(bar_main, "loop_threshold_normal", TW_TYPE_DOUBLE, &configTemp.loop_threshold_normal, "group=config label='loop thr normal' min=0 max=1 step=0.01");
    tw_util::set_var_opened(bar_main, "config", false);
    // camera |
    //--------+
    tw_util::AddVarCB<bool>(bar_main, "camera_upright",
        [&](const bool& value) {
            Vector3d eye    = camera->get_eye();
            Vector3d center = camera->center;
            Vector3d up     = camera->get_up();
            if (value)
                camera = &camera_upright;
            else
                camera = &camera_free;
            camera->init(eye, center, up);
            camera->update_center(center);
        }, [&](bool& value) {
            value = camera == &camera_upright;
        }, "group=camera label=upright");
    TwAddVarRW(bar_main, "use_ortho"  , TW_TYPE_BOOLCPP, &configRender.use_ortho  , "group=camera label=ortho");
    TwAddVarRW(bar_main, "auto_camera_center" , TW_TYPE_BOOLCPP, &configRender.auto_camera_center, "group=camera label='auto center'");
    TwAddVarRW(bar_main, "turntable_speed", TW_TYPE_DOUBLE, &configRender.turntable_speed, "group=camera label='turntable speed' max=1 min=-1 step=0.001");
    tw_util::set_var_opened(bar_main, "camera", false);
    // render |
    //--------+
    //TwAddVarRW(bar_main, "render_quad_only", TW_TYPE_BOOLCPP, &configRender.render_quad_only, "group=render label='quad only'");
    //TwAddVarRW(bar_main, "render_basemesh_only", TW_TYPE_BOOLCPP, &configRender.render_basemesh_only, "group=render label='basemesh only'");
    TwAddVarRW(bar_main, "render_mode", TwDefineEnum("render_mode_type", 0, 0), &configRender.mode, "group = render label='mode' enum='"
        "0 {default}           , "
        "1 {no basemesh}       , "
        "2 {basemesh only}     ' ");
    TwAddVarRW(bar_main, "always_show_quads", TW_TYPE_BOOLCPP, &configRender.always_show_quads, "group=render label='always show quads'");
    TwAddVarRW(bar_main, "basemesh_render_line", TW_TYPE_BOOLCPP, &configRender.basemesh_render_line, "group=render label='basemesh line'");
    TwAddVarRW(bar_main, "show_axis", TW_TYPE_BOOLCPP, &configRender.show_axis, "group=render label=axis");
    TwAddVarRW(bar_main, "show_bbox", TW_TYPE_BOOLCPP, &configRender.show_bbox, "group=render label=bbox");
    tw_util::AddVarCB_default<bool  >(bar_main, "phongTessellationEnabled", phong_tessellation::internal::enabled(),
        [&](){
            for (auto& p : curvenetwork.patches)
                p.invalidate_displist();
        }, "group=render label='phong tessell'");
    tw_util::AddVarCB_default<int   >(bar_main, "phongTessellationSubdiv" , phong_tessellation::subdiv(),
        [&](){
            for (auto& p : curvenetwork.patches)
                p.invalidate_displist();
        }, "group=render label='phong subdiv' min=1 max=6");
    tw_util::AddVarCB_default<double>(bar_main, "phongTessellationWeight" , phong_tessellation::weight(),
        [&](){
            for (auto& p : curvenetwork.patches)
                p.invalidate_displist();
        }, "group=render label='phong weight' min=0 max=1 step=0.01");
    TwAddVarRW(bar_main, "show_expmap", TW_TYPE_BOOLCPP, &configRender.show_expmap, "group=render");
    TwAddVarRW(bar_main, "overlay_alpha", TW_TYPE_FLOAT, &configRender.overlay_alpha, "group=render min=0 max=1 step=0.01");
    TwAddVarRW(bar_main, "overlay_v_flipped", TW_TYPE_BOOLCPP, &configRender.overlay_v_flipped, "group=render");
    TwAddVarRW(bar_main, "bgcolor_bottom", TW_TYPE_COLOR3F, &configRender.color.background_bottom, "group=render label='bg color btm'");
    TwAddVarRW(bar_main, "bgcolor_top"   , TW_TYPE_COLOR3F, &configRender.color.background_top   , "group=render label='bg color top'");
    tw_util::AddVarCB_default<double>(bar_main, "corner_radius" , configRender.corner_radius,
        [&](){
            curvenetwork.invalidate_displist();
        }, "group=render label='corner radius' min=0 step=0.001");
    TwAddVarRW(bar_main, "edge_width_interior", TW_TYPE_DOUBLE , &configRender.edge_width_interior, "group=render label='edge width i' min=0 step=0.1");
    TwAddVarRW(bar_main, "edge_width_boundary", TW_TYPE_DOUBLE , &configRender.edge_width_boundary, "group=render label='edge width b' min=0 step=0.1");
    TwAddVarRW(bar_main, "singularity_radius" , TW_TYPE_DOUBLE , &configRender.singularity_radius, "group=render label='singularity radius' min=0 step=0.1");
    TwAddVarRW(bar_main, "light_pos", TW_TYPE_DIR3F, &configRender.light_pos[0], "group=render label='light pos'");
    TwAddVarRW(bar_main, "show_auxiliary", TW_TYPE_BOOLCPP, &configRender.quadmesh_only_show_auxiliary, "group=render label='show auxiliary'");
    TwAddVarRW(bar_main, "show_left_side", TW_TYPE_BOOLCPP, &configRender.quadmesh_only_show_left_side, "group=render label='show left side'");
    tw_util::set_var_opened(bar_main, "render", false);
}
