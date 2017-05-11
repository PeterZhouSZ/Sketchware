#include <OpenMesh/Core/IO/MeshIO.hh>
#include "SketchRetopo.hh"
#include <tinyxml2/tinyxml2.h>
#include <lodepng/lodepng.h>
#include <base64/encode.h>
#include <kt84/Clock.hh>
#include <kt84/openmesh/merge_nearby_vertices.hh>
#include <kt84/zenity_util.hh>
#include <kt84/graphics/graphics_util.hh>
#include <kt84/safe_istream.hh>
#include <patchgen/decl.hh>
#include "helper.hh"
#include "curvenetwork/helper.hh"
using namespace std;
using namespace Eigen;
using namespace kt84;
using namespace kt84::graphics_util;

bool SketchRetopo::texture_overlay_load(const string& fname) {
    vector<unsigned char> pixels;
    unsigned int width, height;
    if (lodepng::decode(pixels, width, height, fname)) {
        cerr << "Error while reading " << fname << endl;
        return false;
    }

    int max_texture_size = glGet1i(ParamName::MAX_TEXTURE_SIZE);
    if (max_texture_size < width || max_texture_size < height) {
        cerr << "Requested resolution of " << width << "x" << height << " exceeds the maximum of " << max_texture_size << "x" << max_texture_size << endl;
        return false;
    }
    
    texture_overlay.bind();
    texture_overlay.allocate(width, height);
    texture_overlay.copy_cpu2gpu(GL_UNSIGNED_BYTE, pixels.data());
    texture_overlay.unbind();
    configRender.overlay_alpha = 0.5;

    return true;
}
bool SketchRetopo::material_texture_load(const string& fname) {
    ifstream fin(fname, ios_base::binary);
    if (!fin) return false;
    
    stringstream ss_encoded;
    base64::encoder E;
    E.encode(fin, ss_encoded);
    material_texture_png_base64 = ss_encoded.str();
    
    material_texture_update();
    
    return true;
}

bool SketchRetopo::import_basemesh(const string& fname) {
    {
        ClkSimple clk("OpenMesh::IO::read_mesh");
        BaseMesh basemesh_temp;
        OpenMesh::IO::Options opt;
        opt += OpenMesh::IO::Options::FaceTexCoord;
        if (!OpenMesh::IO::read_mesh(basemesh_temp, fname, opt))
            return false;
        basemesh_temp.has_texture = (opt & OpenMesh::IO::Options::FaceTexCoord) != 0;
        basemesh = basemesh_temp;
    }
    
    {
        ClkSimple clk("BaseMesh::init");
        basemesh.init();
    }
    
    double bb_diagonal = basemesh.boundingBox_diagonal_norm();

	auto & bbox = basemesh.boundingBox;
	auto x = bbox.center().x() * bbox.sizes().x();
	auto y = bbox.center().y() * bbox.sizes().y();
	auto z = bbox.center().z() * bbox.sizes().z();
    
    curvenetwork.clear();
    
    // clear undo/redo data
    memento.init();
    
    // reset all config to default
    configRender = ConfigRender();
    configSaved  = ConfigSaved ();
    configTemp   = ConfigTemp  ();
    
    // parameters dependent on model size
    configTemp.quadSize   .value = bb_diagonal * ConfigTemp::default_quadSize_ratio;
    configTemp.segmentSize.value = bb_diagonal * ConfigTemp::default_segmentSize_ratio;
    curvenetwork::Patch::quadSize = configTemp.quadSize.value;
    
    configSaved.projOffset = 0.0005 * bb_diagonal;
    double step_projOffset = 0.1 * configSaved.projOffset;
    //TwSetParam(bar_main, "projOffset", "step", TW_PARAM_DOUBLE, 1, &step_projOffset);
    
    // init camera
    camera->init(basemesh.centerOfMass + Vector3d(0, 0, bb_diagonal), basemesh.centerOfMass, Vector3d::UnitY());
    
    {   // init embree
        ClkSimple clk("embree_init");
        embree_init();
    }
    
    {   // init geidesic
        ClkSimple clk("geidesic_init");
        geodesic_init();
    }

	{   //init contour
		if (contour2D != nullptr) delete contour2D;
		contour2D = new SketcherCore::Contour2D(basemesh);
	}
    
    state->enter_state();
    
    return true;
}

bool SketchRetopo::export_basemesh(std::string fname) const {
    return OpenMesh::IO::write_mesh(basemesh, fname);
}

bool SketchRetopo::export_retopomesh(string fname) const {
    struct RetopoMeshTraits : public OpenMesh::DefaultTraits {
        typedef OpenMesh::Vec3d Point;
        typedef OpenMesh::Vec3d Normal;
    };
    typedef OpenMesh::PolyMesh_ArrayKernelT<RetopoMeshTraits> RetopoMeshBase;
    struct RetopoMesh
        : public RetopoMeshBase
#ifndef NDEBUG
        , public DebugInfo<RetopoMeshBase, RetopoMesh>
#endif
    {} retopoMesh;
    
    // concatenate all patches
    for (auto& patch : curvenetwork.patches) {
        vector<RetopoMesh::VHandle> added_vertices;
        for (auto v : patch.vertices()) {
            RetopoMesh::Point point(&patch.data(v).pn[0]);
            
            //if (configSaved.symmetric && abs(point[0]) < threshold)
            //    point[0] = 0;
            
            added_vertices.push_back(retopoMesh.add_vertex(point));
        }
        for (auto f : patch.faces()) {
            vector<RetopoMesh::VHandle> face_vhandles;
            for (auto v : patch.fv_range(f))
                face_vhandles.push_back(added_vertices[v.idx()]);
            retopoMesh.add_face(face_vhandles);
        }
    }
    
    if (configSaved.symmetric) {
        // symmetric part
        for (auto& patch : curvenetwork.patches) {
            vector<RetopoMesh::VHandle> added_vertices;
            for (auto v : patch.vertices()) {
                RetopoMesh::Point point(&patch.data(v).pn[0]);
                
                //if (abs(point[0]) < threshold)
                //    point[0] = 0;
                
                point[0] *= -1;
                added_vertices.push_back(retopoMesh.add_vertex(point));
            }
            for (auto f : patch.faces()) {
                vector<RetopoMesh::VHandle> face_vhandles;
                for (auto v : patch.fv_range(f))
                    face_vhandles.push_back(added_vertices[v.idx()]);
                reverse(face_vhandles.begin(), face_vhandles.end());
                retopoMesh.add_face(face_vhandles);
            }
        }
    }
    
#ifndef NDEBUG
    retopoMesh.debugInfo_get(false, false, true);
#endif
    //double merge_threshold_percent = 0.1;
    //zenity_util::entryT<double>(merge_threshold_percent, "SketchRetopo2", "Vertex merging threshold (percentage w.r.t. bounding box diagonal)", merge_threshold_percent);
    // fu*k! this never works right! should be done properly based on topological info from the curvenetwork
    //kt84::merge_nearby_vertices(retopoMesh, basemesh.boundingBox_diagonal_norm() * 0.01 * merge_threshold_percent);
    
    return OpenMesh::IO::write_mesh(retopoMesh, fname);
}

bool SketchRetopo::xml_load(const string& fname) {
    tinyxml2::XMLDocument xml_doc;
    if (xml_doc.LoadFile(fname.c_str()))
        return false;
    
    auto xml_root = xml_doc.FirstChildElement("sketchretopo");
    if (!xml_root)
        return false;
    
    // base mesh
    BaseMesh basemesh_temp;
    auto xml_basemesh = xml_root->FirstChildElement("basemesh");
    if (!xml_basemesh)
        return false;
    {
        basemesh_temp.has_texture = xml_basemesh->IntAttribute("has_texture");
        
        // vertices
        auto xml_basemesh_vertices = xml_basemesh->FirstChildElement("vertices");
        int num_vertices = xml_basemesh_vertices->IntAttribute("num");
        istringstream ss_basemesh_vertices;
        ss_basemesh_vertices.str(xml_basemesh_vertices->GetText());
        for (int i = 0; i < num_vertices; ++i) {
            BaseMesh::Point p;
            make_safe_istream(ss_basemesh_vertices) >> p[0] >> p[1] >> p[2];
            basemesh_temp.add_vertex(p);
        }
        
        // faces
        auto xml_basemesh_faces = xml_basemesh->FirstChildElement("faces");
        int num_faces = xml_basemesh_faces->IntAttribute("num");
        istringstream ss_basemesh_faces;
        ss_basemesh_faces.str(xml_basemesh_faces->GetText());
        for (int i = 0; i < num_faces; ++i) {
            int vidx0, vidx1, vidx2;
            make_safe_istream(ss_basemesh_faces) >> vidx0 >> vidx1 >> vidx2;
            auto v0 = basemesh_temp.vertex_handle(vidx0);
            auto v1 = basemesh_temp.vertex_handle(vidx1);
            auto v2 = basemesh_temp.vertex_handle(vidx2);
            auto f = basemesh_temp.add_face(v0, v1, v2);
            if (f.is_valid() && basemesh_temp.has_texture) {
                for (auto h : basemesh_temp.fh_range(f)) {
                    BaseMesh::TexCoord2D uv;
                    make_safe_istream(ss_basemesh_faces) >> uv[0] >> uv[1];
                    basemesh_temp.set_texcoord2D(h, uv);
                }
            }
        }
    }
    
    ConfigSaved configSaved_temp;
    {   // config
        auto xml_config = xml_root->FirstChildElement("config");
        if (!xml_config)
            return false;
        configSaved_temp.projOffset    = xml_config->DoubleAttribute("projoffset");
        configSaved_temp.symmetric     = xml_config->Attribute("symmetric", "true") ? true : false;
    }
    
    // curveNetwork
    curvenetwork::Core curvenetwork_temp;
    auto xml_curveNetwork = xml_root->FirstChildElement("curvenetwork");
    if (!xml_curveNetwork)
        return false;
    
    {   // vertices
        auto xml_curveNetwork_vertices = xml_curveNetwork->FirstChildElement("vertices");
        if (!xml_curveNetwork_vertices)
            return false;
        int num_curveNetwork_vertices = xml_curveNetwork_vertices->IntAttribute("num");
        istringstream ss_curveNetwork_vertices(num_curveNetwork_vertices ? xml_curveNetwork_vertices->GetText() : "");
        for (int i = 0; i < num_curveNetwork_vertices; ++i) {
            auto v = curvenetwork_temp.new_vertex();
            make_safe_istream(ss_curveNetwork_vertices)
                >> v.id
                >> v->halfedge.id
                >> v->pn[0] >> v->pn[1] >> v->pn[2]
                >> v->pn[3] >> v->pn[4] >> v->pn[5];
            v->id = v.id;
        }
    }
    
    {   // halfedges
        auto xml_curveNetwork_halfedges = xml_curveNetwork->FirstChildElement("halfedges");
        if (!xml_curveNetwork_halfedges)
            return false;
        int num_curveNetwork_halfedges = xml_curveNetwork_halfedges->IntAttribute("num");
        istringstream ss_curveNetwork_halfedges(num_curveNetwork_halfedges ? xml_curveNetwork_halfedges->GetText() : "");
        for (int i = 0; i < num_curveNetwork_halfedges; ++i) {
            auto h = curvenetwork_temp.new_halfedge();
            int is_corner;
            make_safe_istream(ss_curveNetwork_halfedges)
                >> h.id
                >> h->vertex   .id
                >> h->next     .id
                >> h->prev     .id
                >> h->opposite .id
                >> h->halfchain.id
                >> is_corner;
            h->is_corner = is_corner != 0;
            h->id = h.id;
        }
    }
    
    {   // halfchains
        auto xml_curveNetwork_halfchains = xml_curveNetwork->FirstChildElement("halfchains");
        if (!xml_curveNetwork_halfchains)
            return false;
        int num_curveNetwork_halfchains = xml_curveNetwork_halfchains->IntAttribute("num");
        istringstream ss_curveNetwork_halfchains(num_curveNetwork_halfchains ? xml_curveNetwork_halfchains->GetText() : "");
        for (int i = 0; i < num_curveNetwork_halfchains; ++i) {
            auto c = curvenetwork_temp.new_halfchain();
            make_safe_istream(ss_curveNetwork_halfchains)
                >> c.id
                >> c->halfedge_front.id
                >> c->halfedge_back .id
                >> c->patch         .id
                >> c->edgechain     .id;
            c->id = c.id;
        }
    }
    
    {   // edgechains
        auto xml_curveNetwork_edgechains = xml_curveNetwork->FirstChildElement("edgechains");
        if (!xml_curveNetwork_edgechains)
            return false;
        int num_curveNetwork_edgechains = xml_curveNetwork_edgechains->IntAttribute("num");
        istringstream ss_curveNetwork_edgechains(num_curveNetwork_edgechains ? xml_curveNetwork_edgechains->GetText() : "");
        for (int i = 0; i < num_curveNetwork_edgechains; ++i) {
            auto e = curvenetwork_temp.new_edgechain();
            make_safe_istream(ss_curveNetwork_edgechains)
                >> e.id
                >> e->halfchain[0].id
                >> e->halfchain[1].id
                >> e->num_subdiv;
            e->id = e.id;
        }
    }
    
    {   // patches
        auto xml_curveNetwork_patches = xml_curveNetwork->FirstChildElement("patches");
        if (!xml_curveNetwork_patches)
            return false;
        int num_curveNetwork_patches = xml_curveNetwork_patches->IntAttribute("num");
        
        for (auto xml_curveNetwork_patch = xml_curveNetwork_patches->FirstChildElement("patch");
            xml_curveNetwork_patch;
            xml_curveNetwork_patch = xml_curveNetwork_patch->NextSiblingElement("patch"))
        {
            auto patch = curvenetwork_temp.new_patch();
            patch->id = patch.id = xml_curveNetwork_patch->IntAttribute("id"        );
            patch->halfchain.id  = xml_curveNetwork_patch->IntAttribute("halfchain" );
            
            {   // patch vertices
                auto xml_patch_vertices = xml_curveNetwork_patch->FirstChildElement("patch_vertices");
                if (!xml_patch_vertices)
                    return false;
                
                int num_patch_vertices = xml_patch_vertices->IntAttribute("num");
                istringstream ss_patch_vertices;
                if (num_patch_vertices)
                    ss_patch_vertices.str(xml_patch_vertices->GetText());
                
                for (int i = 0; i < num_patch_vertices; ++i) {
                    auto v = patch->add_vertex(OpenMesh::Vec3d());
                    auto& vdata = patch->data(v);
                    
                    int tag;
                    make_safe_istream(ss_patch_vertices)
                        >> vdata.pn[0] >> vdata.pn[1] >> vdata.pn[2]
                        >> vdata.pn[3] >> vdata.pn[4] >> vdata.pn[5]
                        >> vdata.patchgen.corner_index
                        >> tag;
                    vdata.patchgen.tag = static_cast<patchgen::PatchVertexTag>(tag);
                }
            }
            
            {   // patch faces
                auto xml_patch_faces = xml_curveNetwork_patch->FirstChildElement("patch_faces");
                if (!xml_patch_faces)
                    return false;
                
                int num_patch_faces = xml_patch_faces->IntAttribute("num");
                istringstream ss_patch_faces;
                if (num_patch_faces)
                    ss_patch_faces.str(xml_patch_faces->GetText());
                
                for (int i = 0; i < num_patch_faces; ++i) {
                    int vidx[4];
                    make_safe_istream(ss_patch_faces) >> vidx[0] >> vidx[1] >> vidx[2] >> vidx[3];
                    
                    curvenetwork::Patch::VHandle v[4];
                    for (int j = 0; j < 4; ++j)
                        v[j] = patch->vertex_handle(vidx[j]);
                    
                    patch->add_face(v[0], v[1], v[2], v[3]);
                }
            }
#ifndef NDEBUG
            patch->debugInfo_get(false, false, false);
#endif
        }
    }

    {   // material texture
        auto xml_texture = xml_root->FirstChildElement("texture");
        if (xml_texture) {
            material_texture_png_base64 = xml_texture->GetText();
            material_texture_update();
        }
    }
    
    // all green, replace current data
    basemesh = basemesh_temp;
    
    // init process mostly the same as in import_basemesh()
    {
        ClkSimple clk("BaseMesh::init");
        basemesh.init();
    }
    
    {   // init embree
        ClkSimple clk("embree_init");
        embree_init();
    }
    
    {   // init geidesic
        ClkSimple clk("geidesic_init");
        geodesic_init();
    }
    
    curvenetwork = curvenetwork_temp;
    curvenetwork.validate_all_ptr();
    // factorize direct Laplace solver
    for (auto& patch : curvenetwork.patches)
        patch.laplaceSolver_init();
    
    // clear undo/redo data
    memento.init();
    
    configSaved = configSaved_temp;
    
    // reset all config to default
    configRender = ConfigRender();
    configTemp   = ConfigTemp  ();
    
    // parameters dependent on model size
    double bb_diagonal = basemesh.boundingBox_diagonal_norm();
    configTemp.quadSize   .value = bb_diagonal * ConfigTemp::default_quadSize_ratio;
    configTemp.segmentSize.value = bb_diagonal * ConfigTemp::default_segmentSize_ratio;
    curvenetwork::Patch::quadSize = configTemp.quadSize.value;
    
    double step_projOffset = 0.1 * configSaved.projOffset;
    TwSetParam(bar_main, "projOffset", "step", TW_PARAM_DOUBLE, 1, &step_projOffset);
    
    // init camera
    camera->init(basemesh.centerOfMass + Vector3d(0, 0, bb_diagonal), basemesh.centerOfMass, Vector3d::UnitY());
    
    state->enter_state();
    
    configTemp.autoSave.filename = fname;
    
    return true;
}

bool SketchRetopo::xml_save(string fname) {
    tinyxml2::XMLDocument xml_doc;
    
    // root element
    auto xml_root = xml_doc.InsertEndChild(xml_doc.NewElement("sketchretopo"))->ToElement();
    if (!xml_root) return false;
    
    {   // base mesh
        auto xml_basemesh = xml_root->InsertEndChild(xml_doc.NewElement("basemesh"))->ToElement();
        xml_basemesh->SetAttribute("has_texture", basemesh.has_texture ? 1 : 0);
        
        {   // vertices
            auto xml_basemesh_vertices = xml_basemesh->InsertEndChild(xml_doc.NewElement("vertices"))->ToElement();
            xml_basemesh_vertices->SetAttribute("num", static_cast<int>(basemesh.n_vertices()));
        
            ostringstream ss_basemesh_vertices;
            ss_basemesh_vertices << endl;
        
            for (auto v : basemesh.vertices()) {
                auto p = basemesh.point(v);
                ss_basemesh_vertices << p[0] << " " << p[1] << " " << p[2] << endl;
            }
            xml_basemesh_vertices->InsertEndChild(xml_doc.NewText(ss_basemesh_vertices.str().c_str()));
        }
        
        {   // faces
            auto xml_basemesh_faces = xml_basemesh->InsertEndChild(xml_doc.NewElement("faces"))->ToElement();
            xml_basemesh_faces->SetAttribute("num", static_cast<int>(basemesh.n_faces()));
        
            ostringstream ss_basemesh_faces;
            ss_basemesh_faces << endl;
        
            for (auto f : basemesh.faces()) {
                for (auto h : basemesh.fh_range(f)) {
                    auto v = basemesh.to_vertex_handle(h);
                    ss_basemesh_faces << v.idx() << " ";
                }
                if (basemesh.has_texture) {
                    for (auto h : basemesh.fh_range(f)) {
                        auto t = basemesh.texcoord2D(h);
                        ss_basemesh_faces << t[0] << " " << t[1] << " ";
                    }
                }
                ss_basemesh_faces << endl;
            }
            xml_basemesh_faces->InsertEndChild(xml_doc.NewText(ss_basemesh_faces.str().c_str()));
        }
    }
    
    {   // config
        auto xml_config = xml_root->InsertEndChild(xml_doc.NewElement("config"))->ToElement();
        xml_config->SetAttribute("projoffset", configSaved.projOffset);
        xml_config->SetAttribute("symmetric" , configSaved.symmetric ? "true" : "false");
    }
    
    // curveNetwork
    auto xml_curveNetwork = xml_root->InsertEndChild(xml_doc.NewElement("curvenetwork"))->ToElement();
    
    {   // vertices
        auto xml_curveNetwork_vertices = xml_curveNetwork->InsertEndChild(xml_doc.NewElement("vertices"))->ToElement();
        xml_curveNetwork_vertices->SetAttribute("num", static_cast<int>(curvenetwork.vertices.size()));
        ostringstream ss_curveNetwork_vertices;
        ss_curveNetwork_vertices << endl;
        
        for (auto& v : curvenetwork.vertices) {
            if (v.is_deleted) continue;
            ss_curveNetwork_vertices
                << v.id << " "
                << v.halfedge.id << " "
                << v.pn[0] << " " << v.pn[1] << " " << v.pn[2] << " "
                << v.pn[3] << " " << v.pn[4] << " " << v.pn[5] << endl;
        }
        
        xml_curveNetwork_vertices->InsertEndChild(xml_doc.NewText(ss_curveNetwork_vertices.str().c_str()));
        xml_curveNetwork_vertices->InsertEndChild(xml_doc.NewComment("id halfedge px py pz nx ny nz"));
    }
    
    {   // halfedges
        auto xml_curveNetwork_halfedges = xml_curveNetwork->InsertEndChild(xml_doc.NewElement("halfedges"))->ToElement();
        xml_curveNetwork_halfedges->SetAttribute("num", static_cast<int>(curvenetwork.halfedges.size()));
        ostringstream ss_curveNetwork_halfedges;
        ss_curveNetwork_halfedges << endl;
        
        for (auto& h : curvenetwork.halfedges) {
            if (h.is_deleted) continue;
            ss_curveNetwork_halfedges
                << h.id << " "
                << h.vertex   .id << " "
                << h.next     .id << " "
                << h.prev     .id << " "
                << h.opposite .id << " "
                << h.halfchain.id << " "
                << (h.is_corner ? 1 : 0) << endl;
        }
        
        xml_curveNetwork_halfedges->InsertEndChild(xml_doc.NewText(ss_curveNetwork_halfedges.str().c_str()));
        xml_curveNetwork_halfedges->InsertEndChild(xml_doc.NewComment("id vertex next prev opposite halfchain is_corner"));
    }
    
    {   // halfhcains
        auto xml_curveNetwork_halfchains = xml_curveNetwork->InsertEndChild(xml_doc.NewElement("halfchains"))->ToElement();
        xml_curveNetwork_halfchains->SetAttribute("num", static_cast<int>(curvenetwork.halfchains.size()));
        ostringstream ss_curveNetwork_halfchains;
        ss_curveNetwork_halfchains << endl;
        
        for (auto& c : curvenetwork.halfchains) {
            if (c.is_deleted) continue;
            ss_curveNetwork_halfchains
                << c.id << " "
                << c.halfedge_front.id << " "
                << c.halfedge_back .id << " "
                << c.patch         .id << " "
                << c.edgechain     .id << endl;
        }
        
        xml_curveNetwork_halfchains->InsertEndChild(xml_doc.NewText(ss_curveNetwork_halfchains.str().c_str()));
        xml_curveNetwork_halfchains->InsertEndChild(xml_doc.NewComment("id halfedge_front halfedge_back patch edgechain"));
    }
    
    {   // edgecains
        auto xml_curveNetwork_edgechains = xml_curveNetwork->InsertEndChild(xml_doc.NewElement("edgechains"))->ToElement();
        xml_curveNetwork_edgechains->SetAttribute("num", static_cast<int>(curvenetwork.edgechains.size()));
        ostringstream ss_curveNetwork_edgechains;
        ss_curveNetwork_edgechains << endl;
        
        for (auto& e : curvenetwork.edgechains) {
            if (e.is_deleted) continue;
            ss_curveNetwork_edgechains
                << e.id << " "
                << e.halfchain[0].id << " "
                << e.halfchain[1].id << " "
                << e.num_subdiv << endl;
        }
        
        xml_curveNetwork_edgechains->InsertEndChild(xml_doc.NewText(ss_curveNetwork_edgechains.str().c_str()));
        xml_curveNetwork_edgechains->InsertEndChild(xml_doc.NewComment("id halfchain_0 halfchain_1 num_subdiv"));
    }
    
    {   // patches
        auto xml_curveNetwork_patches = xml_curveNetwork->InsertEndChild(xml_doc.NewElement("patches"))->ToElement();
        xml_curveNetwork_patches->SetAttribute("num", static_cast<int>(curvenetwork.patches.size()));
        
        for (auto& patch : curvenetwork.patches) {
            if (patch.is_deleted) continue;
            auto xml_curveNetwork_patch = xml_curveNetwork_patches->InsertEndChild(xml_doc.NewElement("patch"))->ToElement();
            xml_curveNetwork_patch->SetAttribute("id"        , patch.id          );
            xml_curveNetwork_patch->SetAttribute("halfchain" , patch.halfchain.id);
            
            {   // patch vertices
                auto xml_patch_vertices = xml_curveNetwork_patch->InsertEndChild(xml_doc.NewElement("patch_vertices"))->ToElement();
                xml_patch_vertices->SetAttribute("num", static_cast<int>(patch.n_vertices()));
                ostringstream ss_patch_vertices;
                ss_patch_vertices << endl;
                
                for (auto v : patch.vertices()) {
                    auto& vdata = patch.data(v);
                    
                    ss_patch_vertices
                        << vdata.pn[0] << " " << vdata.pn[1] << " " << vdata.pn[2] << " "
                        << vdata.pn[3] << " " << vdata.pn[4] << " " << vdata.pn[5] << " "
                        << vdata.patchgen.corner_index << " "
                        << static_cast<int>(vdata.patchgen.tag) << endl;
                }
                xml_patch_vertices->InsertEndChild(xml_doc.NewText(ss_patch_vertices.str().c_str()));
                xml_patch_vertices->InsertEndChild(xml_doc.NewComment("px py pz nx ny nz corner_index tag"));
            }
            
            {   // patch faces
                auto xml_patch_faces = xml_curveNetwork_patch->InsertEndChild(xml_doc.NewElement("patch_faces"))->ToElement();
                xml_patch_faces->SetAttribute("num", static_cast<int>(patch.n_faces()));
                ostringstream ss_patch_faces;
                ss_patch_faces << endl;
                
                for (auto f : patch.faces()) {
                    for (auto v : patch.fv_range(f))
                        ss_patch_faces << v.idx() << " ";
                    ss_patch_faces << endl;
                }
                xml_patch_faces->InsertEndChild(xml_doc.NewText(ss_patch_faces.str().c_str()));
            }
        }
        
    }
    
    {   // material texture (PNG encoded using base64)
        auto xml_texture = xml_root->InsertEndChild(xml_doc.NewElement("texture"))->ToElement();
        xml_texture->InsertEndChild(xml_doc.NewText((material_texture_png_base64).c_str()));
    }
    
    // save (file extension automatically added)
    string fname_ext = fname.substr(fname.size() - 4, 4);
    if (fname_ext != ".xml" && fname_ext != ".XML")
        fname += ".xml";
    if (xml_doc.SaveFile(fname.c_str())) return false;
    
    configTemp.autoSave.filename = fname;
    configTemp.autoSave.unsaved  = false;
    
    return true;
}


