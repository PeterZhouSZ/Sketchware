#include "SketchRetopo.hh"
#include <algorithm>
#include <unordered_map>
#include <boost/range/algorithm_ext.hpp>
#include <Eigen/Sparse>
#include <kt84/container_util.hh>
#include <boost/range/algorithm.hpp>
#include <kt84/Clock.hh>
#include <kt84/eigen_util.hh>
#include <kt84/loop_util.hh>
#include <kt84/MinSelector.hh>
#include <kt84/adjacent_pairs.hh>
#include <kt84/math/RBFKernel.hh>
#include <kt84/openmesh/edgeloop.hh>
#include <patchgen/decl.hh>
#include "curvenetwork/Circulator.hh"
#include "curvenetwork/helper.hh"
#include "learnquad/convert_patch.hh"
using namespace std;
using namespace Eigen;
using namespace kt84;

namespace {
    inline float bc(const embree::Hit& hit, int index) {
        return
            index == 0 ? 1 - hit.u - hit.v :
            index == 1 ? hit.u :
            index == 2 ? hit.v : numeric_limits<float>::quiet_NaN();
    }
}

bool SketchRetopo::is_loop_ccw(const Polyline_PointNormal& loop) const {
    assert(loop.is_loop);
    
    int n = loop.size();
    
    // average
    PointNormal avg = accumulate(loop.begin(), loop.end(), PointNormal::Zero().eval()) / n;
    
    // reject if normal directions vary widely
    if (avg.tail(3).norm() < configTemp.loop_threshold_normal)
        return false;
    
    // rough estimate of imagined surface area surrounded by this loop
    double area_sum = 0;
    for (int i = 0; i < loop.size(); ++i) {
        Vector3d d0 = (loop[ i         ] - avg).head(3);
        Vector3d d1 = (loop[(i + 1) % n] - avg).head(3);
        Vector3d n  = loop[i].tail(3);
        area_sum += 0.5 * d0.cross(d1).dot(n);
    }
    
    double loop_length = loop.length();
    double area_max = loop_length * loop_length / (4 * util::pi());      // maximum (i.e. disc) area for the loop length
    
    return area_max * configTemp.loop_threshold_area < area_sum;
}

void SketchRetopo::generate_patches() {
    using curvenetwork::Patch;
    curvenetwork.invalidate_displist();
    
    // visited flag
    curvenetwork.set_flag_halfchains(curvenetwork::Core::FlagOp::SUBST, 0);
    
    for (auto& c : curvenetwork.halfchains) {
        if (c.is_deleted || c.flag || c.patch) continue;
        
        auto patch_halfchain = &c;
        if (!Patch::is_valid_for_patch(patch_halfchain))
            continue;
        for (auto c1 = patch_halfchain; ; ) {
            c1->flag = 1;
            c1 = c1->next();
            if (c1 == patch_halfchain)
                break;
        }
        
        int num_corners = Patch::num_corners(patch_halfchain);
        if (num_corners < 2 || 6 < num_corners)
            // don't automatically generate patch if it's not supported by pattern-based method
            continue;
        
        // determine if this loop is counterclockwise
        Polyline_PointNormal polyline;
        for (auto c1 = patch_halfchain; ; ) {
            auto polyline_sub = c1->toPolyline();
            polyline_sub.pop_back();
            polyline.insert(polyline.end(), polyline_sub.begin(), polyline_sub.end());
            c1 = c1->next();
            if (c1 == patch_halfchain)
                break;
        }
        polyline.is_loop = true;
        double loop_length = polyline.length();
        if (loop_length > basemesh.boundingBox_diagonal_norm() * 2)
            // loop is too long
            continue;
        
        if (!is_loop_ccw(polyline))
            // doesn't form a reasonable counterclockwise loop --> skip
            continue;

        generate_patch(patch_halfchain);
    }
    curvenetwork.garbage_collect();
}
curvenetwork::Patch* SketchRetopo::generate_patch(curvenetwork::Halfchain* halfchain) {
    curvenetwork::Patch::set_default_num_subdiv(halfchain);
    // check if the boundary condition is valid
    VectorXi num_subdiv = curvenetwork::Patch::num_subdiv(halfchain);
    if (num_subdiv.sum() % 2 != 0)
        return nullptr;
    
    // instantiate a Patch
    auto patch = curvenetwork.new_patch();
    patch->set_halfchain(halfchain);
    
    // switch between two methods (pattern-based vs. data-driven) depending on num_corners
    int num_corners = num_subdiv.size();
    if (2 <= num_corners && num_corners <= 6) {
        patch->generate_topology(num_subdiv);
    } else {
        learnquad.notifiable->func = [&] (
            vcg::tri::pl::MT_PatchPriorityLookupNotifiable::NotificationType notificationType,
            vcg::tri::pl::count_type nTemplatePatches,
            vcg::tri::pl::count_type nPatches,
            std::chrono::nanoseconds duration)
        {
            if (nPatches == 0)
                return;
            lock_guard<mutex> lock(learnquad.mtx_display);
            convert_patch(learnquad.lookup, 0, *patch);
            learnquad.lookup.sendInterrupt();
        };
        ClkSimple clk("querying DB");
        learnquad.lookup.findPatchesFromBoundaryLength(num_corners, vector_cast<vector<vcg::tri::pl::num_type>>(num_corners, num_subdiv));
        learnquad.lookup.waitForThreads();
        if (learnquad.lookup.numberOfPatches() == 0) {
            // couldn't find anything --> delete patch
            curvenetwork::delete_patch(patch);
            patch = nullptr;
        }
    }
    if (patch)
        compute_patch_interior_pn(patch);
    return patch;
}
void SketchRetopo::add_curve_open_delegate(Polyline_PointNormal& curve, bool corner_at_openend) {
    // helper function to get offset to nearby curvenetwork::Vertex
    auto find_snap = [this] (const PointNormal& pn, PointNormal& result) -> bool {
        MinSelector<curvenetwork::Vertex*> v_openend;
        MinSelector<curvenetwork::Vertex*> v_corner ;
        MinSelector<curvenetwork::Vertex*> v_side   ;
        
        for (auto& v : curvenetwork.vertices) {
            if (v.is_deleted)
                continue;
            
            bool is_openend = v.is_openend();
            bool is_corner  = v.is_corner ();
            
            double dist = pn_norm(pn - v.pn);
            if (configTemp.snapSize() < dist)
                continue;
            
            if (is_openend)
                v_openend.update(dist, &v);
            else if (is_corner)
                v_corner.update(dist, &v);
            else
                v_side.update(dist, &v);
        }
        
        if (v_openend.value) { result = v_openend.value->pn; return true; }
        if (v_corner .value) { result = v_corner .value->pn; return true; }
        if (v_side   .value) { result = v_side   .value->pn; return true; }
        
        return false;
    };
    
    // offset endpoints based on snapping
    PointNormal pn_snap;
    bool is_snapped = false;
    if (find_snap(curve.front(), pn_snap)) {
        curve.offset_front(pn_snap);
        is_snapped = true;
    }
    if (find_snap(curve.back(), pn_snap)) {
        curve.offset_back(pn_snap);
        is_snapped = true;
    }
    
    if (is_snapped)
        // project again
        project(curve);
    
    // speed-dependent snap radius
    double snapSize = configTemp.snapSize();
    double delta_max = 10;
    double mouse_delta = min<double>((common_mouse_pos - common_mouse_pos_prev).cast<double>().norm(), delta_max);
    snapSize *= 1 + mouse_delta / delta_max;
    
    curvenetwork.add_curve_open(curve, configTemp.snapSize(), corner_at_openend);
}

void SketchRetopo::compute_patch_interior_pn(curvenetwork::Patch* patch) {
    // initial solution by direct Laplace solve
    int n_interior_vertices = 0;
    for (auto v : patch->vertices()) {
        if (!patch->is_boundary(v)) {
            ++n_interior_vertices;
            continue;
        }
        
        auto& vdata = patch->data(v);
        vdata.laplaceDirect.value = vdata.pn;
    }
    patch->laplaceDirect_solve();
    
    // pass result to laplace iterative solver
    for (auto v : patch->vertices()) {
        auto& vdata = patch->data(v);
        vdata.laplaceIterative.value = vdata.laplaceDirect.value;
    }
    
    // interleaved iterations of Laplacian smoothing + projection
    int n_iterations = max<int>(static_cast<int>(n_interior_vertices * 0.01), 5);
    for (int i = 0; i < n_iterations; ++i) {
        // laplace smoothing
        patch->laplaceIterative_compute(1);
        
        for (auto v : patch->vertices()) {
            if (patch->is_boundary(v))
                continue;
            
            auto& pn = patch->data(v).laplaceIterative.value;
                
            if (pn.tail(3).isZero())
                continue;
            
            // projection
            pn_normalize(pn);
			//---------------------------------------------------------------------------------------------------
			//Because there's no need to generate patch on the surface, so the projection is commented
			//---------------------------------------------------------------------------------------------------
            //project(pn);
        }
    }
    
    // copy result back
    for (auto v : patch->vertices()) {
        if (patch->is_boundary(v))
            continue;
        
        patch->data(v).pn = patch->data(v).laplaceIterative.value;
    }
    
    patch->invalidate_displist();
}
vector<BaseMesh::FHandle> SketchRetopo::parameterize_basemesh_under_patch(curvenetwork::Patch* patch, BaseMesh::FHandle submesh_seed) {
    using Patch = curvenetwork::Patch;
    
    Patch::VHandle p_corner0 = patch->patch_corner(0);                           // Patch corner vertex with flag 0
    
    struct Constraint {                     // Tuple representing a constraint. Can handle face/edge/vertex-wise constraints.
        BaseMesh::VHandle vertex[3];
        double coord[3];
        Vector2d value;
    };
    vector<Constraint> constraints;
    
    // Clear floodfill flag
    for (auto f : basemesh.faces())
        basemesh.data(f).floodfill_flag = false;
    
    vector<BaseMesh::FHandle> submesh_faces;
    
    // For every patch boundary edge, set constraints
    auto p_h = patch->halfedge_handle(p_corner0);
    for (int i = 0; i < patch->num_corners(); ++i) {
        for (int j = 0; j < patch->num_subdiv(i); ++j, p_h = patch->prev_halfedge_handle(p_h)) {
            auto p_v0v1 = patch->util_halfedge_to_vertex_pair(patch->opposite_halfedge_handle(p_h));
            auto p_v0 = p_v0v1.first;
            auto p_v1 = p_v0v1.second;
            
            // Face constraint
            embree::Hit hit      = intersect(patch->data(p_v0).pn);
            embree::Hit hit_next = intersect(patch->data(p_v1).pn);
            if (hit) {
                auto b_f = basemesh.face_handle(hit.id0);
                auto b_fv = basemesh.util_fv_vec(b_f);
                constraints.push_back(Constraint{
                    {b_fv[0]          ,     b_fv[1],    b_fv[2] },
                    {1 - hit.u - hit.v,     hit.u  ,    hit.v   },
                    patch->data(p_v0).harmonic_uv
                });
            }
            if (!hit || !hit_next)
                continue;
            
            // Traverse along the geodesics toward the next patch boundary vertex, adding edge constraints
    	    geodesic::SurfacePoint source(&geodesic_mesh.faces()[hit     .id0], 1 - hit     .u - hit     .v, hit     .u, hit     .v);
    	    geodesic::SurfacePoint target(&geodesic_mesh.faces()[hit_next.id0], 1 - hit_next.u - hit_next.v, hit_next.u, hit_next.v);
		    vector<geodesic::SurfacePoint> sources(1, source);
		    vector<geodesic::SurfacePoint> stop_points(1, target);
		    vector<geodesic::SurfacePoint> path;
            geodesic_algorithm.propagate(sources, geodesic::GEODESIC_INF, &stop_points);
		    geodesic_algorithm.trace_back(target, path);
            boost::reverse(path);                              // originally path is oriented from target to source
            double path_length_total = geodesic::length(path);
            double path_length_acc = 0;
            for (size_t k = 1; k < path.size() - 1; ++k) {
                geodesic::SurfacePoint& p = path[k];
                path_length_acc += p.distance(&path[k - 1]);
                double t = path_length_acc / path_length_total;
                Vector2d uv = (1 - t) * patch->data(p_v0).harmonic_uv +
                               t      * patch->data(p_v1).harmonic_uv;
                
                if (p.type() == geodesic::VERTEX) {
                    geodesic::vertex_pointer vertex = static_cast<geodesic::vertex_pointer>(p.base_element());
                    // vertex constraint
                    constraints.push_back(Constraint{
                        { basemesh.vertex_handle(vertex->id()), BaseMesh::VHandle(), BaseMesh::VHandle() },
                        { 1                                   , 0                  , 0                   },
                        uv
                    });
                
                } else if (p.type() == geodesic::EDGE) {
                    geodesic::edge_pointer edge = static_cast<geodesic::edge_pointer>(p.base_element());
                    auto b_v0 = basemesh.vertex_handle(edge->adjacent_vertices()[0]->id());
                    auto b_v1 = basemesh.vertex_handle(edge->adjacent_vertices()[1]->id());
                    auto p0 = o2e(basemesh.point(b_v0));
                    auto p1 = o2e(basemesh.point(b_v1));
                    double s = (Vector3d(p.x(), p.y(), p.z()) - p0).dot(p1 - p0) / (p1 - p0).squaredNorm();
                    // edge constraint
                    constraints.push_back(Constraint{
                        { b_v0 , b_v1, BaseMesh::VHandle() },
                        { 1 - s, s   , 0                   },
                        uv
                    });
                }
                
                // set floodfill flag for adjacent faces
                for (size_t m = 0; m < p.base_element()->adjacent_faces().size(); ++m) {
                    auto b_f = basemesh.face_handle(p.base_element()->adjacent_faces()[m]->id());
                    basemesh.data(b_f).floodfill_flag = true;
                    submesh_faces.push_back(b_f);
                }
            }
        }
    }
    
    // Collect basemesh face IDs inside the patch boundary by flood filling starting from the seed face
    const size_t n_submesh_faces_limit = submesh_faces.size() * 10000;
    vector<BaseMesh::FHandle> candidates = { submesh_seed };
    while (!candidates.empty()) {
        if (n_submesh_faces_limit < submesh_faces.size()) {
            cout << "Sub-mesh extraction process doesn't seem to terminate!\n";
            return {};
        }
        auto b_f = candidates.back();
        candidates.pop_back();
        basemesh.data(b_f).floodfill_flag = true;
        submesh_faces.push_back(b_f);
        for (auto b_ff : basemesh.ff_range(b_f))
            if (!basemesh.data(b_ff).floodfill_flag)
                candidates.push_back(b_ff);
    }
    
    container_util::remove_duplicate(submesh_faces);
    
    // Mapping from vertices in the submesh to indices in the system matrix
    struct Hash {
        size_t operator()(const BaseMesh::VHandle& key) const { return key.idx(); }
    };
    unordered_map<BaseMesh::VHandle, int, Hash> vtx2var;
    for (auto b_f : submesh_faces)
        for (auto b_v : basemesh.fv_range(b_f))
            if (vtx2var.find(b_v) == vtx2var.end())
                vtx2var[b_v] = vtx2var.size();
    int n = vtx2var.size();
    
    // Construct Laplacian matrix
    vector<Triplet<double, int>> L_triplets;
    for (auto p_vtx_var : vtx2var) {
        auto b_v = p_vtx_var.first;
        int i    = p_vtx_var.second;
        
        double weight_sum = 0;
        for (auto b_ve : basemesh.ve_range(b_v)) {
            auto b_vv = basemesh.util_opposite_vertex(b_ve, b_v);
            auto j = container_util::at_optional(vtx2var, b_vv);
            if (!j) continue;
            //double weight = basemesh.cotanWeight(b_ve);           // Not sure if this is appropriate.
            double weight = 0;
            for (int k = 0; k < 2; ++k) {
                auto b_h = basemesh.halfedge_handle(b_ve, k);
                if (basemesh.is_boundary(b_h)) continue;
                if (basemesh.data(basemesh.face_handle(b_h)).floodfill_flag)
                    weight += basemesh.data(b_h).cotangent;             //**********************weight += cotangent
            }
            L_triplets.push_back( { i, *j, weight } );                            // Off-diagonal
            weight_sum += weight;
        }
        L_triplets.push_back({ i, i, -weight_sum });                              // Diagonal
    }
    SparseMatrix<double> L(n, n);
    L.setFromTriplets(L_triplets.begin(), L_triplets.end());
    
    // Weight for soft constraint
    double constraint_weight = 0;
    for (auto triplet : L_triplets)
        constraint_weight += abs(triplet.value());
    constraint_weight *= 0.1;
    
    // Specify soft constraints on the matrix and right hand side
    vector<Triplet<double, int>> C_triplets;
    MatrixX2d rhs = MatrixX2d::Zero(n, 2);
    for (auto constraint : constraints) {
        for (int k0 = 0; k0 < 3; ++k0) {
            auto i = container_util::at_optional(vtx2var, constraint.vertex[k0]);
            if (!i) continue;
            for (int k1 = 0; k1 < 3; ++k1) {
                auto j = container_util::at_optional(vtx2var, constraint.vertex[k1]);
                if (!j) continue;
                C_triplets.push_back({*i, *j, constraint_weight * constraint.coord[k0] * constraint.coord[k1]});
            }
            rhs.row(*i) += Vector2d(constraint_weight * constraint.coord[k0] * constraint.value);
        }
    }
    SparseMatrix<double> C(n, n);
    C.setFromTriplets(C_triplets.begin(), C_triplets.end());
    
    // Solve
    SparseMatrix<double> A = -L + C;
    //SparseMatrix<double> A = L * L + C;                 // This may be better? The sign could be wrong...
    SimplicialCholesky<SparseMatrix<double>> solver(A);
    MatrixX2d x = solver.solve(rhs);
    
    // Copy results
    for (auto p_vtx_var : vtx2var) {
        auto b_v = p_vtx_var.first;
        int i    = p_vtx_var.second;
        basemesh.data(b_v).harmonic_uv = x.row(i);
    }

    return submesh_faces;
}
void SketchRetopo::compute_patch_interior_pn_harmonic(curvenetwork::Patch* patch, BaseMesh::FHandle submesh_seed) {
    using Patch = curvenetwork::Patch;
    
    Patch::VHandle p_corner0;                           // Patch corner vertex with flag 0
    for (auto p_v : patch->vertices()) {                // For clarity, handles for Patch and BaseMesh are prefixed by p_ and b_, respectively.
        if (patch->data(p_v).patchgen.corner_index == 0) {
            p_corner0 = p_v;
            break;
        }
    }
    
    int num_sides = patch->num_corners();
    VectorXi l;    
    l.resize(num_sides);
    for (int i = 0; i < num_sides; ++i)
        l[i] = patch->num_subdiv(i);
    
    // Step I: Compute harmonic parameterization for the patch |
    //---------------------------------------------------------+
    auto p_h = patch->halfedge_handle(p_corner0);
    for (int i = 0; i < num_sides; ++i) {
        for (int j = 0; j < l[i]; ++j, p_h = patch->prev_halfedge_handle(p_h)) {
            double t = i + j / static_cast<double>(l[i]);
            patch->data(patch->from_vertex_handle(p_h)).laplaceDirect.value << patchgen::get_boundary_geometry(num_sides, t), 0, 0, 0, 0; // Fixed boundary
        }
    }
    patch->laplaceDirect_solve();
    for (auto p_v : patch->vertices()) {
        auto& vdata = patch->data(p_v);
        vdata.harmonic_uv = vdata.laplaceDirect.value.head(2);
    }
    
    // Step II: Compute harmonic parameterization for the base mesh |
    //--------------------------------------------------------------+
    vector<BaseMesh::FHandle> submesh_faces = parameterize_basemesh_under_patch(patch, submesh_seed);
    
    // Step III: For each patch interior vertex, find position on the basemesh with the same harmonic_uv |
    //---------------------------------------------------------------------------------------------------+
    for (auto p_v : patch->vertices()) {                            // Brute force search. Could be accelerated.
        if (patch->is_boundary(p_v)) continue;
        Vector2d p_uv = patch->data(p_v).harmonic_uv;
        
        for (auto b_f : submesh_faces) {
            auto b_fv = basemesh.util_fv_vec(b_f);
            Vector2d b_uv0 = basemesh.data(b_fv[0]).harmonic_uv;
            Vector2d b_uv1 = basemesh.data(b_fv[1]).harmonic_uv;
            Vector2d b_uv2 = basemesh.data(b_fv[2]).harmonic_uv;
            Vector3d t;
            if (!eigen_util::project_to_triangle(b_uv0, b_uv1, b_uv2, p_uv, t)) continue;
            if (t.minCoeff() < 0) continue;
            
            PointNormal b_pn0;      b_pn0 << o2e(basemesh.point(b_fv[0])), o2e(basemesh.normal(b_fv[0]));
            PointNormal b_pn1;      b_pn1 << o2e(basemesh.point(b_fv[1])), o2e(basemesh.normal(b_fv[1]));
            PointNormal b_pn2;      b_pn2 << o2e(basemesh.point(b_fv[2])), o2e(basemesh.normal(b_fv[2]));
            PointNormal& p_pn = patch->data(p_v).pn;
            p_pn = t[0] * b_pn0 +
                   t[1] * b_pn1 +
                   t[2] * b_pn2;

            pn_normalize(p_pn);
            project(p_pn); //The projection should be removed, but I don't know if it's just that simple yet.
            
            break;
        }
    }
    
    patch->invalidate_displist();
}

void SketchRetopo::add_cylinder_curve(Polyline_PointNormal cylinder_curve) {
    // set flags for all halfedges that are on patch boundaries and form loops
    curvenetwork.set_flag_halfedges(curvenetwork::Core::FlagOp::SUBST, 0);
    for (auto& c : curvenetwork.halfchains) {
        if (c.patch)
            // this halfchain is not on a patch boundary
            continue;
        
        if (c.halfedge_front->flag)
            // we already know it's on a loop
            continue;
        
        // see if this halfchain is on a loop
        curvenetwork::Halfchain* c_front(nullptr);
        curvenetwork::Halfchain* c_back (nullptr);
        curvenetwork::trace_halfchains(&c, c_front, c_back);
        if (c_front->prev() == c_back) {
            // set flags for all halfedges on the same loop
            int num_corner = 0;
            for (auto h = c.halfedge_front; ; ) {
                h->flag = 1;
                if (h->vertex->is_corner())
                    ++num_corner;
                h = h->next;
                if (h == c.halfedge_front)
                    break;
            }
            if (num_corner == 0) {
                // if there's no corner on this loop, set a special flag for each halfedge
                for (auto h = c.halfedge_front; ; ) {
                    h->flag = 2;
                    h = h->next;
                    if (h == c.halfedge_front)
                        break;
                }
            }
        }
    }
    
    // find a halfedge each endpoint snaps to
    curvenetwork::Halfedge* h_snapped[2] = { 0, 0 };
    for (int i = 0; i < 2; ++i) {
        auto& pn_endpoint = i == 0 ? cylinder_curve.front() : cylinder_curve.back();
        Vector3d d;
        if (i == 0) d = (*++cylinder_curve. begin() - *cylinder_curve. begin()).head(3);
        else        d = (*++cylinder_curve.rbegin() - *cylinder_curve.rbegin()).head(3);
        d.normalize();
        
        double dist_min = util::dbl_max();
        for (auto& h : curvenetwork.halfedges) {
            if (h.is_deleted || !h.flag) continue;
            
            if (h.flag == 1 && !h.vertex->is_corner())
                // if this loop has any corners, endpoint should always snap to corners
                continue;
            
            double dist = pn_norm(h.vertex->pn - pn_endpoint);
            
            double dp = h.toVector3d().cross(d).dot(pn_endpoint.tail(3));       // to avoid choosing halfedge on the wrong side
            
            if (dist < dist_min && dist < configTemp.snapSize() && dp > 0) {
                dist_min = dist;
                h_snapped[i] = &h;
            }
        }
    }
    
    if (!h_snapped[0] || !h_snapped[1])
        // both endpoints have to be snapped!
        return;
    
    // make sure these loops are different ones, check number of corners
    int num_corners[2] = { 0, 0 };
    for (int i = 0; i < 2; ++i) {
        for (auto h = h_snapped[i]; ; ) {
            if (h == h_snapped[(i + 1) % 2]) {
                // two halfedges are on the same edge flow!
                h_snapped[0] = 0;
                break;
            }
            
            if (h->vertex->is_corner())
                ++num_corners[i];
            
            h = h->next;
            if (h == h_snapped[i])
                break;
        }
        
        if (!h_snapped[0])
            return;
    }
    
    // check if corners on the loops match
    if (num_corners[0] && num_corners[1] && num_corners[0] != num_corners[1])
        return;
    
    // obtain loop curve geometry
    Polyline_PointNormal loop_curve[2];
    for (int i = 0; i < 2; ++i) {
        for (auto h = h_snapped[i]; ; ) {
            loop_curve[i].push_back(h->vertex->pn);
            
            h = i == 0 ? h->next : h->prev;
            if (h == h_snapped[i])
                break;
        }
        loop_curve[i].is_loop = true;
    }
    
    // reorder such that num_corners[1] is equal or greater than num_corner[0]
    if (num_corners[0] > num_corners[1]) {
        reverse(cylinder_curve.begin(), cylinder_curve.end());
        swap(h_snapped  [0], h_snapped  [1]);
        swap(num_corners[0], num_corners[1]);
        swap(loop_curve [0], loop_curve [1]);
        loop_curve[0].reverse();
        loop_curve[1].reverse();
    }
    
    vector<PointNormal> geodesic_endpoint[2];
    
    if (num_corners[0] > 0) {
        assert(num_corners[0] == num_corners[1]);
        
        // just connect corresponding corners
        for (int i = 0; i < 2; ++i) {
            for (auto h = i == 0 ? h_snapped[i]->next : h_snapped[i]->prev; ; ) {
                if (h->vertex->is_corner())
                    geodesic_endpoint[i].push_back(h->vertex->pn);
                
                if (h == h_snapped[i])
                    break;
                h = i == 0 ? h->next : h->prev;
            }
        }
        
    } else if (num_corners[1] > 0) {
        assert(!num_corners[0]);
        
        // get arc length parameter from num_corners[1] side
        vector<double> arc_length_parameter(num_corners[1], 0);
        auto c = h_snapped[1]->halfchain;
        for (int i = 0; i < num_corners[1]; ++i, c = c->prev())
            arc_length_parameter[i] = (i == 0 ? 0 : arc_length_parameter[i - 1]) + c->length();
        assert(c == h_snapped[1]->halfchain);
        
        for (int i = 0; i < num_corners[1]; ++i)
            arc_length_parameter[i] /= arc_length_parameter.back();
        
        // sample point on loop_curve[0] according to the above arc length parameter
        for (int i = 0; i < num_corners[1]; ++i)
            geodesic_endpoint[0].push_back(loop_curve[0].point_at(arc_length_parameter[i]));
        
        // sample corners on loop_curve[1]
        for (auto h = h_snapped[1]->prev; ; ) {
            if (h->vertex->is_corner())
                geodesic_endpoint[1].push_back(h->vertex->pn);
            
            if (h == h_snapped[1])
                break;
            h = h->prev;
        }
    
    } else {
        assert(!num_corners[0] && !num_corners[1]);
        
        for (int i = 0; i < 2; ++i) {
            for (int j = 1; j <= configTemp.cylinder_num_div; ++j)
                geodesic_endpoint[i].push_back(loop_curve[i].point_at(j / static_cast<double>(configTemp.cylinder_num_div)));
        }

        assert(!num_corners[0] && !num_corners[1]);
    }
    assert(geodesic_endpoint[0].size() == geodesic_endpoint[1].size());
    
    // last points correspond to cylinder_curve --> omit
    geodesic_endpoint[0].pop_back();
    geodesic_endpoint[1].pop_back();
    
    memento_store();
    
    // add geodesic traced curve
    loop_util::for_each(
        geodesic_endpoint[0].begin(), geodesic_endpoint[0].end(), geodesic_endpoint[1].begin(), 
        [&] (const PointNormal& pn_from, const PointNormal& pn_to) {
            auto geodesic_curve = geodesic_compute(pn_from, pn_to);
            if (geodesic_curve.empty())
                return;
            
            // reampling, projection
            geodesic_curve.resample_by_length(configTemp.segmentSize());
            for_each(
                geodesic_curve.begin(), geodesic_curve.end(),
                [&] (PointNormal& pn) { project(pn); }); //No need to change the project, because the method is abandoned.
            
            add_curve_open_delegate(geodesic_curve);
    });
    
    // add sketched curve
    add_curve_open_delegate(cylinder_curve);
    
    // check if the opposite side of each snapped halfedge forms a reasonable loop. if yes, activate the corner flag
    for (int i = 0; i < 2; ++i) {
        if (i == 0)
            loop_curve[i].reverse();

        if (!is_loop_ccw(loop_curve[i]))
            continue;
        
        curvenetwork::Halfedge* h_start = h_snapped[i]->opposite;
        for (auto h = h_start; ; ) {
            if (h->vertex->is_corner())
                h->is_corner = true;
            
            h = h->next;
            if (h == h_start)
                break;
        }
    }
}

Polyline_PointNormal SketchRetopo::trace_on_plane(const PointNormal& pn_start, const Vector3d& plane_normal, double dist_max) {
    const Polyline_PointNormal empty_result;
    
    // intersected basemesh face
    auto hit = intersect(pn_start);
    if (!hit)
        return empty_result;
    
    // helper functions
    auto plane_func = [pn_start, plane_normal] (const Vector3d& p) { return plane_normal.dot(p - pn_start.head(3)); };
    
    auto is_crossing = [plane_func] (const BaseMesh& mesh, const BaseMesh::HHandle h) {
        auto p01 = mesh.util_halfedge_to_point_pair(h);
        return plane_func(o2e(p01.first)) * plane_func(o2e(p01.second)) <= 0;
    };
    
    // find a starting halfedge
    BaseMesh::HHandle h_start;
    for (auto h : basemesh.fh_range(basemesh.face_handle(hit.id0))) {
        if (is_crossing(basemesh, h)) {
            h_start = h;
            break;
        }
    }
    if (!h_start.is_valid())
        return empty_result;
    
    // check if the chain of halfedges is open or closed
    for (auto h = h_start; ; ) {
        auto h1 = basemesh.next_halfedge_handle(h);
        auto h2 = basemesh.next_halfedge_handle(h1);
        
        auto h_next = is_crossing(basemesh, h1) ? h1 : h2;
        h_next = basemesh.opposite_halfedge_handle(h_next);
        
        if (basemesh.is_boundary(h_next)) {
            // reached a boundary
            h_start = basemesh.opposite_halfedge_handle(h_next);
            break;
        
        } else if (h_next == h_start) {
            // loop detected!
            break;
        }
        
        h = h_next;
    }
    
    // start walking on basemesh
    Polyline_PointNormal result;
    double dist_sum = 0;
    for (auto h = h_start; ; ) {
        // get edge segment
        auto v0v1 = basemesh.util_halfedge_to_vertex_pair(h);
        PointNormal pn0 = basemesh.get_pn(v0v1.first );
        PointNormal pn1 = basemesh.get_pn(v0v1.second);
        
        // get intersected point
        double t0 = plane_func(pn0.head(3));
        double t1 = plane_func(pn1.head(3));
        assert(t0 * t1 <= 0);
        double u = - t0 / (t1 - t0);
        PointNormal pn = (1 - u) * pn0 + u * pn1;
        
        pn_normalize(pn);
        project(pn); //Have no idea yet
        
        if (!result.empty()) {
            dist_sum += pn_norm(result.back() - pn);
            if (dist_max < dist_sum)
                // have traveled too far
                return empty_result;
        }
        
        result.push_back(pn);
        
        // look for next halfedge
        auto h1 = basemesh.next_halfedge_handle(h);
        auto h2 = basemesh.next_halfedge_handle(h1);
        
        auto h_next = is_crossing(basemesh, h1) ? h1 : h2;
        h_next = basemesh.opposite_halfedge_handle(h_next);
        
        if (basemesh.is_boundary(h_next) || h_next == h_start) {        // reached a boundary, or loop detected
            if (h_next == h_start)
                result.is_loop = true;
            
            // resampling
            result.resample_by_length(configTemp.segmentSize());
            //result.insert_points_per_segment(9);
            project(result);
            
            return result;
        }
        
        h = h_next;
    }
    
    // should not reach here
    return empty_result;
}

void                 SketchRetopo::edgeLoop_insert (curvenetwork::Patch* patch_start, curvenetwork::Patch::HHandle h_start, double t) {
    if (!patch_start || !h_start.is_valid() || !patch_start->is_boundary(h_start) || t <= 0 || 1 <= t)
        return;
    
    memento_store();
    
    // check if this global (i.e., inter-patch) edge loop is open or closed
    auto patch = patch_start;
    auto h     = h_start;
    while (true) {
        auto edgeLoop = find_edgeloop(*patch, h);
        assert(edgeLoop.front() != edgeLoop.back());
        
        auto h_opposite = patch->opposite_halfedge_handle(edgeLoop.front());
        assert(patch->is_boundary(h_opposite));
        
        auto patch_next = patch->opposite_patch            (h_opposite);
        auto h_next     = patch->opposite_boundary_halfedge(h_opposite);
        
        if (!patch_next) {
            // reached a boundary
            patch_start = patch;
            h_start     = h_opposite;
            t = 1 - t;
            break;
        } else if (patch_next == patch_start && h_next == h_start) {
            // loop detected
            break;
        }
        
        patch = patch_next;
        h     = h_next;
    }
    
    vector<curvenetwork::Edgechain*> e_affected;
    vector<curvenetwork::Patch    *> patch_affected;
    
    patch = patch_start;
    h     = h_start;
    while (true) {
        auto edgeLoop = find_edgeloop(*patch, h);
        
        auto h_opposite = patch->opposite_halfedge_handle(edgeLoop.front());
        
        e_affected    .push_back(patch->data(h         ).halfchain->edgechain);
        e_affected    .push_back(patch->data(h_opposite).halfchain->edgechain);
        patch_affected.push_back(patch);
        
        auto patch_next = patch->opposite_patch            (h_opposite);
        auto h_next     = patch->opposite_boundary_halfedge(h_opposite);
        
        insert_edgeloop<curvenetwork::Patch>(*patch, h, t,
            [&] (curvenetwork::Patch& patch_, OpenMesh::HalfedgeHandle h_old, OpenMesh::VertexHandle v_new) {
                auto v0v1 = patch_.util_halfedge_to_vertex_pair(h_old);
                auto& vdata0    = patch_.data(v0v1.first );
                auto& vdata1    = patch_.data(v0v1.second);
                auto& vdata_new = patch_.data(v_new      );
                
                vdata_new.pn = (1 - t) * vdata0.pn + t * vdata1.pn;
                pn_normalize(vdata_new.pn);
                project(vdata_new.pn); //Temporily keep, because this method is not used yet
        });
        
        if (!patch_next || patch_next == patch_start && h_next == h_start)
            break;
        patch = patch_next;
        h     = h_next;
    }
    
    container_util::remove_duplicate(e_affected);
    for_each(e_affected.begin(), e_affected.end(),
        [] (curvenetwork::Edgechain* e) { ++e->num_subdiv; });
    
    container_util::remove_duplicate(patch_affected);
    for_each(patch_affected.begin(), patch_affected.end(),
        [] (curvenetwork::Patch* patch_) {
            patch_->set_halfedgeData();
            patch_->invalidate_displist();
    });
}
Polyline_PointNormal SketchRetopo::edgeLoop_preview(curvenetwork::Patch* patch_start, curvenetwork::Patch::HHandle h_start, double t) const {
    auto patch_start_orig = patch_start;
    auto h_start_orig = h_start;
    double t_orig = t;
    
    if (!patch_start || !h_start.is_valid() || !patch_start->is_boundary(h_start) || t <= 0 || 1 <= t)
        return Polyline_PointNormal();
    
    Polyline_PointNormal result;
    
    // check if this global (i.e., inter-patch) edge loop is open or closed
    auto patch = patch_start;
    auto h     = h_start;
    while (true) {
        if (!h.is_valid())
            // bug!
            return result;
        
        auto edgeLoop = find_edgeloop(*patch, h);
        
        auto h_opposite = patch->opposite_halfedge_handle(edgeLoop.front());
        
        auto patch_next = patch->opposite_patch            (h_opposite);
        auto h_next     = patch->opposite_boundary_halfedge(h_opposite);
        
        if (!patch_next) {
            // reached a boundary
            patch_start = patch;
            h_start     = h_opposite;
            t = 1 - t;
            break;
        } else if (patch_next == patch_start && h_next == h_start) {
            // loop detected
            break;
        }
        
        patch = patch_next;
        h     = h_next;
    }
    
    patch = patch_start;
    h     = h_start;
    while (true) {
        auto edgeLoop = find_edgeloop(*patch, h);
        
        for_each(
            edgeLoop.rbegin(), edgeLoop.rend(),
            [&] (curvenetwork::Patch::HHandle h) {
                auto v0v1 = patch->util_halfedge_to_vertex_pair(h);
                auto& pn0 = patch->data(v0v1.first ).pn;
                auto& pn1 = patch->data(v0v1.second).pn;
                PointNormal pn = (1 - t) * pn0 + t * pn1;
                pn_normalize(pn);
                result.push_back(pn);
        });
        
        auto h_opposite = patch->opposite_halfedge_handle(edgeLoop.front());
        
        auto patch_next = patch->opposite_patch            (h_opposite);
        auto h_next     = patch->opposite_boundary_halfedge(h_opposite);
        
        if (!patch_next || patch_next == patch_start && h_next == h_start)
            break;
        patch = patch_next;
        h     = h_next;
    }
    
    return result;
}

vector<pair<curvenetwork::Patch*, curvenetwork::Patch::HHandle>> SketchRetopo::edgeLoop_walk(curvenetwork::Patch* patch_start, curvenetwork::Patch::HHandle h_start) const {
    typedef vector<pair<curvenetwork::Patch*, curvenetwork::Patch::HHandle>> Result;
    
    auto patch_start_orig = patch_start;
    auto h_start_orig = h_start;
    
    if (!patch_start || !h_start.is_valid() || !patch_start->is_boundary(h_start))
        return Result();
    
    Result result;
    
    // check if this global (i.e., inter-patch) edge loop is open or closed
    auto patch = patch_start;
    auto h     = h_start;
    while (true) {
        auto edgeLoop = find_edgeloop(*patch, h);
        
        auto h_opposite = patch->opposite_halfedge_handle(edgeLoop.front());
        
        auto patch_next = patch->opposite_patch            (h_opposite);
        auto h_next     = patch->opposite_boundary_halfedge(h_opposite);
        
        if (!patch_next) {
            // reached a boundary
            patch_start = patch;
            h_start     = h_opposite;
            break;
        } else if (patch_next == patch_start && h_next == h_start) {
            // loop detected
            break;
        }
        
        patch = patch_next;
        h     = h_next;
    }
    
    patch = patch_start;
    h     = h_start;
    while (true) {
        result.push_back(make_pair(patch, h));
        
        auto edgeLoop = find_edgeloop(*patch, h);
        
        auto h_opposite = patch->opposite_halfedge_handle(edgeLoop.front());
        
        auto patch_next = patch->opposite_patch            (h_opposite);
        auto h_next     = patch->opposite_boundary_halfedge(h_opposite);
        
        if (!patch_next) {
            result.push_back(make_pair(patch, h_opposite));
            break;
        }
        
        if (patch_next == patch_start && h_next == h_start)
            break;
        
        patch = patch_next;
        h     = h_next;
    }
    
    return result;
}

void SketchRetopo::vertices_move  (const PointNormal& pn_center, const Vector3d& center_offset, double radius) {
    for (auto& patch : curvenetwork.patches) {
        for (auto v : patch.vertices()) {
            auto& pn = patch.data(v).pn;
            
            double dist = pn_norm(pn_center - pn);
            double dot  = pn_center.tail(3).dot(pn.tail(3));
            if (dist > radius || dot < 0)
                continue;
            
            pn.head(3) += RBFKernel_Wendland()(dist / radius) * center_offset;
            project(pn); //This needs to be rewritten.
            
            patch.invalidate_displist();
        }
    }
    
    vector<curvenetwork::Edgechain*> affected_edgechains;
    for (auto& v : curvenetwork.vertices) {
        double dist = pn_norm(pn_center - v.pn);
        double dot = pn_center.tail(3).dot(v.pn.tail(3));
        if (dist > radius || dot < 0)
            continue;
        
        for (curvenetwork::VOCIter c(&v); c; ++c)
            affected_edgechains.push_back(c->edgechain);
    }
    
    container_util::remove_duplicate(affected_edgechains);
    for (auto e : affected_edgechains) {
        for (int i = 0; i < 2; ++i) {
            if (e->halfchain[i]->patch) {
                set_halfchain_pn_from_patch(e->halfchain[i]);
                break;
            }
        }
    }
}
void SketchRetopo::vertices_smooth(const PointNormal& pn_center, double radius) {
    vector<curvenetwork::Edgechain*> affected_edgechains;
    
    for (auto& patch : curvenetwork.patches) {
        for (auto patch_v : patch.vertices()) {
            auto& vdata = patch.data(patch_v);
            vdata.pn_temp = vdata.pn;
            
            // too far or too differently oriented vertices are skipped
            double dist = pn_norm(vdata.pn - pn_center);
            double dot = vdata.pn.tail(3).dot(pn_center.tail(3));
            if (dist > radius || dot < 0)
                continue;
            
            auto average = vertices_smooth_sub(&patch, patch_v, affected_edgechains);
            if (!average)
                continue;
            
            // store falloff-weighted result into pn_temp
            double falloff = RBFKernel_Wendland()(dist / radius);
            vdata.pn_temp = (1 - falloff) * vdata.pn + falloff * (*average);
            pn_normalize(vdata.pn_temp);
            
            // projection
            project(vdata.pn_temp); //The projection should be removed, but I don't know if it's just that simple yet.
            
            // update diplay list
            patch.invalidate_displist();
        }
    }
    
    // copy result from pn_temp
    for (auto& patch : curvenetwork.patches) {
        for (auto v : patch.vertices()) {
            auto& vdata = patch.data(v);
            vdata.pn = vdata.pn_temp;
        }
    }
    
    // propagate position update to curve network vertices
    container_util::remove_duplicate(affected_edgechains);
    for (auto e : affected_edgechains) {
        for (int i = 0; i < 2; ++i) {
            if (e->halfchain[i]->patch) {
                set_halfchain_pn_from_patch(e->halfchain[i]);
                break;
            }
        }
    }
}
auto SketchRetopo::vertices_smooth_sub(curvenetwork::Patch* patch, curvenetwork::Patch::VHandle patch_v, std::vector<curvenetwork::Edgechain*>& affected_edgechains) -> boost::optional<PointNormal> const {
    PointNormal sum_pn     = PointNormal::Zero();
    double      sum_weight = 0;
    
    if (patch->is_boundary(patch_v)) {
        auto v = patch->vertex_patch_to_curveNetwork(patch_v);
        if (v) {
            // patch corner vertex
            if (v->is_boundary())
                // if this corner is on the boundary of curve network --> skip it
                return boost::none;
            
            // walk over the connected patches
            for (curvenetwork::VOCIter c(v); c; ++c) {
                auto c_patch = c->patch;
                
                auto patch_v2 = c_patch->vertex_curveNetwork_to_patch(v);
                if (!patch_v2.is_valid())
                    // bug!
                    continue;
                
                // sum up adjacent patch vertices
                for (auto w : c_patch->vv_range(patch_v2)) {
                    double weight = c_patch->is_boundary(w) ? 0.5 : 1.0;
                    sum_pn     += weight * c_patch->data(w).pn;
                    sum_weight += weight;
                }
                affected_edgechains.push_back(c->edgechain);
            }
            
        } else {
            // patch side vertex
            auto patch_h = patch->prev_halfedge_handle(patch->halfedge_handle(patch_v));
            auto c = patch->data(patch_h).halfchain;
            
            affected_edgechains.push_back(c->edgechain);
            
            auto& pn_prev = patch->data(patch->from_vertex_handle(patch_h                             )).pn;
            auto& pn_next = patch->data(patch->to_vertex_handle  (patch->next_halfedge_handle(patch_h))).pn;
            
            sum_pn     += pn_prev + pn_next;
            sum_weight += 2;
            
            auto patch_opposite = patch->opposite_patch(patch_h);
            if (patch_opposite) {
                // if vertex is not on the boundary of curve network, also add up patch interior vertices
                for (auto w : patch->vv_range(patch_v)) {
                    if (patch->is_boundary(w))
                        continue;
                    sum_pn     += patch->data(w).pn;
                    sum_weight += 1;
                }
                
                // from opposite patch
                auto patch_opposite_h = patch->opposite_boundary_halfedge(patch_h);
                auto patch_opposite_v = patch_opposite->from_vertex_handle(patch_opposite_h);
                for (auto w : patch_opposite->vv_range(patch_opposite_v)) {
                    if (patch_opposite->is_boundary(w))
                        continue;
                    sum_pn     += patch_opposite->data(w).pn;
                    sum_weight += 1;
                }
            }
        }
        
    } else {
        // patch interior vertex
        for (auto w : patch->vv_range(patch_v)) {
            sum_pn     += patch->data(w).pn;
            sum_weight += 1;
        }
        
    }
    
    return sum_pn /= sum_weight;
}
void SketchRetopo::vertices_smooth_global() {
    for (auto& patch : curvenetwork.patches) {
        for (auto patch_v : patch.vertices()) {
            auto& vdata = patch.data(patch_v);
            vdata.pn_temp = vdata.pn;
            
            vector<curvenetwork::Edgechain*> affected_edgechains;
            auto average = vertices_smooth_sub(&patch, patch_v, affected_edgechains);
            if (!average)
                continue;
            
            vdata.pn_temp = *average;
            pn_normalize(vdata.pn_temp);
            project(vdata.pn_temp); //The projection should be removed, but I don't know if it's just that simple yet.
        }
        patch.invalidate_displist();
    }
    // copy result from pn_temp
    for (auto& patch : curvenetwork.patches) {
        for (auto patch_v : patch.vertices()) {
            auto& vdata = patch.data(patch_v);
            vdata.pn = vdata.pn_temp;
        }
    }
    
    // propagate position update to curve network vertices
    for (auto& e : curvenetwork.edgechains) {
        for (int i = 0; i < 2; ++i) {
            if (e.halfchain[i]->patch) {
                set_halfchain_pn_from_patch(e.halfchain[i]);
                break;
            }
        }
    }
}
void SketchRetopo::set_halfchain_pn_from_patch(curvenetwork::Halfchain* c) {
    auto patch = c->patch;
    if (!patch)
        return;
    
    // find patch boundary halfedge corresponding to this halfchain
    auto patch_h_start = patch->prev_halfedge_handle(patch->halfedge_handle(patch->patch_corner(0)));
    auto patch_h = patch_h_start;
    while (true) {
        if (patch->data(patch_h).halfchain == c)
            break;
        
        patch_h = patch->prev_halfedge_handle(patch_h);
        if (patch_h == patch_h_start)
            // this should never happen!
            return;
    }
    
    // sample patch boundary points
    Polyline_PointNormal patch_boundary_curve;
    patch_boundary_curve.push_back(patch->data(patch->to_vertex_handle(patch_h)).pn);
    for (int i = 0; i < c->num_subdiv(); ++i) {
        patch_boundary_curve.push_back(patch->data(patch->from_vertex_handle(patch_h)).pn);
        patch_h = patch->prev_halfedge_handle(patch_h);
    }
    
    // update curve network vertex positions
    int n = c->toPolyline().size() - 1;
    auto h = c->halfedge_front;
    h->from_vertex()->pn = patch_boundary_curve.front();
    for (int i = 1; i <= n; ++i) {
        double t = i / static_cast<double>(n);
        h->vertex->pn = patch_boundary_curve.point_at(t);
        project(h->vertex->pn); //The projection should be removed, but I don't know if it's just that simple yet.
        
        h = h->next;
    }
}
void SketchRetopo::snap_symmetry() {
    memento_store();
    auto snap = [&](PointNormal& pn){ if (abs(pn[0]) < configTemp.segmentSize()) pn[0] = 0; };
    for (auto& v : curvenetwork.vertices) {
        if (v.is_deleted) continue;
        snap(v.pn);
    }
    for (auto& p : curvenetwork.patches) {
        if (p.is_deleted) continue;
        for (auto v : p.vertices())
            snap(p.data(v).pn);
    }
    curvenetwork.invalidate_displist();
    for (auto& p : curvenetwork.patches)
        p.invalidate_displist();
}
void SketchRetopo::trace_basemesh_boundary(BaseMesh::HHandle h_start, bool keep_exact) {
    if (!basemesh.is_boundary(h_start))
        return;
    memento_store();
    Polyline_PointNormal curve;
    for (auto h = h_start; ; ) {
        auto v0 = basemesh.from_vertex_handle(h);
        auto v1 = basemesh.to_vertex_handle(h);
        auto v2 = basemesh.to_vertex_handle(basemesh.next_halfedge_handle(basemesh.opposite_halfedge_handle(h)));
        Vector3d weight;
        if (keep_exact)
            weight << 1, 0, 0;
        else
            weight << 0.4, 0.4, 0.2;
        PointNormal pn =
            weight[0] * basemesh.get_pn(v0) +
            weight[1] * basemesh.get_pn(v1) +
            weight[2] * basemesh.get_pn(v2);
        pn_normalize(pn);
        curve.push_back(pn);
        h = basemesh.next_halfedge_handle(h);
        if (h == h_start)
            break;
    }
    curve.is_loop = true;
    if (configSaved.symmetric) {
        // don't add boundary loop that is entirely on the negative side
        bool is_all_negative = boost::count_if(curve, [] (PointNormal& pn) { return pn.x() >= 0; }) == 0;
        if (is_all_negative)
            return;
        // if there exists a segment having both positive and negative x coordinates, move one endpoint to x=0
        for (int i = 0; i < curve.size(); ++i) {
            int i_next = (i + 1) % curve.size();
            auto& pn0 = curve[i];
            auto& pn1 = curve[i_next];
            if (pn0.x() * pn1.x() >= 0)
                continue;
            double t = -pn0.x() / (pn1.x() - pn0.x());
            PointNormal pn = (1 - t) * pn0 + t * pn1;
            pn[0] = pn[3] = 0;
            pn_normalize(pn);
            pn0 = pn;
        }
        // look for curve point with negative x adjacent to x=0
        int rotate_pos = -1;
        for (int i = 0; i < curve.size(); ++i) {
            int i_prev = i == 0 ? curve.size() - 1 : i - 1;
            if (curve[i_prev].x() < 0 && curve[i].x() >= 0) {
                assert(curve[i].x() == 0);
                rotate_pos = i;
                break;
            }
        }
        // erase all points with negative x
        if (rotate_pos != -1) {
            rotate(curve.begin(), curve.begin() + rotate_pos, curve.end());
            boost::remove_erase_if(curve, [] (const PointNormal& pn) { return pn.x() < 0; });
            curve.is_loop = false;
        }
    }
    if (!keep_exact)
        // resample
        curve.resample_by_length(configTemp.segmentSize());
    // projOffset
    for (auto& pn : curve)
        pn.head(3) += configSaved.projOffset * pn.tail(3);
    if (curve.is_loop)
        curvenetwork.add_curve_loop(curve);
    else
        add_curve_open_delegate(curve, false);
    generate_patches();
}
Polyline_PointNormal SketchRetopo::geodesic_compute(const PointNormal& pn0, const PointNormal& pn1) {
    auto hit0 = intersect(pn0);
    auto hit1 = intersect(pn1);
    if (!hit0 || !hit1)
        return Polyline_PointNormal();
    
    geodesic::SurfacePoint source(&geodesic_mesh.faces()[hit0.id0], 1 - hit0.u - hit0.v, hit0.u, hit0.v);
    geodesic::SurfacePoint target(&geodesic_mesh.faces()[hit1.id0], 1 - hit1.u - hit1.v, hit1.u, hit1.v);
    
    std::vector<geodesic::SurfacePoint> path;
    geodesic_algorithm.geodesic(source, target, path);
    
    Polyline_PointNormal result;
    for (auto& sp : path) {
        auto sp_type = sp.type();
        if (sp_type == geodesic::UNDEFINED_POINT)
            continue;
        
        PointNormal pn = PointNormal::Zero();
        pn.head(3) << sp.x(), sp.y(), sp.z();
        if (sp_type == geodesic::FACE) {
            for (int i = 0; i < 3; ++i) {
                auto v = basemesh.vertex_handle(sp.base_element()->adjacent_vertices()[i]->id());
                pn.tail(3) += o2e(basemesh.normal(v));
            }
        
        } else if (sp_type == geodesic::EDGE) {
            for (int i = 0; i < 2; ++i) {
                auto v = basemesh.vertex_handle(sp.base_element()->adjacent_vertices()[i]->id());
                pn.tail(3) += o2e(basemesh.normal(v));
            }
        
        } else if (sp.type() == geodesic::VERTEX) {
            auto v = basemesh.vertex_handle(sp.base_element()->id());
            pn.tail(3) = o2e(basemesh.normal(v));
        }
        pn_normalize(pn);
        
        result.push_back(pn);
    }
    return result;
}
