#pragma once

#include <Eigen/Core>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <kt84/openmesh/base/CotanWeight.hh>
#include <kt84/openmesh/base/CenterOfMass.hh>
#include <kt84/openmesh/base/BoundingBox.hh>
#include <kt84/openmesh/base/LaplaceIterative.hh>
#include <kt84/openmesh/base/Utility.hh>
#ifndef NDEBUG
#   include <kt84/openmesh/base/DebugInfo.hh>
#endif
#include <kt84/geometry/PointNormal.hh>
#include <kt84/graphics/DisplayList.hh>

struct BaseMeshTraits : public OpenMesh::DefaultTraits {
    typedef OpenMesh::Vec3d Point;
    typedef OpenMesh::Vec3d Normal;
    typedef OpenMesh::Vec2d TexCoord2D;
    
    VertexTraits
        , public kt84::LaplaceIterative_VertexTraits<3>
    {
        Eigen::Vector2d harmonic_uv;                // UV computed with harmonic parameterization in SketchRetopo::compute_patch_interior_pn_harmonic(). Not to be confused with the texture UVs
    };
    
    FaceTraits
        , public kt84::CotanWeight_FaceTraits
    {
        bool floodfill_flag;
        FaceT()
            : floodfill_flag()
        {}
    };
    
    EdgeTraits
        , public kt84::CotanWeight_EdgeTraits
    {};
    
    HalfedgeTraits
        , public kt84::LaplaceIterative_HalfedgeTraits
        , public kt84::CotanWeight_HalfedgeTraits
    {};
};

typedef OpenMesh::TriMesh_ArrayKernelT<BaseMeshTraits> BaseMeshBase;

struct BaseMesh
    : public BaseMeshBase
    , public kt84::CotanWeight     <BaseMeshBase, BaseMesh>
    , public kt84::CenterOfMass    <BaseMeshBase, BaseMesh>
    , public kt84::BoundingBox     <BaseMeshBase, BaseMesh>
    , public kt84::LaplaceIterative<BaseMeshBase, BaseMesh, 3>
    , public kt84::Utility         <BaseMeshBase, BaseMesh>
#ifndef NDEBUG
    , public kt84::DebugInfo       <BaseMeshBase, BaseMesh>
#endif
{
    kt84::DisplayList displist_fill;
    kt84::DisplayList displist_line;
    bool has_texture;
    
    BaseMesh();
    void init();
    void render_fill();
    void render_line();
    void invalidate_displist() {
        displist_fill.invalidate();
        displist_line.invalidate();
    }
    kt84::PointNormal get_pn(VHandle v) const;
};
