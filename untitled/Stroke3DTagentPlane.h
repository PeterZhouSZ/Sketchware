#pragma once
#include "Stroke3D.h"

namespace SketcherCore {

class Stroke3DTagentPlane :
	public Stroke3D
{
public:
	Stroke3DTagentPlane(const Stroke2D<Point2D> & stroke) : Stroke3D(stroke) {}
	virtual ~Stroke3DTagentPlane();

	static Point shallowest_point(const Stroke2D<Point2D> & stroke);

protected:
	DirectionVector get_tangent_plane_normal_when_endpoints_unknown(const Stroke2D<Point2D> & stroke)const;
	Plane get_min_skew_plane_when_endpoints_known(const Point & a, const Point & b, const Stroke2D<Point2D> & stroke)const;
	Plane get_min_skew_plane_when_endpoints_known(const Point & a, const Stroke2D<Point2D> & stroke)const;
	//virtual std::vector<PointNormal> get_points(const Stroke2D<Point2D> & stroke, const Plane & min_skew)const;

	virtual kt84::Polyline_PointNormal Stroke3DTagentPlane::polyline_pn(const Stroke2D<Point2D> & stroke);
};

}