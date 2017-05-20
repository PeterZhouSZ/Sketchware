#pragma once
#include "Stroke3D.h"

namespace SketcherCore {

class Stroke3DNormalPlane :
	public Stroke3D
{
public:
	Stroke3DNormalPlane(const Stroke2D<Point2D> & stroke) : Stroke3D(stroke) { set_polyline_pn(stroke); }
	virtual ~Stroke3DNormalPlane();

protected:
	Plane get_min_skew_plane(const Stroke2D<Point2D> & stroke)const;
	//virtual std::vector<PointNormal> get_points(const Stroke2D<Point2D> & stroke, const Plane & min_skew)const
	kt84::Polyline_PointNormal  Stroke3DNormalPlane::polyline_pn(const Stroke2D<Point2D> & stroke);

	std::vector<PointNormal> get_end_points_of_the_stroke(const Stroke2D<Point2D> & stroke);
};

}