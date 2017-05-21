#pragma once
#include "Stroke3D.h"

namespace SketcherCore {

class Stroke3DShellProjection :
	public Stroke3D
{
public:
	Stroke3DShellProjection(const Stroke2D<Point2D> & stroke) : Stroke3D(stroke) { set_polyline_pn(stroke); }
	virtual ~Stroke3DShellProjection();

	double height(int index)const { if (index >= 0 && index < _heights.size()) return _heights[index]; qDebug() << "Index of height is unacceptable!!!"; return -1; }

protected:
	void set_heights_when_endpoints_are_known(const PointNormal & c1, const PointNormal & cn, Stroke2D<Point2D> & stroke);
	void set_heights_when_endpoints_are_unknown(const Stroke2D<Point2D> & stroke);
	double convert_height(double height) const; //convert the height to the 3D space height

	boost::optional<Stroke3D::PointNormal> get_point_when_projectable(const Point2D & point, double height)const; //If projectiont fails, the method will return a uninitialized optional value
	PointNormal get_point_when_fail_to_project(const Point & a, const Point & b, const Point2D & stroke_point)const;
	//kt84::Polyline_PointNormal get_points(const Stroke2D<Point2D> & stroke);

	virtual kt84::Polyline_PointNormal polyline_pn(const Stroke2D<Point2D> & stroke);
	boost::optional<PointNormal> get_end_point_of_the_stroke(const Point2D & stroke_endpoint);

private:
	std::vector<double> _heights;
};

}