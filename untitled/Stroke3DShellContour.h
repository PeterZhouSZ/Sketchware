#pragma once
#include "Stroke3D.h"
#include "Contour2D.h"

namespace SketcherCore {

class Stroke3DShellContour :
	public Stroke3D
{
public:

	Stroke3DShellContour(const Stroke2D<Point2D>& stroke, bool isShapeMatching = false) : Stroke3D(stroke), _isShapeMatching(isShapeMatching) {}
	virtual ~Stroke3DShellContour();

	static Point nearest_point_line_to_model(const Point2D & stroke_point);
	static Point nearest_point_line_to_model(const Line & line);

	

protected:
	std::vector<Stroke3D::PointNormal> featureCurveContour(const Stroke2D<Point2D> & stroke, const std::vector<Point>& contour)const;
	

	virtual kt84::Polyline_PointNormal polyline_pn(const Stroke2D<Point2D> & stroke);
	

private:
	bool _isShapeMatching;

};

}