#include "Stroke3DTagentPlane.h"
#include "kt84\graphics\graphics_util.hh"
#include "Stroke2D.h"
#include "kt84\graphics\graphics_util.hh"

using namespace kt84;
using namespace kt84::graphics_util;
using namespace Eigen;

namespace {
	auto & core = SketchRetopo::get_instance();
}

namespace SketcherCore {


Stroke3DTagentPlane::~Stroke3DTagentPlane()
{
}

Stroke3D::DirectionVector Stroke3DTagentPlane::get_tangent_plane_normal_when_endpoints_unknown(const Stroke2D<Point2D> & stroke)const {
	
	Stroke3D::DirectionVector normal(0.0, 0.0, 0.0);
	int count = 0;
	for (auto it = stroke.begin(); it != stroke.end(); it++) {
		auto pn = core.intersect_convert(core.intersect(it->x(), it->y()));
		if (pn.is_initialized()) {
			auto n = pn->tail<3>();
			n.normalize();
			normal += n;
			count += 1;
		}
	}

	return normal / count;
}

Stroke3D::Plane Stroke3DTagentPlane::get_min_skew_plane_when_endpoints_known(const Point & a, const Point & b, const Stroke2D<Point2D> & stroke)const {
	auto pn = core.intersect_convert(core.intersect_directly(merge(a, b-a)));
	if (pn.is_initialized()) { return get_min_skew_plane_when_endpoints_known(a, stroke); }
	return min_skew_plane(a, b - a);
}

Stroke3D::Plane Stroke3DTagentPlane::get_min_skew_plane_when_endpoints_known(const Point & a, const Stroke2D<Point2D> & stroke)const {
	return min_skew_plane(a, a - shallowest_point(stroke));
}

Stroke3D::Point Stroke3DTagentPlane::shallowest_point(const Stroke2D<Point2D> & stroke) {
	Point2D shallowest_point;
	double depth = 1;
	for (auto it = stroke.begin(); it != stroke.end(); it++) {
		double d = core.read_depth(it->x(), it->y());
		if (d < depth) { depth = d; shallowest_point = *it; }
	}

	auto pn = core.intersect_convert(core.intersect(shallowest_point.x(), shallowest_point.y()));
	return pn->head<3>().eval();

}

Polyline_PointNormal Stroke3DTagentPlane::polyline_pn(const Stroke2D<Point2D> & stroke) {
	auto first_end_point = get_end_point_of_the_stroke(stroke.front());
	auto last_end_point = get_end_point_of_the_stroke(stroke.back());

	Plane plane;
	//DirectionVector normal;
	if (first_end_point.is_initialized() && last_end_point.is_initialized()) {
		auto a = first_end_point->head<3>().eval();
		auto b = last_end_point->head<3>().eval();
		plane = get_min_skew_plane_when_endpoints_known(a, b, stroke);
	}
	else if (first_end_point.is_initialized()) {
		auto a = first_end_point->head<3>().eval();
		plane = get_min_skew_plane_when_endpoints_known(a, stroke);
	}
	else if (last_end_point.is_initialized()) {
		auto a = last_end_point->head<3>().eval();
		plane = get_min_skew_plane_when_endpoints_known(a, stroke);
	}
	else {
		auto point = shallowest_point(stroke);
		auto normal = get_tangent_plane_normal_when_endpoints_unknown(stroke);
		plane = merge(point, normal);
	}

	
	vector<PointNormal> result;
	auto ray = [&](const Point2D & point) {
		return merge(core.camera->get_eye(), unproject(Vector3d(point.x(), point.y(), 1)) - core.camera->get_eye());
	};

	auto normal = plane.tail<3>().eval();
	for (auto it = stroke.begin(); it != stroke.end(); it++) {
		auto point = intersect_point(ray(*it), plane);
		result.push_back(merge(point, normal));
	}

	return Polyline_PointNormal(result, false);

}

}