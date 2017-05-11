#include "Stroke3DNormalPlane.h"
#include "kt84\graphics\graphics_util.hh"
#include "Stroke2D.h"
using namespace Eigen;
using namespace kt84::graphics_util;
using namespace std;

namespace {
	auto & core = SketchRetopo::get_instance();
}

namespace SketcherCore {


Stroke3DNormalPlane::~Stroke3DNormalPlane()
{
}

Stroke3D::Plane Stroke3DNormalPlane::get_min_skew_plane(const Stroke2D<Point2D> & stroke)const {
	auto endPoint_1 = core.intersect_convert(core.intersect(stroke.front().x(), stroke.front().y()));
	auto endPoint_n = core.intersect_convert(core.intersect(stroke.back().x(), stroke.back().y()));

	if (endPoint_1.is_initialized() && endPoint_n.is_initialized()) {
		return min_skew_plane(((endPoint_1->head<3>() + endPoint_n->head<3>()) / 2).eval(), ((endPoint_1->tail<3>() + endPoint_n->tail<3>()) / 2).eval());
	}
	else if (endPoint_1.is_initialized()) {
		return min_skew_plane(endPoint_1->head<3>().eval(), endPoint_1->tail<3>().eval());
	}
	else {
		return min_skew_plane(endPoint_n->head<3>().eval(), endPoint_n->tail<3>().eval());
	}
}

kt84::Polyline_PointNormal  Stroke3DNormalPlane::polyline_pn(const Stroke2D<Point2D> & stroke) {
	auto & endPoints = get_end_points_of_the_stroke(stroke);

	vector<PointNormal> result;
	auto min_skew = get_min_skew_plane(stroke);

	auto ray = [&](const Point2D & point) {
		return merge(core.camera->get_eye(), unproject(Vector3d(point.x(), point.y(), 1)) - core.camera->get_eye());
	};

	
	if (endPoints.size() == 1) { //与表面垂直的线，暂时只考虑这种情况
		auto d = min_skew.tail<3>().eval();
		for (auto point : stroke) {
			result.push_back(merge(intersect_point(ray(point), min_skew), d));
		}
	}
	else { //两端点都在表面上的弯曲的线
		result.push_back(endPoints.front());
		auto last_point = endPoints.front().head<3>().eval();
		for (auto it = stroke.begin() + 1; it != stroke.end(); it++){
			auto point = intersect_point(ray(*it), min_skew);
			result.push_back(merge(point, last_point - point));
			last_point = point;
		}
	}

	return kt84::Polyline_PointNormal(result, false);
}

std::vector<Stroke3D::PointNormal> Stroke3DNormalPlane::get_end_points_of_the_stroke(const Stroke2D<Point2D> & stroke) {
	auto get_result = [&](const Point2D point) {
		auto first = get_end_point_of_the_stroke(point);
		if (first.is_initialized()) return std::vector<PointNormal>(1, *first);
		first = core.intersect_convert(core.intersect(point.x(), point.y()));
		if (first.is_initialized()) return std::vector<PointNormal>(1, *first);
		return std::vector<PointNormal>();
	};
	auto & v = get_result(stroke.front());
	if (v.size() > 0) return v;
	return get_result(stroke.back());
}

}