#include "Stroke3D.h"
#include "SketchRetopo.hh"
#include "Stroke2D.h"
#include "kt84\graphics\graphics_util.hh"
#include "curvenetwork\Core.hh"
using namespace kt84;
using namespace kt84::graphics_util;

using namespace Eigen;

namespace {
	auto& core = SketchRetopo::get_instance();
}

namespace SketcherCore {

Stroke3D::Plane Stroke3D::min_skew_plane(const Point & point, const DirectionVector & direction) {
	auto ray = (point - core.camera->get_eye()).eval();
	auto normal = direction.cross(ray.cross(direction));
	normal.normalize();
	return merge(point, normal);
}

Stroke3D::Point Stroke3D::intersect_point(const Line & line, const Plane & plane) {
	auto linePoint_to_planePoint = (line.head<3>() - plane.head<3>()).eval();
	auto length = -linePoint_to_planePoint.dot(plane.tail<3>());
	auto weight = line.tail<3>().dot(plane.tail<3>()) / length;
	return line.head<3>().eval() + linePoint_to_planePoint * weight;
}

Stroke3D::Point Stroke3D::nearest_point_line_to_point(const Line & line, const Point & point) {

	auto line_direction = line.tail<3>().eval();
	line_direction.normalize();

	auto point_to_linePoint = (line.head<3>() - point).eval();
	double length = line_direction.dot(point_to_linePoint);
	auto direction_vector = (point_to_linePoint - line_direction * length).eval();

	return point + direction_vector;
}

std::vector<Stroke3D::PointNormal> Stroke3D::get_points(const Stroke2D<Point2D> & stroke, const Plane & min_skew)const {
	std::vector<Stroke3D::PointNormal> result;
	for (auto it = stroke.begin(); it != stroke.end(); it++) {
		auto point = kt84::graphics_util::unproject(Eigen::Vector3d(it->x(), it->y(), 1));
		result.push_back(merge(intersect_point(merge(core.camera->get_eye(), (point - core.camera->get_eye())), min_skew), Vector3d::Zero().eval()));
	}

	return result;
}

Stroke3D::PointNormal Stroke3D::nearest_point_line_to_face_when_parellel(const Line & line, Face face, const DirectionVector & face_normal) {
	auto perpendicular_face_normal = line.tail<3>().eval().cross(face_normal);
	perpendicular_face_normal.normalize();


	auto is_homosign = [=](double a, double b) { return a > 0 && b > 0 || a < 0 && b < 0; };
	vector<double> cos_result;
	auto line_point = line.head<3>().eval();
	for (auto point : face) {
		//cos_result.push_back((point - line_point).dot(perpendicular_face_normal));
	}

	auto get_point_when_interseted = [&](int single, int other1, int other2){
		Plane perpendicular = merge(line_point, perpendicular_face_normal);
		auto point1 = intersect_point(merge(face[single], (face[other1] - face[single]).eval()), perpendicular);
		auto point2 = intersect_point(merge(face[single], (face[other2] - face[single]).eval()), perpendicular);

		auto result_on_face = ((point1 + point2) / 2).eval();
		auto result_on_line = nearest_point_line_to_point(line, result_on_face);
		return merge(result_on_line, (result_on_face - result_on_line).eval());
	};

	auto get_point_when_not_interseted = [&](){
		struct PointForCompare {
			double distance;
			Point nearest_point;
			Point orignal_on_face;

			PointForCompare(double d, const Point & n_p, const Point & orignal) : distance(d), nearest_point(n_p), orignal_on_face(orignal) {}
			bool operator<(const PointForCompare & a)const { return distance < a.distance; }
			double operator-(const PointForCompare & a)const { return  distance - a.distance; }
		};

		vector<PointForCompare> point_array;
		for (auto point : face) {
			auto nearest_point = nearest_point_line_to_point(line, point);
			point_array.push_back(PointForCompare((nearest_point - point).eval().norm(), nearest_point, point));
		}

		std::sort(point_array.begin(), point_array.end());

		if (point_array[1] - point_array[0] < 0.0001) { auto result = ((point_array[0].nearest_point + point_array[1].nearest_point) / 2).eval();  return merge(result, ((point_array[0].orignal_on_face + point_array[1].orignal_on_face) / 2 - result).eval()); }
		else { return merge(point_array[0].nearest_point, point_array[0].orignal_on_face - point_array[0].nearest_point); }
	};

	//Check whether the points of face is in the same side of the perpendicular face
	if (is_homosign(cos_result[0], cos_result[1])) {
		if (is_homosign(cos_result[0], cos_result[2])){
			return get_point_when_not_interseted();
		}
		else {
			return get_point_when_interseted(2, 0, 1);
		}
	}
	else {
		if (is_homosign(cos_result[0], cos_result[2])){
			return get_point_when_interseted(1, 0, 2);
		}
		else {
			return get_point_when_not_interseted();
		}
	}
}

Stroke3D::PointNormal Stroke3D::nearest_point_line_to_face_when_unparellel(const Line & line, const Face & face, const DirectionVector & face_normal) {
	auto  get_nearest_point_normal = [&](SegmentLine segment, Line line) {
		auto result_on_segment = nearest_point_segment_line_to_line(segment, line);
		auto result_on_line = nearest_point_line_to_point(line, result_on_segment);

		return merge(result_on_line, result_on_segment - result_on_line);
	};

	struct PointForCompare {
		double distance;
		PointNormal point_normal;

		PointForCompare(double d, const PointNormal & pn) : distance(d), point_normal(pn) {}
		bool operator<(const PointForCompare & a)const { return distance < a.distance; }
	};

	vector<PointForCompare> result_array;
	for (int i = 0; i < face.size(); i++) {
		auto pn = get_nearest_point_normal(merge(face[i], face[(i + 1 == face.size()) ? 0 : i + 1]), line);
		result_array.push_back(PointForCompare(pn.tail<3>().norm(), pn));
	}

	std::sort(result_array.begin(), result_array.end());

	return result_array.front().point_normal;
}

Stroke3D::PointNormal Stroke3D::nearest_point_line_to_face(const Line & line, const Face & face) {
	if (face.size() < 3) { qDebug() << "Stroke3DShellContour face is wrong"; return PointNormal(); }

	auto face_normal = (face[1] - face[0]).cross(face[2] - face[0]);
	if (face_normal.dot(line.tail<3>()) == 0) {
		return nearest_point_line_to_face_when_parellel(line, face, face_normal);
	}
	else {
		return nearest_point_line_to_face_when_unparellel(line, face, face_normal);
	}
}

Stroke3D::Point Stroke3D::nearest_point_segment_line_to_line(const SegmentLine & segment_line, const Line & line) {
	auto nearest_point_line_to_line_when_unparallel = [&](Line des, Line src) {
		auto normal = des.tail<3>().cross(src.tail<3>());
		normal.normalize();
		return intersect_point(des, merge(src.head<3>(), normal));
	};

	auto is_between_two_point = [&](Point des, Point p1, Point p2){
		return (p1 - des).dot(p2 - des) <= 0;
	};

	auto endpoint1 = segment_line.head<3>();
	auto endpoint2 = segment_line.tail<3>();
	auto point = nearest_point_line_to_line_when_unparallel(merge(endpoint1, endpoint1 - endpoint2), line);
	if (is_between_two_point(point, endpoint1, endpoint2)) {
		return nearest_point_line_to_point(line, point);
	}
	else {
		if (std::abs(endpoint1.x() - point.x()) < std::abs(endpoint2.x() - point.x())) {
			return nearest_point_line_to_point(line, endpoint1);
		}
		else {
			nearest_point_line_to_point(line, endpoint2);
		}
	}
}

boost::optional<Stroke3D::PointNormal> Stroke3D::get_end_point_of_the_stroke(const Point2D & stroke_endpoint) {
	auto point = core.intersect_convert(core.intersect(stroke_endpoint.x(), stroke_endpoint.y()));
	if (!point.is_initialized()) {
		point = merge(unproject(Vector3d(stroke_endpoint.x(), stroke_endpoint.y(), 1)), Vector3d::Zero().eval());
	}
	auto segment_line = merge(point->head<3>().eval(), core.camera->get_eye());
	curvenetwork::Vertex * vertex_min = nullptr;
	double dist = core.curvenetwork.find_snap_nearest_point(vertex_min, segment_line);
	if (dist < core.configTemp.snapSize()) {
		return vertex_min->pn;
	}
	else {
		return boost::none;
	}
	
}

Stroke3D::Point Stroke3D::nearest_point_segment_line_to_point(const SegmentLine & segment_line, const Point & point) {
	auto line_point1 = segment_line.head<3>().eval();
	auto line_point2 = segment_line.tail<3>().eval();
	auto p = nearest_point_line_to_point(merge(line_point1, line_point2 - line_point1), point);

	auto smaller = line_point1.x() < line_point2.x() ? line_point1 : line_point2;
	auto bigger = line_point1.x() < line_point2.x() ? line_point2 : line_point1;
	if (p.x() >= smaller.x() && p.x() <= bigger.x()) {
		return p;
	}
	else if (p.x() < smaller.x()) {
		return smaller;
	}
	else {
		return bigger;
	}
}

}