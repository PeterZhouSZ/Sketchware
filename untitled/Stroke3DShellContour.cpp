#include "Stroke3DShellContour.h"
#include "SketchRetopo.hh"
#include "kt84\graphics\graphics_util.hh"
#include <algorithm>
#include <qdebug.h>
#include <cmath>
#include "Stroke2D.h"
using namespace kt84;
using namespace Eigen;
using namespace kt84::graphics_util;
using namespace std;

namespace {
	auto& core = SketchRetopo::get_instance();
}


namespace SketcherCore {

//Stroke3DShellContour::Stroke3DShellContour(Stroke2D<Contour2DVertex> & contour) {
//	if (contour.size() == 0)  return;
//	else {
//		auto id_to_point = [&](int id) {
//			return o2e(core.basemesh.point(core.basemesh.vertex_handle(id)));
//		};
//
//		push_back(id_to_point(contour.front().idx3D()));
//		int start = 0;
//		int end = 1;
//		while (true) {
//			while (end < contour.size() && contour[end].idx3D() == -1) end++;
//			if (end == contour.size())break;
//			auto start_point = id_to_point(contour[start].idx3D());
//			auto end_point = id_to_point(contour[end].idx3D());
//
//			for (int i = start + 1; i <= end; i++) {
//				double w0 = contour[i].weight();
//				double w1 = 1 - w0;
//				Vector3d point = start_point * w0 + end_point * w1;
//				push_back(point);
//			}
//		}
//	}
//	
//}


Stroke3DShellContour::~Stroke3DShellContour(){

}

Polyline_PointNormal Stroke3DShellContour::polyline_pn(const Stroke2D<Point2D> & stroke) {

	
	if (_isShapeMatching) {
		auto contour = core.contour2D->getCorrespondingContour(stroke);
		return Polyline_PointNormal(featureCurveContour(stroke, contour), false);
	}
	else {
		vector<PointNormal> result_points;
		auto s_1 = nearest_point_line_to_model(stroke.front()).head<3>().eval();
		auto s_n = nearest_point_line_to_model(stroke.back()).head<3>().eval();
		auto d = (s_n - s_1).eval();
		auto s1_to_model_face = -(s_1 - core.camera->get_eye()).cross(d);
		auto direction = d.cross(s1_to_model_face);
		direction.normalize();
		Plane min_skew = merge(s_1, direction);
		for (auto it = stroke.begin(); it != stroke.end(); it++) {
			//Find the corresponding point on the face of basemesh
			auto point = intersect_point(merge(core.camera->get_eye(), unproject(Vector3d(it->x(), it->y(), 1.0)) - core.camera->get_eye()), min_skew);
			auto point_on_face = core.intersect_convert(core.intersect(merge(point, s1_to_model_face)));

			result_points.push_back(merge(point, point_on_face->head<3>().eval() - point));
		}

		return Polyline_PointNormal(result_points, false);
	}
}

vector<PointNormal> Stroke3DShellContour::featureCurveContour(const Stroke2D<Point2D> & stroke, const vector<Vector3d>& contour)const {
	
	vector<PointNormal> result;
	for (int i = 0; i < stroke.size(); i++){
		auto ray_direction = unproject(Vector3d(stroke[i].x(), stroke[i].y(), 1)) - core.camera->get_eye();
		auto result_on_face = contour[i].head<3>();
		auto point3d = nearest_point_line_to_point(merge(core.camera->get_eye(), ray_direction), result_on_face);
		result.push_back(merge(point3d, result_on_face - point3d));
	}

	return result;
}



Stroke3D::PointNormal Stroke3DShellContour::nearest_point_line_to_model(const Point2D & stroke_point) {
	return nearest_point_line_to_model(merge(core.camera->get_eye(), unproject(Vector3d(stroke_point.x(), stroke_point.y(), 1.0)) - core.camera->get_eye()));
}

Stroke3D::PointNormal Stroke3DShellContour::nearest_point_line_to_model(const Line & line) {

	//The line interset with the model
	auto pn = core.intersect_convert(core.intersect(line));
	if (pn.is_initialized()) {
		auto direction = pn->tail<3>().eval();
		direction.normalize();
		direction *= DEFAULT_HEIGHT;
		return *pn + merge(direction, Vector3d::Zero().eval());
	}

	//The line uninterset with the model
	Point nearest_point;
	double min_distance = INFINITY;
	for (BaseMesh::FaceIter f_it = core.basemesh.faces_begin(); f_it != core.basemesh.faces_end(); ++f_it) {
		vector<Point> temp;
		for (BaseMesh::FaceVertexIter fv_it = core.basemesh.fv_iter(*f_it); fv_it.is_valid(); fv_it++) {
			temp.push_back(o2e(core.basemesh.point(*fv_it)));
		}
		auto point_normal = nearest_point_line_to_face(line, temp);

		double distance = point_normal.tail<3>().norm();
		if (distance < min_distance) {
			nearest_point = point_normal.head<3>();
			min_distance = distance;
		}
	}

	auto result = nearest_point_line_to_point(line, nearest_point);

	return merge(result, nearest_point - result);
}

}