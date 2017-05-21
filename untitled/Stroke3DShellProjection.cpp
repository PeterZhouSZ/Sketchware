#include "Stroke3DShellProjection.h"
#include "SketchRetopo.hh"
#include "Stroke2D.h"
#include "kt84\graphics\graphics_util.hh"
#include "Stroke3DShellContour.h"
using namespace kt84;
using namespace Eigen;
using namespace kt84::graphics_util;



namespace {
	auto & core = SketchRetopo::get_instance();
}

namespace SketcherCore {


Stroke3DShellProjection::~Stroke3DShellProjection()
{
}

void Stroke3DShellProjection::set_heights_when_endpoints_are_known(const PointNormal & c1, const PointNormal & cn, Stroke2D<Point2D> & stroke) {
	
	double h1 = c1.tail<3>().norm();
	double hn = cn.tail<3>().norm();

	_heights.clear();
	_heights.push_back(h1);
	double stroke_length = stroke.length();
	double current_length = 0;
	for (auto it = stroke.edge_begin(); it != stroke.edge_end(); it++) {
		current_length += it->length();
		double w = current_length / stroke_length;
		_heights.push_back(convert_height(h1 * w + hn * (1 - w)));
	}
}

void Stroke3DShellProjection::set_heights_when_endpoints_are_unknown(const Stroke2D<Point2D> & stroke) {
	double height = DEFAULT_HEIGHT;

	auto set_height = [&](double &current_height, Point2D point) {
		auto hit = core.intersect(point.x(), point.y());
		if (hit.id0 == -1) {
			double min_height = INFINITY;
			for (auto point_it = core.contour2D->begin(); point_it != core.contour2D->end(); point_it++) {
				double d = point.distance(*point_it);
				if (d < min_height) min_height = d;
			}
			if (min_height > current_height) current_height = min_height;
		}
	};

	set_height(height, stroke.front());
	set_height(height, stroke.back());

	_heights.clear();
	height = convert_height(height);
	for (int i = 0; i < stroke.size(); i++) {
		_heights.push_back(height);
	}
}

double Stroke3DShellProjection::convert_height(double height)const {
	return height;
}

boost::optional<Stroke3D::PointNormal> Stroke3DShellProjection::get_point_when_projectable(const Point2D & point, double height)const {
	auto position = unproject(Vector3d(point.x(), point.y(), core.read_depth(point.x(), point.y()) + height)); // This could make height > 1. I don't know whether it's ok
	
	//Move forward the  camera
	auto d = core.camera->center_to_eye();
	d.normalize();
	d = d * height;
	auto eye_forward_position = core.camera->get_eye() + d;
	
	auto interest_position = core.intersect_convert(core.intersect(merge(eye_forward_position, position - eye_forward_position)));

	if (interest_position.is_initialized()) {
		auto pn = *interest_position;
		auto direction = pn.tail<3>();
		direction.normalize();
		direction *= height;
		return boost::optional<PointNormal>(merge((*interest_position).head<3>() + direction, -direction)); 
	}
	return  boost::none;
}

Stroke3D::PointNormal Stroke3DShellProjection::get_point_when_fail_to_project(const Point & a, const Point &b, const Point2D& stroke_point)const {
	Plane min_skew = min_skew_plane(a, b - a);
	auto direction = unproject(Vector3d(stroke_point.x(), stroke_point.y(), 1)) - core.camera->get_eye();
	return merge(intersect_point(merge(core.camera->get_eye(), direction), min_skew), min_skew.tail<3>().eval());
}

//Polyline_PointNormal Stroke3DShellProjection::get_points(const Stroke2D<Point2D> & stroke) {
//	
//}

Polyline_PointNormal Stroke3DShellProjection::polyline_pn(const Stroke2D<Point2D> & stroke) {
	if (stroke.size() < 2) return Polyline_PointNormal();
	auto first_end_point = get_end_point_of_the_stroke(stroke.front());
	auto last_end_point = get_end_point_of_the_stroke(stroke.back());
	vector<PointNormal> result;
	result.push_back(*first_end_point);

	vector<Point2D> fail_to_project;
	vector <PointNormal> succeed_to_project;

	int current_height_index = 1;
	auto stroke_itr = stroke.begin();
	stroke_itr++;
	auto last_success_point = first_end_point;
	while (stroke_itr != stroke.end() - 1)
	{
		//Store the projection failure points
		while (stroke_itr != stroke.end() - 1) {
			auto point = get_point_when_projectable(*stroke_itr, height(current_height_index));
			current_height_index += 1; stroke_itr++;

			if (point.is_initialized()) {
				succeed_to_project.push_back(*point);
				break;
			}
			fail_to_project.push_back(*stroke_itr);
		}

		//Project the failure points on the min_skew
		list<PointNormal> before_success;
		if (stroke_itr == stroke.end() - 1) { before_success.push_front(*last_end_point); }
		else { before_success.push_front(succeed_to_project.front()); }
		for (vector<Point2D>::reverse_iterator it = fail_to_project.rbegin(); it != fail_to_project.rend(); it++) {
			before_success.push_front(get_point_when_fail_to_project(last_success_point->head<3>().eval(), before_success.front().head<3>().eval(), *it));
		}
		before_success.pop_back();

		while (stroke_itr != stroke.end() - 1) {
			auto point = get_point_when_projectable(*stroke_itr, height(current_height_index));
			current_height_index += 1; stroke_itr++;
			if (point.is_initialized()) { succeed_to_project.push_back(*point); }
			else { break; }
		}

		last_success_point = before_success.back();
		for (auto point : before_success) {
			result.push_back(point);
		}
		for (auto pn : succeed_to_project) {
			result.push_back(pn);
		}
	}

	result.push_back(*last_end_point);

	return Polyline_PointNormal(result, false);
}

boost::optional<PointNormal> Stroke3DShellProjection::get_end_point_of_the_stroke(const Point2D & stroke_endpoint) {
	auto end_point = Stroke3D::get_end_point_of_the_stroke(stroke_endpoint);
	if (end_point.is_initialized()) {
		return end_point;
	}

	auto stroke3D = unproject(Vector3d(stroke_endpoint.x(), stroke_endpoint.y(), 1.0));
	Line line = merge(core.camera->get_eye(), (stroke3D - core.camera->get_eye()).eval());
	return Stroke3DShellContour::nearest_point_line_to_model(line);
}

}