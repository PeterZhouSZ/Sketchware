#include "Contour2D.h"
#include "SketchRetopo.hh"
#include <kt84/graphics/graphics_util.hh>
#include "OpenGLDisplayWidget.h"
#include <cstring>
#include <qdebug.h>
#include "Contour2DEdge.h"
#include "Contour2DVertex.h"
#include "Stroke2D.h"
using namespace Eigen;
using namespace kt84;
using namespace kt84::graphics_util;

#define MIN_SEGMENT_LENGTH 0.0000001

namespace {
	auto& core = SketchRetopo::get_instance();
}

namespace SketcherCore {

const double  Contour2D::mid_width_divider = 25;

Contour2D::~Contour2D()
{
	delete[] _faceCondition;
}

void Contour2D::generateFaceConditions() {
	memset(_faceCondition, 0, sizeof(FaceCondition) * core.basemesh.n_faces());

	auto make_Ray = [](const Vector3d& org, const Vector3d& dir) {
		auto org_f = org.cast<float>();
		auto dir_f = dir.cast<float>();
		return embree::Ray(
			embree::Vec3f(org_f[0], org_f[1], org_f[2]),
			embree::Vec3f(dir_f[0], dir_f[1], dir_f[2]));
	};
	auto center_of_face = [&](const BaseMesh::FaceHandle &f_h) {
		auto fv_iter = basemesh.fv_iter(f_h);
		Vector3d center(0,0,0);
		int i = 0;
		for (; fv_iter.is_valid(); fv_iter++) {
			center += o2e(basemesh.point(*fv_iter));
			i++;
		}
		center /= i;
		//qDebug() << "center:(" << center.x() << ", " << center.y() << ", " << center.z();
		return center;
	};

	for (BaseMesh::FaceIter f_it = basemesh.faces_begin(); f_it != basemesh.faces_end(); f_it++) {
		auto f_h = basemesh.face_handle(f_it->idx());
		auto face_normal = o2e(basemesh.normal(f_h));

		auto ray = center_of_face(f_h) - core.camera->get_eye();

		// Set the face condtion
		if (ray.dot(face_normal) <= 0) {
			embree::Hit hit;
			core.embree_intersector->intersect(make_Ray(SketchRetopo::get_instance().camera->get_eye(), ray), hit);

			if (hit.id0 != f_h.idx()) { _faceCondition[f_h.idx()] = FaceCondition::INVISIBLE; }
			else qDebug() << hit.id0 << ": VISIBLE";
		}
		else {
			_faceCondition[f_h.idx()] = FaceCondition::BACK;
		}
		
	}
}

void Contour2D::generateContour() {

	generateFaceConditions();

	_lines.clear();
	_points.clear();

	map<int, int> pointsMap;
	auto isContour = [&](const int first, const int second) {
		return _faceCondition[first] == FaceCondition::VISIBLE && _faceCondition[second] == FaceCondition::BACK ||
				_faceCondition[second] == FaceCondition::VISIBLE && _faceCondition[first] == FaceCondition::BACK;
	};

	int edge_start_index = 0;
	for (auto edge : basemesh.edges()) {

		auto first_face = basemesh.face_handle(basemesh.halfedge_handle(edge, 0));
		auto second_face = basemesh.face_handle(basemesh.halfedge_handle(edge, 1));
		
		if (isContour(first_face.idx(), second_face.idx()))
		{
			addEdge(edge, pointsMap, edge_start_index);
			//contour.push_back(projectEdge(edge, pointsMap));
		}
	}

	generateTopoGraph();
}

void Contour2D::addEdge(BaseMesh::EdgeHandle edge_handle, map<int, int>& pointsMap, int & edge_start_index) {
	//The order of points can't be changed
	auto half_edge = basemesh.halfedge_handle(edge_handle, 0);
	auto pos_a = basemesh.from_vertex_handle(half_edge);
	auto pos_b = basemesh.to_vertex_handle(half_edge);

	auto index_of_point = [&](BaseMesh::VertexHandle vertex_handle, double weight, int edge_index) {
		int pointIndex = vertex_handle.idx();
		auto search = pointsMap.find(pointIndex);
		int idx;
		if (search == pointsMap.end()) { 
			idx = _points.size();
			pointsMap[pointIndex] = idx; 
			_points.push_back(Contour2DVertex(project(o2e(basemesh.point(vertex_handle))), weight, edge_index ,pointIndex)); 
		}
		else {
			idx = search->second;
		}
		return idx;
	};

	

	int v_a = index_of_point(pos_a, 0, edge_start_index);
	int v_b = index_of_point(pos_b, 1, -1);
	double length =_points[v_a].distance(_points[v_b]);
	int size = std::floor(length / _segmentLength);
	_points.back().setEdgeIndex(edge_start_index + size + 1);

	vector<int> result;
	result.push_back(v_a);
	if (size > 0) {
		double unit = _segmentLength / length;
		for (int i = 0; i < size; i++){
			edge_start_index++;
			double w0 = unit * (i + 1);
			double w1 = 1 - w0;
			auto pointPos = _points[v_a].point() * w0 + _points[v_b].point() * w1;
			result.push_back( _points.size());
			_points.push_back(Contour2DVertex(pointPos.eval(), w0, _points.size(),edge_start_index));
		}
	}
	result.push_back(v_b);
	edge_start_index++;

	for (int i = 0; i < result.size() - 1; i++) {
		_lines.push_back(Contour2DEdge(std::pair<int, int>(result[i], result[i + 1]), point(result[i]).distance(point(result[i+1])), edge_handle.idx()));
	}
	
	
	//return Contour2DEdge(std::pair<int, int>(index_of_point(pos_a),index_of_point(pos_b)), edge_handle.idx());
}




std::vector<Eigen::Vector3d> Contour2D::getCorrespondingContour(const Stroke2D<Point2D>& stroke) {

	if (stroke.size() == 0) return vector<Vector3d>();

	bool * visited = new bool[_points.size()];
	int first_ver = -1;
	double min_distance = _points[0].distance(stroke[0]);
	for (int i = 1; i < _points.size(); i++) {
		double distance = _points[i].distance(stroke[0]);
		if (distance < min_distance) {
			first_ver = i;
			min_distance = distance;
		}
	}


	if (min_distance > SketchRetopo::get_instance().camera->width / mid_width_divider) { return vector<Vector3d>(); }

	visited[first_ver] = true;
	int now = first_ver;
	vector<Contour2DVertex> result;
	result.push_back(_points[first_ver]);

	auto distance_between = [&](int idx, Point2D point) {
		return vertex(idx).distance(point);
	};
	
	for(int i = 1; i < stroke.size(); i++) {
		double min = -1;
		int next = -1;
		for (auto next_edge : _topoGraph[now]) {
			int next_ver = edge(next_edge).nextPoint(now);
			if (visited[next_ver])continue;
			double distance = distance_between(next_ver, stroke[i]);
			if (min == -1 || min > distance) { min = distance; next = next_ver; }
		}
		if (next == -1) return vector<Vector3d>();
		result.push_back(_points[next]);
		visited[next] = true;
		now = next;
	}

	delete []visited;

	vector<Vector3d> contour;
	for (auto vertex : result) {
		auto first_point = _lines[vertex.edge_index()].firstPoint_3DPosition();
		auto second_point = _lines[vertex.edge_index()].secondPoint_3DPosition();
		contour.push_back(first_point * vertex.weight() + second_point * (1 - vertex.weight()));
	}

	return contour;
}

void Contour2D::resample_by_length(double segment_length) {
	//vector<Contour2DVertex> vers;
	vector<Contour2DEdge> edges;
	for (auto edge : _lines) {
		auto set_endpoints_of_edge = [&](const Contour2DEdge & edge, int first_index, int second_index) {
			point(edge.firstPoint()).setEdgeIndex(first_index);
			point(edge.secondPoint()).setEdgeIndex(second_index);
		};
		int size = std::ceil(edge.length() / segment_length);
		double s_l = segment_length;
		
		if (edge.length() <= segment_length) { set_endpoints_of_edge(edge, edges.size(), edges.size()); edges.push_back(edge); continue; }
		if (edge.length() - segment_length < MIN_SEGMENT_LENGTH) {
			s_l = segment_length / 2 +MIN_SEGMENT_LENGTH;
		}

		set_endpoints_of_edge(edge, edges.size(), edges.size() + size - 1);


		auto get_weight = [&](const Contour2DEdge & edge, double w) {
			double w0 = point(edge.firstPoint()).weight();
			double w1 = point(edge.secondPoint()).weight();
			return w0 * w + w1 * (1 - w);
		};

		auto get_position = [&](const Contour2DEdge & edge, double w) {
			return (point(edge.firstPoint()).point() * w + point(edge.secondPoint()).point() * (1 - w)).eval();
		};
		double w = s_l / edge.length();
		double len = s_l;
		
		edges.push_back(Contour2DEdge(std::make_pair<int, int>(_points.size() - 2, _points.size() - 1), len, edge.idx3D()));

		_points.push_back(Contour2DVertex(get_position(edge, w), get_weight(edge, w), _points.size(), edges.size()));
		edges.push_back(Contour2DEdge(std::make_pair<int, int>(edge.firstPoint(), _points.size() - 1), s_l, edge.idx3D()));
		for (int i = 1; i < size - 1; i++) {
			len += s_l;
			w = len / edge.length();
			_points.push_back(Contour2DVertex(get_position(edge, w), get_weight(edge, w), _points.size(), edges.size()));
			edges.push_back(Contour2DEdge(std::make_pair<int, int>(_points.size() - 2, _points.size() - 1), s_l, edge.idx3D()));
		}
		edges.push_back(Contour2DEdge(std::make_pair<int, int>(_points.size() - 1, edge.secondPoint()), edge.length() - len, edge.idx3D()));
	}

	_lines.clear();
	_lines = edges;
}

}



