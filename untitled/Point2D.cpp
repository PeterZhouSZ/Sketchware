#include "Point2D.h"
#include "SketchRetopo.hh"

using namespace Eigen;
using namespace kt84;
using namespace kt84::graphics_util;

namespace {
	auto& core = SketchRetopo::get_instance();
}

namespace SketcherCore {

Point2D::~Point2D()
{
}

bool Point2D::is_on_surface() {

	auto hit = core.intersect(_point.x(), _point.y());
	return hit.id0 != -1;
}

}