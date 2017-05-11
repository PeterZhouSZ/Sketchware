#pragma once
#include "../curvenetwork/Patch.hh"
#include <LearningQuadrangulationsCore/learning_quadrangulations.h>

bool convert_patch(vcg::tri::pl::MT_PatchPriorityLookup<vcg::tri::pl::PolyMesh>& lookup, int index, curvenetwork::Patch& result);
