#pragma once
#include "Permutation.hh"

namespace patchgen {
    struct PatchParam {
        int pattern_id = -1;
        Eigen::VectorXi l;              // original input without permutation
        Permutation permutation;
        Eigen::VectorXi p;              // padding
        Eigen::VectorXi q;              // padding inserted inside the pattern (p[i] and q[i] have the same effect on the boundary. i.e. same basis vector)
        int x = -1;                          // number of edgeflows
        int y = -1;
        int z = -1;                          // parameters describing auxiliary edgeflow counts (only relevant for 6-sided) TODO: better explanation...
        int w = -1;
        bool is_valid() const { return pattern_id != -1; }
        int get_num_sides() const { return l.size(); }
        Eigen::VectorXi get_l_permuted() const { return permutation(l); }
    };
}
