#pragma once

#include <string>
#include <vector>

#ifndef IGL_NO_EIGEN
    #include <Eigen/Core>
#endif

#include "glm/glm.hpp"
#include "utilities.h"

using namespace std;
using namespace Eigen;
using namespace glm;

// reorder the selected faces of the input mesh so that the selected part can form a regular mesh
//
// @param all_verts, all_faces: input mesh vertices and faces
// @param part_faces: selected face ids
// @param out_verts, out_faces: output reordered vertices and faces of the selected mesh part
void reorder_mesh_indices(const MatrixXd& all_verts, const MatrixXi& all_faces,
                          const vector<int>& part_faces, MatrixXd& out_verts, MatrixXi& out_faces);


// compute the 3-pole signed distance for input points in the given cell
vector<double> compute_3psdf_per_cell(
    const MatrixXd& V,                    // input entire mesh vertices
    const MatrixXi& F,                    // input entire mesh faces
    const vector<Vector3d>& pts,          // input points
    const pair<Vector3d, Vector3d>& cell,  // input cell; first - min corner, second - cell length
    const int cell_id);

