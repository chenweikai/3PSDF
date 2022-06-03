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


void reOrderMeshIndices(const MatrixXd& allV, const MatrixXi& allF, 
                        const vector<int>& partF, MatrixXd& outV, MatrixXi& outF);
            

// compute the local 3-pole signed distance for input point in the given cell
vector<double> computeT3PoleDistForPtsInCell(
    const MatrixXd& V,                    // input entire mesh vertices
    const MatrixXi& F,                    // input entire mesh faces
    const vector<Vector3d>& pts,          // input points
    const pair<Vector3d, Vector3d>& cell,  // input cell; first - min corner, second - cell length
    const int cell_id
);

