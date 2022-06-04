#pragma once

#include <igl/readOBJ.h>
#include <igl/read_triangle_mesh.h>
#include "PQP.h"

#ifndef IGL_NO_EIGEN
    #include <Eigen/Core>
#endif

using namespace std;
using namespace Eigen;


// initialize PQP query model
void build_pqp(PQP_Model* ptr, string objName);

// initialize PQP query model
void build_pqp(PQP_Model* ptr, const MatrixXd& V, const MatrixXi& F);

// Compute the unsigned distance from a 3D point to the mesh (PQP model required to be built first)
// 
// queryPnt - input query point
// nearestPnt - return the nearest point on mesh regarding to the input point
// closestFaceIdx - return the id of the closest triangle
double pqp_abs_dist(PQP_Model* m_pqp_model, Vector3d queryPnt, Vector3d& nearestPnt, int& closestFaceIdx);
