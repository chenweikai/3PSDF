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
void buildPQP(PQP_Model* ptr, string objName);


void buildPQP(PQP_Model* ptr, const MatrixXd& V, const MatrixXi& F);

// 1) load mesh 2) compute PQP model from mesh 
// void buildPQPModel(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

// return the unsigned distance from a 3D point to the mesh (PQP model required to be built first)
// args:
// queryPnt - input query point
// nearestPnt - return the nearest point on mesh regarding to the input point
// closestFaceIdx - return the id of the closest triangle
double PQPABSDist(PQP_Model* m_pqp_model, Vector3d queryPnt, Vector3d& nearestPnt, int& closestFaceIdx);
