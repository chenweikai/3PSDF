// Utilitiy functions for computing distances, e.g. point to mesh distance and 3PSDF.

#ifndef _3PSDF_SRC_LIB_DIST_COMPUTE_H_
#define _3PSDF_SRC_LIB_DIST_COMPUTE_H_

#include <string>
#include <utility>
#include <vector>

#ifndef IGL_NO_EIGEN
    #include <Eigen/Core>
#endif
#include "PQP.h"

namespace l3psdf {

// Initialize PQP model for computing unsigned distance from point to mesh
//
// @param pqp_model: the pqp model pointer to be initialized
// @param verts: the mesh vertices
// @param faces: the mesh faces
void BuildPqp(PQP_Model* pqp_model, const Eigen::MatrixXd& verts, const Eigen::MatrixXi& faces);

// Compute the absolute L2 distance from a 3D point to the mesh (PQP model required to be built first)
//
// @param pqp_model: the pre-built pqp query model for computing point to mesh distance
// @param query_pnt: the input query point
// @param nearest_pnt: return the nearest point on mesh regarding to the input point
// @param closest_face_id: return the id of the closest triangle face
//
// @return: the absolution L2 distance from the query point to the mesh
double PqpAbsDist(PQP_Model* pqp_model, Eigen::Vector3d query_pnt, Eigen::Vector3d& nearest_pnt, int& closest_face_id);

// Compute the 3-pole signed distance for input points in the given cell
//
// @param verts: the verts of the input mesh
// @param faces: the faces of the input mesh
// @param points: the input query points
// @param cell: the input cell: first - min corner of cell, second - cell size
// @param cell_id: the id of the input cell
//
// @return: the computed 3psdf distance for query points
std::vector<double> Compute3psdfPerCell(
    const Eigen::MatrixXd& verts,
    const Eigen::MatrixXi& faces,
    const std::vector<Eigen::Vector3d>& points,
    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& cell,  // input cell; 
    const int cell_id);

}  // namespace l3psdf

#endif // _3PSDF_SRC_LIB_DIST_COMPUTE_H_
