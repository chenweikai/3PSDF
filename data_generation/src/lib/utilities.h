#pragma once

#include <experimental/filesystem>
#include <fstream>
#include <map>
#include <string>
#include <utility>


#include <Eigen/Core>
#include <igl/read_triangle_mesh.h>


using namespace std;
using namespace Eigen;

namespace fs = std::experimental::filesystem;

static bool comparePoint(const RowVector3d& p1, const RowVector3d& p2)
{
  if (p1[0] > p2[0])
    return true;
  else if (p1[0] < p2[0])
    return false;
  else if (p1[1] > p2[1])
    return true;
  else if (p1[1] < p2[1])
    return false;
  else if (p1[2] > p2[2])
    return true;
  else if (p1[2] < p2[2])
    return false;
  
  return false;
}

static bool equalPoint(const RowVector3d& p1, const RowVector3d& p2)
{
  double th = 1e-6;
	if ( fabs(p1[0] - p2[0]) < th
      && fabs(p1[1] - p2[1]) < th 
      && fabs(p1[2] - p2[2]) < th)
		return true;
	return false;
}


// remove those faces that have idential vertex ids
void removeIdenticalVertices(const MatrixXi& F, MatrixXi& outF);


// customized mesh save function -- automatically remove NAN vertices
void save_obj_mesh(string filename,
                   const MatrixXd& verts,
                   const MatrixXi& faces);

void assembleMeshParts(const vector<pair<MatrixXd, MatrixXi>>& meshParts, MatrixXd& outV, MatrixXi& outF);

void computeBBox(string bboxObjName, VectorXd& bboxmin, VectorXd& bboxmax);

// normalize mesh to [-0.5, 0.5] and centered at [0, 0, 0]
void normalizeMesh(string input_obj_name, string output_obj_name);