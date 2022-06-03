/*
    Implemented 1) Octree Generation
      2) Octree based importance sampling
      3) Octree based inside/outside determination
    Weikai Chen
*/
#ifndef OctreeUtilities_H_
#define OctreeUtilities_H_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "Octree.h"
// #include "BVH.h"
#include <map>
#include <Eigen/Core>
#include <cstdlib>
#include <igl/readOBJ.h>
#include <set>
#include <random>
using namespace std;
using namespace Eigen;
/*************************************************************************** 
  OBJ Loading 
 ***************************************************************************/
 
class Model_OBJ
{
  public: 
  	struct Edge_Info
  	{
  		int face_x, face_y;
  		map<int, int> loop_index;
  	};
  vector<set<int> > v_faces;

	Model_OBJ();			
  int Load(string filename);	// Loads the model

  // get the octree leaf nodes 
  // pair<Vector3d, Vector3d> : pair<min_cornor of the cell, the 3D length of the cell>
  vector<pair<Vector3d, Vector3d>> getTreeCells(int resolution, const Vector3d& bboxmin, const Vector3d& bboxmax);
  vector<pair<Vector3d, Vector3d>> getTreeCells(int resolution);
  vector<pair<Vector3d, Vector3d>> getTreeCells(int resolution, MatrixXd& cellCornerPts);
  vector<pair<Vector3d, Vector3d>> getOccupiedCells(int resolution, MatrixXd& cellCornerPts);
  vector<pair<Vector3d, Vector3d>> getEmptyCells(int resolution, MatrixXd& cellCornerPts);
  // return the cells that contain the query point
  vector<pair<Vector3d, Vector3d>> getBoundingCellsForQueryPnt(const RowVector3d& p);

  // perform importance sampling on the obtained octree
  // [param] - cellRes: number of Octree child cells we want to create
  // [param] - targetSampleNum: target number of sampling points
  // return: a vector of 3D sampling points based on importance sampling around the surface
  vector<Vector3d> generateImpSamples(int cellRes, int targetSampleNum);
  set<Octree*> inOrOutTest(const MatrixXd& pts, vector<double>& outputSign, const vector<Vector3d>& legPoints);
  set<Octree*> BuildTreeGridPts(const MatrixXd& pts, vector<double>& outputSign);
  void ReconOnGridPts(const MatrixXd& pts);
  MatrixXd getCellCornerPts(const set<Octree*>& cells);
  void displayCells(const vector<pair<Vector3d, Vector3d>>& cells, const MatrixXd& gridPts);

  void setBBox(const glm::dvec3& bmin, const glm::dvec3& bmax) {
    min_corner = bmin;
    max_corner = bmax;
    flag_use_external_bbox = true;
  }
 	
  void Calc_Bounding_Box();

  void Process_Manifold(int resolution);

  void Build_Tree(int resolution, const Vector3d& bboxMin, const Vector3d& bboxMax);
  void Build_Tree(int resolution);
  set<Octree*> GetEmptyCells(int resolution);
  // void Build_BVH();
  void Construct_Manifold();
  void Project_Manifold();
  // bool Project(glm::dvec3& o, glm::dvec3& d);
  void Save(const char* filename, bool color);
  void SaveOBJ(string filename);
  glm::dvec3 Closest_Point( const glm::dvec3 *triangle, const glm::dvec3 &sourcePosition );
  glm::dvec3 Find_Closest(int i);
  int is_manifold();
  bool Split_Grid(map<Grid_Index,int>& vcolor, vector<glm::dvec3>& nvertices, vector<glm::ivec4>& nface_indices, vector<set<int> >& v_faces, vector<glm::ivec3>& triangles);
  double clamp(double d1, double l, double r)
  {
    if (d1 < l)
      return l;
    if (d1 > r)
      return l;
    return d1;
  }
  vector<Grid_Index > v_info;
  
	vector<glm::dvec3> vertices, vertices_buf;
  vector<glm::dvec3> colors;
	vector<glm::ivec3> face_indices, face_indices_buf;
  vector<glm::dvec3> face_normals;
	
  bool flag_use_external_bbox = false;
	glm::dvec3 min_corner, max_corner;
  Octree* tree;
  // BVH* bvh;
  // vector<BV*> bvs;
  string fn;
  // vector field
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

};

#endif
