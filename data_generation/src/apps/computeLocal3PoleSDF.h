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

void addItem(
  vector<vector<double>>& gridDists, 
  const RowVector3i& grid_size,
  const vector<double>& distances,
  int i,
  int j,
  int k,
  int& cnt);

void addItem(
  vector<vector<double>>& gridDists, 
  const RowVector3i& grid_size,
  const vector<double>& distances,
  int i,
  int j,
  int k,
  int& cnt,
  double threshold);

// generate regular sampling points and its localized 3PSD to the input mesh
void generateRegularSamplesinBBox(
  const MatrixXd& V,              // input mesh vertices
  const MatrixXi& F,              // input mesh faces
  const RowVector3d& bmin,           // input min corner of input bounding box
  const double& min_grid_width,     // input width of minimum grid cell
  const RowVector3i& grid_size,    // number of samples in each dimension
  MatrixXd& samplePoints,         // output regular sampling points
  VectorXd& distPerPoint          // output local 3PSD for each point
);

// generate regular sampling points and its localized 3PSD to the input mesh
vector<vector<double>> generateRegularSamplesinBBox(
  const MatrixXd& V,              // input mesh vertices
  const MatrixXi& F,              // input mesh faces
  const MatrixXd& bmin,           // input min corner of input bounding box
  const double& min_grid_width,   // input width of minimum grid cell
  const RowVector3i& grid_size,   // number of samples in each dimension
  double threshold,                // threshold for determining if the grid value is computed
  int idx_z
);

// generate the thin 3-Pole distance field proposed by Weikai
// Here the 3-Pole field definition is that only the region close to surface will be set as valid inside/outside label
// all the other regions will be set to NULL
void generateThin3PoleSDF(string meshFileName,
                          string outReconMeshName,
                          int resolution,
                          int batchSize);


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

