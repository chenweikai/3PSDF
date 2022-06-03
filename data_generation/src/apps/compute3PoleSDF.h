#include <iostream>
#include <string>

#include <igl/readOBJ.h>
#include <igl/copyleft/marching_cubes.h>
#include <igl/signed_distance.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/parula.h>
#include <igl/point_mesh_squared_distance.h>
#include "igl/AABB.h"
#include "igl/WindingNumberAABB.h"
#include <Eigen/Core>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <chrono> 
#include "distComputeUtility.h"
#include "utilities.h"
// #include "pointTriangleDist.h"
#include <math.h> 
#include <random>
#include <chrono>
#include <queue>
#include <set>
#include <stack>
#include <sstream>
#include <deque>

#ifndef IGL_NO_EIGEN
    #include <Eigen/Core>
#endif
using namespace std;
using namespace igl;
using namespace Eigen;
using namespace std::chrono; 


// generate the 3-pole distance field proposed by Weikai
// Here the 3-pole field definition is the region within the specified angles are set to NULL
void generate3PoleSDF(string meshFileName, string outReconMeshName, int resolution,
                        int batchSize);

void compute3PoleDistForInputPoints(const MatrixXd& V, const MatrixXi& F, const MatrixXd& inputPoints,
                                    VectorXd& outputDist, double minGridWidth,
                                    int maxBufferSize=2);


void reOrderMeshIndices(const MatrixXd& allV, const MatrixXi& allF, const vector<int>& partF, MatrixXd& outV, MatrixXi& outF);

void findBoundaryEdges(map<pair<int, int>, vector<int>> edge2Face, vector<pair<int,int>>& boundEdges);
void findBoundaryPoints(map<pair<int, int>, vector<int>> edge2Face, vector<int>& outBoundPoints);