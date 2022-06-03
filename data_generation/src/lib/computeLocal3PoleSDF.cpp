#include "computeLocal3PoleSDF.h"

#include <math.h> 

#include <random>
#include <queue>
#include <set>
#include <stack>
#include <deque>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <chrono> 
#include <iostream>

#include "utilities.h"
#include "distComputeUtility.h"
#include "glm/gtc/matrix_transform.hpp"
#include "manifold/Intersection.h"
#include <igl/readOBJ.h>
#include <igl/writePLY.h>
#include <igl/copyleft/marching_cubes.h>
#include <igl/signed_distance.h>
#include <igl/read_triangle_mesh.h>
// #include <igl/opengl/glfw/Viewer.h>
// #include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/parula.h>
#include <igl/point_mesh_squared_distance.h>
#include "igl/AABB.h"
#include "igl/WindingNumberAABB.h"

#define PI 3.14159265

// for debug only!
#define FOR_DEBUG 1

using namespace igl;
using namespace std::chrono; 

// given point p and triangle (a,b,c), return p's barycentric coordinate
void Barycentric(const Vector3d& p, const Vector3d& a, const Vector3d& b, const Vector3d& c, float &u, float &v, float &w)
{
    Vector3d v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = v0.dot(v0); 
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

string vec2String(const RowVector3d& v){
	ostringstream strs;
	strs << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
	return strs.str();
}

typedef pair<int,int> CEdge;

// Compute sign of the query point via angle weighted pseudonormals
double computeSignViaPseudonormal(
        const int& vertID,
        const MatrixXd& all_verts,
        const MatrixXi& all_faces,
        const Vector3d& dir, 
        const MatrixXd& face_normals,
        map<int, vector<int>>& vert2Face) {
  auto iter = vert2Face.find(vertID);
  if (iter == vert2Face.end()) {
      std::cout << "The vertex is not found in function verifyAngleViaVertConnection!" << std::endl;
  }

  vector<int> faces = vert2Face[vertID];
  map<int, int> fids; // use map to avoid duplicated face ids
  for(auto f : faces)
      fids[f] = 1;
  RowVector3d sumNormal(0.0, 0.0, 0.0);
  for(auto i=fids.begin(); i!=fids.end(); i++) {
    // Compute angles
    RowVector3i f_verts = all_faces.row(i->first);
    int v0 = -1;
    int v1 = -1;
    // find out the other two vertices other than the input vertex id
    for (int j = 0; j < 3; j++) {
      if (f_verts[j] == vertID)
        continue;
      if (v0 == -1) {
        v0 = f_verts[j];
        continue;
      }
      if (v1 == -1) {
        v1 = f_verts[j];
      }
    }
    RowVector3d edge0 = all_verts.row(v0) - all_verts.row(vertID);
    RowVector3d edge1 = all_verts.row(v1) - all_verts.row(vertID);
    // std::cout << "before normalization: edge0: " << std::endl << edge0 << std::endl << " edge1: " << std::endl << edge1 << std::endl;
    edge0.normalize();
    edge1.normalize();
    // std::cout << "after normalization: edge0: " << std::endl << edge0 << std::endl << " edge1: " << std::endl << edge1 << std::endl;
    double dot_p = edge0.dot(edge1);
    double angle = acos(dot_p);
    RowVector3d n = face_normals.row(i->first);
    sumNormal += angle * n;
    // std::cout << "dot p: " << dot_p << std::endl << "angle: " << angle << std::endl << "sumNormal: " << sumNormal << std::endl;
  }

  sumNormal.normalize();
  // std::cout << "normalized sum normal: " << sumNormal << std::endl;
  double sign = sumNormal.dot(dir);
  // std::cout << "dir : " << dir << std::endl << "sign: " << sign << std::endl;
  return sign;
}

double getMaxAngleViaVertConnection(
                    const int& vertID,
                    const Vector3d& dir, 
                    const MatrixXd& face_normals,
                    map<int, vector<int>>& vert2Face
                    )
{
    auto iter = vert2Face.find(vertID);
    if (iter == vert2Face.end())
    {
        std::cout << "The vertex is not found in function verifyAngleViaVertConnection!" << std::endl;
    }

    vector<int> faces = vert2Face[vertID];
    map<int, int> fids; // use map to avoid duplicated face ids
    for(auto f : faces)
        fids[f] = 1;
    double maxV = -1e10;
    RowVector3d aveNormal(0.0, 0.0, 0.0);
    for(auto i=fids.begin(); i!=fids.end(); i++)
    {
        RowVector3d n = face_normals.row(i->first);
        aveNormal += n;
        // double dotprod = dir.dot(n);
        // if (dotprod > maxV)
        //     maxV = dotprod;
    }

    aveNormal = aveNormal / double(fids.size());
    aveNormal.normalize();
    maxV = aveNormal.dot(dir);
    return maxV;
}

// input params:
// a, b, c -- the barycentric coordinate of v0, v1, v2 of a triangle
double getMaxAngleViaEdgeConnection(const float& a, const float& b, const float& c, 
                    const int& faceID,
                    const Vector3d& dir, 
                    const MatrixXi& F,
                    const MatrixXd& face_normals,
                    map<CEdge, vector<int>>& edge2Face)
{
    float eps = 1e-4;
    map<int, int> fids;
    int v0 = F.row(faceID)[0], v1 = F.row(faceID)[1], v2 = F.row(faceID)[2];
    CEdge e;
    if (abs(a) < eps){
        if (v1 < v2)
            e = CEdge(v1, v2);
        else
            e = CEdge(v2, v1);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            std::cout << "The edge is not found in function getMaxAngleViaEdgeConnection!" << std::endl;
        }
        vector<int> faces = itr->second;
        for(auto f : faces)
            fids[f] = 1;
        // fids.insert(fids.end(), edge2Face[e].begin(), edge2Face[e].end());
    }
    if (abs(b) < eps){
        if (v2 < v0)
            e = CEdge(v2, v0);
        else
            e = CEdge(v0, v2);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            std::cout << "The edge is not found in function getMaxAngleViaEdgeConnection!" << std::endl;
        }
        vector<int> faces = itr->second;
        for(auto f : faces)
            fids[f] = 1;
    }
    if (abs(c) < eps){
        if (v1 < v0)
            e = CEdge(v1, v0);
        else
            e = CEdge(v0, v1);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            std::cout << "The edge is not found in function getMaxAngleViaEdgeConnection!" << std::endl;
        }
        vector<int> faces = itr->second;
        for(auto f : faces)
            fids[f] = 1;
    }

    double maxV = -1e10;
    RowVector3d aveNormal(0.0, 0.0, 0.0);
    
    for(auto i=fids.begin(); i!=fids.end(); i++)
    {
        RowVector3d n = face_normals.row(i->first);
        aveNormal += n;
        // double dotprod = dir.dot(n);
        // if (dotprod > maxV)
        //     maxV = dotprod;
    }

    aveNormal = aveNormal / double(fids.size());
    aveNormal.normalize();
    maxV = aveNormal.dot(dir);
    return maxV;
}

void reOrderMeshIndices(const MatrixXd& allV, const MatrixXi& allF, const vector<int>& partF, MatrixXd& outV, MatrixXi& outF)
{
    map<int, int> old2New;  // mapp from old index to new index in new mesh
    for(int i=0; i < partF.size(); ++i)
    {
        int fid = partF[i];
        old2New[allF(fid,0)] = -1;
        old2New[allF(fid,1)] = -1;
        old2New[allF(fid,2)] = -1;
    }
    // std::cout << "New mesh includes: " << old2New.size() << " vertices." << std::endl;
    // only keep selected vertices and update vertex index
    outV = MatrixXd(old2New.size(), 3);
    int cnt = 0;
    for (auto iter = old2New.begin(); iter != old2New.end(); iter++)
    {
        outV.row(cnt) = allV.row(iter->first);
        iter->second = cnt++;
    }
    // update face index
    outF = MatrixXi(partF.size(), 3);
    for(int i=0; i < partF.size(); ++i)
    {
        int fid = partF[i];
        outF(i,0) = old2New[allF(fid,0)];
        outF(i,1) = old2New[allF(fid,1)];
        outF(i,2) = old2New[allF(fid,2)];
    }    
}

class distItem{
public:
    bool is_nan;
    float dist;
#if FOR_DEBUG
    // Vector3d closestPnt;
#endif
public:
    distItem(): is_nan(), dist() {}
    distItem(bool nan, float distance): is_nan(nan), dist(distance) {}

};

void computeVert2FacesAndEdge2Faces(const MatrixXd& V, const MatrixXi& F,
                                        map<pair<int, int>, vector<int>>& edge2Face,
                                        map<int, vector<int>>& vert2Face)
{
    for (int i=0; i < F.rows(); ++i)
    {
        int v0 = F.row(i)[0], v1 = F.row(i)[1], v2 = F.row(i)[2];
        if (v0 < v1)
            edge2Face[CEdge(v0, v1)].push_back(i);
        else
            edge2Face[CEdge(v1, v0)].push_back(i);
        if (v0 < v2)
            edge2Face[CEdge(v0, v2)].push_back(i);
        else
            edge2Face[CEdge(v2, v0)].push_back(i);
        if (v2 < v1)
            edge2Face[CEdge(v2, v1)].push_back(i);
        else
            edge2Face[CEdge(v1, v2)].push_back(i);
        
        vert2Face[v0].push_back(i);
        vert2Face[v1].push_back(i);
        vert2Face[v2].push_back(i);            
    }
}


void addItem(
  vector<vector<double>>& gridDists, 
  const RowVector3i& grid_size,
  const vector<double>& distances,
  int i,
  int j,
  int k,
  int& cnt) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        // std::cout << "cnt: " << cnt << std::endl << "distances: " << std::endl << distances[cnt] << std::endl;
        int pos = 0;
        while (pos < gridDists[0].size()) {
          // for each inner grid point, it should be enclosed by 8 cells
          // for grid points on the surface but not at the corner or edge, it should be enclosed by 4 cells
          // for grid points on the edge, it is enclosed by 2 cells
          // for grid points on the corner, it is enclosed by 1 cell
          if (!isnan(gridDists[(i + x) + grid_size(0) * (j + y + grid_size(1) * (k + z))][pos]) 
              && gridDists[(i + x) + grid_size(0) * (j + y + grid_size(1) * (k + z))][pos] < -1e10) {
            // if the current values haven't been filled (it is not NAN and not the initial value), 
            // then update it witht the new distance
            gridDists[ (i + x) + grid_size(0) * (j + y + grid_size(1) * (k + z))][pos] = distances[cnt];
            break;
          }
          pos++;
        }        
        cnt++;
      }
    }
  }    
}

void addItem(
  vector<vector<double>>& gridDists, 
  const RowVector3i& grid_size,
  const vector<double>& distances,
  int i,
  int j,
  int k,
  int& cnt,
  double threshold) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        // std::cout << "cnt: " << cnt << std::endl << "distances: " << std::endl << distances[cnt] << std::endl;
        int pos = 0;
        while (pos < gridDists[0].size()) {
          // if the current values haven't been filled (it is not NAN and not the initial value), 
            // then update it witht the new distance
          if (!isnan(gridDists[(i + x) + grid_size(0) * (j + y + grid_size(1) * (k + z))][pos]) 
              && gridDists[(i + x) + grid_size(0) * (j + y + grid_size(1) * (k + z))][pos] < threshold) {
            gridDists[ (i + x) + grid_size(0) * (j + y + grid_size(1) * (k + z))][pos] = distances[cnt];
            break;
          }
            pos++;
        }        
        cnt++;
      }
    }
  }
}

// generate regular sampling points and its localized 3PSD to the input mesh
void generateRegularSamplesinBBox(
  const MatrixXd& V,              // input mesh vertices
  const MatrixXi& F,              // input mesh faces
  const RowVector3d& bmin,           // input min corner of input bounding box
  const double& min_grid_width,     // input width of minimum grid cell
  const RowVector3i& grid_size,    // number of samples in each dimension
  MatrixXd& samplePoints,         // output regular sampling points
  VectorXd& distPerPoint          // output local 3PSD for each point
) {
  // std::cout << "Creating grid ... ";
  // std::cout << "bbmin: " << bmin[0] << ", " << bmin[1] << ", " << bmin[2] << std::endl;
	// std::cout << "Grid resolution: " << grid_size[0] << " " << grid_size[1] << " " << grid_size[2] << std::endl;
  // std::cout << "Grid minimum width: " << min_grid_width << std::endl;
	MatrixXd GV(grid_size[0]*grid_size[1]*grid_size[2], 3);	    // GV to store grid query points
	VectorXd finalS(grid_size[0]*grid_size[1]*grid_size[2], 1);  // final vector to stored computed distance values

  std::cout << "Before initializing GV" << std::endl;
  for(int x = 0; x< grid_size[0]; x++) {
    const double xi = x * min_grid_width + bmin(0);
    for(int y = 0; y < grid_size[1]; y++) {
      const double yi = y * min_grid_width + bmin(1);
      for(int z = 0; z < grid_size[2]; z++) {
        const double zi = z * min_grid_width + bmin(2);
        GV.row(x+grid_size(0)*(y + grid_size(1)*z)) = RowVector3d(xi,yi,zi);
      }
    }
  }

  RowVector3d tmp(1.0, 1.0, 1.0);
  std::cout << "size of RowVector3d: " << sizeof(RowVector3d) << " total size: " << tmp * grid_size[0] * grid_size[1] * grid_size[2] << std::endl;
  std::cout << "Done initial GV!" << std::endl;

  std::cout << "Computing 3-pole distance field ..." << std::endl;
  auto start = high_resolution_clock::now();
  int w = grid_size(0) - 1;
  int h = grid_size(1) - 1;
  int d = grid_size(2) - 1;
  int faceNum = F.rows();

  // map<int, vector<double>> gridDists;  // store the point to mesh distance for grid points (GV)
  vector<vector<double>> gridVec(finalS.rows());
  double threshold = -1e10;
  
  for (int i = 0; i < gridVec.size(); i++) {
    gridVec[i] = vector<double>(8, threshold - 1.0);
  }
  std::cout << " Done initial gridVec!" << std::endl;

  // try to compute in batches
  // int totalRes = w * h * d;
  // int min_batch_size = 150 * 150 * 150;
  // int num_batch = totalRes / min_batch_size;
  // int step_w_size = w / num_batch;
  // if (step_w_size == 0) step_w_size = 1;
  // std::cout << "There are " << num_batch << " batches!" << " step_w_size: " << step_w_size << std::endl;
  // int start_num = 0;
  // int stop_num = 0;

  // while (1) {
  //   stop_num = start_num + step_w_size;
  //   if (stop_num > w)
  //     stop_num = w;
  //   if (start_num >= stop_num)
  //     break;
  //   std::cout << "start num: " << start_num << " stop num: " << stop_num << std::endl;
  //   for (int i = start_num; i < stop_num; i++) {
  //     #pragma omp parallel for
  //     for (int j = 0; j < h; j++) {
  //       for (int k = 0; k < d; k++) {

  //       }
  //     }
  //   }
    
  //   start_num += step_w_size; 
  // }

  // looping over the grid cells
  #pragma omp parallel for
  for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      for (int k = 0; k < d; k++) {

        vector<Vector3d> queryPnts;
        for (int x = 0; x < 2; x++) {
          for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
              Vector3d base = Vector3d(bmin(0) + i * min_grid_width,
                  bmin(1) + j * min_grid_width,
                  bmin(2) + k * min_grid_width);
              Vector3d pnt = base + Vector3d(x * min_grid_width, y * min_grid_width, z * min_grid_width);
              queryPnts.push_back(pnt);
              // std::cout << "pnt : "  << pnt[0] << ", " << pnt[1] << ", " << pnt[2] << std::endl;   
            }
          }
        }

        Vector3d cellCenter(bmin(0) + (i + 0.5) * min_grid_width,
            bmin(1) + (j + 0.5) * min_grid_width,
            bmin(2) + (k + 0.5) * min_grid_width);
        Vector3d length(min_grid_width, min_grid_width, min_grid_width);
        pair<Vector3d, Vector3d> cell;
        cell.first = cellCenter;
        cell.second = length;

        // std::cout << "cell center: " << cellCenter[0] << ", " << cellCenter[1] << ", " << cellCenter[2]
        //   << " length: " << length << std::endl;

        vector<double> distances = computeT3PoleDistForPtsInCell(V, F, queryPnts, cell, 0);
        int cnt = 0;        
        addItem(gridVec, grid_size, distances, i, j, k, cnt);
      }  // end for k
    }  // end for j
  }  // end for i

  // std::cout << "start merging result!" << std::endl;
  for (int i = 0; i < gridVec.size(); i++) {
    const vector<double>& dists = gridVec[i];
    double sum = 0.0;
    int nanCount = 0;
    int validCount = 0;
    for (int j = 0; j < dists.size(); j++) {
      if (isnan(dists[j])) {
        nanCount++;
        continue;
      }
      if (dists[j] < threshold)
        continue;
      validCount++;
      sum += dists[j];
    }
    if (nanCount == dists.size()) {
        finalS(i) = NAN;
    } else {
        double finalD = sum / static_cast<double>(validCount);
        finalS(i) = finalD;
    }
  }
  samplePoints = GV;
  distPerPoint = finalS;

  // for debug only
  map<int, int> stats;
  for (int i = 0; i < grid_size[0]; i++) {
    for (int j = 0; j < grid_size[1]; j++) {
      for (int k = 0; k < grid_size[2]; k++) {
        // std::cout << "grid " << i << " " << j << " " << k << ": " << std::endl;
        int global_grid_pos = i + grid_size(0) * (j + grid_size(1) * k);
        vector<double> vec = gridVec[global_grid_pos];
        int validCnt = 0;
        for (int j = 0; j < vec.size(); j++) {
          // std::cout << vec[j] << " ";
          if (isnan(vec[i]) || vec[j] > threshold)
            validCnt++;
        }
        stats[validCnt]++;
        // std::cout << std::endl;
      }
    }
  }
  for (auto iter = stats.begin(); iter != stats.end(); iter++) {
    std::cout << iter->first << ": " << iter->second << std::endl;
  }
 
}
// generate regular sampling points and its localized 3PSD to the input mesh
vector<vector<double>> generateRegularSamplesinBBox(
  const MatrixXd& V,              // input mesh vertices
  const MatrixXi& F,              // input mesh faces
  const MatrixXd& bmin,           // input min corner of input bounding box
  const double& min_grid_width,   // input width of minimum grid cell
  const RowVector3i& grid_size,   // number of samples in each dimension
  double threshold,                // threshold for determining if the grid value is computed
  int idx_z
) {
  // std::cout << "Creating grid ... ";
  // std::cout << "bbmin: " << bmin[0] << ", " << bmin[1] << ", " << bmin[2] << std::endl;
	// std::cout << "Grid resolution: " << grid_size[0] << " " << grid_size[1] << " " << grid_size[2] << std::endl;
  // std::cout << "Grid minimum width: " << min_grid_width << std::endl;
	MatrixXd GV(grid_size[0]*grid_size[1]*grid_size[2], 3);	    // GV to store grid query points
	VectorXd finalS(grid_size[0]*grid_size[1]*grid_size[2], 1);  // final vector to stored computed distance values

  for(int x = 0; x< grid_size[0]; x++) {
    const double xi = x * min_grid_width + bmin(0);
    for(int y = 0; y < grid_size[1]; y++) {
      const double yi = y * min_grid_width + bmin(1);
      for(int z = 0; z < grid_size[2]; z++) {
        const double zi = z * min_grid_width + bmin(2);
        GV.row(x+grid_size(0)*(y + grid_size(1)*z)) = RowVector3d(xi,yi,zi);
      }
    }
  }
  // std::cout << std::endl << std::endl;
  // std::cout << "Computing 3-pole distance field ..." << std::endl;
  auto start = high_resolution_clock::now();
  int w = grid_size(0) - 1;
  int h = grid_size(1) - 1;
  int d = grid_size(2) - 1;
  int faceNum = F.rows();

  // map<int, vector<double>> gridDists;  // store the point to mesh distance for grid points (GV)
  vector<vector<double>> gridVec(finalS.rows());
  
  for (int i = 0; i < gridVec.size(); i++) {
    gridVec[i] = vector<double>(8, threshold - 10.0);
  }

  // looping over the grid cells
  #pragma omp parallel for
  for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      for (int k = 0; k < d; k++) {

        vector<Vector3d> queryPnts;
        for (int x = 0; x < 2; x++) {
          for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
              Vector3d base = Vector3d(bmin(0) + i * min_grid_width,
                  bmin(1) + j * min_grid_width,
                  bmin(2) + k * min_grid_width);
              Vector3d pnt = base + Vector3d(x * min_grid_width, y * min_grid_width, z * min_grid_width);
              queryPnts.push_back(pnt);
              // std::cout << "pnt : "  << pnt[0] << ", " << pnt[1] << ", " << pnt[2] << std::endl;   
            }
          }
        }

        Vector3d cellCenter(bmin(0) + (i + 0.5) * min_grid_width,
            bmin(1) + (j + 0.5) * min_grid_width,
            bmin(2) + (k + 0.5) * min_grid_width);
        Vector3d length(min_grid_width, min_grid_width, min_grid_width);
        pair<Vector3d, Vector3d> cell;
        cell.first = cellCenter;
        cell.second = length;

        vector<double> distances = computeT3PoleDistForPtsInCell(V, F, queryPnts, cell, 0);
        if (queryPnts.size() != distances.size())
          std::cout << "ERROR! Returned distance size is not equal to that of query points!" << std::endl;
        if (distances.size() != 8)
          std::cout << "Error! Returned distance size is not equal to 8!" << std::endl;
        int cnt = 0;
        addItem(gridVec, grid_size, distances, i, j, k, cnt, threshold);
      }  // end for k
    }  // end for j
  }  // end for i


  // for debug only
  // for (int i = 0; i < gridVec.size(); i++) {
  //   vector<double> vec = gridVec[i];
  //   for (int j = 0; j < vec.size(); j++)
  //     std::cout << vec[j] << " ";
    
  //   std::cout << std::endl;
  // }

  return gridVec;
}

vector<double> computeT3PoleDistForPtsInCell(
    const MatrixXd& V,
    const MatrixXi& F,
    const vector<Vector3d>& pts,
    const pair<Vector3d, Vector3d>& cell,  // first - cell center, second - cell length
    const int cell_id
) {
  int faceNum = F.rows();
  Vector3d cc = cell.first;
  Vector3d length = cell.second;
  vector<int> includedFaces;  // store all faces that intersect with the current cell
  // collect intersected face ids
  // #pragma omp parallel for shared(count)
  for (int fid = 0; fid < faceNum; fid++) {
    float cellCenter[3] = {cc[0], cc[1], cc[2]};
    float halfSize[3] = {0.51 * length[0], 0.51 * length[1], 0.51 * length[2]};
    Vector3i f = F.row(fid);
    float triVerts[3][3] = {{V.row(f[0])[0], V.row(f[0])[1], V.row(f[0])[2]},
        {V.row(f[1])[0], V.row(f[1])[1], V.row(f[1])[2]},
        {V.row(f[2])[0], V.row(f[2])[1], V.row(f[2])[2]}};

    if (triBoxOverlap(cellCenter, halfSize, triVerts)) {
      includedFaces.push_back(fid);
    }
  }  // end for fid

  vector<double> gridDists; // output

  // // for debug only
  // if (cell_id == 855 || cell_id == 856 || cell_id == 859 || cell_id == 860) {
  //   std::cout << "included faces: " << std::endl;
  //   for (auto idx : includedFaces)
  //     std::cout << idx << std::endl;
  // }

  if (includedFaces.size() == 0) {
    for (int x = 0; x < pts.size(); x++) {
      gridDists.push_back(NAN);
    }
    return gridDists;
  }
  // std::cout << "There are " << includedFaces.size() << " faces inside!" << std::endl;


  // reorder the included trianlge faces
  MatrixXi partF;
  MatrixXd partV;
  reOrderMeshIndices(V, F, includedFaces, partV, partF);
  // igl::writeOBJ("../data/output.obj", partV, partF);

  // initialize PQP model
  PQP_Model* m_pqp_model = new PQP_Model();
  buildPQP(m_pqp_model, partV, partF); 


  map<pair<int, int>, vector<int>> part_edge2Face;
  map<int, vector<int>> part_vert2Face;
  computeVert2FacesAndEdge2Faces(partV, partF, part_edge2Face, part_vert2Face);

  MatrixXd face_normals; // per face normal
  // MatrixXd part_Z;
  // igl::writeOBJ("test.obj", partV, partF);
  igl::per_face_normals(partV, partF, Vector3d(1,1,1).normalized(), face_normals);

  // iterate over cell corners - compute its distance to the included faces
  // #pragma omp parallel for
  for (int k = 0; k < pts.size(); k++) {    
    const Vector3d& pnt = pts[k];
    // std::cout << pnt[0] << " " << pnt[1] << " " << pnt[2] << std::endl;

    Vector3d nearestPnt;
    int closestTriID;
    double dist = PQPABSDist(m_pqp_model, pnt, nearestPnt, closestTriID);
    RowVector3d n = face_normals.row(closestTriID);
    Vector3d dir_org = pnt - nearestPnt;
    Vector3d dir = dir_org.normalized();

    // double p2pdist = dir_org.norm();
    if (dist < 1e-10 || isnan(dir[0])) {
      dist = 0.0;
      gridDists.push_back(dist);
      continue;
    }
    double eps = 1e-4;
    double dotprod = dir.dot(n);
    if (abs(dotprod-1.0) < eps)
      dotprod = 1.0;
    if (abs(dotprod+1.0) < eps)
      dotprod = -1.0;
    double angle = acos(dotprod) * 180.0 / PI;

    if (abs(dotprod - 1.0) < eps) {
      // in this case, the nearest point lies inside certain triangle
      gridDists.push_back(dist);
      continue;
    } else if (abs(dotprod + 1.0) < eps) {
      gridDists.push_back(-dist);
      continue;
    } else {
      float a, b, c;  // barycentric coordinate
      int vid0 = partF.row(closestTriID)[0], vid1 = partF.row(closestTriID)[1], vid2 = partF.row(closestTriID)[2];
      Vector3d v0 = partV.row(vid0);
      Vector3d v1 = partV.row(vid1);
      Vector3d v2 = partV.row(vid2);
      Barycentric(nearestPnt, v0, v1, v2, a, b, c);
      float eps = 1e-4;

      if ( abs(a) < eps || abs(b) < eps || abs(c) < eps) {
        // closest point lies on the edge
        double prod = -10;
        if (abs(a-1.0) < eps ||  abs(b-1.0) < eps ||  abs(c-1.0) < eps) {
          int vid = vid0;
          if (abs(b-1.0) < eps)
              vid = vid1;
          if (abs(c-1.0) < eps)
              vid = vid2;
          prod = getMaxAngleViaVertConnection(vid, dir, face_normals, part_vert2Face);
          // prod = computeSignViaPseudonormal(vid, V, F, dir, face_normals, part_vert2Face);
        }else {
          prod = getMaxAngleViaEdgeConnection(a, b, c, closestTriID, dir, partF, face_normals, part_edge2Face);
        }
        if (abs(prod-1.0) < eps)
          prod = 1.0;
        if (abs(prod+1.0) < eps)
          prod = -1.0;
        double finalAngle = acos(prod) * 180.0 / PI;
        double newDist;
        if (finalAngle < 90.0) {
          newDist = dist;
        } else {
          newDist = -dist;
        }
        gridDists.push_back(newDist);
      } else {
        // do not lie on the edge and the angle is not smaller than 90 degree
        // std::cout << "sounds like impossible case just happened!" << std::endl;
        gridDists.push_back(dist);
      }
    }
  }

  delete m_pqp_model;
  return gridDists;
}

void generateThin3PoleSDF(string meshFileName,
    string outReconMeshName,
    int resolution,
    int batchSize) {
  // load obj mesh
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  std::cout << "Loading " << meshFileName << std::endl;
	// Read in inputs as double precision floating point meshes
  igl::readOBJ(meshFileName,V,F);
	std::cout << "Mesh load done!" << std::endl;

  MatrixXi newF;
  removeIdenticalVertices(F, newF);

  F = newF;

  /****** create grid query points ******/
  // compute tight bounding box of the input mesh
	RowVector3d tightBmin = V.colwise().minCoeff();
	RowVector3d tightBmax = V.colwise().maxCoeff();
  RowVector3d diff = tightBmax - tightBmin;
  // create proper bounding box for axis-aligned planes
  if (diff[0] == 0.0 || diff[1] == 0.0 || diff[1] == 0.0)
  {
      double maxItem = diff.maxCoeff();
      int minID = 0;
      if (diff[1] == 0)
          minID = 1;
      if (diff[2] == 0)
          minID = 2;
      tightBmin[minID] -= 0.2 * maxItem;
      tightBmax[minID] += 0.2 * maxItem;
  }
	RowVector3d center = (tightBmax + tightBmin) / 2.0;
	// enlarge bounding box little bit
	RowVector3d bmin = center + 1.05 * (tightBmin - center);
	RowVector3d bmax = center + 1.05 * (tightBmax - center);
	RowVector3d bboxSize = bmax - bmin;
	std::cout << "Bounding corners - min: " << vec2String(bmin) << " max: " << vec2String(bmax) << std::endl;

	// number of vertices on the largest side
	const int s = resolution;
	const double minGridWidth = (bmax-bmin).maxCoeff()/(double)s;
	RowVector3i gridSize = ( s * ((bmax-bmin) / (bmax-bmin).maxCoeff()) ).cast<int>();
  gridSize += RowVector3i(1, 1, 1);
	std::cout << "Grid resolution: " << gridSize[0] << " " << gridSize[1] << " " << gridSize[2] << std::endl;
  std::cout << "Grid minimum width: " << minGridWidth << std::endl;

	
	std::cout << "Creating grid ... ";
	MatrixXd GV(gridSize(0)*gridSize(1)*gridSize(2), 3);	// GV to store grid query points
	VectorXd finalS(gridSize[0]*gridSize[1]*gridSize[2], 1); // final vector to stored computed distance values
  finalS.fill(-1.0);
	std::cout << " Done " << std::endl;

	const auto lerp = [&](const int di, const int d)->double
	{return bmin(d)+(double)di/(double)(gridSize(d)-1)*(bmax(d)-bmin(d));};

  
  for(int x = 0; x< gridSize[0]; x++) {
    const double xi = lerp(x, 0);
    for(int y = 0; y < gridSize[1]; y++) {
      const double yi = lerp(y, 1);
      for(int z = 0; z < gridSize[2]; z++) {
        const double zi = lerp(z, 2);
        GV.row(x+gridSize(0)*(y + gridSize(1)*z)) = RowVector3d(xi,yi,zi);
      }
    }
  }


  /********************************************************************************************
   * Compute 3-pole distance field
  *********************************************************************************************/
  std::cout << "Computing 3-pole distance field ..." << std::endl;
  auto start = high_resolution_clock::now();
  int maxBufferSize = 2; // buffer size of stored closest distItems; should be at least 2!!

  // int totalNum = gridSize(0)*gridSize(1)*gridSize(2);
  // int batchNum = ceil(double(totalNum) / double(batchSize));
  // std::cout << "Total points: " << totalNum << " Batch size: " << batchSize << std::endl;
  // std::cout << "There will be " << batchNum << " batches in total!" << std::endl;

  int w = gridSize(0) - 1;
  int h = gridSize(1) - 1;
  int d = gridSize(2) - 1;
  int faceNum = F.rows();

  vector<glm::dvec3> vertices;
  for (int i = 0; i < V.rows(); i++) {
      vertices.push_back(glm::dvec3(V.row(i)[0], V.row(i)[1], V.row(i)[2]));
  }  // end for i

  map<int, vector<double>> gridDists;  // store the point to mesh distance for grid points (GV)

  // looping over the grid cells
  // omp_set_num_threads(8);
  
  // #pragma omp parallel for
  for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      for (int k = 0; k < d; k++) {
          vector<int> includedFaces;  // store all faces that intersect with the current cell
          // collect intersected face ids
          // int count = 0;
          // #pragma omp parallel for shared(count)
          // for (int fid = 0; fid < faceNum; fid++) {
            // float cellCenter[3] =
            //     {bmin[0] + (i + 0.5) * minGridWidth,
            //     bmin[1] + (j + 0.5) * minGridWidth,
            //     bmin[2] + (k + 0.5) * minGridWidth};
            // float halfSize[3] = {0.5 * minGridWidth, 0.5 * minGridWidth, 0.5 * minGridWidth};
          //   Vector3i f = F.row(fid);
          //   float triVerts[3][3] = {{V.row(f[0])[0], V.row(f[0])[1], V.row(f[0])[2]},
          //       {V.row(f[1])[0], V.row(f[1])[1], V.row(f[1])[2]},
          //       {V.row(f[2])[0], V.row(f[2])[1], V.row(f[2])[2]}};

          //   if (triBoxOverlap(cellCenter, halfSize, triVerts)) {
          //     includedFaces.push_back(fid);
          //     // includedFaces[count] = fid;
          //     // count++;
          //   }                  
          // }  // end for fid

          // vector<int> xi{i, i + 1};
          // vector<int> yi{j, j + 1};
          // vector<int> zi{k, k + 1};

          // if (includedFaces.size() == 0)
          // {
          //   for (int x = 0; x < 2; x++) {
          //     for (int y = 0; y < 2; y++) {
          //       for (int z = 0; z < 2; z++) {
          //         gridDists[ (i + x) + gridSize(0) * (j + y + gridSize(1) * (k + z))].push_back(NAN);
          //       }
          //     }
          //   }
          //   continue;
          // }
          

          // // record the included trianlge faces
          // MatrixXi partF;
          // MatrixXd partV;
          // reOrderMeshIndices(V, F, includedFaces, partV, partF);

          // // initialize PQP model
          // PQP_Model* m_pqp_model = new PQP_Model();        
          // buildPQP(m_pqp_model, partV, partF); 

          // map<pair<int, int>, vector<int>> part_edge2Face;
          // map<int, vector<int>> part_vert2Face;
          // computeVert2FacesAndEdge2Faces(partV, partF, part_edge2Face, part_vert2Face);


          // MatrixXd face_normals; // per face normal
          // MatrixXd part_Z;
          // igl::per_face_normals(partV, partF, part_Z, face_normals);   

          

          // iterate over cell corners - compute its distance to the included faces
          // #pragma omp parallel for
          vector<Vector3d> queryPnts;
          for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
              for (int z = 0; z < 2; z++) {
                Vector3d base = Vector3d(bmin[0] + i * minGridWidth,
                    bmin[1] + j * minGridWidth,
                    bmin[2] + k * minGridWidth);
                Vector3d pnt = base + Vector3d(x * minGridWidth, y * minGridWidth, z * minGridWidth);
                // std::cout << pnt[0] << " " << pnt[1] << " " << pnt[2] << std::endl;
                queryPnts.push_back(pnt);

                // Vector3d nearestPnt;
                // int closestTriID;
                // double dist = PQPABSDist(m_pqp_model, pnt, nearestPnt, closestTriID);
                // RowVector3d n = face_normals.row(closestTriID);
                // Vector3d dir_org = pnt - nearestPnt;
                // Vector3d dir = dir_org.normalized();

                // // double p2pdist = dir_org.norm();
                // if (dist < 1e-10 || isnan(dir[0])) {
                //   dist = 0.0;
                //   gridDists[(i + x) + gridSize(0) * (j + y + gridSize(1) * (k + z))].push_back(dist);
                //   continue;
                // }
                // double eps = 1e-4;
                // double dotprod = dir.dot(n);
                // if (abs(dotprod-1.0) < eps)
                //   dotprod = 1.0;
                // if (abs(dotprod+1.0) < eps)
                //   dotprod = -1.0;
                // double angle = acos(dotprod) * 180.0 / PI;

                // if (abs(dotprod-1.0) < eps) {
                //   // in this case, the nearest point lies inside certain triangle
                //   gridDists[(i + x) + gridSize(0) * (j + y + gridSize(1) * (k + z))].push_back(dist);
                //   continue;
                // } else if (abs(dotprod+1.0) < eps) {
                //   gridDists[(i + x) + gridSize(0) * (j + y + gridSize(1) * (k + z))].push_back(-dist);
                //   continue;
                // } else {
                //   float a, b, c;  // barycentric coordinate
                //   int vid0 = partF.row(closestTriID)[0], vid1 = partF.row(closestTriID)[1], vid2 = partF.row(closestTriID)[2];
                //   Vector3d v0 = partV.row(vid0);
                //   Vector3d v1 = partV.row(vid1);
                //   Vector3d v2 = partV.row(vid2);
                //   Barycentric(nearestPnt, v0, v1, v2, a, b, c);
                //   float eps = 1e-4;

                //   if ( abs(a) < eps || abs(b) < eps || abs(c) < eps) {
                //     // closest point lies on the edge
                //     double prod = -10;
                //     if (abs(a-1.0) < eps ||  abs(b-1.0) < eps ||  abs(c-1.0) < eps) {
                //       int vid = vid0;
                //       if (abs(b-1.0) < eps)
                //           vid = vid1;
                //       if (abs(c-1.0) < eps)
                //           vid = vid2;
                //       prod = getMaxAngleViaVertConnection(vid, dir, face_normals, part_vert2Face);
                //     }else {
                //       prod = getMaxAngleViaEdgeConnection(a, b, c, closestTriID, dir, partF, face_normals, part_edge2Face);
                //     }
                //     if (abs(prod-1.0) < eps)
                //       prod = 1.0;
                //     if (abs(prod+1.0) < eps)
                //       prod = -1.0;
                //     double finalAngle = acos(prod) * 180.0 / PI;
                //     double newDist;
                //     if (finalAngle < 90.0) {
                //       newDist = dist;
                //     } else {
                //       newDist = -dist;                        
                //     }
                //     gridDists[ (i + x) + gridSize(0) * (j + y + gridSize(1) * (k + z))].push_back(newDist);
                //   } else {
                //     // do not lie on the edge and the angle is not smaller than 90 degree
                //     std::cout << "sounds like impossible case just happened!" << std::endl;
                //     gridDists[ (i + x) + gridSize(0) * (j + y + gridSize(1) * (k + z))].push_back(dist);
                //   }
                // }
              }
            }
          }

          Vector3d cellCenter(bmin[0] + (i + 0.5) * minGridWidth,
              bmin[1] + (j + 0.5) * minGridWidth,
              bmin[2] + (k + 0.5) * minGridWidth);
          Vector3d length(minGridWidth, minGridWidth, minGridWidth);
          // float halfSize[3] = {0.5 * minGridWidth, 0.5 * minGridWidth, 0.5 * minGridWidth};
          pair<Vector3d, Vector3d> cell;
          cell.first = cellCenter;
          cell.second = length;
          // std::cout << "i: " << i << " j: " << j << " k: " << k << std::endl;
          // std::cout << "cell.first: " << cell.first << " " << " length: " << cell.second << std::endl;
          // std::cout << "query point: " << queryPnts << std::endl;
          vector<double> distances = computeT3PoleDistForPtsInCell(V, F, queryPnts, cell, 0);
          int cnt = 0;
          for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
              for (int z = 0; z < 2; z++) {
                // std::cout << "cnt: " << cnt << std::endl << "distances: " << std::endl << distances[cnt] << std::endl;
                gridDists[ (i + x) + gridSize(0) * (j + y + gridSize(1) * (k + z))].push_back(distances[cnt]);
                cnt++;
              }
            }
          }



          // delete m_pqp_model; 
      }  // end for k
    }  // end for j
  }  // end for i

  assert(gridDists.size() == finalS.rows());

  // merge the results
  for (int i = 0; i < gridDists.size(); i++) {
      const vector<double>& dists = gridDists[i];
      double sum = 0.0;
      int nanCount = 0;
      for (int j = 0; j < dists.size(); j++) {
          if (isnan(dists[j])) {
              nanCount++;
              continue;
          }
          sum += dists[j];
      }

      if (nanCount == dists.size()) {
          finalS(i) = NAN;
      } else {
          double finalD = sum / static_cast<double>(dists.size());
          finalS(i) = finalD;
      }
  }
    
	auto stop = high_resolution_clock::now(); 
	auto duration = duration_cast<microseconds>(stop - start); 
	std::cout << "Computation time: " << duration.count() / double(1000000.0) << " seconds" << std::endl;	


    /************* Reconstruction ****************/
	// use marching cube to reconstruct
	std::cout << " Marching cubes ... " << std::endl;
	start = high_resolution_clock::now();

	MatrixXd SV;
	MatrixXi SF;
	igl::copyleft::marching_cubes(finalS, GV, gridSize(0), gridSize(1), gridSize(2), SV, SF);

	stop = high_resolution_clock::now();
	duration = duration_cast<microseconds>(stop - start); 
	std::cout << "Marching Cube Used time: " << duration.count() / double(1000000.0) << " seconds" << std::endl;

    
	// save the reconstructed mesh
	save_obj_mesh(outReconMeshName, SV, SF);
    std::cout << "there are " << SV.rows() << " vertices and " << SF.rows() << " faces!" << std::endl;
	std::cout << "Finished writing reconstruction to " << outReconMeshName << "!" << std::endl;   
}
                    

