/* 
    Visual Hull Computation for Orthogonal Projection
    Weikai Chen
*/
#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <chrono> 
#include <fstream>
#include <sstream>
#include <experimental/filesystem>
#include <queue>
#define cimg_use_png
#define cimg_use_jpg
#include "CImg.h"
#ifdef Success
  #undef Success
#endif
#include "OpenGLCamera.h"
#ifndef IGL_NO_EIGEN
    #include <Eigen/Core>
#endif
#include <igl/copyleft/marching_cubes.h>
#include <igl/read_triangle_mesh.h>
#include "igl/readOBJ.h"
#include "igl/writeOBJ.h"
#include "utility_file_read.h"
using namespace std;
using namespace std::chrono; 
using namespace Eigen;
using namespace cimg_library;

// Extract silhouette from the input image
// delete pixels in the background that are similar to the input bgColor
// input args:
// bgColor - background color
// threshold - if the l2 difference between the background pixels and the input bgColor
//              is smaller than this threshold, the background pixels will be deleted
void extractSil(string inImgName, string outImgName, const Vector3i& bgColor, double threshold);

// Generate visual hull for input images in a row
void batchComputeVisualHull(string inputDir, int resolution);

// Compute visual hull for input images
void computeVisualHull(const vector<string>& inImageNames, string inMeshName, string outMeshName, 
    const vector<Vector3d>& viewPoints, int resolution);

// function for projecting mesh vertices to the input image
void projectMeshVertsToImage(string meshName);

// project 3D points onto the input image using orthogonal projection
// the camera parameters are given
// input is n_verts *3 materix; output is n_verts * 2 matrix recording screen coordinates
MatrixXi project3DPointsToImage(const MatrixXd& V, float l, float r, float b, float t,
    float near, float far, const Vector3d& eye, const Vector3d& center, const Vector3d& up, int imgHeight, int imgWidth);