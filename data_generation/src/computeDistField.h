#pragma once
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
#include "OctreeUtilities.h"
// #include "IGL_SIGN.h"
#include "visualization.h"

#ifndef IGL_NO_EIGEN
    #include <Eigen/Core>
#endif
using namespace std;
using namespace Eigen;




// generate signed distance field using PQP
// args:
// meshFileName - input mesh filename
// meshReoncName - reconstructed mesh filename using marching cube
// outputFile - output SDF file name
// resolution - the resolution in the longest axis
void generateSDFUsePQP(string meshFileName, string meshReconName, string outSDFName, int resolution = 100);

// compute signed distance field using winding number
// args:
// meshFileName - input mesh filename
// meshReoncName - reconstructed mesh filename using marching cube
// outputFile - output SDF file name
// resolution - the resolution in the longest axis
void computeSDFUseWindNum(string meshFileName, string meshReconName, string outSDFName, int resolution);




void projectMeshToMesh(string sourceMeshName, string targetMeshName);




// void saveSDF()

