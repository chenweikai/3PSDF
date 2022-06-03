#pragma once
#include <iostream>
#include <string>
#include <igl/readOBJ.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/parula.h>
#include <Eigen/Core>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <chrono> 
// #include <igl/copyleft/marching_cubes.h>
#include <igl/signed_distance.h>
#include <igl/parula.h>
#include <igl/point_mesh_squared_distance.h>
#include "Octree.h"
#include "utilities.h"

#ifndef IGL_NO_EIGEN
    #include <Eigen/Core>
#endif
using namespace std;
using namespace Eigen;

void generateVisField(igl::opengl::glfw::Viewer& viewer, const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F, const RowVector3d& bmin, const RowVector3d& bmax, int resolution, int dim, int slice = 0);

// display the reconstructed mesh and marching cube grid for debug
void displayForDebug(const MatrixXd& V, const MatrixXi& F, const MatrixXd& SV, const MatrixXi& SF, 
	const RowVector3d& Vmax, const RowVector3d& Vmin, const MatrixXd& gridPts, const VectorXd& S);

void displayEmptyOctreeCelss(const MatrixXd& V, const MatrixXi& F, const set<Octree*>& emptyCells);

void displayMeshAndSamples(const MatrixXd& V, const MatrixXi& F, const MatrixXd& gridPts);

void displayMeshAndSignedSamples(const MatrixXd& V, const MatrixXi& F, const MatrixXd& gridPts, const vector<double>& signs);

// void displayParentAndChildren(const Octree* parent, const Octree* child1, const Octree* child2);
