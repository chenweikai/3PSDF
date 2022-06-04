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

// get the folder name from the input file name
string get_folder_name (const string& file_name);

// remove those faces that have idential vertex ids
void remove_identical_verts(const MatrixXi& F, MatrixXi& outF);

// customized mesh save function -- automatically remove NAN vertices
void save_obj_mesh(string filename, const MatrixXd& verts, const MatrixXi& faces);

// assemble independent mesh parts into one mesh
void assemble_mesh_parts(const vector<pair<MatrixXd, MatrixXi>>& meshParts, MatrixXd& outV, MatrixXi& outF);

// compute bounding box of the input mesh
void computeBBox(string bboxObjName, VectorXd& bboxmin, VectorXd& bboxmax);

// normalize mesh to [-0.5, 0.5] and centered at [0, 0, 0]
void normalizeMesh(string input_obj_name, string output_obj_name);