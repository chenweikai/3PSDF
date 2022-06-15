// Utility functions

#ifndef _3PSDF_SRC_LIB_UTILITIES_H_
#define _3PSDF_SRC_LIB_UTILITIES_H_

#include <experimental/filesystem>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <igl/read_triangle_mesh.h>

namespace l3psdf {

// Compute bounding box of the input mesh
//
// @param input_obj_name: input obj mesh name
// @param bbox_min: returned bounding box min corner
// @param bbox_max: returned bounding box max corner
void ComputeBbox(std::string input_obj_name, Eigen::VectorXd& bbox_min, Eigen::VectorXd& bbox_max);

// Get the folder name from the input file name
//
// @param file_name: input file name
//
// @return: the immediate parent folder name that contains the input file
std::string GetFolderName(const std::string& file_name);

// Remove those faces that have idential vertex ids
//
// @param input_faces: input mesh faces
// @param output_faces: output face meshes
//
// @return: output_faces: faces with idential vertex ids removed
void RemoveIdenticalVerts(const Eigen::MatrixXi& input_faces, Eigen::MatrixXi& output_faces);

// reorder the selected faces of the input mesh so that the selected part can form a regular mesh
//
// @param all_verts, all_faces: input mesh vertices and faces
// @param part_faces: selected face ids
// @param out_verts, out_faces: output reordered vertices and faces of the selected mesh part
void ReorderMeshIndices(const Eigen::MatrixXd& all_verts, const Eigen::MatrixXi& all_faces,
                          const std::vector<int>& part_faces, Eigen::MatrixXd& out_verts, Eigen::MatrixXi& out_faces);

// Customized mesh save function -- automatically remove NAN vertices
//
// @param filename: output mesh name
// @param verts: mesh vertices to be saved
// @param faces: mesh faces to be saved
void SaveObjMesh(std::string filename, const Eigen::MatrixXd& verts, const Eigen::MatrixXi& faces);

// Assemble independent mesh parts into one mesh
//
// @param mesh_parts: separated mesh parts to be assembled
// @param out_verts: returned output mesh vertices
// @param out_faces: returned output mesh faces
void AssembleMeshParts(const std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>>& mesh_parts,
                       Eigen::MatrixXd& out_verts, Eigen::MatrixXi& out_faces);

}  // namespace l3psdf

#endif // _3PSDF_SRC_LIB_UTILITIES_H_
