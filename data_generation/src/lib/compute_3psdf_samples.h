// Implementation of computing 3-Pole Signed Distance (3PSDF) and training data from a given mesh

#ifndef _3PSDF_SRC_LIB_COMPUTE_3PSDF_SAMPLES_H_
#define _3PSDF_SRC_LIB_COMPUTE_3PSDF_SAMPLES_H_

#include <vector>

#ifndef IGL_NO_EIGEN
    #include <Eigen/Core>
#endif

// Perform marching cubes on each octree cells and merge the local meshes into a global one.
//
// @param octree_verts: the file name of the output SDF file
// @param reconObjName: the file name of the output reconstructed mesh
// @param output_pts_name: the file name of the saved sampling points
// @param octreeDepth: octree depth for computing the L3PSDF
std::pair<Eigen::MatrixXd, Eigen::MatrixXi> localized_marching_cubes(
    const std::vector<Eigen::Vector3d>& octree_verts,
    const std::vector<std::vector<int>>& octree_faces,
    const std::vector<double>& distances);

// Generate 3PSDF samples that are the vertices of the octree
//
// @param objName: the input file name of obj mesh
// @param outSDFName: the file name of the output SDF file
// @param reconObjName: the file name of the output reconstructed mesh
// @param output_pts_name: the file name of the saved sampling points
// @param octreeDepth: octree depth for computing the L3PSDF
// @param flag_write_ply: flag of whether to output the sampling points to PLY file for visualization
// @param flag_recon_obj: flag of whether to reconstruct the mesh from the computed L3PSDF field
// @param flag_write_sdf: flag of whether to output SDF file
void GenerateOctree3psdfSamples(
    std::string input_obj_name, std::string out_sdf_name,
    std::string recon_obj_name, std::string output_pts_name,
    int octree_depth, bool flag_write_ply,
    bool flag_recon_obj, bool flag_write_sdf);

#endif // _3PSDF_SRC_LIB_COMPUTE_3PSDF_SAMPLES_H_
