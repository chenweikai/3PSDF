/*
    1) Generate samples on Octree cell vertices
    2) Distance computed based on local version of 3-Pole signed distance
    For each mesh, the output is saved to a sdf file
    Weikai Chen
*/

#include <chrono> 
#include <map>
#include <utility>

#include <igl/copyleft/marching_cubes.h>
#include <igl/writeOBJ.h>
#include <igl/writePLY.h>

#include "computeLocal3PoleSDF.h"
#include "utilities.h"
#include "OctreeUtilities.h"

using namespace std;
using namespace Eigen;
using namespace igl;

struct CompareVector3d {
  bool operator()(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) const {
    if (p1[0] > p2[0])
      return true;
    else if (p1[0] < p2[0])
      return false;
    else if (p1[1] > p2[1])
      return true;
    else if (p1[1] < p2[1])
      return false;
    else if (p1[2] > p2[2])
      return true;
    else if (p1[2] < p2[2])
      return false;
    
    return false;
  }
};

// Given the octree cells, organize the data into a format like obj mesh, which has a
// matrix store unique vertex cooridnate and the other storing the vertex id of each cell
//
// @param cells: the input octree cells
//               for each cell, pair.first - min corner; pair.second - max corner
// @param verts: the output vertices
// @param faces: the output faces - each face contains 8 vertice ids
void BuildOctreeVertFaceConnection(
    const vector<pair<Vector3d, Vector3d>>& cells,
    vector<Vector3d>& verts,
    vector<vector<int>>& faces) {
  // clear up input verts and faces
  vector<Vector3d>().swap(verts);  // clear vector and release memory
  vector<vector<int>>().swap(faces);

  map<Eigen::Vector3d, int, CompareVector3d> vert_map;
  for (auto cell : cells) {
    const Vector3d& min_corner = cell.first;
    const Vector3d& len = cell.second;
    double x = len[0], y = len[1], z = len[2];
		vector<Vector3d> corner_points;
    corner_points.push_back(min_corner);  // cell minimum corner
		corner_points.push_back(min_corner + Vector3d(0, 0, z));
		corner_points.push_back(min_corner + Vector3d(0, y, 0));
		corner_points.push_back(min_corner + Vector3d(0, y, z));
		corner_points.push_back(min_corner + Vector3d(x, 0, 0));
		corner_points.push_back(min_corner + Vector3d(x, 0, z));
		corner_points.push_back(min_corner + Vector3d(x, y, 0));
		corner_points.push_back(min_corner + Vector3d(x, y, z));  // // cell maximum corner
    vector<int> vert_ids;
    for (int i = 0; i < corner_points.size(); i++) {
      if (vert_map.find(corner_points[i]) ==  vert_map.end()) {
        verts.push_back(corner_points[i]);
        vert_map[corner_points[i]] = verts.size() - 1;  // map the vertice to its id
        vert_ids.push_back(verts.size() - 1);
      } else {
        int vid = vert_map[corner_points[i]];
        vert_ids.push_back(vid);
      }
    }
    faces.push_back(vert_ids);
  }
}

// Given the octree vertice andf faces, compute the vertice to face mapping
//
// @param verts: the input octree vertices
// @param faces: the input octree faces
//
// return: mapping from vertex (id) to the ids of faces that it connects to
map<Vector3d, vector<int>, CompareVector3d> BuildOctreeVertToFaceMapping(
    const vector<Vector3d>& verts,
    const vector<vector<int>>& faces) {
  map<Vector3d, vector<int>, CompareVector3d> vert_to_face;
  for (int i = 0; i < faces.size(); i++) {
    const vector<int>& f = faces[i];
    for (int j = 0; j < f.size(); j++) {
      int vert_id = f[j];
      vert_to_face[verts[vert_id]].push_back(i);
    }
  }
  return vert_to_face;
}

// Perform marching cubes on each octree cells and merge the local meshes into a global one
void LocalizedMarchingCubes(
    string output_obj_name,
    const vector<Vector3d>& verts,
    const vector<vector<int>>& faces,
    const vector<double>& distances) {
  string output_folder = "../output/cells/";
  vector<pair<MatrixXd, MatrixXi>> mesh_parts;
  for (int i = 0; i < faces.size(); i++) {
    VectorXd local_dist(8, 1);
    MatrixXd cell_corners(8, 3);
    bool null_cell = false;
    for (int j = 0; j < 8; j++) {
      local_dist(j) = distances[faces[i][j]];
      if (isnan(local_dist(j))) {
        null_cell = true;
        break;
      }
      cell_corners.row(j) = verts[faces[i][j]].transpose();
    }
    if (null_cell)
      continue;

    MatrixXd points(8, 3);
    for (int j = 0; j < 8; j++) {
      local_dist(j) = distances[faces[i][j]];
      points.row(j) = verts[faces[i][j]].transpose();
    }
    MatrixXd part_verts;
    MatrixXi part_faces;
    local_dist = -1.0 * local_dist;
    igl::copyleft::marching_cubes(local_dist, cell_corners, 2, 2, 2, part_verts, part_faces);

    mesh_parts.push_back(make_pair(part_verts, part_faces));
  }
  MatrixXd total_verts;
  MatrixXi total_faces;
  assembleMeshParts(mesh_parts, total_verts, total_faces);
  save_obj_mesh(output_obj_name, total_verts, total_faces);
  std::cout << "Finished writing the output sampling points into " << output_obj_name << "!" << std::endl;
}

// Generate samples that are on the vertices of the octree
//
// @param objName: the input file name of obj mesh
// @param outSDFName: the file name of the output SDF file
// @param reconObjName: the file name of the output reconstructed mesh
// @param output_pts_name: the file name of the saved sampling points
// @param octreeDepth: octree depth for computing the L3PSDF
// @param flag_write_ply: flag of whether to output the sampling points to PLY file for visualization
// @param flag_recon_obj: flag of whether to reconstruct the mesh from the computed L3PSDF field
// @param flag_write_sdf: flag of whether to output SDF file
void GenerateOctVexSamples(
    string objName,
    string outSDFName,
    string reconObjName,
    string output_pts_name,
    int octreeDepth,
    bool flag_write_ply,
    bool flag_recon_obj,
    bool flag_write_sdf) {

  auto start = std::chrono::high_resolution_clock::now();
  // compute octree cells
  Model_OBJ obj;
  int sucess = obj.Load(objName);
  if (sucess != 0) {
    std::cout << "Failed to load the input mesh! Quit!" << std::endl;
    exit(0);
  }
  vector<pair<Vector3d, Vector3d>> cells;
  MatrixXd cellCornerPts;
  // uncommnet the following line if you are using a specified bounding box
  obj.setBBox(glm::dvec3(-0.65, -1, -0.16), glm::dvec3(0.65, 0.8, 0.16));

  cells = obj.getTreeCells(octreeDepth, cellCornerPts);
  std::cout << "cell number: " << cells.size() << std::endl;

  // extract vertices, faces and vert-to-face mapping from octree cells
  vector<Vector3d> octree_verts;
  vector<vector<int>> octree_faces;
  BuildOctreeVertFaceConnection(cells, octree_verts, octree_faces);
  map<Vector3d, vector<int>, CompareVector3d> vert_to_face = BuildOctreeVertToFaceMapping(octree_verts, octree_faces);

  // load obj mesh and remove the duplicated vertices
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::readOBJ(objName,V,F);
  MatrixXi newF;
  removeIdenticalVertices(F, newF);
  F = newF;

  /* Calculation of the Localized 3-Pole Signed Distance Field (L3PSDF) */
  vector<vector<double>> l3psdf_to_merge(octree_verts.size());  // for each vertex, store its l3psd in all cells that enclose it
  double threshold = -1e10;
  for (int i = 0; i < l3psdf_to_merge.size(); i++) {
    l3psdf_to_merge[i] = vector<double>(8, threshold - 1.0);  // one cell vertex is enclosed by at most 8 cells
  }

  #pragma omp parallel for
  for (int i = 0; i < octree_faces.size(); i++) {
    const vector<int>& face = octree_faces[i];
    assert(face.size() == 8);  // each octree cell should have 8 vertices
    vector<Vector3d> query_points;
    for (int j = 0; j < face.size(); j++) {
      query_points.push_back(octree_verts[face[j]]);
    }
    Vector3d cell_min = octree_verts[face[0]];  // cell minimum corner
    Vector3d cell_max = octree_verts[face[7]];  // cell maximum corner
    Vector3d cell_center = (cell_min + cell_max) / 2.0;
    Vector3d cell_length = cell_max - cell_min;
    vector<double> distances = computeT3PoleDistForPtsInCell(V, F, query_points, make_pair(cell_center, cell_length), i);
    assert(distances.size() == 8);
    for (int j = 0; j < distances.size(); j++) {
      int vid = face[j];
      int pos = 0;
      while (pos < l3psdf_to_merge[vid].size()) {
        if (l3psdf_to_merge[vid][pos] < threshold) {
          l3psdf_to_merge[vid][pos] = distances[j];
          break;
        }
        pos++;
      }
    }
  }

  // merge the computed l3psd value for each cell
  int inside_cnt = 0;
  int outside_cnt = 0;
  int nan_cnt = 0;
  vector<double> final_dist(octree_verts.size(), -1);
  for (int i = 0; i < l3psdf_to_merge.size(); i++) {
    const vector<double>& dists = l3psdf_to_merge[i];
    double sum = 0.0;
    int nan_count = 0;
    int valid_count = 0;
    for (int j = 0; j < dists.size(); j++) {
      if (isnan(dists[j])) {
        nan_count++;
        continue;
      }
      if (dists[j] < threshold)
        continue;
      valid_count++;
      sum += dists[j];
    }
    if (valid_count == 0) {
        final_dist[i] = NAN;
        nan_cnt++;
    } else {
        double final_d = sum / static_cast<double>(valid_count);
        final_dist[i] = final_d;
        if (final_d > 0) {
          outside_cnt++;
        } else {
          inside_cnt++;
        }
    }
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
	std::cout << "Computing L3PSDF at depth " << octreeDepth << " Used time: " << duration.count() / double(1000000.0) << " seconds" << std::endl;

  std::cout << "number of inside : outside : nan = " << inside_cnt << " ： " << outside_cnt << " : " << nan_cnt << std::endl;
  std::cout << "ratio of inside : outside : nan = " << 1.0 << " ： " << double(outside_cnt) / inside_cnt << " : " << double(nan_cnt) / inside_cnt << std::endl;

  if (flag_recon_obj) {
    // truncated L3PSDF
    vector<double> truncated_dist;
    for (int i = 0; i < final_dist.size(); i++) {
      if (isnan(final_dist[i])) {
        truncated_dist.push_back(NAN);
        nan_cnt++;
      } else if (final_dist[i] > 0) {
        truncated_dist.push_back(1.0);
        outside_cnt++;
      } else {
        truncated_dist.push_back(-1.0);
        inside_cnt++;
      }
    }

    // reconstruct the computed L3PSDF
    // MatrixXd GV(grid_size[0]*grid_size[1]*grid_size[2], 3);	    // GV to store grid query points
    // VectorXd finalS(grid_size[0]*grid_size[1]*grid_size[2], 1);  // final vector to stored computed distance values

    // std::cout << "Before initializing GV" << std::endl;
    // for(int x = 0; x< grid_size[0]; x++) {
    //   const double xi = x * min_grid_width + bmin(0);
    //   for(int y = 0; y < grid_size[1]; y++) {
    //     const double yi = y * min_grid_width + bmin(1);
    //     for(int z = 0; z < grid_size[2]; z++) {
    //       const double zi = z * min_grid_width + bmin(2);
    //       GV.row(x+grid_size(0)*(y + grid_size(1)*z)) = RowVector3d(xi,yi,zi);
    //     }
    //   }
    // }
    // MatrixXd SV1;
    // MatrixXi SF1;
    // igl::copyleft::marching_cubes(final_dist, points, grid_size(0), grid_size(1), grid_size(2), SV1, SF1);

    // reconstruct using localized marching cubes
    LocalizedMarchingCubes(reconObjName, octree_verts, octree_faces, final_dist);
    string truncated_obj_name = reconObjName.substr(0, reconObjName.size() - 4);
    // reconstruct truncated field which is the GT for deep learning
    truncated_obj_name += "_truncated.obj";
    LocalizedMarchingCubes(truncated_obj_name, octree_verts, octree_faces, truncated_dist);
  }


  // output the 3D sampling points to PLY
  if (flag_write_ply) {
    MatrixXd points = MatrixXd(octree_verts.size(), 3);
    for (int i = 0; i < octree_verts.size(); i++) {
      points.row(i) = octree_verts[i].transpose();
    }
    MatrixXi tmpF;
    igl::writePLY(output_pts_name, points, tmpF);
    std::cout << "Finished writing the output sampling points into " << output_pts_name << "!" << std::endl;
  }

  if (flag_write_sdf) {
    // write into txt format
    ofstream fout(outSDFName);
    fout << final_dist.size() << std::endl;
    for(int i=0; i < final_dist.size(); ++i) {
      int label = 1;  // outside
      if (isnan(final_dist[i]))
          label = 2;  // nan region
      if (final_dist[i] < 0)
          label = 0;  // inside
      fout << octree_verts[i](0) << " " << octree_verts[i](1) << " " << octree_verts[i](2)
           << " " << label << " " << final_dist[i] << std::endl; 
    }
    fout.close();
    std::cout << "Finished writing the samples into " << outSDFName << "!" << std::endl;
  }
}

int main(int argc, char** argv){
  if (argc < 8){
      std::cout << "usage: ./genOctreeL3PSDFSamples input.obj samples.sdf recon_obj.obj output_samples.ply octree_depth [default=5] flag_writePLY flag_writeOBJ flag_writeSDF" << std::endl;
  }

  string objName = "../data/airplane.obj";
  string outSDFName = "../output/airplane.sdf";
  string reconObjName = "../output/airplane_recon.obj";
  string output_pts_name = "../output/pts_octvex.ply";
  int writePLY = 1;
  int writeOBJ = 1;
  int writeSDF = 0;
  int octreeDepth = 3;
  for(int i = 1; i < argc; ++i){
    if (i == 1)
        objName = argv[i];
    if (i == 2)
        outSDFName = argv[i];
    if (i == 3)
        reconObjName = argv[i];
    if (i == 4)
        output_pts_name = argv[i];
    if (i == 5)
        sscanf(argv[i], "%d", &octreeDepth);
    if (i == 6)
        sscanf(argv[i], "%d", &writePLY);
    if (i == 7)
        sscanf(argv[i], "%d", &writeOBJ);
    if (i == 8)
        sscanf(argv[i], "%d", &writeSDF);
  }


  bool flag_writePLY = true;
  bool flag_recon_obj = true;
  bool flag_writeSDF = true;

  if (writePLY < 1)
    flag_writePLY = false;
  if (writeOBJ < 1)
    flag_recon_obj = false;
  if (writeSDF < 1)
    flag_writeSDF = false;

  std::cout << "input: writePLY: " << writePLY << " writeOBJ: " << writeOBJ << " writeSDF: " << writeSDF << std::endl;

  std::cout << "Current setting: ./genOctreeL3PSDFSamples " << objName << " " << outSDFName << " " << reconObjName << " " << output_pts_name << " "
      << octreeDepth << " " << flag_writePLY  << " " << flag_recon_obj  << " " << flag_writeSDF << std::endl;

  GenerateOctVexSamples(objName, outSDFName, reconObjName, output_pts_name, octreeDepth, flag_writePLY, flag_recon_obj, flag_writeSDF);

  return 1;
}