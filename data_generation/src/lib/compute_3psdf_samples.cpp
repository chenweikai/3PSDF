#include "compute_3psdf_samples.h"

#include <chrono> 
#include <map>
#include <utility>

#include <igl/copyleft/marching_cubes.h>
#include <igl/writeOBJ.h>
#include <igl/writePLY.h>

#include "dist_compute.h"
#include "utilities.h"
#include "OctreeUtilities.h"

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
// @param faces: the output faces, each face contains 8 vertice ids
void BuildOctreeVertFaceConnection(
    const std::vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& cells,
    std::vector<Eigen::Vector3d>& verts,
    std::vector<std::vector<int>>& faces) {
  // clear up input verts and faces
  std::vector<Eigen::Vector3d>().swap(verts);  // clear std::vector and release memory
  std::vector<std::vector<int>>().swap(faces);

  map<Eigen::Vector3d, int, CompareVector3d> vert_map;
  for (auto cell : cells) {
    const Eigen::Vector3d& min_corner = cell.first;
    const Eigen::Vector3d& len = cell.second;
    double x = len[0], y = len[1], z = len[2];
		std::vector<Eigen::Vector3d> corner_points;
    corner_points.push_back(min_corner);  // cell minimum corner
		corner_points.push_back(min_corner + Eigen::Vector3d(0, 0, z));
		corner_points.push_back(min_corner + Eigen::Vector3d(0, y, 0));
		corner_points.push_back(min_corner + Eigen::Vector3d(0, y, z));
		corner_points.push_back(min_corner + Eigen::Vector3d(x, 0, 0));
		corner_points.push_back(min_corner + Eigen::Vector3d(x, 0, z));
		corner_points.push_back(min_corner + Eigen::Vector3d(x, y, 0));
		corner_points.push_back(min_corner + Eigen::Vector3d(x, y, z));  // cell maximum corner
    std::vector<int> vert_ids;
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
map<Eigen::Vector3d, std::vector<int>, CompareVector3d> BuildOctreeVert2FaceMap(
    const std::vector<Eigen::Vector3d>& verts,
    const std::vector<std::vector<int>>& faces) {
  map<Eigen::Vector3d, std::vector<int>, CompareVector3d> vert_to_face;
  for (int i = 0; i < faces.size(); i++) {
    const std::vector<int>& f = faces[i];
    for (int j = 0; j < f.size(); j++) {
      int vert_id = f[j];
      vert_to_face[verts[vert_id]].push_back(i);
    }
  }
  return vert_to_face;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXi>  localized_marching_cubes(
    const std::vector<Eigen::Vector3d>& octree_verts,
    const std::vector<std::vector<int>>& octree_faces,
    const std::vector<double>& distances) {
  std::vector<pair<Eigen::MatrixXd, Eigen::MatrixXi>> mesh_parts;
  for (int i = 0; i < octree_faces.size(); i++) {
    Eigen::VectorXd local_dist(8, 1);
    Eigen::MatrixXd cell_corners(8, 3);
    bool null_cell = false;
    for (int j = 0; j < 8; j++) {
      local_dist(j) = distances[octree_faces[i][j]];
      if (isnan(local_dist(j))) {
        null_cell = true;
        break;
      }
      cell_corners.row(j) = octree_verts[octree_faces[i][j]].transpose();
    }
    if (null_cell)
      continue;

    Eigen::MatrixXd points(8, 3);
    for (int j = 0; j < 8; j++) {
      local_dist(j) = distances[octree_faces[i][j]];
      points.row(j) = octree_verts[octree_faces[i][j]].transpose();
    }
    Eigen::MatrixXd part_verts;
    Eigen::MatrixXi part_faces;
    local_dist = -1.0 * local_dist;
    igl::copyleft::marching_cubes(local_dist, cell_corners, 2, 2, 2, part_verts, part_faces);

    mesh_parts.push_back(make_pair(part_verts, part_faces));
  }
  Eigen::MatrixXd total_verts;
  Eigen::MatrixXi total_faces;
  AssembleMeshParts(mesh_parts, total_verts, total_faces);

  return std::make_pair(total_verts, total_faces);
}

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
    std::string input_obj_name,
    std::string out_sdf_name,
    std::string recon_obj_name,
    std::string output_pts_name,
    int octree_depth,
    bool flag_write_ply,
    bool flag_recon_obj,
    bool flag_write_sdf) {
  auto start = std::chrono::high_resolution_clock::now();

  // compute octree cells
  Model_OBJ obj;
  int sucess = obj.Load(input_obj_name);
  if (sucess != 0) {
    std::cout << "Failed to load the input mesh! Quit!" << std::endl;
    exit(0);
  }

  std::vector<pair<Eigen::Vector3d, Eigen::Vector3d>> cells;
  Eigen::MatrixXd cellCornerPts;
  // uncommnet the following line if you are using a specified bounding box
  // obj.setBBox(glm::dvec3(-0.65, -1, -0.16), glm::dvec3(0.65, 0.8, 0.16));
  cells = obj.GetTreeCells(octree_depth, cellCornerPts);
  std::cout << "cell number: " << cells.size() << std::endl;

  // extract vertices, faces and vert-to-face mapping from octree cells
  std::vector<Eigen::Vector3d> octree_verts;
  std::vector<std::vector<int>> octree_faces;
  BuildOctreeVertFaceConnection(cells, octree_verts, octree_faces);
  map<Eigen::Vector3d, std::vector<int>, CompareVector3d> vert_to_face = BuildOctreeVert2FaceMap(octree_verts, octree_faces);

  // load obj mesh and remove the duplicated vertices
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::readOBJ(input_obj_name,V,F);
  Eigen::MatrixXi newF;
  RemoveIdenticalVerts(F, newF);
  F = newF;

  /* Calculation of the Localized 3-Pole Signed Distance Field (L3PSDF) */
  std::vector<std::vector<double>> l3psdf_to_merge(octree_verts.size());  // for each vertex, store its l3psd in all cells that enclose it
  double threshold = -1e10;
  for (int i = 0; i < l3psdf_to_merge.size(); i++) {
    l3psdf_to_merge[i] = std::vector<double>(8, threshold - 1.0);  // one cell vertex is enclosed by at most 8 cells
  }

  #pragma omp parallel for
  for (int i = 0; i < octree_faces.size(); i++) {
    const std::vector<int>& face = octree_faces[i];
    assert(face.size() == 8);  // each octree cell should have 8 vertices
    std::vector<Eigen::Vector3d> query_points;
    for (int j = 0; j < face.size(); j++) {
      query_points.push_back(octree_verts[face[j]]);
    }
    Eigen::Vector3d cell_min = octree_verts[face[0]];  // cell minimum corner
    Eigen::Vector3d cell_max = octree_verts[face[7]];  // cell maximum corner
    Eigen::Vector3d cell_center = (cell_min + cell_max) / 2.0;
    Eigen::Vector3d cell_length = cell_max - cell_min;
    std::vector<double> distances = Compute3psdfPerCell(V, F, query_points, make_pair(cell_center, cell_length), i);
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
  std::vector<double> final_dist(octree_verts.size(), -1);
  for (int i = 0; i < l3psdf_to_merge.size(); i++) {
    const std::vector<double>& dists = l3psdf_to_merge[i];
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
  std::cout << "Computing L3PSDF at depth " << octree_depth << " Used time: " << duration.count() / double(1000000.0) << " seconds" << std::endl;
  std::cout << "number of inside : outside : nan = " << inside_cnt << " ： " << outside_cnt << " : " << nan_cnt << std::endl;
  std::cout << "ratio of inside : outside : nan = " << 1.0 << " ： " << double(outside_cnt) / inside_cnt << " : " << double(nan_cnt) / inside_cnt << std::endl;

  if (flag_recon_obj) {
    // truncated L3PSDF
    std::vector<double> truncated_dist;
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

    // reconstruct using localized marching cubes
    auto recon_result = localized_marching_cubes(octree_verts, octree_faces, final_dist);
    SaveObjMesh(recon_obj_name, recon_result.first, recon_result.second);
    std::cout << "Finished writing the reconstructed mesh into " << recon_obj_name << "!" << std::endl;

    std::string truncated_obj_name = recon_obj_name.substr(0, recon_obj_name.size() - 4);
    // reconstruct truncated field which is the GT for deep learning
    truncated_obj_name += "_truncated.obj";
    auto truncated_recon_result = localized_marching_cubes(octree_verts, octree_faces, truncated_dist);
    SaveObjMesh(truncated_obj_name, recon_result.first, recon_result.second);
    std::cout << "Finished writing the reconstructed mesh into " << truncated_obj_name << "!" << std::endl;
  }

  // output the 3D sampling points to PLY
  if (flag_write_ply) {
    Eigen::MatrixXd points = Eigen::MatrixXd(octree_verts.size(), 3);
    for (int i = 0; i < octree_verts.size(); i++) {
      points.row(i) = octree_verts[i].transpose();
    }
    Eigen::MatrixXi tmpF;
    igl::writePLY(output_pts_name, points, tmpF);
    std::cout << "Finished writing the output sampling points into " << output_pts_name << "!" << std::endl;
  }

  // output the generated 3-pole signed distance field into a .sdf file
  if (flag_write_sdf) {
    // write into txt format
    ofstream fout(out_sdf_name);
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
    std::cout << "Finished writing the samples into " << out_sdf_name << "!" << std::endl;
  }
}
