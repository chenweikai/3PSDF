// utility functions

#include "utilities.h"

#include <fstream>
#include <map>
#include <sstream>

void ComputeBbox(std::string input_obj_name, Eigen::VectorXd& bbox_min, Eigen::VectorXd& bbox_max) {
  Eigen::MatrixXd verts;
  Eigen::MatrixXi faces;
  igl::read_triangle_mesh(input_obj_name, verts, faces);
  Eigen::VectorXd max_val = verts.colwise().maxCoeff();
  Eigen::VectorXd minVal = verts.colwise().minCoeff();
  bbox_min = minVal;
  bbox_max = max_val;
}

std::string GetFolderName(const std::string& file_name){
  size_t found;
  found = file_name.find_last_of("/\\");
  return file_name.substr(0, found);
}

void RemoveIdenticalVerts(const Eigen::MatrixXi& input_faces, Eigen::MatrixXi& output_faces) {
  std::vector<int> valid_face_id;
  for (int i = 0; i < input_faces.rows(); i++) {
    Eigen::RowVector3i f = input_faces.row(i);
    std::map<int, int> faces;
    faces[f[0]] = 1;
    faces[f[1]] = 1;
    faces[f[2]] = 1;
    if (faces.size() == 3)
      valid_face_id.push_back(i);
  }

  // only keep those valide faces
  output_faces = Eigen::MatrixXi(valid_face_id.size(), 3);
  output_faces.fill(0);
  for (int i = 0; i < valid_face_id.size(); i++) {
    output_faces.row(i) = input_faces.row(valid_face_id[i]);
  }
}

void ReorderMeshIndices(
    const Eigen::MatrixXd& all_verts, const Eigen::MatrixXi& all_faces,
    const std::vector<int>& part_faces, Eigen::MatrixXd& out_verts, Eigen::MatrixXi& out_faces) {
  std::map<int, int> old_to_New; // mapp from old index to new index in new mesh
  for(int i = 0; i < part_faces.size(); ++i) {
    int fid = part_faces[i];
    old_to_New[all_faces(fid,0)] = -1;
    old_to_New[all_faces(fid,1)] = -1;
    old_to_New[all_faces(fid,2)] = -1;
  }
  // only keep selected vertices and update vertex index
  out_verts = Eigen::MatrixXd(old_to_New.size(), 3);
  int cnt = 0;
  for (auto iter = old_to_New.begin(); iter != old_to_New.end(); iter++) {
    out_verts.row(cnt) = all_verts.row(iter->first);
    iter->second = cnt++;
  }
  // update face index
  out_faces = Eigen::MatrixXi(part_faces.size(), 3);
  for(int i = 0; i < part_faces.size(); ++i) {
    int fid = part_faces[i];
    out_faces(i, 0) = old_to_New[all_faces(fid, 0)];
    out_faces(i, 1) = old_to_New[all_faces(fid, 1)];
    out_faces(i, 2) = old_to_New[all_faces(fid, 2)];
  }
}

void AssembleMeshParts(
    const std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>>& mesh_parts,
    Eigen::MatrixXd& out_verts, Eigen::MatrixXi& out_faces) {
  // get total number of vertices and faces
  int vert_number = 0, face_number = 0;
  for(auto part : mesh_parts) {
    const Eigen::MatrixXd& verts = part.first;
    const Eigen::MatrixXi& faces = part.second;
    vert_number += verts.rows();
    face_number += faces.rows();
  }

  out_verts = Eigen::MatrixXd(vert_number, 3);
  out_faces = Eigen::MatrixXi(face_number, 3);
  // construct the output mesh
  int vert_cnt = 0, face_cnt = 0;
  for(auto part : mesh_parts) {
    const Eigen::MatrixXd& cur_verts = part.first;
    const Eigen::MatrixXi& cur_faces = part.second;
    for (int i = 0; i < cur_verts.rows(); i++) {
      out_verts.row(vert_cnt + i) = cur_verts.row(i);
    }
    for (int i=0; i < cur_faces.rows(); i++) {
      out_faces(face_cnt + i, 0) = cur_faces(i, 0) + vert_cnt;
      out_faces(face_cnt + i, 1) = cur_faces(i, 1) + vert_cnt;
      out_faces(face_cnt + i, 2) = cur_faces(i, 2) + vert_cnt;
    }

    vert_cnt += cur_verts.rows();
    face_cnt += cur_faces.rows();
  }
}

void SaveObjMesh(
    std::string filename,
    const Eigen::MatrixXd& verts,
    const Eigen::MatrixXi& faces) {
  std::ofstream fout (filename);
  if (fout.is_open()) {
    std::map<int, int> indexMap; // used to filter out NAN vertices
    int idx = 0;
    for(int i = 0; i < verts.rows(); ++i) {
      if (isnan(verts.row(i)[0])) {
        indexMap[i] = -1;
      } else {
        indexMap[i] = ++idx;
        fout << "v " << verts.row(i)[0] << " " << verts.row(i)[1] << " " << verts.row(i)[2] << std::endl;
      }
    }

    for(int i = 0; i < faces.rows(); ++i) {
      int vid0 = faces.row(i)[0];
      int vid1 = faces.row(i)[1];
      int vid2 = faces.row(i)[2];

      auto it = indexMap.find(vid0);
      if (it == indexMap.end()) {
        std::cerr << "Cannot find the vertex in save_obj_mesh function!" << std::endl;
      } else {
        if (indexMap[vid0] == -1)
          continue;
      }
      it = indexMap.find(vid1);
      if (it == indexMap.end()) {
        std::cerr << "Cannot find the vertex in save_obj_mesh function!" << std::endl;
      } else {
        if (indexMap[vid1] == -1)
          continue;
      }
      it = indexMap.find(vid2);
      if (it == indexMap.end()) {
        std::cerr << "Cannot find the vertex in save_obj_mesh function!" << std::endl;
      } else {
        if (indexMap[vid2] == -1)
          continue;
      }
      int newvid0 = indexMap[vid0];
      int newvid1 = indexMap[vid1];
      int newvid2 = indexMap[vid2];
      fout << "f " << newvid0 << " " << newvid1 << " " << newvid2 << std::endl;
    }
    fout.close();
  }
}
