#include "utilities.h"
// #include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <sstream>
#include <map>


void computeBBox(string bboxObjName, VectorXd& bboxmin, VectorXd& bboxmax) {
  MatrixXd gV;
  MatrixXi gF;
  igl::read_triangle_mesh(bboxObjName, gV, gF);
  VectorXd maxVal = gV.colwise().maxCoeff();
  VectorXd minVal = gV.colwise().minCoeff();
  bboxmin = minVal;
  bboxmax = maxVal;
}

void removeIdenticalVertices(const MatrixXi& F, MatrixXi& outF) {
  vector<int> validFaceID;
  for (int i = 0; i < F.rows(); i++) {
    RowVector3i f = F.row(i);
    map<int, int> faces;
    faces[f[0]] = 1;
    faces[f[1]] = 1;
    faces[f[2]] = 1;
    if (faces.size() == 3)
      validFaceID.push_back(i);
  }

  // cout << "There are " << F.rows() << " faces and  " << validFaceID.size() << " valid ones!" << endl;
  // only keep those valide faces
  outF = MatrixXi(validFaceID.size(), 3);
  outF.fill(0);
  for (int i = 0; i < validFaceID.size(); i++) {
    outF.row(i) = F.row(validFaceID[i]);
  }
}

void assembleMeshParts(const vector<pair<MatrixXd, MatrixXi>>& meshParts, MatrixXd& outV, MatrixXi& outF)
{
    // get total number of vertices and faces
    int totalV = 0, totalF = 0;
    for(auto i : meshParts)
    {
        const MatrixXd& V = i.first;
        const MatrixXi& F = i.second;
        totalV += V.rows();
        totalF += F.rows();
    }

    outV = MatrixXd(totalV, 3);
    outF = MatrixXi(totalF, 3);
    // construct the output mesh
    int Vcnt = 0, Fcnt = 0;
    for(auto it : meshParts)
    {
        const MatrixXd& V = it.first;
        const MatrixXi& F = it.second;
        for (int i = 0; i < V.rows(); i++)
        {
            outV.row(Vcnt + i) = V.row(i);
        }
        for (int i=0; i < F.rows(); i++)
        {
            outF(Fcnt+i,0) = F(i,0) + Vcnt;
            outF(Fcnt+i,1) = F(i,1) + Vcnt;
            outF(Fcnt+i,2) = F(i,2) + Vcnt;
        }

        Vcnt += V.rows();
        Fcnt += F.rows();
    }
}

// customized mesh save function -- automatically remove NAN vertices
void save_obj_mesh(
    string filename,
    const MatrixXd& verts,
    const MatrixXi& faces) {
  ofstream fout (filename);
  if (fout.is_open()) {
    map<int, int> indexMap; // used to filter out NAN vertices
    int idx = 0;
    for(int i=0; i < verts.rows(); ++i) {
        if (isnan(verts.row(i)[0])) {
            indexMap[i] = -1;
        } else {
            indexMap[i] = ++idx;
            fout << "v " << verts.row(i)[0] << " " << verts.row(i)[1] << " " << verts.row(i)[2] << endl;
        }
    }

    for(int i =0; i < faces.rows(); ++i) {
      int vid0 = faces.row(i)[0];
      int vid1 = faces.row(i)[1];
      int vid2 = faces.row(i)[2];

      auto it = indexMap.find(vid0);
      if (it == indexMap.end()) {
          cerr << "Cannot find the vertex in save_obj_mesh function!" << endl;
          system("pause");
      } else {
        if (indexMap[vid0] == -1)
          continue;
      }
      it = indexMap.find(vid1);
      if (it == indexMap.end()) {
          cerr << "Cannot find the vertex in save_obj_mesh function!" << endl;
          system("pause");
      } else {
        if (indexMap[vid1] == -1)
          continue;
      }
      it = indexMap.find(vid2);
      if (it == indexMap.end()) {
          cerr << "Cannot find the vertex in save_obj_mesh function!" << endl;
          system("pause");
      } else {
        if (indexMap[vid2] == -1)
          continue;
      }
      int newvid0 = indexMap[vid0];
      int newvid1 = indexMap[vid1];
      int newvid2 = indexMap[vid2];
      fout << "f " << newvid0 << " " << newvid1 << " " << newvid2 << endl;
    }
    fout.close();
  }
}

void normalizeMesh(string input_obj_name, string output_obj_name) {
  MatrixXd gV;
  MatrixXi gF;
  igl::read_triangle_mesh(input_obj_name, gV, gF);
  VectorXd bboxmax = gV.colwise().maxCoeff();
  VectorXd bboxmin = gV.colwise().minCoeff();
  gV.rowwise() -= bboxmin.transpose();
  VectorXd length = bboxmax - bboxmin;
  double maxLength = length.maxCoeff();
  gV = gV / maxLength;
  VectorXd maxVal = gV.colwise().maxCoeff();
  VectorXd minVal = gV.colwise().minCoeff();
  VectorXd newLength = maxVal - minVal;
  VectorXd half_size = newLength / 2.0;
  gV.rowwise() -= half_size.transpose();
  save_obj_mesh(output_obj_name, gV, gF);
}