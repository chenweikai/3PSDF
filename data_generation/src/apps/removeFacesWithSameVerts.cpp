// Remove non-manifold faces with identical vertices but opposite normals

#include <algorithm>
#include <map>
#include <iostream>
#include <string>
#include <vector>

#include <igl/read_triangle_mesh.h>
#include <igl/writeOBJ.h>

using namespace std;
using namespace Eigen;

static bool compareFaces(const RowVector3i& face1, const RowVector3i& face2)
{
  map<int, int> f1_map;
  map<int, int> f2_map;
  for (int i = 0; i < 3; i++)
    f1_map[face1[i]] = -1;
  for (int i = 0; i < 3; i++)
    f2_map[face2[i]] = -1;
  vector<int> f1;
  vector<int> f2;
  for (auto iter = f1_map.begin(); iter != f1_map.end(); iter++) {
    f1.push_back(iter->first);
  }
  for (auto iter = f2_map.begin(); iter != f2_map.end(); iter++) {
    f2.push_back(iter->first);
  }
  if (f1[0] > f2[0])
    return true;
  else if (f1[0] < f2[0])
    return false;
  else if (f1[1] > f2[1])
    return true;
  else if (f1[1] < f2[1])
    return false;
  else if (f1[2] > f2[2])
    return true;
  else if (f1[2] < f2[2])
    return false;
  
  return false;
}

static bool equalFace(const RowVector3i& face1, const RowVector3i& face2)
{
  map<int, int> f1_map;
  map<int, int> f2_map;
  for (int i = 0; i < 3; i++)
    f1_map[face1[i]] = -1;
  for (int i = 0; i < 3; i++)
    f2_map[face2[i]] = -1;
  vector<int> f1;
  vector<int> f2;
  for (auto iter = f1_map.begin(); iter != f1_map.end(); iter++) {
    f1.push_back(iter->first);
  }
  for (auto iter = f2_map.begin(); iter != f2_map.end(); iter++) {
    f2.push_back(iter->first);
  }
  if ( f1[0] == f2[0] && f1[1] == f2[1] && f1[2] == f2[2])
    return true;
  return false;
}

void RemoveFacesWithSameVerts(
    string input_obj_name,
    string output_obj_name
) {
  MatrixXd in_verts;
  MatrixXi in_faces;
  igl::read_triangle_mesh(input_obj_name, in_verts, in_faces);

  int original_face_num = in_faces.rows();
  vector<RowVector3i> sorted_faces;
  for (int i = 0; i < original_face_num; i++) {
    RowVector3i face = in_faces.row(i);
    sorted_faces.push_back(face);
  }
  std::sort(sorted_faces.begin(), sorted_faces.end(), compareFaces);
  auto unique_end = std::unique(sorted_faces.begin(), sorted_faces.end(), equalFace);
	sorted_faces.erase(unique_end, sorted_faces.end());
  cout << "Previous face number: " << original_face_num << " after face number: " << sorted_faces.size() << endl;
  MatrixXi new_faces(sorted_faces.size(), 3);
  for (int i = 0; i < sorted_faces.size(); i++) {
    new_faces.row(i) = sorted_faces[i];
  }
  igl::writeOBJ(output_obj_name, in_verts, new_faces);
}

int main(int argc, char** argv){
    
  if (argc != 3){
      cout << "usage: ./removeFacesWithSameVerts input.obj output.obj" << endl;
  }
  string input_obj_name;
  string output_obj_name;
  for(int i = 1; i < argc; ++i){
      if (i == 1)
          input_obj_name = argv[i];
      if (i == 2)
          output_obj_name = argv[i];
  }


  cout << "Current setting: ./removeFacesWithSameVerts " << input_obj_name << " " << output_obj_name << endl;

  RemoveFacesWithSameVerts(input_obj_name, output_obj_name);

  return 1;
}