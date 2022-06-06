/*
    Batch generate 3PSDF using octree-based sampling
*/

#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
namespace fs = std::experimental::filesystem;


int main(int argc, char** argv){

  string dataDir = "/cfs-cq-dcc/weikaichen/3DReconData/OBJ/boy";
  string outSDFDir = "/cfs-cq-dcc/weikaichen/3DReconData/SDF_raw/boy/coat_and_tshirt_depth10";
  string outObjDir = "/cfs-cq-dcc/weikaichen/3DReconData/ReconObj/boy_depth10";
  string testSet = "../data/test.txt";
  string todo_filename = "/cfs-cq-dcc/weikaichen/3DReconData/OBJ/coat_and_tshirt_train.txt";

  int depth = 10;
  int writePLY = 0;
  int writeSDF = 1;
  int writeOBJ = 1;

  if (argc < 9){
      // Note the higher the numOctreeCells is, the kmore samples distributed inside/near the surface and less the outside of surface
      std::cout << "usage: ./batchGenOctL3PSDF inDir outSDFDir outObjDir todo_file depth flag_writeSDF [Default: 1] flag_writeOBJ flag_writePLY" << std::endl;
  }
  
  for(int i = 1; i < argc; ++i){
      if (i == 1)
          dataDir = argv[i];
      if (i == 2)
          outSDFDir = argv[i];  
      if (i == 3)
          outObjDir = argv[i];
      if (i == 4)
          todo_filename = argv[i]; 
      if (i == 5)
          sscanf(argv[i], "%d", &depth);
      if (i == 6)
          sscanf(argv[i], "%d", &writeSDF);
      if (i == 7)
          sscanf(argv[i], "%d", &writeOBJ);
      if (i == 8)
          sscanf(argv[i], "%d", &writePLY);
  }

  std::vector<int> todoList;
  ifstream fin;
  fin.open(todo_filename);
  int idx;
  while (fin >> idx) {
    todoList.push_back(idx);
  }
  std::cout << std::endl;
  
  if (!fs::exists(dataDir)){
    fs::create_directories(dataDir);
    std::cout << "Created directory: " << dataDir << "!" << std::endl;
  }

  if (!fs::exists(outSDFDir)){
    fs::create_directories(outSDFDir);
    std::cout << "Created directory: " << outSDFDir << "!" << std::endl;
  }

  if (!fs::exists(outObjDir)){
    fs::create_directories(outObjDir);
    std::cout << "Created directory: " << outObjDir << "!" << std::endl;
  }


  // process according to the order of the todo list
  for (int i = 0; i < todoList.size(); i++) {
    std::cout << std::endl << "Processing " << i + 1  << "/" << todoList.size() << " ..." << std::endl;
    string id = std::to_string(todoList[i]);
    string objName = dataDir + "/" + id + ".obj";
    std::cout << "Computing " << objName << std::endl;
    if (!fs::exists(dataDir)) {
      std::cout << "Does NOT exist file: " << objName << "!" << std::endl;
      continue;
    }
    string SDFName = outSDFDir + "/" + id + ".sdf";  
    string reconObjName =  outObjDir + "/" + id + ".obj";

    stringstream ss;
    string output_ply_name = outObjDir + "/" + id + ".ply";
    ss << "./gen_3psdf_samples " << objName << " " << SDFName << " " << reconObjName << " " << output_ply_name << " "
         << depth << " " << writePLY << " " << writeOBJ << " " << writeSDF << std::endl;
    string command = ss.str();
    std::cout << "executing command: " << command << std::endl;
    system(command.c_str());
  }
}
