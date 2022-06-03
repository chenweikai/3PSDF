/*
    Batch generate L3PSDF using octree based sampling
    Only performed on original mesh
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdlib.h> 
#include <stdio.h> 
#include <sstream>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
using namespace std;

namespace fs = std::experimental::filesystem;



vector<string> loadNames(string fileName){
    ifstream fin(fileName);
    if (!fin.is_open())
        cout << "Cannot open " << fileName << "!" << endl;

    int num;
    fin >> num;
    string name;
    vector<string> outputNames;
    while (fin >> name)
    {
        outputNames.push_back(name);
        cout << name << endl;
    }

    return outputNames;
}

int main(int argc, char** argv){

  // string dataDir = "/home/weikai/data/arthub/corrected_3Views_and_mesh/OBJ/girl";
  string dataDir = "/cfs-cq-dcc/weikaichen/3DReconData/OBJ/boy";
  // string dataDir = "/home/weikai/data/arthub/corrected_3Views_and_mesh/OBJ/boy/used_for_now";
  // string outSDFDir = "/home/weikai/data/arthub/corrected_3Views_and_mesh/SDF_raw/girl";
  string outSDFDir = "/cfs-cq-dcc/weikaichen/3DReconData/SDF_raw/boy/coat_and_tshirt_depth10";
  // string outSDFDir = "/home/weikai/data/arthub/corrected_3Views_and_mesh/L3PSD_whole_100";
  // string outObjDir = "/home/weikai/data/arthub/corrected_3Views_and_mesh/ReconObj/girl/depth6";
  string outObjDir = "/cfs-cq-dcc/weikaichen/3DReconData/ReconObj/boy_depth10";
  // string outObjDir = "/data2/weikaichen/L3PSD_whole_1000/boy";
  // string outObjDir = "/home/weikai/data/arthub/corrected_3Views_and_mesh/OBJ/selected/recon";
  string testSet = "../data/test.txt";
  string todo_filename = "/cfs-cq-dcc/weikaichen/3DReconData/OBJ/coat_and_tshirt_train.txt";
  // string todo_filename = "/home/weikai/data/arthub/corrected_3Views_and_mesh/OBJ/selected/todo.txt";
  // string todo_filename = "/home/weikai/data/arthub/corrected_3Views_and_mesh/IMG/girl_front_views/TODO1.txt";
  string existingDir = "/cfs-cq-dcc/weikaichen/3DReconData/SDF_raw/boy/coat_and_tshirt_depth10";
  // string existingDir = outSDFDir;

  int depth = 10;

  int writePLY = 0;
  int writeSDF = 1;
  int writeOBJ = 1;

  if (argc < 9){
      // Note the higher the numOctreeCells is, the kmore samples distributed inside/near the surface and less the outside of surface
      cout << "usage: ./batchGenOctL3PSDF inDir outSDFDir outObjDir todo_file depth flag_writeSDF [Default: 1] flag_writeOBJ flag_writePLY" << endl;
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

  vector<int> todoList;
  ifstream fin;
  fin.open(todo_filename);
  int a;
  cout << "Todos: ";
  while (fin >> a)
  {
    todoList.push_back(a);
    cout << a << " ";
  }
  cout << endl;
  
  if (!fs::exists(dataDir)){
    cout << "Does not exist path: " << dataDir << "!" << endl;
    fs::create_directories(dataDir);
  }

  if (!fs::exists(outSDFDir)){
    cout << "Does not exist path: " << outSDFDir << "!" << endl;
    fs::create_directories(outSDFDir);
  }

  if (!fs::exists(outObjDir)){
    cout << "Does not exist path: " << outObjDir << "!" << endl;
    fs::create_directories(outObjDir);
  }

  // if (!fs::exists(existingDir)){
  //   cout << "Does not exist path: " << existingDir << "!" << endl;
  //   fs::create_directories(existingDir);
  // }

  // process according to the order of the todo list
  for (int i = 0; i < todoList.size(); i++) {
    cout << endl << "Processing " << i + 1  << "/" << todoList.size() << " ..." << endl;
    string id = std::to_string(todoList[i]);
    string objName = dataDir + "/" + id + ".obj";
    cout << "Computing " << objName << endl;
    if (!fs::exists(dataDir)) {
      cout << "Does NOT exist file: " << objName << "!" << endl;
      continue;
    }
    string SDFName = outSDFDir + "/" + id + ".sdf";  
    string reconObjName =  outObjDir + "/" + id + ".obj";
    string existSDFName = existingDir + "/" + id + ".sdf";
    if (fs::exists(existSDFName)) {
        cout << "Already computed! Skip!" << endl;
        continue;
    }
    stringstream ss;
    // ss << "./genSamplesLocal3Pole " << objName << " " << SDFName << " " << reconObjName << " " << regularRes << " " << writeSDF << endl;
    string output_ply_name = outObjDir + "/" + id + ".ply";
    ss << "./genOctreeL3PSDFSamples " << objName << " " << SDFName << " " << reconObjName << " " << output_ply_name << " "
         << depth << " " << writePLY << " " << writeOBJ << " " << writeSDF << endl;
    string command = ss.str();
    cout << "executing command: " << command << endl;
    system(command.c_str());
  }

}