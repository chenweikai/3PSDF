/*
  Batch generate 3PSDF using octree-based sampling
*/

#include <stdlib.h>

#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "compute_3psdf_samples.h"

namespace fs = std::experimental::filesystem;

using namespace l3psdf;

int main(int argc, char** argv) {
  std::string input_data_dir = "../data";
  std::string out_sdf_dir = "../output/SDF";
  std::string out_recon_obj_dir = "../output/OBJ";
  std::string out_recon_ply_dir = "../output/PLY";
  std::string todo_filename = "../data/todo.txt";  // a txt file containing the names of the subjects to be computed
  std::vector<std::string> todo_list;

  int octree_depth = 9;
  int write_ply = 0;
  int write_sdf = 1;
  int write_obj = 1;

  if (argc < 9) {
    std::cout << "usage: ./batch_generate input_mesh_dir out_sdf_dir out_recon_obj_dir out_ply_dir octree_depth flag_writeSDF [Default: 1] flag_writeOBJ [Default: 1] flag_writePLY [Default: 0] [todo_list.txt (optional)]" << std::endl;
    std::cout << "Insufficient input argument. Please refer to the usage info for more details. Currently run the default setting for illustration!" << std::endl;
  } else {

    input_data_dir = std::string(argv[1]);
    out_sdf_dir = std::string(argv[2]);
    out_recon_obj_dir = std::string(argv[3]);
    out_recon_ply_dir = std::string(argv[4]);
    octree_depth = atoi(argv[5]);
    write_sdf = atoi(argv[6]);
    write_obj = atoi(argv[7]);
    write_ply = atoi(argv[8]);


    std::ifstream fin;
    fin.open(todo_filename);
    std::string name;
    while (fin >> name) {
      todo_list.push_back(name);
    }
    std::cout << std::endl;
  }

  if (!fs::exists(input_data_dir)) {
    std::cout << "Cannot find the input data directory!" << std::endl;
    std::exit(0);
  }

  if (!fs::exists(todo_filename)) {
    std::cout << "Cannot find the todo list for batch computing!" << std::endl;
    std::exit(0);
  }

  if (!fs::exists(out_sdf_dir)) {
    fs::create_directories(out_sdf_dir);
    std::cout << "Created directory: " << out_sdf_dir << "!" << std::endl;
  }

  if (!fs::exists(out_recon_obj_dir)) {
    fs::create_directories(out_recon_obj_dir);
    std::cout << "Created directory: " << out_recon_obj_dir << "!" << std::endl;
  }

  if (!fs::exists(out_recon_ply_dir)) {
    fs::create_directories(out_recon_ply_dir);
    std::cout << "Created directory: " << out_recon_ply_dir << "!" << std::endl;
  }

  // process according to the todo list
  for (int i = 0; i < todo_list.size(); i++) {
    std::cout << std::endl << "Processing " << i + 1  << "/" << todo_list.size() << " ..." << std::endl;
    std::string subject = todo_list[i];
    std::string input_obj_name = input_data_dir + "/" + subject + ".obj";
    std::cout << "Computing " << input_obj_name << std::endl;
    if (!fs::exists(input_obj_name)) {
      std::cout << "Does NOT exist file: " << input_obj_name << "! Skip!" << std::endl;
      continue;
    }

    std::string sdf_name = out_sdf_dir + "/" + subject + ".sdf";  
    std::string recon_obj_name =  out_recon_obj_dir + "/" + subject + ".obj";
    std::string output_ply_name = out_recon_obj_dir + "/" + subject + ".ply";

    GenerateOctree3psdfSamples(input_obj_name, sdf_name, recon_obj_name, output_ply_name, octree_depth, write_ply, write_obj, write_sdf);
  }

  return 1;
}
