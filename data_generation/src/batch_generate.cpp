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
  string input_data_dir = "../data";
  string out_sdf_dir = "../output/SDF";
  string out_recon_obj_dir = "../output/OBJ";
  string out_recon_ply_dir = "../output/PLY";
  string todo_filename = "../data/todo.txt";  // a txt file containing the names of the subjects to be computed

  int octree_depth = 9;
  int write_ply = 0;
  int write_sdf = 1;
  int write_obj = 1;

  if (argc < 9){
    std::cout << "usage: ./batch_generate todo_list.txt inDir outSDFDir outObjDir outPLYDir octree_depth flag_writeSDF [Default: 1] flag_writeOBJ [Default: 1] flag_writePLY [Default: 0]" << std::endl;
  }
  
  for(int i = 1; i < argc; ++i){
    if (i == 1)
        todo_filename = argv[i];
    if (i == 2)
        input_data_dir = argv[i];
    if (i == 3)
        out_sdf_dir = argv[i];
    if (i == 4)
        out_recon_obj_dir = argv[i];
    if (i == 5)
        out_recon_ply_dir = argv[i];
    if (i == 6)
        sscanf(argv[i], "%d", &octree_depth);
    if (i == 7)
        sscanf(argv[i], "%d", &write_sdf);
    if (i == 8)
        sscanf(argv[i], "%d", &write_obj);
    if (i == 9)
        sscanf(argv[i], "%d", &write_ply);
  }

  std::vector<string> todo_list;
  ifstream fin;
  fin.open(todo_filename);
  string name;
  while (fin >> name) {
    todo_list.push_back(name);
  }
  std::cout << std::endl;
  
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
    string subject = todo_list[i];
    string obj_name = input_data_dir + "/" + subject + ".obj";
    std::cout << "Computing " << obj_name << std::endl;
    if (!fs::exists(obj_name)) {
      std::cout << "Does NOT exist file: " << obj_name << "!" << std::endl;
      continue;
    }

    string sdf_name = out_sdf_dir + "/" + subject + ".sdf";  
    string recon_obj_name =  out_recon_obj_dir + "/" + subject + ".obj";
    string output_ply_name = out_recon_obj_dir + "/" + subject + ".ply";

    stringstream ss;
    ss << "./gen_3psdf_samples " << obj_name << " " << sdf_name << " " << recon_obj_name << " " << output_ply_name << " "
         << octree_depth << " " << write_sdf << " " << write_obj << " " << write_ply << std::endl;
    string command = ss.str();
    std::cout << "executing command: " << command << std::endl;
    system(command.c_str());
  }
}
