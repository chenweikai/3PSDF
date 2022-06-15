/*
  1) Generate samples on octree cell vertices.
  2) 3PSDF distance computation based on local octree cells.
*/

#include <chrono> 
#include <map>
#include <utility>

#include <igl/copyleft/marching_cubes.h>
#include <igl/writeOBJ.h>
#include <igl/writePLY.h>

#include "compute_3psdf_samples.h"
#include "utilities.h"

namespace fs = std::experimental::filesystem;

using namespace l3psdf;

int main(int argc, char** argv) {
  if (argc < 9) {
      std::cout << "usage: ./gen_3psdf_samples input.obj output.sdf recon_obj.obj output_samples.ply octree_depth flag_writeSDF flag_writeOBJ flag_writePLY" << std::endl;
  }

  std::string objName = "../data/soldier_fight.obj";
  std::string outSDFName = "../output/soldier_fight.sdf";
  std::string reconObjName = "../output/soldier_fight.obj";
  std::string output_pts_name = "../output/soldier_fight.ply";

  std::string output_folder_name = GetFolderName(outSDFName);
  if (!fs::exists(output_folder_name)) {
    fs::create_directories(output_folder_name);
    std::cout << "Created output directory: " << output_folder_name << "!" << std::endl;
  }

  int writePLY = 1;
  int writeOBJ = 1;
  int writeSDF = 1;
  int octreeDepth = 9;
  for(int i = 1; i < argc; ++i) {
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
        sscanf(argv[i], "%d", &writeSDF);
    if (i == 7)
        sscanf(argv[i], "%d", &writeOBJ);
    if (i == 8)
        sscanf(argv[i], "%d", &writePLY);
  }

  bool flag_writeSDF = true;  // flag of whether to save the geneated 3PSDF to a .sdf file
  bool flag_reconObj = true;  // flag of whether to save the reconstructed obj mesh from sampled 3PSDF
  bool flag_writePLY = true;  // flag of whether to dump the sampling points into .ply file

  if (writePLY < 1)
    flag_writePLY = false;
  if (writeOBJ < 1)
    flag_reconObj = false;
  if (writeSDF < 1)
    flag_writeSDF = false;

  std::cout << "Current setting: ./gen_3psdf_samples " << objName << " " << outSDFName << " " << reconObjName << " " << output_pts_name << " "
      << octreeDepth << " " << flag_writeSDF << " " << flag_reconObj << " " << flag_writePLY << std::endl;

  GenerateOctree3psdfSamples(objName, outSDFName, reconObjName, output_pts_name, octreeDepth, flag_writeSDF, flag_reconObj, flag_writePLY);

  return 1;
}
