#include "computeLocal3PoleSDF.h"
#include <iostream>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <sys/types.h>

string obj_name = "/home/weikai/Projects/Human3D/data/H_00011.obj";
string ReconMeshName = "/home/weikai/Projects/Human3D/output/H_00011_recon.obj";

int res = 100;
int batch_size = 300*300*300;


int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::cout << "usage: ./generateLocal3PoleSDF input.obj outReconMeshName resulution [default=100] batchSize" << endl;
    }

    for (int i = 1; i < argc; ++i){
        if (i == 1)
            obj_name = argv[i]; 
        if (i == 2)
            ReconMeshName = argv[i];  
        if (i == 3)
            sscanf(argv[i], "%d", &res);  
        if (i == 4)
            sscanf(argv[i], "%d", &batch_size);      
    }
  
    generateThin3PoleSDF(obj_name, ReconMeshName, res, batch_size);
}