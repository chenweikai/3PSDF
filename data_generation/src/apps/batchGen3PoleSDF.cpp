/*
    Batch compute SDF using winding number
    Weikai Chen
*/
#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h> 
#include <stdio.h> 
#include <sstream>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include "utility_file_read.h"
using namespace std;

// string dataDir = "/home/weikai/data/mixamo_error";
string dataDir = "/home/weikai/data/arthub/OBJ/boys_clean";
// string dataDir = "/home/weikai/data/bodyRecon_data/train";
string outDir = "/home/weikai/data/arthub/Recon_test/boys_clean";
string objName = "../data/OBJ/character.obj";
string outObjName = "../output/character_recon.obj";
string outSDFName = "../output/character.sdf";

int resolution = 100;
int batchSize = 100;

int main(int argc, char** argv){

    if (argc < 5){
        // Note the higher the numOctreeCells is, the more samples distributed inside/near the surface and less the outside of surface
        cout << "usage: ./batchGen3PoleSDF inputDir outputDir resolution batchSize" << endl;
    }

    for(int i = 1; i < argc; ++i){
        if (i == 1)
            dataDir = argv[i];
        if (i == 2)
            outDir = argv[i];   
        if (i == 3)
            sscanf(argv[i], "%d", &resolution); 
        if (i == 4)
            sscanf(argv[i], "%d", &batchSize);   
    }

    if (!fs::exists(dataDir)){
         cout << "Does not exist path: " << dataDir << "!" << endl;
         return 1;
    }
       
    vector<string> ObjFileNames;
    getAllFormatFiles(dataDir, ObjFileNames, ".obj");
    vector<string> names;
    for(auto s : ObjFileNames)
    {
        cout << "processing " << s << endl;
        string inMeshName = s;
        // extract the folder path and obj file name from the input path
        string folderPath = s.substr(0, s.find_last_of("/\\"));
        string file = s.substr(s.find_last_of("/\\")+1);
        file = file.substr(0, file.find_last_of("."));

        
        // reconstruct to separate folder
        string targetFoldDir = outDir;
        if (!fs::exists(targetFoldDir)){
            int status = mkdir(targetFoldDir.c_str(), S_IRWXU);
            if (status == 0)
                cout << "Created folder: " << targetFoldDir << "!" << endl;
            else
                cout << "Cannot Creat folder: " << targetFoldDir << "!" << endl;
        }
          
        string reconName =  outDir + "/" + file + ".obj";    


        // // reconstruct to the same folder
        // string SDFName = folderPath + "/" + file + ".sdf";   
        // string reconName =  outDir + "/" + file + "_recon.obj"; 
        stringstream ss;
        ss << "./generate3PoleSDF " << inMeshName << " " << reconName
             << " " << resolution << " " << batchSize*batchSize*batchSize << endl;
        string command = ss.str();
        system(command.c_str());
    }    
  
}