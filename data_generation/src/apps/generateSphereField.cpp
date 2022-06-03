#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <string>
using namespace Eigen;
using namespace std;

float func(float radius, float x, float y, float z){
    return (x*x + y*y + z*z - radius*radius);
}

// generated a implicit field center at origin with specified reslution and bounding box
void generateImplicitField(int resX, int resY, int resZ, Eigen::Vector3d bmax, Eigen::Vector3d bmin, string outputFileName){
    Vector3d step_size;
    step_size[0] = (bmax[0] - bmin[0]) / float(resX);
    step_size[1] = (bmax[1] - bmin[1]) / float(resY);
    step_size[2] = (bmax[2] - bmin[2]) / float(resZ); 
    // initialize writing file stream
    ofstream outFile;
    outFile.open("sphere.sdf", std::ios::binary);
    int cnt = 0;
    for (size_t ix = 0; ix <= resX; ix++)
    {
        for (size_t iy = 0; iy <= resY; iy++)
        {
            for (size_t iz = 0; iz < resZ; iz++)
            {
                Vector3d pos = Vector3d(bmin[0] + ix * step_size[0], bmin[1] + iy*step_size[1], bmin[2] + iz*step_size[2]);
                float val = func(1.0, pos[0], pos[1], pos[2]);
                outFile << val;
                cnt ++;
            }
            
        }        
    }
    cout << cnt << endl;
    outFile.close();

}

int main(){
    int resX = 99, resY = 99, resZ = 99;
    Vector3d bmin(-1,-1,-1);
    Vector3d bmax(1,1,1);
    string outputFileName = "sphere.sdf";
    generateImplicitField(resX, resY, resZ, bmax, bmin, outputFileName);
}