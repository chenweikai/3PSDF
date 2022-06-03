#include "compute3PoleSDF.h"
#include "utility_file_read.h"
#include "utilities.h"
#include <experimental/filesystem>
#include <sys/stat.h>
#include <sys/types.h>

string obj_name = "/home/weikai/data/arthub/OBJ/boys_clean/failed/717.obj";
string ReconMeshName = "/home/weikai/Projects/Human3D/output/717_recon.obj";
// string objName = "/home/weikai/Projects/Human3D/data/test.obj";
// string outReconMeshName = "/home/weikai/Projects/Human3D/output/test_recon.obj";
// string obj_name = "/home/weikai/Projects/Human3D/data/scene.obj";
// string ReconMeshName = "/home/weikai/Projects/Human3D/output/scene_recon_mine.obj";
int res = 200;
int batch_size = 300*300*300;

void removeFacesWithIdenticalVerts(string objName, MatrixXd& outV, MatrixXi& outF)
{
    MatrixXd vb;
    MatrixXi vf;
    igl::readOBJ(objName,vb,vf);

    vector<Vector3d> collapseVerts;
    vector<int> validFaces;
    int cnt = 0;
    for(int i=0; i < vf.rows(); i++)
    {
        map<int, int> ind;
        int v0 = vf.row(i)[0];
        int v1 = vf.row(i)[1];
        int v2 = vf.row(i)[2];
        ind[v0] = 1;
        ind[v1] = 1;
        ind[v2] = 1;
        if (ind.size() < 3)
        {
            collapseVerts.push_back(Vector3d(vb.row(v0)));
            collapseVerts.push_back(Vector3d(vb.row(v1)));
            collapseVerts.push_back(Vector3d(vb.row(v2)));
            cnt++;
        }else
        {
            validFaces.push_back(i);
        }
        
    }

    cout << "There are " << cnt << " faces that have identical vertices!" << endl; 
 
    
    reOrderMeshIndices(vb, vf, validFaces, outV, outF);
    // visualize those problematic vertices
    MatrixXd points(collapseVerts.size(), 3);
    for (int i=0; i < collapseVerts.size(); i++)
        points.row(i) = RowVector3d(collapseVerts[i]);
    
    MatrixXd Ccc = RowVector3d(0.4,0.8,0.3).replicate(vf.rows(),1);
	igl::opengl::glfw::Viewer viewerb;
	viewerb.data().set_mesh(vb, vf);
	viewerb.data().set_colors(Ccc);
    // viewerb.data().add_points(points,Eigen::RowVector3d(1,0,0));
	viewerb.launch();

}

void batchRemoveIdenticalVertices(string inDir, string outDir)
{
    vector<string> ObjFileNames;
    getAllFormatFiles(inDir, ObjFileNames, ".obj");
    vector<string> names;
    for(auto s : ObjFileNames)
    {
        cout << "processing " << s << endl;
        string inMeshName = s;
        // extract the folder path and obj file name from the input path
        string folderPath = s.substr(0, s.find_last_of("/\\"));
        // string file = s.substr(s.find_last_of("/\\")+1);
        // file = file.substr(0, file.find_last_of("."));
        string file = s.substr(0, s.find_last_of("/\\"));
        file = file.substr(file.find_last_of("/\\")+1);
        cout << file << endl;

        
        // reconstruct to separate folder
        string targetFoldDir = outDir;
        if (!fs::exists(targetFoldDir)){
            int status = mkdir(targetFoldDir.c_str(), S_IRWXU);
            if (status == 0)
                cout << "Created folder: " << targetFoldDir << "!" << endl;
            else
                cout << "Cannot Creat folder: " << targetFoldDir << "!" << endl;
        }
        
        MatrixXd V; 
        MatrixXi F;
        removeFacesWithIdenticalVerts(s, V, F);
        string reconName =  outDir + "/" + file + ".obj";    
        save_obj_mesh(reconName, V, F);
        cout << "Saved obj to " << reconName << endl;
        
    }    
}


int main(int argc, char** argv)
{
    if (argc < 5)
    {
        cout << "usage: ./generate3PoleSDF input.obj outReconMeshName resulution [default=100] batchSize" << endl;
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
    // MatrixXd V; MatrixXi F;
    // removeFacesWithIdenticalVerts(obj_name, V, F);
    // batchRemoveIdenticalVertices("/home/weikai/data/arthub/OBJ/girls", "/home/weikai/data/arthub/OBJ/girls_clean");
    generate3PoleSDF(obj_name, ReconMeshName, res, batch_size);
}