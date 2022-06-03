#include <iostream>
#include <vector>
#include <string>
#include <experimental/filesystem>
// #include "PQP.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/write_triangle_mesh.h>
#include "computeVisualHull.h"
#include "computeDistField.h"
#include "OctreeUtilities.h"
#define cimg_use_png
#define cimg_use_jpg
#include "CImg.h"
using namespace std;

// string objName = "/home/weikai/data/mixamo_mesh_and_render/Ch02_nonPBR/Ch02_nonPBR.obj";
// string objName = "/home/weikai/Projects/Manifold/build/output.obj";
string objName = "../data/plane.obj";
// string objName = "../data/OBJ/character.obj";
string outObjName = "../output/plane_recon.obj";
string outSDFName = "../output/character.sdf";

string data_dir = "/home/weikai/data/mixamo_mesh_and_render/";
int resolution = 100;
enum SDFMethod{
    SDF_IGL = 0,
    SDF_PQP = 1
};
SDFMethod method = SDF_IGL;
// PQP_Model* m_pqp_model;

int main(int argc, char** argv){
    
    if (argc < 6){
        cout << "usage: ./GEO_PROCESSING input.obj output.obj output.sdf resolution -PQP [or -IGL]" << endl;
    }

    for(int i = 1; i < argc; ++i){
        if (i == 1)
            objName = argv[i];
        if (i == 2)
            outObjName = argv[i];
        if (i == 3)
            outSDFName = argv[i];
        if (i == 4)
            sscanf(argv[i], "%d", &resolution);
        if (i == 5){
            if (strcmp(argv[i], "-PQP") == 0){
                method = SDF_PQP;
            }else if(strcmp(argv[i], "-IGL") == 0){
                method = SDF_IGL;
            }             
        }
            
    }

    cout << "Current setting: ./GEO_PROCESSING " << objName << " " << outObjName << " " << outSDFName 
            << " " << "resolution: " << resolution << endl;

    /* test 3-way distance field computation */

    // generate3WaySDFUsePQP(objName, outObjName, outSDFName, resolution);

    /* End - test 3-way distance field computation */
    
    // batchComputeVisualHull(data_dir, 200);

    // vector<string> imgNames({"/home/weikai/data/mixamo_mesh_and_render/aj/aj_0.png",
    //                         "/home/weikai/data/mixamo_mesh_and_render/aj/aj_1.png",
    //                         "/home/weikai/data/mixamo_mesh_and_render/aj/aj_2.png",
    //                         "/home/weikai/data/mixamo_mesh_and_render/aj/aj_3.png"});
    // string meshName = "/home/weikai/data/mixamo_mesh_and_render/aj/aj.obj";

    // computeVisualHull(imgNames, meshName, 100);


    // extractSil("/home/weikai/Projects/bodyRecon/data/aj_0.png", 
    //     "/home/weikai/Projects/bodyRecon/output/test_sil.png", Vector3i(255,255,255), 10);
    
    /*
        Compute signed distance field
    */
    // if (method == SDF_IGL)
    //     computeSDFUseIGL(objName, outObjName, outSDFName, resolution);
    // else
    //     generateSDFUsePQP(objName, outObjName, outSDFName, resolution);

    // string targetName = "/home/weikai/data/mixamo_mesh_and_render/Ch02_nonPBR/Ch02_nonPBR.obj";
    // projectMeshToMesh(objName, targetName);
     /*
        End Compute signed distance field
    */

    /* 
        testing sign determination
    */
    // Model_OBJ obj;
    // obj.Load(objName);
    
    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
	// // Read in inputs as double precision floating point meshes
  	// igl::readOBJ(objName,V,F);
    // // compute tight bounding box of the input mesh
	// RowVector3d tightBmin = V.colwise().minCoeff();
	// RowVector3d tightBmax = V.colwise().maxCoeff();
	// RowVector3d center = (tightBmax + tightBmin) / 2.0;
	// // enlarge bounding box little bit
	// RowVector3d bmin = center + 1.0 * (tightBmin - center);
	// RowVector3d bmax = center + 1.0 * (tightBmax - center);
	// RowVector3d bboxSize = bmax - bmin;

	// // number of vertices on the largest side
	// const int s = resolution;
	// const double h = (bmax-bmin).maxCoeff()/(double)s;
	// const RowVector3i gridSize = ( s * ((bmax-bmin) / (bmax-bmin).maxCoeff()) ).cast<int>();
	// cout << "Grid resolution: " << gridSize[0] << " " << gridSize[1] << " " << gridSize[2] << endl;
    // MatrixXd GV(gridSize(0)*gridSize(1)*gridSize(2), 3);	// GV to store grid points
    // const auto lerp = [&](const int di, const int d)->double
	// {return bmin(d)+(double)di/(double)(gridSize(d)-1)*(bmax(d)-bmin(d));};
    // for(int x = 0; x< gridSize[0]; x++){
	// 	const double xi = lerp(x, 0);
	// 	for(int y = 0; y < gridSize[1]; y++){
	// 		const double yi = lerp(y, 1);
	// 		for(int z = 0; z < gridSize[2]; z++)
	// 		{	
	// 			const double zi = lerp(z, 2);
	// 			// construct grid points
	// 			GV.row(x+gridSize(0)*(y + gridSize(1)*z)) = RowVector3d(xi,yi,zi);			
	// 		}
	// 	}
	// }

    // // load leg points
    // ifstream fin("/home/weikai/Projects/bodyRecon/data/legPoints.txt");
    // if (!fin.is_open())
    //     cerr << "Faile to open leg points!" << endl;
    // int num;
    // fin >> num;
    // vector<Vector3d> points;
    // for(int i=0; i < num; ++i)
    // {
    //     double x, y, z;
    //     fin >> x >> y >> z; 
    //     Vector3d p(x,y,z);
    //     points.push_back(p);
    // }
    // cout << points[points.size()-1] << endl;


    // // MatrixXd queryPoint = MatrixXd::Zero(1,3);
    // vector<double> signs;
    // set<Octree*> emptyCells = obj.inOrOutTest(GV, signs, points);
    // set<Octree*> emptyCells = obj.BuildTreeGridPts(GV, signs);
    // MatrixXd cellCornerPts = obj.getCellCornerPts(emptyCells);
    // obj.ReconOnGridPts(GV);
    // obj.SaveOBJ(outObjName);

    // MatrixXd cellCornerPts;
    // obj.getTreeCells(resolution, cellCornerPts);
    // MatrixXd cellCornerPts;
    // vector<pair<Vector3d, Vector3d>> cells = obj.getOccupiedCells(resolution, cellCornerPts);
    // obj.displayCells(cells, GV);
    // cells = obj.getOccupiedCells(resolution, cellCornerPts);
    // obj.displayCells(cells, GV);

    // MatrixXi Fv = MatrixXi::Zero(100,3);
    // igl::writePLY("occupiedCell.ply", cellCornerPts, Fv);
    
    // displayMeshAndSamples(V, F, cellCornerPts);
    // displayMeshAndSignedSamples(V,F,GV,signs);
    // set<Octree*> emptyCells = obj.GetEmptyCells(500);
    // displayEmptyOctreeCelss(V, F, emptyCells);
    /* 
    END - testing sign determination
    */

    /* 
    testing importance sampling code 
    */
    // Model_OBJ obj;
    // obj.Load(objName);
    // vector<Vector3d> samples;
    // samples = obj.generateImpSamples(5000, 100000);
    // MatrixXd sample3D(samples.size(),3);
    // for(int i=0; i < samples.size(); ++i)
    // {
    //     sample3D.row(i) = samples[i].transpose();
    // }

    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
	// // Read in inputs as double precision floating point meshes
  	// igl::readOBJ(objName,V,F);
    // displayMeshAndSamples(V,F,sample3D);

    // MatrixXd V(2,3);
    // V.row(0) = RowVector3d(0, 23, 0);
    // V.row(1) = RowVector3d(5, 15, 0);


    // // camera paramters
    // float l = -55;      // left
    // float r = 55;       // right
    // float b = -55;      // bottom
    // float t = 55;       // top
    // float near = 0.1;        // near
    // float far = 10000;  // far

    // // for computing lookat matrix
    // Vector3d eye = Vector3d(0, 0, 2000);
    // Vector3d center = Vector3d(0, 0, 0);
    // Vector3d up = Vector3d(0, 1, 0);

    // // image size
    // int img_height = 500;
    // int img_width = 500;

    // MatrixXi screen = project3DPointsToImage(V, l, r, b, t, near, far, eye, center, up, img_height, img_width);
    // cout << screen << endl;
    // string imageName = "/home/weikai/data/mixamo_mesh_and_render/Andromeda/Andromeda_0.png";
    // // string imageName = "/home/weikai/data/mixamo_mesh_and_render/akai_e_espiritu/akai_e_espiritu_0.png";
    // cout << "Start projecting to image" << endl;
    // CImg<unsigned char> img(imageName.c_str());
    // cout << "Loaded image " << imageName << " image width: " << img.width() << " height: " << img.height() << endl;
    // for(int i=0; i < screen.rows(); i++)
    // {
    //     int x = screen(i,0);
    //     int y = screen(i,1);
    //     if (x < 0 || y < 0 || x >= img_width || y >= img_height)
    //         continue;
    //     cout <<  " height y : " << img_height - y << "width x: " << x  << endl;
    //     cout << int(img(x, img_height-y, 0)) << " " << int(img(x, img_height-y, 1)) << " " << int(img(x, img_height-y, 2)) << endl;
    //     img(x, img_height-y, 0) = 255;
    //     img(x, img_height-y, 1) = 0;
    //     img(x, img_height-y, 2) = 0;
    // }
    // cout << "249 353: " << int(img(249, 353, 0)) << " " << int(img(249, 353, 1)) << " " << int(img(249, 353, 2)) << endl;
    // cout << "249 353: " << (float(img(249, 353, 0)) - 127.5)/127.5 << " " << (float(img(249, 353, 1))-127.5)/127.5 << " " << (float(img(249, 353, 2))-127.5)/127.5 << endl;
    // cout << "272 317: " << int(img(272, 317, 0)) << " " << int(img(272, 317, 1)) << " " << int(img(272, 317, 2)) << endl;
    // cout << "272 317: " << (float(img(272, 317, 0))-127.5)/127.5 << " " << (float(img(272, 317, 1))-127.5)/127.5 << " " << (float(img(272, 317, 2))-127.5)/127.5 << endl;
    // // img.save("test.png");
    // img.display();
    /* 
    End - testing importance sampling code 
    */

    /* test if the generated sampels are correct */
    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
	// // Read in inputs as double precision floating point meshes
    // vector<string> sdfFileNames;
    // getAllFormatFiles("/home/weikai/data/bodyRecon_data/SDF/train", sdfFileNames, ".sdf");

    // for(int i=0; i < sdfFileNames.size(); ++i)
    // {        
    //     // read the SDF file
    //     string sdfName = sdfFileNames[i];
    //     cout << "Loading SDF: " << sdfName << endl;
    //     ifstream fin(sdfName);
    //     int numPoint;
    //     fin >> numPoint;
    //     double x,y,z,dist;
    //     int outside;
    //     vector<Vector3d> inPoints;
    //     vector<Vector3d> outPoints;
    //     // vector<double> distances;
    //     // vector<int> signs;
    //     int row = 0;
    //     while (fin >> x >> y >> z >> outside >> dist)
    //     {
    //         // distances.push_back(dist);
    //         // signs.push_back(outside);
    //         if (outside == 1)
    //             outPoints.push_back(Vector3d(x,y,z));
    //         if (outside == 0)
    //             inPoints.push_back(Vector3d(x,y,z));
    //         row++;
    //     }
    //     assert((outPoints.size() + inPoints.size()) == numPoint);

    //     // visualize the results
    //     string folderPath = sdfName.substr(0, sdfName.find_last_of("/\\"));
    //     folderPath = folderPath.substr(0, folderPath.find_last_of("/\\"));
    //     folderPath = folderPath.substr(0, folderPath.find_last_of("/\\"));
    //     string file = sdfName.substr(sdfName.find_last_of("/\\")+1);
    //     file = file.substr(0, file.find_last_of("."));

    //     objName = folderPath + "/mesh/train/" + file + "/" + file + ".obj";
    //     cout << objName << endl;
    //     igl::readOBJ(objName,V,F);

    //     igl::opengl::glfw::Viewer viewer;
    //     viewer.data().set_mesh(V, F);
    //     MatrixXd pin(inPoints.size(), 3);
    //     MatrixXd pout(outPoints.size(), 3);
    //     for(int i=0; i < inPoints.size(); ++i)
    //     {
    //         pin.row(i) = inPoints[i].transpose();
    //     }
    //     for(int i=0; i < outPoints.size(); ++i)
    //     {
    //         pout.row(i) = outPoints[i].transpose();
    //     }
    //     viewer.data().add_points(pin, RowVector3d(1,0,0));
    //     viewer.data().add_points(pout, RowVector3d(0,1,0));
    //     viewer.launch();
    // }

    

    /* End - test if the generated sampels are correct */
}