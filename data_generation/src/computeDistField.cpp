#include "computeDistField.h"

using namespace std::chrono; 
using namespace Eigen;
using namespace igl;

// extern PQP_Model* m_pqp_model;
igl::opengl::glfw::Viewer viewer;


string vec2String(const RowVector3d& v){
	ostringstream strs;
	strs << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
	return strs.str();
}

void projectMeshToMesh(string sourceMeshName, string targetMeshName)
{
	// PQP_Model* ptr1 = new PQP_Model();
	PQP_Model* ptr2 = new PQP_Model();
	// buildPQP(ptr1, sourceMeshName);
	buildPQP(ptr2, targetMeshName);
	MatrixXd V1, V2;
	MatrixXi F1, F2;
	igl::read_triangle_mesh(sourceMeshName, V1, F1);
	igl::read_triangle_mesh(targetMeshName, V2, F2);
	int step_times = 100;
	double step_ratio = 0.1;
	for (size_t k = 0; k < step_times; k++)
	{
		cout << "step " << k << " ... " << endl;
		if (k < 50)
			step_ratio = 0.1;
		if (k> 50 && k < 80)
			step_ratio = 0.3;
		if (k > 80)
			step_ratio = 0.5;
		cout << "step " << k << "  step ratio: " << step_ratio << endl;
		for(int i=0; i < V1.rows(); i++)
		{
			Vector3d p = V1.row(i);
			Vector3d nearestPt;
			int closestTriID;
			double dist = PQPABSDist(ptr2, p, nearestPt, closestTriID);
			Vector3d dir = nearestPt - p;
			V1.row(i) = p + dir * step_ratio;
			// // for debug only - visualization
			// igl::opengl::glfw::Viewer viewer;
			// viewer.data().set_mesh(V2, F2);
			// cout << "load mesh. " << endl;
			// MatrixXd tmp1(1, 3);
			// tmp1.row(0) = p.transpose();
			// viewer.data().add_points(tmp1, RowVector3d(1, 0, 0));
			// cout << "added point 1. " << endl;
			// MatrixXd tmp2(1, 3);
			// tmp2.row(0) = nearestPt.transpose();
			// viewer.data().add_points(tmp2, RowVector3d(0, 0, 1));
			// cout << "added point 2. " << endl;
			// viewer.launch();
		}
	}

	for(int i=0; i < V1.rows(); i++)
	{
		Vector3d p = V1.row(i);
		Vector3d nearestPt;
		int closestTriID;
		double dist = PQPABSDist(ptr2, p, nearestPt, closestTriID);
		V1.row(i) = nearestPt;
	
	}
	
	

	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V1, F1);
	igl::writeOBJ("/home/weikai/Projects/bodyRecon/output/output.obj", V1, F1);
	viewer.launch();
	
}




// generate signed distance field 
// args:
// (V,F) - (mesh vertex positions, mesh face indices)
// outputFile - output SDF file name
// resolution - the resolution in x,y,z direction
void generateSDFUsePQP(string meshFileName, string meshReconName, string outSDFName, int resolution){
	// load obj mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    cout << "Loading " << meshFileName << endl;
	// Read in inputs as double precision floating point meshes
  	igl::readOBJ(meshFileName,V,F);
	cout << "Mesh load done!" << endl;

	// initialize PQP model
	PQP_Model* m_pqp_model = new PQP_Model();
	buildPQP(m_pqp_model, meshFileName);
	// buildPQPModel(V, F);

	cout << "Computing signed distance field ..." << endl;

	// compute tight bounding box of the input mesh
	RowVector3d tightBmin = V.colwise().minCoeff();
	RowVector3d tightBmax = V.colwise().maxCoeff();
	RowVector3d center = (tightBmax + tightBmin) / 2.0;
	// enlarge bounding box little bit
	RowVector3d bmin = center + 1.0 * (tightBmin - center);
	RowVector3d bmax = center + 1.0 * (tightBmax - center);
	RowVector3d bboxSize = bmax - bmin;
	cout << "Tight bounding corners - min: " << vec2String(tightBmin) << " max: " << vec2String(tightBmax) << endl;
	cout << "Enlarged bounding corners - min: " << vec2String(bmin) << " max: " << vec2String(bmax) << endl;

	// number of vertices on the largest side
	const int s = resolution;
	const double h = (bmax-bmin).maxCoeff()/(double)s;
	const RowVector3i gridSize = ( s * ((bmax-bmin) / (bmax-bmin).maxCoeff()) ).cast<int>();
	cout << "Grid resolution: " << gridSize[0] << " " << gridSize[1] << " " << gridSize[2] << endl;


	// initialize write file
	ofstream fout(outSDFName, std::ios::binary);
	// write grid size and bounding box size into output binary file
	fout.write((const char*)&gridSize[0], sizeof(gridSize[0]));
	fout.write((const char*)&gridSize[1], sizeof(gridSize[0]));
	fout.write((const char*)&gridSize[2], sizeof(gridSize[0]));
	
	fout.write((const char*)&bmin[0], sizeof(double));
	fout.write((const char*)&bmin[1], sizeof(double));
	fout.write((const char*)&bmin[2], sizeof(double));
	fout.write((const char*)&bmax[0], sizeof(double));
	fout.write((const char*)&bmax[1], sizeof(double));
	fout.write((const char*)&bmax[2], sizeof(double));

	// for debug only Weikai
	// cout << "int size: " << sizeof(int) << " float size: " << sizeof(float) << " double size: " << sizeof(double) << endl;
	ofstream foutTest("test.txt");
	foutTest << gridSize[0] << " " << gridSize[1] << " " << gridSize[2] << endl;

	
	Vector3d step(resolution, resolution, resolution); 
	vector<Vector3d> closestPts;

	// create grid
	cout << "Creating grid ..." << endl;
	MatrixXd GV(gridSize(0)*gridSize(1)*gridSize(2), 3);	// GV to store grid points
	VectorXd S(gridSize(0)*gridSize(1)*gridSize(2), 1);		// S to store distance values
	
	const auto lerp = [&](const int di, const int d)->double
	{return bmin(d)+(double)di/(double)(gridSize(d)-1)*(bmax(d)-bmin(d));};

	auto start = high_resolution_clock::now();
	Model_OBJ obj;
    obj.Load(meshFileName);
	for(int x = 0; x< gridSize[0]; x++){
		const double xi = lerp(x, 0);
		for(int y = 0; y < gridSize[1]; y++){
			const double yi = lerp(y, 1);
			for(int z = 0; z < gridSize[2]; z++)
			{	
				const double zi = lerp(z, 2);
				// construct grid points
				GV.row(x+gridSize(0)*(y + gridSize(1)*z)) = RowVector3d(xi,yi,zi);
			
				// compute grid value				
				Vector3d pnt(xi, yi, zi);
				Vector3d nearestPnt;
				int closestTriID;
				float dist = PQPABSDist(m_pqp_model, pnt, nearestPnt, closestTriID);
				// float dist = PQPSignedDist(pnt, V, F, nearestPnt, closestTriID);
				closestPts.emplace_back(nearestPnt);

				// update the S vector
				S(x+gridSize(0)*(y + gridSize(1)*z)) = dist;
				
				// write to output file
				fout.write((const char*)&dist, sizeof(float));

				// // for debug only Weikai
				// foutTest << dist << " ";
			}
			// foutTest << endl;
			// cout << "finished z" << endl;
		}
		// cout << "finished y " << endl;
	}

	// for debug only!
	// double xmin = 0, xmax = 20;
	// double ymin = -50, ymax = 0;
	// double zmin = -9.67, zmax = 9.67;
	// vector<RowVector3d> legInnerPts;
	// for(int i=0; i < GV.rows(); i++)
	// {
	// 	RowVector3d p = GV.row(i);
	// 	if (p[0] > xmin && p[0] < xmax && p[1] > ymin && p[1] < ymax && p[2] >zmin && p[2] < zmax)
	// 	{
	// 		legInnerPts.push_back(p);
	// 	}
	// }
	// MatrixXd legPts(legInnerPts.size(), 3);
	// int cnt = 0;
	// for(auto iter = legInnerPts.begin(); iter != legInnerPts.end(); iter++)
	// {
	// 	legPts.row(cnt++) = *iter;
	// }
	// cout << "starting sign computing ... " << endl;
	// VectorXd signIGL(legInnerPts.size(), 1);	/// store inside/outside sign
	// signForPoints(legPts, V, F, signIGL);
	// cout << " Done ." << endl;
	// vector<RowVector3d> realInner;
	// for(int i=0; i < legInnerPts.size(); ++i)
	// {
	// 	if(signIGL(i) < 0)
	// 		realInner.push_back(legInnerPts[i]);
	// }

	// // write to the file
	// ofstream fpts("legPoints.txt");
	// cout << "leg point size: " << realInner.size() << endl;
	// fpts << realInner.size() << endl;
	// for(auto r : realInner)
	// {
	// 	fpts << r[0] << " " << r[1] << " " << r[2] << endl;
	// }			
	// fpts.close();
	// igl::opengl::glfw::Viewer viewer;
	// viewer.data().set_mesh(V,F);
	// viewer.data().add_points(legPts, RowVector3d(1,1,0));
	// viewer.launch();


	// compute inside or outside of each query points
	vector<double> signs;
	vector<Vector3d> points;
    set<Octree*> emptyCells = obj.inOrOutTest(GV, signs, points);
	// MatrixXd cellCornerPts;
	// vector<pair<Vector3d, Vector3d>>  cells = obj.getOccupiedCells(resolution, cellCornerPts);
	// obj.displayCells(cells, GV);
	// vector<pair<Vector3d, Vector3d>>  cell2s = obj.getEmptyCells(resolution, cellCornerPts);
	// obj.displayCells(cell2s, GV);
	assert(S.rows() == signs.size());
	for(int i=0; i < S.rows(); ++i)
	{
		S(i) = S(i) * signs[i] *  -1;
	}
	// VectorXd signs(gridSize(0)*gridSize(1)*gridSize(2), 1);	/// store inside/outside sign
	// signForPoints(GV, V, F, signs);
	// for(int i=0; i < S.rows(); ++i)
	// {
	// 	if (signs(i) < 0)
	// 	{
	// 		// cout << "distance before: " << S(i);
	// 		S(i) = S(i) * -1;
	// 		// cout << " distance after: " << S(i) << " sign: " << signs(i) << endl;
	// 	}			
	// }


	auto stop = high_resolution_clock::now(); 
	auto duration = duration_cast<microseconds>(stop - start); 
	cout << "PQP Computation Used time: " << duration.count() / double(1000000.0) << " seconds" << endl;	
	
	fout.close();
	foutTest.close();
	cout << "Finished writing signed distance field to " << outSDFName << "!" << endl;

	// use IGL marching cube to reconstruct
	cout << " Marching cubes ... " << endl;
	start = high_resolution_clock::now();

	MatrixXd SV;
	MatrixXi SF;
	igl::copyleft::marching_cubes(S, GV, gridSize(0), gridSize(1), gridSize(2), SV, SF);

	stop = high_resolution_clock::now(); 
	duration = duration_cast<microseconds>(stop - start); 
	cout << "Marching Cube Used time: " << duration.count() / double(1000000.0) << " seconds" << endl;

	// save the reconstructed mesh
	igl::writeOBJ(meshReconName, SV, SF);
	cout << "Finished writing reconstruction to " << meshReconName << "!" << endl;

	// display the mesh and grid points
	// displayForDebug(V, F, SV, SF, bmax, bmin, GV, S);
}

void computeSDFUseWindNum(string meshFileName, string meshReconName, string outSDFName, int resolution){
	// load obj mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    cout << "Loading " << meshFileName << endl;
	// Read in inputs as double precision floating point meshes
  	igl::readOBJ(meshFileName,V,F);
	cout << "Mesh load done!" << endl;
	// number of vertices on the largest side
	const int s = resolution;
	const RowVector3d Vmin = V.colwise().minCoeff();
	const RowVector3d Vmax = V.colwise().maxCoeff();
	const double h = (Vmax-Vmin).maxCoeff()/(double)s;
	const RowVector3i res = ( s * ((Vmax-Vmin) / (Vmax-Vmin).maxCoeff()) ).cast<int>();
	cout << "Grid resolution: " << res[0] << " " << res[1] << " " << res[2] << endl;
	cout << "Bounding box min: " << Vmin << " bounding box max: " << Vmax << endl;
	// create grid
	cout << "Creating grid ..." << endl;
	MatrixXd GV(res(0)*res(1)*res(2), 3);
	for(int zi = 0; zi<res(2); zi++)
	{
		const auto lerp = [&](const int di, const int d)->double
		{return Vmin(d)+(double)di/(double)(res(d)-1)*(Vmax(d)-Vmin(d));};
		const double z = lerp(zi,2);
		for(int yi = 0;yi<res(1);yi++)
		{
			const double y = lerp(yi,1);
			for(int xi = 0;xi<res(0);xi++)
			{
				const double x = lerp(xi,0);
				GV.row(xi+res(0)*(yi + res(1)*zi)) = RowVector3d(x,y,z);
			}
		}
	}
	// compute values
	cout<<"Computing distances ... " << endl;
	auto start = high_resolution_clock::now();

	VectorXd S,B;
	{
		VectorXi I;
		MatrixXd C,N;
		signed_distance(GV,V,F,SIGNED_DISTANCE_TYPE_WINDING_NUMBER,S,I,C,N);
		// point_mesh_squared_distance(GV, V, F, S, I, C);
		// signed_distance(GV,V,F,SIGNED_DISTANCE_TYPE_PSEUDONORMAL,S,I,C,N);
	}
	// // determine the sign of the distance
	// cout << "Computing the sign of distance ... ";
	// vector<double> signs;
	// Model_OBJ obj;
	// obj.Load(meshFileName);
	// vector<Vector3d> points;
    // set<Octree*> emptyCells = obj.inOrOutTest(GV, signs, points);
	// assert(S.rows() == signs.size());
	// for(int i=0; i < S.rows(); ++i)
	// {
	// 	S(i) = sqrt(S(i)) * signs[i] *  -1;
	// }
	// cout << "Done." << endl;

	auto stop = high_resolution_clock::now(); 
	auto duration = duration_cast<microseconds>(stop - start); 
	cout << "Min distance: " << S.minCoeff() << " max distance: " << S.maxCoeff() << endl;
	cout << " Signed distance field Creation Used time: " << duration.count() / double(1000000.0) << " seconds" << endl; 

	cout << " Marching cubes... " << endl;
	start = high_resolution_clock::now();
 
	MatrixXd SV,BV;
	MatrixXi SF,BF;
	igl::copyleft::marching_cubes(S,GV,res(0),res(1),res(2),SV,SF);

	stop = high_resolution_clock::now(); 
	duration = duration_cast<microseconds>(stop - start); 
	cout << "Marching Cube Used time: " << duration.count() / double(1000000.0) << " seconds" << endl;

	// save the reconstructed mesh
	igl::writeOBJ(meshReconName, SV, SF);
	cout << "Finished writing reconstruction to " << meshReconName << "!" << endl;

	// save distance field to binary file
	// initialize write file
	// ofstream fout(outSDFName, std::ios::binary);
	// // write grid size and bounding box size into output binary file
	// fout.write((const char*)&res[0], sizeof(res[0]));
	// fout.write((const char*)&res[1], sizeof(res[0]));
	// fout.write((const char*)&res[2], sizeof(res[0]));
	
	// fout.write((const char*)&Vmin[0], sizeof(double));
	// fout.write((const char*)&Vmin[1], sizeof(double));
	// fout.write((const char*)&Vmin[2], sizeof(double));
	// fout.write((const char*)&Vmax[0], sizeof(double));
	// fout.write((const char*)&Vmax[1], sizeof(double));
	// fout.write((const char*)&Vmax[2], sizeof(double));
	// for(int i=0; i<S.rows(); i++)
	// {
	// 	// write to output file
	// 	float dist = S(i);
	// 	fout.write((const char*)&dist, sizeof(float));
	// }
	// cout << "Saving signed distance field to " << outSDFName << endl;

	// displayForDebug(V, F, SV, SF, Vmax, Vmin, GV, S);
}

template <
  typename DerivedP,
  typename DerivedV,
  typename DerivedF,
  typename DerivedS>
   void signForPoints(
  const Eigen::MatrixBase<DerivedP> & P,
  const Eigen::MatrixBase<DerivedV> & V,
  const Eigen::MatrixBase<DerivedF> & F,
  Eigen::PlainObjectBase<DerivedS> & S)
{
    typedef Eigen::Matrix<typename DerivedV::Scalar,1,3> RowVector3S;
    WindingNumberAABB<RowVector3S,DerivedV,DerivedF> hier3;
    hier3.set_mesh(V,F);
    hier3.grow();
    typename DerivedV::Scalar s=1;
    // parallel_for(P.rows(),[&](const int p)
    for(int p = 0;p<P.rows();p++)
    {
        RowVector3S q3;
        q3.head(P.row(p).size()) = P.row(p);
        s = 1.-2.*hier3.winding_number(q3.transpose());
        S(p) = s;
    }
}