#include "visualization.h"

void displayEmptyOctreeCelss(const MatrixXd& V, const MatrixXi& F, const set<Octree*>& emptyCells)
{
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V, F);

	MatrixXd GV(8 * emptyCells.size(), 3);
	int cnt = 0;
	for(auto it = emptyCells.begin(); it != emptyCells.end(); it++)
	{
		glm::dvec3 mc = (*it)->min_corner;
		glm::dvec3 l = (*it)->length;
		Vector3d minP(mc[0], mc[1], mc[2]);
		Vector3d len(l[0], l[1], l[2]);
		
		double x = len[0], y = len[1], z = len[2];
		GV.row(8*cnt + 0) = minP.transpose();
		GV.row(8*cnt + 1) = (minP + Vector3d(x, 0, 0)).transpose();
		GV.row(8*cnt + 2) = (minP + Vector3d(0, y, 0)).transpose();
		GV.row(8*cnt + 3) = (minP + Vector3d(0, 0, z)).transpose();
		GV.row(8*cnt + 4) = (minP + Vector3d(x, y, 0)).transpose();
		GV.row(8*cnt + 5) = (minP + Vector3d(x, 0, z)).transpose();
		GV.row(8*cnt + 6) = (minP + Vector3d(0, y, z)).transpose();
		GV.row(8*cnt + 7) = (minP + Vector3d(x, y, z)).transpose();
		// if ((*it)->exterior == 1)
		// {
		// 	// viewer.data().add_points(GV, RowVector3d(1,0,0));
		// }else
		// {
		// 	cout << "Points interior!" << endl;
			
		// }		
		cnt++;
	}
	viewer.data().add_points(GV, RowVector3d(0,1,0));
	viewer.launch();
}

void displayMeshAndSamples(const MatrixXd& V, const MatrixXi& F, const MatrixXd& gridPts)
{
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V, F);
	viewer.data().add_points(gridPts, RowVector3d(1,0,0));
	viewer.launch();
}

void displayMeshAndSignedSamples(const MatrixXd& V, const MatrixXi& F, const MatrixXd& gridPts, const vector<double>& signs)
{
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V, F);
	assert(gridPts.rows() ==  signs.size());
	int cnt = 0;
	for(int i=0; i<signs.size(); ++i)
	{
		if (signs[i] == -1)
			cnt++;
	}
	cout << "there are " << cnt << " points outside!" << endl;
	MatrixXd ptsOut(cnt, 3);
	MatrixXd ptsIn(signs.size() - cnt, 3);
	int outCnt = 0, inCnt = 0;
	for(int i=0; i<signs.size(); ++i)
	{
		if (signs[i] == -1)
			ptsOut.row(outCnt++) = gridPts.row(i);
		else
			ptsIn.row(inCnt++) = gridPts.row(i);
	}
	// viewer.data().add_points(ptsOut, RowVector3d(1,0,0));
	viewer.data().add_points(ptsIn, RowVector3d(0,0,1));
	viewer.launch();
}

void displayForDebug(const MatrixXd& V, const MatrixXi& F, const MatrixXd& SV, const MatrixXi& SF, 
	const RowVector3d& Vmax, const RowVector3d& Vmin, const MatrixXd& gridPts, const VectorXd& S)
{
	// cout<<R"(Usage:
	// '1'  Show original mesh.
	// '2'  Show marching cubes contour of signed distance.
	// '4'  Show part of the marching cube grid.
	// '5'  March the marching cube grid.
	// '6'  Show and march unsigned distance field.
	// )";
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(SV,SF);
	int sid = 1004000;
	int step_size = 1500;
	int visPoints = 0;
	int mesh1 = 1;
	int mesh2 = 1;
	int row_num = gridPts.rows();
	int resF = 1000; 	// resolution for visualizing unsigned distance field
	int slice = 0;		// slice id for visualizing unsigned distance field	
	viewer.callback_key_down =
		[&](igl::opengl::glfw::Viewer & viewer, unsigned char key, int mod)->bool
		{
		switch(key)
		{
			
			default:
			return false;
			case '1':
				viewer.data().clear();
				if (++mesh1 % 2 == 0){					
					viewer.data().set_mesh(V,F);
					mesh2 = 1;
				}
				if (visPoints % 2 == 0){
					MatrixXd tmp = gridPts.block(sid, 0, step_size, 3);
					viewer.data().add_points(tmp, RowVector3d(1,0,0));
					assert(gridPts.rows() ==  S.rows());					
					for(int i = sid; i < std::min((sid + step_size), row_num); i++){
						stringstream l1;
						l1 << S(i);
						viewer.data().add_label(Vector3d(gridPts.row(i)),l1.str());
					}
					cout << "Starting id ..." << sid << endl;
					// drawValueAtPpints(GV, S, viewer);
				}
				break;
			case '2':
				viewer.data().clear();
				if (++mesh2 % 2 == 0){
					viewer.data().set_mesh(SV,SF);
					mesh1 = 1;
				}
				if (visPoints % 2 == 0){
					MatrixXd tmp = gridPts.block(sid, 0, step_size, 3);
					viewer.data().add_points(tmp, RowVector3d(1,0,0));
					assert(gridPts.rows() ==  S.rows());					
					for(int i = sid; i < std::min((sid + step_size), row_num); i++){
						stringstream l1;
						l1 << S(i);
						viewer.data().add_label(Vector3d(gridPts.row(i)),l1.str());
					}
					cout << "Starting id ..." << sid << endl;
					// drawValueAtPpints(GV, S, viewer);
				}
				break;
			// case '3':
			// viewer.data().clear();
			// viewer.data().set_mesh(BV,BF);
			// break;
			case '4':
				viewer.data().clear();
				if (mesh1 % 2 == 0)
					viewer.data().set_mesh(V,F);
				else if (mesh2 % 2 == 0)
					viewer.data().set_mesh(SV,SF);
				if (++visPoints % 2 == 0){
					MatrixXd tmp = gridPts.block(sid, 0, step_size, 3);
					viewer.data().add_points(tmp, RowVector3d(1,0,0));
					assert(gridPts.rows() ==  S.rows());					
					for(int i = sid; i < std::min((sid + step_size), row_num); i++){
						stringstream l1;
						l1 << S(i);
						viewer.data().add_label(Vector3d(gridPts.row(i)),l1.str());
					}
					cout << "Starting id ..." << sid << endl;
					// drawValueAtPpints(GV, S, viewer);
				}
				break;
			case '5':
				viewer.data().clear();
				sid = (sid + step_size) % row_num;
				if (mesh1 % 2 == 0)
					viewer.data().set_mesh(V,F);
				else if (mesh2 %2 == 0)
					viewer.data().set_mesh(SV,SF);
				
				if (visPoints % 2 == 0){
					MatrixXd tmp = gridPts.block(sid, 0, step_size, 3);
					viewer.data().add_points(tmp, RowVector3d(1,0,0));
					assert(gridPts.rows() ==  S.rows());				
					for(int i = sid; i < std::min((sid + step_size), row_num); i++){
						stringstream l1;
						l1 << S(i);
						viewer.data().add_label(Vector3d(gridPts.row(i)),l1.str());
					}
				}
				
				cout << "Starting id ..." << sid << endl;
							
				break;	
			case '6':
				generateVisField(viewer, V, F, Vmin, Vmax, resF, 2, slice++);				
		}
		viewer.data().set_face_based(true);
		return true;
		};
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	menu.callback_draw_viewer_window = [](){};
	viewer.plugins.push_back(&menu);
	viewer.launch();
}

void generateVisField(igl::opengl::glfw::Viewer& viewer, const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F, const RowVector3d& bmin, const RowVector3d& bmax, int resolution, int dim, int slice){
	const int s = resolution;
	const double h = (bmax-bmin).maxCoeff()/(double)s;
	const RowVector3i gridSize = ( s * ((bmax-bmin) / (bmax-bmin).maxCoeff()) ).cast<int>();
	cout << "Grid resolution: " << gridSize[0] << " " << gridSize[1] << " " << gridSize[2] << endl;

	Vector3d step(resolution, resolution, resolution); 

	MatrixXd GV(gridSize(0)*gridSize(1), 3);	// GV to store grid points
	VectorXd S(gridSize(0)*gridSize(1), 1);		// S to store distance values

	const auto lerp = [&](const int di, const int d)->double
	{return bmin(d)+(double)di/(double)(gridSize(d)-1)*(bmax(d)-bmin(d));};

	// dim is disabled for now

	for(int x = 0; x< gridSize[0]; x++){
		const double xi = lerp(x, 0);
		for(int y = 0; y < gridSize[1]; y++){
			const double yi = lerp(y, 1);
			// for(int z = 0; z < gridSize[2]; z++)
			// {	
				const double zi = lerp(slice, 2);
				// construct grid points
				GV.row(x+gridSize(0)*(y)) = RowVector3d(xi,yi,zi);
				VectorXi I;
				MatrixXd C;
				igl::point_mesh_squared_distance(GV,V,F,S,I,C);
				
			// }
		}
	}

	assert(GV.rows() ==  S.rows());	

	MatrixXd color;
	igl::parula(S, false, color);

	// draw grid points
	viewer.data().add_points(GV, color);		
}


