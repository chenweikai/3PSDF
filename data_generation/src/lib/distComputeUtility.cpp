#include "distComputeUtility.h"

#include <sstream>

void build_pqp(PQP_Model* ptr, string objName)
{
	ptr->BeginModel();
	MatrixXd V;
	MatrixXi F;
	igl::read_triangle_mesh(objName, V, F);
	PQP_REAL p1[3], p2[3], p3[3];
	std::cout << "Building Model for mesh " << objName << " ...";
	int numF = F.rows();
	for (int i = 0; i < numF; i++)
	{
		Vector3i vertIdx = F.row(i);
		int vid0 = vertIdx[0], vid1 = vertIdx[1], vid2 = vertIdx[2];
		p1[0] = (PQP_REAL)(V.row(vid0)[0]);
		p1[1] = (PQP_REAL)(V.row(vid0)[1]);
		p1[2] = (PQP_REAL)(V.row(vid0)[2]);
		
		p2[0] = (PQP_REAL)(V.row(vid1)[0]);
		p2[1] = (PQP_REAL)(V.row(vid1)[1]);
		p2[2] = (PQP_REAL)(V.row(vid1)[2]);
		
		p3[0] = (PQP_REAL)(V.row(vid2)[0]);
		p3[1] = (PQP_REAL)(V.row(vid2)[1]);
		p3[2] = (PQP_REAL)(V.row(vid2)[2]);
		ptr->AddTri(p1, p2, p3, i);
	}
	ptr->EndModel();
	std::cout << " Done." << std::endl;
}


void build_pqp(PQP_Model* ptr, const MatrixXd& V, const MatrixXi& F)
{
	ptr->BeginModel();
	PQP_REAL p1[3], p2[3], p3[3];
	// std::cout << "Building Model for mesh subset ... ";
	
	for (int i = 0; i < F.rows(); i++)
	{
		Vector3i vertIdx = F.row(i);
		int vid0 = vertIdx[0], vid1 = vertIdx[1], vid2 = vertIdx[2];
		p1[0] = (PQP_REAL)(V.row(vid0)[0]);
		p1[1] = (PQP_REAL)(V.row(vid0)[1]);
		p1[2] = (PQP_REAL)(V.row(vid0)[2]);
		
		p2[0] = (PQP_REAL)(V.row(vid1)[0]);
		p2[1] = (PQP_REAL)(V.row(vid1)[1]);
		p2[2] = (PQP_REAL)(V.row(vid1)[2]);
		
		p3[0] = (PQP_REAL)(V.row(vid2)[0]);
		p3[1] = (PQP_REAL)(V.row(vid2)[1]);
		p3[2] = (PQP_REAL)(V.row(vid2)[2]);
		ptr->AddTri(p1, p2, p3, i);
	}
	ptr->EndModel();
	// std::cout << " Done." << std::endl;
}


// return the unsigned distance from a 3D point to the mesh (PQP model required to be built first)
// args:
// queryPnt - input query point
// nearestPnt - return the nearest point on mesh regarding to the input point
// closestFaceIdx - return the id of the closest triangle
double pqp_abs_dist(PQP_Model* m_pqp_model, Vector3d queryPnt, Vector3d& nearestPnt, int& closestFaceIdx)
{
	PQP_DistanceResult dres;	PQP_REAL p[3];		int minTriIndex;
	double minDist;				
	int closeTriIdx;
	//QMeshEdge *edge;			QMeshNode *node;

	dres.last_tri = m_pqp_model->last_tri;
	p[0] = queryPnt[0];	p[1] = queryPnt[1];	p[2] = queryPnt[2];
	PQP_Distance(&dres, m_pqp_model, p, 0.0, 0.0);
	double closestPnt[3];
	closestPnt[0] = dres.p1[0];	closestPnt[1] = dres.p1[1];	closestPnt[2] = dres.p1[2];
	closeTriIdx = dres.last_tri->id;
	minDist = dres.Distance();

	// pass result back
	nearestPnt = Vector3d(closestPnt[0], closestPnt[1], closestPnt[2]);
	closestFaceIdx = closeTriIdx;

	return minDist;
}
