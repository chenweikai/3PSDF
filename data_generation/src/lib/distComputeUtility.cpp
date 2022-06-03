#include "distComputeUtility.h"
#include <sstream>

// extern PQP_Model* m_pqp_model;

void buildPQP(PQP_Model* ptr, string objName)
{
	ptr->BeginModel();
	MatrixXd V;
	MatrixXi F;
	igl::read_triangle_mesh(objName, V, F);
	PQP_REAL p1[3], p2[3], p3[3];
	cout << "Building Model for mesh " << objName << " ...";
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
	cout << " Done." << endl;
}

void buildPQP(PQP_Model* ptr, const MatrixXd& V, const MatrixXi& F)
{
	ptr->BeginModel();
	PQP_REAL p1[3], p2[3], p3[3];
	// cout << "Building Model for mesh subset ... ";
	
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
	// cout << " Done." << endl;
}

// initialize PQP query model
// 1) load mesh 2) compute PQP model from mesh 
// void buildPQPModel(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F){
//     m_pqp_model = new PQP_Model();
// 	m_pqp_model->BeginModel();
// 	PQP_REAL p1[3], p2[3], p3[3];
// 	cout << "Building PQP Model ...";
// 	int numF = F.rows();
// 	for (int i = 0; i < numF; i++)
// 	{
// 		Vector3i vertIdx = F.row(i);
// 		int vid0 = vertIdx[0], vid1 = vertIdx[1], vid2 = vertIdx[2];
// 		p1[0] = (PQP_REAL)(V.row(vid0)[0]);
// 		p1[1] = (PQP_REAL)(V.row(vid0)[1]);
// 		p1[2] = (PQP_REAL)(V.row(vid0)[2]);
		
// 		p2[0] = (PQP_REAL)(V.row(vid1)[0]);
// 		p2[1] = (PQP_REAL)(V.row(vid1)[1]);
// 		p2[2] = (PQP_REAL)(V.row(vid1)[2]);
		
// 		p3[0] = (PQP_REAL)(V.row(vid2)[0]);
// 		p3[1] = (PQP_REAL)(V.row(vid2)[1]);
// 		p3[2] = (PQP_REAL)(V.row(vid2)[2]);
// 		m_pqp_model->AddTri(p1, p2, p3, i);
// 	}
// 	m_pqp_model->EndModel();
// 	cout << " Done." << endl;
// }

// return the unsigned distance from a 3D point to the mesh (PQP model required to be built first)
// args:
// queryPnt - input query point
// nearestPnt - return the nearest point on mesh regarding to the input point
// closestFaceIdx - return the id of the closest triangle
double PQPABSDist(PQP_Model* m_pqp_model, Vector3d queryPnt, Vector3d& nearestPnt, int& closestFaceIdx)
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


// float PQPABSDist(PQP_Model* ptr, Vector3d queryPnt, Vector3d& nearestPnt, int& closestFaceIdx)
// {
// 	PQP_DistanceResult dres;	PQP_REAL p[3];		int minTriIndex;
// 	double minDist;				
// 	int closeTriIdx;
// 	//QMeshEdge *edge;			QMeshNode *node;

// 	dres.last_tri = ptr->last_tri;
// 	p[0] = queryPnt[0];	p[1] = queryPnt[1];	p[2] = queryPnt[2];
// 	PQP_Distance(&dres, ptr, p, 0.0, 0.0);
// 	double closestPnt[3];
// 	closestPnt[0] = dres.p1[0];	closestPnt[1] = dres.p1[1];	closestPnt[2] = dres.p1[2];
// 	closeTriIdx = dres.last_tri->id;
// 	minDist = dres.Distance();

// 	// pass result back
// 	nearestPnt = Vector3d(closestPnt[0], closestPnt[1], closestPnt[2]);
// 	closestFaceIdx = closeTriIdx;

// 	return minDist;
// }

// return the distance from a 3D point to the mesh (PQP model required to be built first)
// args:
// queryPnt - input query point
// nearestPnt - return the nearest point on mesh regarding to the input point
// closestFaceIdx - return the id of the closest triangle
float PQPSignedDist(PQP_Model* m_pqp_model, Vector3d queryPnt, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, Vector3d& nearestPnt, int& closestFaceIdx)
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

	// check if it is inside or outside the mesh
	Vector3i vertIdx = F.row(closeTriIdx);
	int vid0 = vertIdx[0], vid1 = vertIdx[1], vid2 = vertIdx[2];
	Vector3d p0 = V.row(vid0);
	Vector3d p1 = V.row(vid1);
	Vector3d p2 = V.row(vid2);
	Vector3d v1 = p1 - p0;
	Vector3d v2 = p2 - p1;
	Vector3d nv = v1.cross(v2);
	double dd = (queryPnt[0] - closestPnt[0])*nv[0] + (queryPnt[1] - closestPnt[1])*nv[1] + (queryPnt[2] - closestPnt[2])*nv[2];
	if (dd < 0.0)
		minDist = -minDist;

	return minDist;
}

// return the distance from a 3D point to the mesh (PQP model required to be built first)
// sign determined by naive normal test -- NOT RELIABLE
// args:
// queryPnt - input query point
// nearestPnt - return the nearest point on mesh regarding to the input point
// closestFaceIdx - return the id of the closest triangle
float PQPSignedDist_PeudoNormal(PQP_Model* m_pqp_model, Vector3d queryPnt, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, Vector3d& nearestPnt, int& closestFaceIdx)
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

	// check if it is inside or outside the mesh
	Vector3i vertIdx = F.row(closeTriIdx);
	int vid0 = vertIdx[0], vid1 = vertIdx[1], vid2 = vertIdx[2];
	Vector3d p0 = V.row(vid0);
	Vector3d p1 = V.row(vid1);
	Vector3d p2 = V.row(vid2);
	Vector3d v1 = p1 - p0;
	Vector3d v2 = p2 - p1;
	Vector3d nv = v1.cross(v2);
	double dd = (queryPnt[0] - closestPnt[0])*nv[0] + (queryPnt[1] - closestPnt[1])*nv[1] + (queryPnt[2] - closestPnt[2])*nv[2];
	if (dd < 0.0)
		minDist = -minDist;

	return minDist;
}
