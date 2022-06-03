#include "compute3PoleSDF.h"
// #include "OctreeUtilities.h"
// #include "IGL_SIGN.h"
// #include "visualization.h"



#define PI 3.14159265

// for debug only!
#define FOR_DEBUG 1
float maxDist = 2;
vector<int> vec_pts_in;
vector<Vector3d> vec_close_in;
vector<int> vec_pts_out;
vector<Vector3d> vec_close_out;
vector<int> vec_pts_nan;
vector<Vector3d> vec_close_nan;

vector<Vector3d> outPnts;
vector<Vector3d> outPnts_close;

// for debug only
MatrixXd vb;
MatrixXi vf;
RowVector3d vbmin;
RowVector3d vbmax;
vector<int> bugPoints;


// given point p and triangle (a,b,c), return p's barycentric coordinate
void Barycentric(const Vector3d& p, const Vector3d& a, const Vector3d& b, const Vector3d& c, float &u, float &v, float &w)
{
    Vector3d v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = v0.dot(v0); 
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}


string vec2String(const RowVector3d& v){
	ostringstream strs;
	strs << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
	return strs.str();
}

typedef pair<int,int> CEdge;

double verifyAngleViaVertConnection(
                    const int& vertID,
                    const Vector3d& dir, 
                    const MatrixXd& face_normals,
                    map<int, vector<int>>& vert2Face,
                    int& closestTriNum)
{
    auto iter = vert2Face.find(vertID);
    if (iter == vert2Face.end())
    {
        system("pause");
        cout << "The vertex is not found in function verifyAngleViaVertConnection!" << endl;
    }

    vector<int> faces = vert2Face[vertID];
    map<int, int> fids; // use map to avoid duplicated face ids
    for(auto f : faces)
        fids[f] = 1;
    double maxV = -1e10;
    RowVector3d aveNormal;
    for(auto i=fids.begin(); i!=fids.end(); i++)
    {
        RowVector3d n = face_normals.row(i->first);
        aveNormal += n;
        // double dotprod = dir.dot(n);
        // if (dotprod > maxV)
        //     maxV = dotprod;
    }
    closestTriNum = fids.size();

    aveNormal = aveNormal / double(fids.size());
    aveNormal.normalize();
    maxV = aveNormal.dot(dir);
    return maxV;
}

double getMaxAngleViaVertConnection(
                    const int& vertID,
                    const Vector3d& dir, 
                    const MatrixXd& face_normals,
                    map<int, vector<int>>& vert2Face
                    )
{
    auto iter = vert2Face.find(vertID);
    if (iter == vert2Face.end())
    {
        system("pause");
        cout << "The vertex is not found in function verifyAngleViaVertConnection!" << endl;
    }

    vector<int> faces = vert2Face[vertID];
    map<int, int> fids; // use map to avoid duplicated face ids
    for(auto f : faces)
        fids[f] = 1;
    double maxV = -1e10;
    RowVector3d aveNormal;
    for(auto i=fids.begin(); i!=fids.end(); i++)
    {
        RowVector3d n = face_normals.row(i->first);
        double dotprod = dir.dot(n);
        if (dotprod > maxV)
            maxV = dotprod;
    }

    return maxV;
}

// input params:
// a, b, c -- the barycentric coordinate of v0, v1, v2 of a triangle
double getMaxAngleViaEdgeConnection(const float& a, const float& b, const float& c, 
                    const int& faceID,
                    const Vector3d& dir, 
                    const MatrixXi& F,
                    const MatrixXd& face_normals,
                    map<CEdge, vector<int>>& edge2Face)
{
    float eps = 1e-4;
    map<int, int> fids;
    int v0 = F.row(faceID)[0], v1 = F.row(faceID)[1], v2 = F.row(faceID)[2];
    CEdge e;
    if (abs(a) < eps){
        if (v1 < v2)
            e = CEdge(v1, v2);
        else
            e = CEdge(v2, v1);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            system("pause");
            cout << "The edge is not found in function verifyAngleViaEdgeConnection!" << endl;
        }
        vector<int> faces = itr->second;
        for(auto f : faces)
            fids[f] = 1;
        // fids.insert(fids.end(), edge2Face[e].begin(), edge2Face[e].end());
    }
    if (abs(b) < eps){
        if (v2 < v0)
            e = CEdge(v2, v0);
        else
            e = CEdge(v0, v2);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            system("pause");
            cout << "The edge is not found in function verifyAngleViaEdgeConnection!" << endl;
        }
        vector<int> faces = itr->second;
        for(auto f : faces)
            fids[f] = 1;
    }
    if (abs(c) < eps){
        if (v1 < v0)
            e = CEdge(v1, v0);
        else
            e = CEdge(v0, v1);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            system("pause");
            cout << "The edge is not found in function verifyAngleViaEdgeConnection!" << endl;
        }
        vector<int> faces = itr->second;
        for(auto f : faces)
            fids[f] = 1;
    }

    double maxV = -1e10;
    for(auto i=fids.begin(); i!=fids.end(); i++)
    {
        RowVector3d n = face_normals.row(i->first);
        double dotprod = dir.dot(n);
        if (dotprod > maxV)
            maxV = dotprod;
    }

    return maxV;
}

// input params:
// a, b, c -- the barycentric coordinate of v0, v1, v2 of a triangle
double verifyAngleViaEdgeConnection(const float& a, const float& b, const float& c, 
                    const int& faceID,
                    const Vector3d& dir, 
                    const MatrixXi& F,
                    const MatrixXd& face_normals,
                    map<CEdge, vector<int>>& edge2Face,
                    // const vector<pair<int,int>>& boundEdges,
                    int& closestTriNum)
{
    float eps = 1e-4;
    map<int, int> fids;
    int v0 = F.row(faceID)[0], v1 = F.row(faceID)[1], v2 = F.row(faceID)[2];
    CEdge e;
    if (abs(a) < eps){
        if (v1 < v2)
            e = CEdge(v1, v2);
        else
            e = CEdge(v2, v1);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            system("pause");
            cout << "The edge is not found in function verifyAngleViaEdgeConnection!" << endl;
        }
        vector<int> faces = itr->second;
        for(auto f : faces)
            fids[f] = 1;
        // fids.insert(fids.end(), edge2Face[e].begin(), edge2Face[e].end());
    }
    if (abs(b) < eps){
        if (v2 < v0)
            e = CEdge(v2, v0);
        else
            e = CEdge(v0, v2);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            system("pause");
            cout << "The edge is not found in function verifyAngleViaEdgeConnection!" << endl;
        }
        vector<int> faces = itr->second;
        for(auto f : faces)
            fids[f] = 1;
    }
    if (abs(c) < eps){
        if (v1 < v0)
            e = CEdge(v1, v0);
        else
            e = CEdge(v0, v1);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            system("pause");
            cout << "The edge is not found in function verifyAngleViaEdgeConnection!" << endl;
        }
        vector<int> faces = itr->second;
        for(auto f : faces)
            fids[f] = 1;
    }

    double maxV = -1e10;
    RowVector3d aveNormal;
    for(auto i=fids.begin(); i!=fids.end(); i++)
    {
        RowVector3d n = face_normals.row(i->first);
        aveNormal += n;
        // double dotprod = dir.dot(n);
        // if (dotprod > maxV)
        //     maxV = dotprod;
    }
    closestTriNum = fids.size();

    aveNormal = aveNormal / double(fids.size());
    aveNormal.normalize();
    maxV = aveNormal.dot(dir);

    return maxV;
}

vector<int> getConnectedFaceID(const float& a, const float& b, const float& c, 
                    const int& faceID,
                    const MatrixXi& F,
                    map<CEdge, vector<int>>& edge2Face)
{
    float eps = 1e-4;
    vector<int> fids;
    int v0 = F.row(faceID)[0], v1 = F.row(faceID)[1], v2 = F.row(faceID)[2];
    CEdge e;
    if (abs(a) < eps){
        if (v1 < v2)
            e = CEdge(v1, v2);
        else
            e = CEdge(v2, v1);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            system("pause");
            cout << "The edge is not found in function verityInorOut!" << endl;
        }
        fids.insert(fids.end(), edge2Face[e].begin(), edge2Face[e].end());
    }
    if (abs(b) < eps){
        if (v2 < v0)
            e = CEdge(v2, v0);
        else
            e = CEdge(v0, v2);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            system("pause");
            cout << "The edge is not found in function verityInorOut!" << endl;
        }
        fids.insert(fids.end(), edge2Face[e].begin(), edge2Face[e].end());
    }
    if (abs(c) < eps){
        if (v1 < v0)
            e = CEdge(v1, v0);
        else
            e = CEdge(v0, v1);
        auto itr = edge2Face.find(e);
        if (itr == edge2Face.end())
        {
            system("pause");
            cout << "The edge is not found in function verityInorOut!" << endl;
        }
        fids.insert(fids.end(), edge2Face[e].begin(), edge2Face[e].end());
    }

    return fids;
}

void extractOpenMeshes(const MatrixXd& V, const MatrixXi& F, 
                        map<int, vector<int>>& vert2Faces,
                        vector<vector<int>>& openMeshes)
{
    MatrixXi BA, K;
	MatrixXi J; // id of boundary faces
	igl::boundary_facets(F, BA, J, K);
    // vector<int> boundary_faceID;
    vector<bool> visited(F.rows(), false); // used to mask the boundary faces
    queue<int> boundFaces;
	for (int i=0; i < J.rows(); ++i)
    {
        // boundary_faceID.push_back(J.row(i)[0]);
        boundFaces.push(J.row(i)[0]);
    }

	// for debug only!
    // cout << "vertex number: " << V.rows() << " face number: " << F.rows() << endl;
    // cout << "There are " << boundFaces.size() << " boundary faces!" << endl;
	// // visualize boundary faces
	// MatrixXd C = RowVector3d(0.4,0.8,0.3).replicate(F.rows(),1);
	// // Red for each in K
	// MatrixXd R = RowVector3d(1.0,0.3,0.3).replicate(J.rows(),1);
	// for(int i=0; i < J.rows(); i++)
	// {
	// 	int r = J.row(i)[0];
	// 	C.row(r) = R.row(0);
	// }
	// igl::opengl::glfw::Viewer viewer;
	// viewer.data().set_mesh(V, F);
	// viewer.data().set_colors(C);
	// viewer.launch();


    while (!boundFaces.empty())
    {
        int fid = boundFaces.front();
        boundFaces.pop();
        if (visited[fid])
            continue;
        vector<int> curMesh;
        stack<int> neighbors;
        neighbors.push(fid);
        while(!neighbors.empty())
        {
            int cur = neighbors.top();
            neighbors.pop();
            if (!visited[cur])
            {
                // cout << cur << endl;
                curMesh.push_back(cur); // has not been visited before
                visited[cur] = true;
            }                
            else{                
                continue;
            }
            int vid0 = F.row(cur)[0], vid1 = F.row(cur)[1], vid2 = F.row(cur)[2];
            auto itr0 = vert2Faces.find(vid0);
            auto itr1 = vert2Faces.find(vid1);
            auto itr2 = vert2Faces.find(vid2);
            if (itr0 == vert2Faces.end() || itr1 == vert2Faces.end() || itr2 == vert2Faces.end())
            {
                system("pause");
                std::cout << "The vertex is not found in function extractOpenMeshes!" << endl;
            }
            for(int f : vert2Faces[vid0])
            {
                if (!visited[f])
                    neighbors.push(f);
            }
            for(int f : vert2Faces[vid1])
            {
                if (!visited[f])
                    neighbors.push(f);
            }
            for(int f : vert2Faces[vid2])
            {
                if (!visited[f])
                    neighbors.push(f);
            }
        }
        // std::cout << "Found one open mesh with " << curMesh.size() << " faces!" << endl;
        openMeshes.push_back(curMesh);
    }
    std::cout << "There are " << openMeshes.size() << " open meshes!" << endl;

}

void reOrderMeshIndices(const MatrixXd& allV, const MatrixXi& allF, const vector<int>& partF, MatrixXd& outV, MatrixXi& outF)
{
    map<int, int> old2New;  // mapp from old index to new index in new mesh
    for(int i=0; i < partF.size(); ++i)
    {
        int fid = partF[i];
        old2New[allF(fid,0)] = -1;
        old2New[allF(fid,1)] = -1;
        old2New[allF(fid,2)] = -1;
    }
    // cout << "New mesh includes: " << old2New.size() << " vertices." << endl;
    // only keep selected vertices and update vertex index
    outV = MatrixXd(old2New.size(), 3);
    int cnt = 0;
    for (auto iter = old2New.begin(); iter != old2New.end(); iter++)
    {
        outV.row(cnt) = allV.row(iter->first);
        iter->second = cnt++;
    }
    // update face index
    outF = MatrixXi(partF.size(), 3);
    for(int i=0; i < partF.size(); ++i)
    {
        int fid = partF[i];
        outF(i,0) = old2New[allF(fid,0)];
        outF(i,1) = old2New[allF(fid,1)];
        outF(i,2) = old2New[allF(fid,2)];
    }    
}

class distItem{
public:
    bool is_nan;
    float dist;
#if FOR_DEBUG
    // Vector3d closestPnt;
#endif
public:
    distItem(): is_nan(), dist() {}
    distItem(bool nan, float distance): is_nan(nan), dist(distance) {}

};



// input: threshold - control the ratio of intermediate NAN region; larger threshold -> smaller NAN region
                     
void fuseDistance(const vector<map<float, distItem>>& distCollect, VectorXd& S, double minGridWidth, float threshold=0.5)
{   

    float fusedDist;
    for (int i=0; i < distCollect.size(); ++i)
    {

        // auto iter = find(bugPoints.begin(), bugPoints.end(), i);
        // bool found = false;
        // if (iter != bugPoints.end())
        //     found = true;

        const map<float, distItem>& vec_dist = distCollect[i];
        if (vec_dist.size() == 0)
            cout << "There is no distance value to be fused!" << endl;
        else if (vec_dist.size() == 1)
        {
            const distItem& item = vec_dist.begin()->second;
            if (item.is_nan)
            {
                fusedDist = NAN;
#if FOR_DEBUG
                // if (abs(item.dist) < maxDist)
                // {
                //     vec_pts_nan.push_back(i);
                //     vec_close_nan.push_back(item.closestPnt);
                // }                
#endif
            }                
            else
            {
                fusedDist = item.dist;
#if FOR_DEBUG
                // if (abs(item.dist) < maxDist)
                // {
                //     if(item.dist<0)
                //     {
                //         // cout << "And it is inside point!" << endl;
                //         vec_pts_in.push_back(i);
                //         vec_close_in.push_back(item.closestPnt);
                //     }else
                //     {
                //         // cout << "And it is outside point!" << endl;
                //         vec_pts_out.push_back(i);
                //         vec_close_out.push_back(item.closestPnt);
                //     }    
                // }
                   
                                      
#endif
            }
                
        }            
        else{
            // map<float, distItem> sortMap;
            // for(auto it : vec_dist)
            //     sortMap[abs(it.dist)] = it;
            
            auto iter = vec_dist.begin();
            distItem dist1 = iter->second;
            iter++;
            distItem dist2 = iter->second;

            if (dist1.dist * dist2.dist == 0)
            {
                fusedDist = 0;
#if FOR_DEBUG
                
                // vec_pts_in.push_back(i);
                // vec_close_in.push_back(dist1.closestPnt);
                
#endif
            } 
            else if( (!dist1.is_nan & !dist2.is_nan) & ( (dist1.dist * dist2.dist) < 0) )
            {
                // if both distances are not NAN but have different signs
                if ( (abs(dist1.dist) / abs(dist2.dist)) >= threshold )
                {
                    
                    // TODO: compute the proper ratio here!
                    if (abs(dist1.dist) < 1 * minGridWidth)
                    {
                        // when the distance is smaller than the minimum grid size, 
                        // if we set the middle part to NAN, marching cube will generate unexpected holes
                        // hence, here we just keep its original value and sign, if additional boundary is generated,
                        // we can remove it using post processing!
                        fusedDist = dist1.dist;
#if FOR_DEBUG
                        // if (abs(dist1.dist) < maxDist)
                        // {
                        //     if (dist1.dist > 0)
                        //     {
                        //         vec_pts_out.push_back(i);
                        //         vec_close_out.push_back(dist1.closestPnt);
                        //     }else{
                        //         vec_pts_in.push_back(i);
                        //         vec_close_in.push_back(dist1.closestPnt);
                        //     }    
                        // }                               
#endif 

                    }else
                    {
                        fusedDist = NAN;    // the middle region is set as NAN
#if FOR_DEBUG            
                        // if (abs(dist1.dist) < maxDist)
                        // {    
                        //     vec_pts_nan.push_back(i);
                        //     vec_close_nan.push_back(dist1.closestPnt);  
                        // }                         
#endif
                    } 
                }else
                {
                    // if not in the middle region, assign the sign according to its closer object
                    fusedDist = dist1.dist;
                }


            }else
            {
                if (dist1.is_nan)
                {
                    fusedDist = NAN;
#if FOR_DEBUG                
                    // if (abs(dist1.dist) < maxDist)
                    // {
                    //     vec_pts_nan.push_back(i);
                    //     vec_close_nan.push_back(dist1.closestPnt); 
                    // }                                              
#endif
                }else
                {
                    fusedDist = dist1.dist;
#if FOR_DEBUG
                   
                        // if (abs(dist1.dist) < maxDist)
                        // {
                        //     if (dist1.dist > 0)
                        //     {
                        //         vec_pts_out.push_back(i);
                        //         vec_close_out.push_back(dist1.closestPnt);
                        //     }else{
                        //         vec_pts_in.push_back(i);
                        //         vec_close_in.push_back(dist1.closestPnt);
                        //     }      
                        // }       
                    
                                          
#endif 
                }
                
            }
            

                         
        }  
        S(i) = fusedDist;
    }
    
    // return fusedDist;
}

/*
    Compute the 3-pole distance field for a single input point
    input: input point - input query points
    input: m_pqp_models - PQP model for compute unsigned distance
    input: V - mesh vertices
    input: F - mesh faces
    input: face_normals - face normals
    input: vert2Face - given a vertex id, this map stores all the face ids that it connects to
    input: edge2Face - given an edge, this map stores all the face ids that it connects to
    output: S - 3-pole distance for query points 
*/
distItem compute3PoleDistForOnePoint(
            const Vector3d& pnt,
            PQP_Model* m_pqp_model, 
            const MatrixXd& V, const MatrixXi& F,
            const MatrixXd& face_normals, 
            map<int, vector<int>>& vert2Face,
            map<pair<int, int>, vector<int>>& edge2Face,
            const vector<int>& boundPoints,
            Vector3d& nan_closestPnt_forDebug
        )
{
    Vector3d nearestPnt;
    int closestTriID;
    float dist = PQPABSDist(m_pqp_model, pnt, nearestPnt, closestTriID);
    RowVector3d n = face_normals.row(closestTriID);
    Vector3d dir_org = pnt - nearestPnt;
    Vector3d dir = dir_org.normalized();
    double p2pdist = dir_org.norm();
    if (p2pdist < 1e-10 || isnan(dir[0]))
    {
        // S(x+gridSize(0)*(y + gridSize(1)*z)) = 0.0;
        dist = 0.0;
    }
    
    double eps = 1e-4;
    double dotprod = dir.dot(n);
    if (abs(dotprod-1.0) < eps)
        dotprod = 1.0;
    if (abs(dotprod+1.0) < eps)
        dotprod = -1.0;
    double angle = acos(dotprod) * 180.0 / PI;
    
    double range = 45;
    // cout << angle << endl;
    distItem newDist;

    float a, b, c;  // barycentric coordinate
    int vid0 = F.row(closestTriID)[0], vid1 = F.row(closestTriID)[1], vid2 = F.row(closestTriID)[2];
    Vector3d v0 = V.row(vid0);
    Vector3d v1 = V.row(vid1);
    Vector3d v2 = V.row(vid2);
    Barycentric(nearestPnt, v0, v1, v2, a, b, c);
    // float eps = 1e-4;
    int closestTriNum = 1;

    if (angle < (90.0 - range))
    {
        dist = dist;
        newDist = distItem(false, dist);
        // vec_pts_out.push_back(pnt);
        // vec_close_out.push_back(nearestPnt);
        // S(x+gridSize(0)*(y + gridSize(1)*z)) = abs(value);
    }
    else //if (angle >= (90 + range))
    {
        
        float a, b, c;  // barycentric coordinate
        int vid0 = F.row(closestTriID)[0], vid1 = F.row(closestTriID)[1], vid2 = F.row(closestTriID)[2];
        Vector3d v0 = V.row(vid0);
        Vector3d v1 = V.row(vid1);
        Vector3d v2 = V.row(vid2);
        Barycentric(nearestPnt, v0, v1, v2, a, b, c);
        float eps = 1e-4;


        
        if ( abs(a) < eps || abs(b) < eps || abs(c) < eps) // closest point lies on the edge
        {
            double prod = -10;
            if ( abs(a-1.0) < eps ||  abs(b-1.0) < eps ||  abs(c-1.0) < eps)
            {
                int vid = vid0;
                if (abs(b-1.0) < eps)
                    vid = vid1;
                if (abs(c-1.0) < eps)
                    vid = vid2;
                prod = verifyAngleViaVertConnection(vid, dir, face_normals, vert2Face, closestTriNum);
            }else
                prod = verifyAngleViaEdgeConnection(a, b, c, closestTriID, dir, F, face_normals, edge2Face, closestTriNum);

            double finalAngle = acos(prod) * 180.0 / PI;

            if (finalAngle < (90.0-range))
            {
                dist = dist;
                newDist = distItem(false, dist);
                // vec_pts_out.push_back(pnt);
                // vec_close_out.push_back(nearestPnt);
            }
            else if (finalAngle >= (90+range))
            {
                dist = -dist;
                newDist = distItem(false, dist);
                // vec_pts_in.push_back(pnt);
                // vec_close_in.push_back(nearestPnt);
            }else
            {
                // vec_pts_nan.push_back(pnt);
                // vec_close_nan.push_back(nearestPnt);
                // cout << "pnt " << pnt[0] << " " << pnt[1] << " " << pnt[2] 
                //         << " faceid: " << closestTriID << " vert id: "
                //         << vid0 << " " << vid1 << " " << vid2 
                //         << " a: " << a << " b: " << b << " c: " << c
                //         << " angle: " << finalAngle << endl;
                // dist = NAN;
                newDist = distItem(true, dist);
            }
            
        }
        else
        {
            // if the closest point does not lie on edge
            // then just follow our original plan
            if (angle >= (90+range))
            {
                dist = -dist;
                newDist = distItem(false, dist);
                // vec_pts_in.push_back(pnt);
                // vec_close_in.push_back(nearestPnt);
                
            }
            else
            {
                // dist = NAN;
                newDist = distItem(true, dist);
                // cout << "pnt " << pnt[0] << " " << pnt[1] << " " << pnt[2] 
                //         << " faceid: " << closestTriID << " vert id: "
                //         << vid0 << " " << vid1 << " " << vid2 
                //         << " " << c
                //         << " angle: " << angle << endl;
                // vec_pts_nan.push_back(pnt);
                // vec_close_nan.push_back(nearestPnt);
                
            }                        
        }        
    }
        
    // if ( abs(a) < eps || abs(b) < eps || abs(c) < eps) // closest point lies on the edge
    // {
    //     double prod = -10;
    //     int closestTriNum = 1; // number of triangles that the nearest point connects with
    //     bool closeToBoundPnt = false;
    //     if ( abs(a-1.0) < eps ||  abs(b-1.0) < eps ||  abs(c-1.0) < eps)
    //     {
    //         int vid = vid0;
    //         if (abs(b-1.0) < eps)
    //             vid = vid1;
    //         if (abs(c-1.0) < eps)
    //             vid = vid2;
    //         prod = verifyAngleViaVertConnection(vid, dir, face_normals, vert2Face, closestTriNum);
    //         auto iter = find(boundPoints.begin(), boundPoints.end(), vid);
    //         if (iter != boundPoints.end())
    //             closeToBoundPnt = true;
    //     }else
    //         prod = verifyAngleViaEdgeConnection(a, b, c, closestTriID, dir, F, face_normals, edge2Face, closestTriNum);

    //     if (abs(prod+1.0) < eps)
    //         prod = -1.0;

    //     if (abs(prod-1.0) < eps)
    //         prod = 1.0;
        
    //     if (abs(prod) < eps)
    //         prod = 0.0;


    //     double finalAngle = acos(prod) * 180.0 / PI;

    //     if (closestTriNum > 1 && closeToBoundPnt == false)
    //     {
    //         // if the closest edge connects to more than 1 triangles, aka, the edge is not a boundary edge
    //         // then the query point is either inside or outside, cannot be NAN!
    //         if (prod > 0)
    //         {
    //             dist = dist;
    //             newDist = distItem(false, dist);
                
    //         }else if (prod < 0)
    //         {
    //             dist = -dist;
    //             newDist = distItem(false, dist);    
    //         }else if (prod == 0)
    //         {
    //             // cout << "Attention!! Inner product equals 0! Query point: [" << pnt[0] << ", " << pnt[1] 
    //             //         << ", " << pnt[2] << "]" << endl; 
    //         }
    //     }else
    //     {
    //         // if the closest edge only connects to one face, aka the edge is boundary edge
    //         // then there could be inside/outside/NAN!
    //         if (finalAngle < (90.0-range))
    //         {
    //             dist = dist;
    //             newDist = distItem(false, dist);

                
    //             // vec_pts_out.push_back(pnt);
    //             // vec_close_out.push_back(nearestPnt);
    //         }
    //         else if (finalAngle >= (90+range))
    //         {
    //             dist = -dist;
    //             newDist = distItem(false, dist);
    //             // vec_pts_in.push_back(pnt);
    //             // vec_close_in.push_back(nearestPnt);
    //         }else
    //         {
    //             // vec_pts_nan.push_back(pnt);
    //             // vec_close_nan.push_back(nearestPnt);
    //             // cout << "pnt " << pnt[0] << " " << pnt[1] << " " << pnt[2] 
    //             //         << " faceid: " << closestTriID << " vert id: "
    //             //         << vid0 << " " << vid1 << " " << vid2 
    //             //         << " a: " << a << " b: " << b << " c: " << c
    //             //         << " prod: " << prod
    //             //         << " angle: " << finalAngle << endl;
    //             // dist = NAN;
    //             newDist = distItem(true, dist);
    //             nan_closestPnt_forDebug = nearestPnt;
    //         }
    //     } 
        
    // }
    // else
    // {
    //     if (angle < (90.0 - range))
    //     {
    //         dist = dist;
    //         newDist = distItem(false, dist);

    //         // outPnts.push_back(pnt);
    //         // outPnts_close.push_back(nearestPnt);
    //     }
    //     // if the closest point does not lie on edge
    //     // then just follow our original plan
    //     else if (angle >= (90+range))
    //     {
    //         dist = -dist;
    //         newDist = distItem(false, dist);
    //         // vec_pts_in.push_back(pnt);
    //         // vec_close_in.push_back(nearestPnt);
            
    //     }
    //     else
    //     {
    //         // dist = NAN;
    //         newDist = distItem(false, dist);
    //         nan_closestPnt_forDebug = nearestPnt;
    //         // cout << "pnt " << pnt[0] << " " << pnt[1] << " " << pnt[2] 
    //         //         << " faceid: " << closestTriID << " vert id: "
    //         //         << vid0 << " " << vid1 << " " << vid2 
    //         //         << " " << c
    //         //         << " angle: " << angle << endl;
    //         // vec_pts_nan.push_back(pnt);
    //         // vec_close_nan.push_back(nearestPnt);
            
    //     }                        
    // }        

    
#if FOR_DEBUG
    // newDist.closestPnt = nearestPnt;
#endif
    return newDist;
}


void addDistItem(vector<map<float, distItem>>& S, int i, const distItem& newDist, int maxSize)
{
    if (S[i].size() == maxSize){
        S[i][abs(newDist.dist)] = newDist;
        if (S[i].size() > maxSize)
        {
            auto it = S[i].end();
            it--;
            S[i].erase(it);
        }        
    }else{
        S[i][abs(newDist.dist)] = newDist;
    }
}

/*
    Compute the 3-pole distance field for input points
    input: inputPoints - input query points
    input: m_pqp_models - PQP model for compute unsigned distance
    input: V - mesh vertices
    input: F - mesh faces
    input: face_normals - face normals
    input: vert2Face - given a vertex id, this map stores all the face ids that it connects to
    input: edge2Face - given an edge, this map stores all the face ids that it connects to
    output: S - 3-pole distance for query points 
*/
void compute3PoleDist(const MatrixXd& inputPoints, PQP_Model* m_pqp_model, 
                        const MatrixXd& V, const MatrixXi& F,
                        const MatrixXd& face_normals, 
                        map<int, vector<int>>& vert2Face,
                        map<pair<int, int>, vector<int>>& edge2Face,
                        const vector<int>& boundPoints,
                        vector<map<float, distItem>>& S,
                        int maxSize,
                        double minGridWidth)
{

  #pragma omp parallel for
	for (int i=0; i < inputPoints.rows(); i++)
  {		
    Vector3d pnt = inputPoints.row(i);
    double xi = pnt[0], yi = pnt[1], zi = pnt[2];

    Vector3d nan_closest_pnt;
    auto iter = find(bugPoints.begin(), bugPoints.end(), i);
    if (iter != bugPoints.end())
    {
      cout << "Find!" << endl;
    }
    distItem newDist = compute3PoleDistForOnePoint(pnt,
                            m_pqp_model, V, F,
                            face_normals, vert2Face,
                            edge2Face,
                            boundPoints,
                            nan_closest_pnt
                            );
    addDistItem(S, i, newDist, maxSize);  
  }
}

void computeVert2FacesAndEdge2Faces(const MatrixXd& V, const MatrixXi& F,
                                        map<pair<int, int>, vector<int>>& edge2Face,
                                        map<int, vector<int>>& vert2Face)
{
    for (int i=0; i < F.rows(); ++i)
    {
        int v0 = F.row(i)[0], v1 = F.row(i)[1], v2 = F.row(i)[2];
        if (v0 < v1)
            edge2Face[CEdge(v0, v1)].push_back(i);
        else
            edge2Face[CEdge(v1, v0)].push_back(i);
        if (v0 < v2)
            edge2Face[CEdge(v0, v2)].push_back(i);
        else
            edge2Face[CEdge(v2, v0)].push_back(i);
        if (v2 < v1)
            edge2Face[CEdge(v2, v1)].push_back(i);
        else
            edge2Face[CEdge(v1, v2)].push_back(i);
        
        vert2Face[v0].push_back(i);
        vert2Face[v1].push_back(i);
        vert2Face[v2].push_back(i);            
    }
}

void compute3PoleDistForInputPoints(const MatrixXd& V, const MatrixXi& F, const MatrixXd& inputPoints,
                                    VectorXd& outputDist, double minGridWidth,
                                    int maxBufferSize)
{
    // build edge to face map and vert to face map
    map<pair<int, int>, vector<int>> edge2Face;
    map<int, vector<int>> vert2Face;
    computeVert2FacesAndEdge2Faces(V, F, edge2Face, vert2Face);

    /********************************************************************************************
     * Divide the mesh into open and closed parts, that are to be handled separately
    *********************************************************************************************/

    // extract open mesh parts
    vector<MatrixXd> openVerts;
    vector<vector<int>> openFaces;
    extractOpenMeshes(V, F, vert2Face, openFaces);

    /****** extract closed surfaces *******/ 
    vector<int> closedFaces;
    vector<bool> mask(F.rows(), false);
    
    for(auto faces: openFaces)
    {
        for(auto i : faces)
        {
            mask[i] = true;
        }
    }
    for(int i=0; i < mask.size(); i++)
    {
        if (!mask[i])
            closedFaces.push_back(i);
    }
    MatrixXd closedV;
    MatrixXi closedF;    
    if (closedFaces.size() > 0)
        reOrderMeshIndices(V, F, closedFaces, closedV, closedF);

    int curBatchSize = inputPoints.rows();
    vector<map<float, distItem>> distCollect(curBatchSize);

    /****** deal with closed meshes *******/
    if (closedFaces.size() > 0)
    {
        cout << "The closed part has " << closedFaces.size() << " faces!" << endl;
        
        VectorXd S(curBatchSize, 1);		// S to store distance values
        VectorXd B;
        {
            VectorXi I;
            MatrixXd C,N;
            signed_distance(inputPoints,closedV,closedF,SIGNED_DISTANCE_TYPE_WINDING_NUMBER,S,I,C,N);
        }
        // update the final distance buffer
        for(int k=0; k < curBatchSize; ++k)
        {
            float dist = S(k);
            addDistItem(distCollect, k, distItem(false, dist), maxBufferSize);
        }
    }
    cout << "finished closed computation!" << endl;

    /****** deal with open meshes *******/        
    int cnt = 0;
    cout << "there are " << openFaces.size() << " open face parts!" << endl;
    for(auto faces: openFaces)
    {   
        cout << "*." ;        
        MatrixXi partF;
        MatrixXd partV;
        reOrderMeshIndices(V, F, faces, partV, partF);

        // initialize PQP model
        PQP_Model* m_pqp_model = new PQP_Model();        
        buildPQP(m_pqp_model, partV, partF);       

        map<pair<int, int>, vector<int>> part_edge2Face;
        map<int, vector<int>> part_vert2Face;
        computeVert2FacesAndEdge2Faces(partV, partF, part_edge2Face, part_vert2Face);

        // vector<pair<int,int>> boundEdges;
        // findBoundaryEdges(edge2Face, boundEdges);
        vector<int> boundPoints;
        findBoundaryPoints(edge2Face, boundPoints);

        MatrixXd part_face_normals; // per face normal
        MatrixXd part_Z;
        igl::per_face_normals(partV, partF, part_Z, part_face_normals);
        

        // compute the 3-pole distance for the query points
        compute3PoleDist(inputPoints, m_pqp_model, partV, partF, part_face_normals, 
                            part_vert2Face, part_edge2Face, boundPoints, distCollect, maxBufferSize, minGridWidth);
    
        delete m_pqp_model;        
    }                  
    cout << endl << "finished open computation!" << endl;

    /******* fuse the distances from different mesh components *******/ 
    cout << "Fusing distance ... ";

    VectorXd curS(curBatchSize, 1);
    cout << "distcollect size: " << distCollect.size() << endl;
    fuseDistance(distCollect, curS, minGridWidth, 0.9);
    outputDist = curS;

    cout << "Done." << endl;

    // igl::opengl::glfw::Viewer viewer2;
}


void findBoundaryEdges(map<pair<int, int>, vector<int>> edge2Face, vector<pair<int,int>>& boundEdges)
{
    vector<pair<int,int>> boundaryEdges;
    for(auto iter = edge2Face.begin(); iter != edge2Face.end(); iter++)
    {
        map<int, int> fids;
        vector<int> faces = iter->second;
        for(auto f : faces)
        {
            fids[f] = 1;
        }
        if (fids.size() == 1)
            boundaryEdges.push_back(iter->first);
    }

    boundEdges = boundaryEdges;
}

// find the boundary points via edge2Face
void findBoundaryPoints(map<pair<int, int>, vector<int>> edge2Face, vector<int>& outBoundPoints)
{
    map<int,int> boundaryPoints;
    for(auto iter = edge2Face.begin(); iter != edge2Face.end(); iter++)
    {
        map<int, int> fids;
        vector<int> faces = iter->second;
        for(auto f : faces)
        {
            fids[f] = 1;
        }
        // if the edge only connects to one face, then it must be boundary edge
        // The end points of the boundary edge must be boundary points
        if (fids.size() == 1)
        {
            auto edge = iter->first;
            int vid0 = edge.first;
            int vid1 = edge.second;
            boundaryPoints[vid0] = 1;
            boundaryPoints[vid1] = 1;
        }
    }
    vector<int> output;
    for(auto iter=boundaryPoints.begin(); iter!=boundaryPoints.end(); iter++)
    {
        output.push_back(iter->first);
    }
    outBoundPoints = output;
}

void removeSmallStrips(const MatrixXd& V, const MatrixXi& F, int maxFaceSize, 
                        MatrixXd& outputV, MatrixXi& outputF)
{
    map<pair<int, int>, vector<int>> edge2Face;
    map<int, vector<int>> vert2Face;
    computeVert2FacesAndEdge2Faces(V, F, edge2Face, vert2Face);

    

    vector<MatrixXd> openVerts;
    vector<vector<int>> openFaces;
    extractOpenMeshes(V, F, vert2Face, openFaces);

    vector<int> facesToKeep;
    vector<bool> mask(F.rows(), false);

    for(auto faces: openFaces)
    {
        if (faces.size() <= maxFaceSize)
        {
            for(auto i : faces)
            {
                mask[i] = true;
            }
        }        
    }

    for(int i=0; i < mask.size(); i++)
    {
        if (!mask[i])
            facesToKeep.push_back(i);
    }
    reOrderMeshIndices(V, F, facesToKeep, outputV, outputF);
}
               

// generate the new 3-way distance field proposed by Weikai
void generate3PoleSDF(string meshFileName, string outReconMeshName, int resolution,
                        int batchSize)
{
    // for debug only
    // string filename = "../data/bugbox2.obj";
    // igl::readOBJ(filename,vb,vf);
    // vbmin = vb.colwise().minCoeff();
    // vbmax = vb.colwise().maxCoeff();

	// load obj mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    cout << "Loading " << meshFileName << endl;
	// Read in inputs as double precision floating point meshes
  	igl::readOBJ(meshFileName,V,F);
	cout << "Mesh load done!" << endl;


    /****** create grid query points ******/
    // compute tight bounding box of the input mesh
	RowVector3d tightBmin = V.colwise().minCoeff();
	RowVector3d tightBmax = V.colwise().maxCoeff();
    RowVector3d diff = tightBmax - tightBmin;
    // create proper bounding box for axis-aligned planes
    if (diff[0] == 0.0 || diff[1] == 0.0 || diff[1] == 0.0)
    {
        double maxItem = diff.maxCoeff();
        int minID = 0;
        if (diff[1] == 0)
            minID = 1;
        if (diff[2] == 0)
            minID = 2;
        tightBmin[minID] -= 0.2 * maxItem;
        tightBmax[minID] += 0.2 * maxItem;
    }
	RowVector3d center = (tightBmax + tightBmin) / 2.0;
	// enlarge bounding box little bit
	RowVector3d bmin = center + 1.05 * (tightBmin - center);
	RowVector3d bmax = center + 1.05 * (tightBmax - center);
	RowVector3d bboxSize = bmax - bmin;
	cout << "Bounding corners - min: " << vec2String(bmin) << " max: " << vec2String(bmax) << endl;

	// number of vertices on the largest side
	const int s = resolution;
	const double minGridWidth = (bmax-bmin).maxCoeff()/(double)s;
	const RowVector3i gridSize = ( s * ((bmax-bmin) / (bmax-bmin).maxCoeff()) ).cast<int>();
	cout << "Grid resolution: " << gridSize[0] << " " << gridSize[1] << " " << gridSize[2] << endl;
    cout << "Grid minimum width: " << minGridWidth << endl;

	
	cout << "Creating grid ... ";
	MatrixXd GV(gridSize(0)*gridSize(1)*gridSize(2), 3);	// GV to store grid query points
	VectorXd finalS(gridSize[0]*gridSize[1]*gridSize[2], 1); // final vector to stored computed distance values
	cout << " Done " << endl;

	const auto lerp = [&](const int di, const int d)->double
	{return bmin(d)+(double)di/(double)(gridSize(d)-1)*(bmax(d)-bmin(d));};

  
    for(int x = 0; x< gridSize[0]; x++){
		const double xi = lerp(x, 0);
		for(int y = 0; y < gridSize[1]; y++){
			const double yi = lerp(y, 1);
			for(int z = 0; z < gridSize[2]; z++){
                const double zi = lerp(z, 2);
                GV.row(x+gridSize(0)*(y + gridSize(1)*z)) = RowVector3d(xi,yi,zi);
            }
        }
    }

    // for debug only
    // // extract open mesh parts
    // vector<MatrixXd> openVerts;
    // vector<vector<int>> openFaces;
    // extractOpenMeshes(V, F, vert2Face, openFaces);

    // save part open meshes
    // for (int i=0; i < openFaces.size(); ++i)
    // {
    //     MatrixXi partF;
    //     MatrixXd partV;
    //     reOrderMeshIndices(V, F, openFaces[i], partV, partF);
    //     string filename = "../output/openParts/mesh_" + std::to_string(i) + ".obj";
    //     save_obj_mesh(filename, partV, partF);
    
    // }
    // cout << "finished saving meshes!" << endl;

    /********************************************************************************************
     * Compute 3-pole distance field
    *********************************************************************************************/
    cout << "Computing 3-pole distance field ..." << endl;
    auto start = high_resolution_clock::now();
    int maxBufferSize = 2; // buffer size of stored closest distItems; should be at least 2!!

    int totalNum = gridSize(0)*gridSize(1)*gridSize(2);
    int batchNum = ceil(double(totalNum) / double(batchSize));
    cout << "Total points: " << totalNum << " Batch size: " << batchSize << endl;
    cout << "There will be " << batchNum << " batches in total!" << endl;

    for (int m=0; m < batchNum; m++)
    {
        int startID = m * batchSize;
        int curBatchSize = batchSize;
        if ((m+1)*batchSize > totalNum)
        {
            curBatchSize = totalNum - startID;
        }
            
        cout << "Processing the " << m << "-th batching --------------------------" << endl;
        vector<map<float, distItem>> distCollect(curBatchSize);

        MatrixXd BV(curBatchSize, 3); // batch points
        for (int n=0; n < curBatchSize; n++)
            BV.row(n) = GV.row(startID+n);
        
        VectorXd curS;
        compute3PoleDistForInputPoints(V, F, BV, curS, minGridWidth, maxBufferSize);

        /************* collect batch results ****************/
        for (int n=0; n < curBatchSize; n++)
            finalS(startID+n) = curS(n);

        cout << m << "-th batch done!" << endl;
    }
    
    
	auto stop = high_resolution_clock::now(); 
	auto duration = duration_cast<microseconds>(stop - start); 
	cout << "Computation time: " << duration.count() / double(1000000.0) << " seconds" << endl;	


    /************* Reconstruction ****************/
	// use marching cube to reconstruct
	cout << " Marching cubes ... " << endl;
	start = high_resolution_clock::now();

	MatrixXd SV;
	MatrixXi SF;
	igl::copyleft::marching_cubes(finalS, GV, gridSize(0), gridSize(1), gridSize(2), SV, SF);

	stop = high_resolution_clock::now(); 
	duration = duration_cast<microseconds>(stop - start); 
	cout << "Marching Cube Used time: " << duration.count() / double(1000000.0) << " seconds" << endl;

    
	// save the reconstructed mesh
	save_obj_mesh(outReconMeshName, SV, SF);
    cout << "there are " << SV.rows() << " vertices and " << SF.rows() << " faces!" << endl;
	cout << "Finished writing reconstruction to " << outReconMeshName << "!" << endl;   

    // remove small floating mesh strips
    // MatrixXd cleanedV;
    // MatrixXi cleanedF;
    // removeSmallStrips(SV, SF, 20, cleanedV, cleanedF);
    // string cleanedOBJName = "../output/cleaned.obj";
    // save_obj_mesh(cleanedOBJName, cleanedV, cleanedF);
	// cout << "Finished writing cleaned reconstruction to " << cleanedOBJName << "!" << endl;  


    // for debug only!

    // MatrixXd Cc = RowVector3d(0.4,0.8,0.3).replicate(F.rows(),1);

    // cout << "Outside points inside" << endl;
    // MatrixXd ptsOut(outPnts.size(), 3);
    // MatrixXd closeOut(outPnts_close.size(), 3);
    // for (int i=0; i < outPnts.size(); ++i)
    // {
    //     ptsOut.row(i) = RowVector3d(outPnts[i]);
    //     closeOut.row(i) = RowVector3d(outPnts_close[i]);
    // }

	// igl::opengl::glfw::Viewer viewer2;
	// viewer2.data().set_mesh(V, F);
	// viewer2.data().set_colors(Cc);
    // viewer2.data().add_points(ptsOut,Eigen::RowVector3d(1,0,0));
    // viewer2.data().add_points(closeOut,Eigen::RowVector3d(0,1,0));
    // viewer2.data().add_edges(ptsOut, closeOut,Eigen::RowVector3d(0,0,1));
	// viewer2.launch();
    
#if FOR_DEBUG   

   
    // MatrixXd Cc = RowVector3d(0.4,0.8,0.3).replicate(F.rows(),1);

    // cout << "Inside points " << endl;
    // MatrixXd pts(vec_pts_in.size(), 3);
    // MatrixXd close(vec_close_in.size(), 3);
    // cout << "there are " << pts.size() << " outside points!" << endl;
    // for (int i=0; i < vec_pts_in.size(); ++i)
    // {
    //     pts.row(i) = RowVector3d(GV.row(vec_pts_in[i]));
    //     close.row(i) = RowVector3d(vec_close_in[i]);
    // }

	igl::opengl::glfw::Viewer viewer2;
	// viewer2.data().set_mesh(V, F);
	// viewer2.data().set_colors(Cc);
    // viewer2.data().add_points(pts,Eigen::RowVector3d(1,0,0));
    // viewer2.data().add_points(close,Eigen::RowVector3d(0,1,0));
    // viewer2.data().add_edges(pts, close,Eigen::RowVector3d(0,0,1));
	// viewer2.launch();

    // cout << "Outside points " << endl;
    // MatrixXd pts2(vec_pts_out.size(), 3);
    // MatrixXd close2(vec_close_out.size(), 3);
    // cout << "there are " << pts2.size() << " outside points!" << endl;
    // for (int i=0; i < vec_pts_out.size(); ++i)
    // {
    //     pts2.row(i) = RowVector3d(GV.row(vec_pts_out[i]));
    //     close2.row(i) = RowVector3d(vec_close_out[i]);
    // }

	// igl::opengl::glfw::Viewer viewer3;
	// viewer3.data().set_mesh(V, F);
	// viewer3.data().set_colors(Cc);
    // viewer3.data().add_points(pts2,Eigen::RowVector3d(1,0,0));
    // viewer3.data().add_points(close2,Eigen::RowVector3d(0,1,0));
    // viewer3.data().add_edges(pts2, close2, Eigen::RowVector3d(0,0,1));
	// viewer3.launch();

    // cout << "Nan points " << endl;
    // MatrixXd pts3(vec_pts_nan.size(), 3);
    // MatrixXd close3(vec_close_nan.size(), 3);
    // cout << "there are " << pts3.size() << " outside points!" << endl;
    // for (int i=0; i < vec_pts_nan.size(); ++i)
    // {
    //     pts3.row(i) = RowVector3d(GV.row(vec_pts_nan[i]));
    //     close3.row(i) = RowVector3d(vec_close_nan[i]);
    // }
    // Cc.row(5342) = RowVector3d(1.0,0.3,0.3);
	// igl::opengl::glfw::Viewer viewer4;
	// viewer4.data().set_mesh(V, F);
	// viewer4.data().set_colors(Cc);
    // viewer4.data().add_points(pts3,Eigen::RowVector3d(1,0,0));
    // viewer4.data().add_points(close3,Eigen::RowVector3d(0,1,0));
    // viewer4.data().add_edges(pts3, close3,Eigen::RowVector3d(0,0,1));
	// viewer4.launch();
#endif
}


