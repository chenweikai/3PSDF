#include "dist_compute.h"

#include <igl/signed_distance.h>
#include "manifold/Intersection.h"

#include "utilities.h"

#define PI 3.14159265

typedef std::pair<int,int> CEdge;

// Given point p and triangle (a, b, c), return p's barycentric coordinate (u, v, w)
void Barycentric(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                 const Eigen::Vector3d& c, double &u, double &v, double &w) {
  Eigen::Vector3d v0 = b - a, v1 = c - a, v2 = p - a;
  double d00 = v0.dot(v0); 
  double d01 = v0.dot(v1);
  double d11 = v1.dot(v1);
  double d20 = v2.dot(v0);
  double d21 = v2.dot(v1);
  double denom = d00 * d11 - d01 * d01;
  v = (d11 * d20 - d01 * d21) / denom;
  w = (d00 * d21 - d01 * d20) / denom;
  u = 1.0 - v - w;
}

// Compute sign of the query point via angle weighted pseudonormals
//
// @param closest_vert_id: the id of query point's closest vertex on the mesh
// @param mesh_verts, mesh_faces: mesh vertices and mesh faces
// @param dir: unit vector that pointing from the closest vertex to the query point
// @param face_normals: face normals of the mesh
// @param vert_to_face: mapping from mesh vertex to all face ids that it connects to
//
// @return: the dot product between dir and the pseudonormal, that is later used to compute angle in between
double GetAngleViaPseudonormal(
        const int& closest_vert_id,
        const Eigen::MatrixXd& mesh_verts,
        const Eigen::MatrixXi& mesh_faces,
        const Eigen::Vector3d& dir, 
        const Eigen::MatrixXd& face_normals,
        std::map<int, std::vector<int>>& vert_to_face) {
  auto iter = vert_to_face.find(closest_vert_id);
  if (iter == vert_to_face.end()) {
      std::cout << "The vertex is not found in function verifyAngleViaVertConnection!" << std::endl;
  }

  std::vector<int> faces = vert_to_face[closest_vert_id];
  std::unordered_map<int, int> fids;
  for(auto f : faces)
    fids[f] = 1;

  Eigen::RowVector3d normal_sum(0.0, 0.0, 0.0);
  for(auto i = fids.begin(); i != fids.end(); i++) {
    // Compute angles
    Eigen::RowVector3i f_verts = mesh_faces.row(i->first);
    int v0 = -1;
    int v1 = -1;
    // Find out the other two vertices other than the input vertex id
    for (int j = 0; j < 3; j++) {
      if (f_verts[j] == closest_vert_id) continue;
      if (v0 == -1) {
        v0 = f_verts[j];
        continue;
      }
      if (v1 == -1) v1 = f_verts[j];
    }
    Eigen::RowVector3d edge0 = mesh_verts.row(v0) - mesh_verts.row(closest_vert_id);
    Eigen::RowVector3d edge1 = mesh_verts.row(v1) - mesh_verts.row(closest_vert_id);
    edge0.normalize();
    edge1.normalize();
    double dot_p = edge0.dot(edge1);
    double angle = acos(dot_p);
    Eigen::RowVector3d n = face_normals.row(i->first);
    normal_sum += angle * n;
  }

  normal_sum.normalize();
  double sign = normal_sum.dot(dir);
  return sign;
}

// Compute the dot product between the input direction and average face normals based on vertex connections
//
// @param closest_vert_id: the id of query point's closest vertex on the mesh
// @param dir: unit vector that pointing from the closest vertex to the query point
// @param face_normals: face normals of the mesh
// @param vert_to_face: mapping from mesh vertex to all face ids that it connects to
//
// @return: the dot product between dir and the average normal, that is later used to compute angle in between
double GetAngleViaVertConnection(
    const int& closest_vert_id,
    const Eigen::Vector3d& dir,
    const Eigen::MatrixXd& face_normals,
    std::map<int, std::vector<int>>& vert_to_face) {
  auto iter = vert_to_face.find(closest_vert_id);
  if (iter == vert_to_face.end()) {
      std::cout << "The vertex is not found in function verifyAngleViaVertConnection!" << std::endl;
  }

  std::vector<int> faces = vert_to_face[closest_vert_id];
  std::map<int, int> fids; // use std::map to avoid duplicated face ids
  for(auto f : faces)
    fids[f] = 1;

  double max_dot_product = -1e10;
  Eigen::RowVector3d aveNormal(0.0, 0.0, 0.0);
  for(auto i=fids.begin(); i!=fids.end(); i++) {
    Eigen::RowVector3d n = face_normals.row(i->first);
    aveNormal += n;
  }

  aveNormal = aveNormal / double(fids.size());
  aveNormal.normalize();
  max_dot_product = aveNormal.dot(dir);
  return max_dot_product;
}


// Compute the dot product between the input direction and face normals based on ege connections
//
// @param a, b, c: the barycentric coordinate of v0, v1, v2 of a triangle
// @param face
double GetAngleViaEdgeConnection(
    double a, double b, double c,
    const int& face_id,
    const Eigen::Vector3d& input_dir,
    const Eigen::MatrixXi& faces,
    const Eigen::MatrixXd& face_normals,
    std::map<CEdge, std::vector<int>>& edge2Face) {
  double eps = 1e-4;
  std::map<int, int> fids;
  int v0 = faces.row(face_id)[0];
  int v1 = faces.row(face_id)[1];
  int v2 = faces.row(face_id)[2];
  CEdge e;
  if (abs(a) < eps) {
    if (v1 < v2)
      e = CEdge(v1, v2);
    else
      e = CEdge(v2, v1);
    auto itr = edge2Face.find(e);
    if (itr == edge2Face.end()) {
      std::cout << "The edge is not found in function GetMaxAngleViaEdgeConnection!" << std::endl;
    }
    std::vector<int> faces = itr->second;
    for(auto f : faces)
      fids[f] = 1;
  }
  if (abs(b) < eps) {
    if (v2 < v0)
      e = CEdge(v2, v0);
    else
      e = CEdge(v0, v2);
    auto itr = edge2Face.find(e);
    if (itr == edge2Face.end()) {
      std::cout << "The edge is not found in function GetMaxAngleViaEdgeConnection!" << std::endl;
    }
    std::vector<int> faces = itr->second;
    for(auto f : faces)
      fids[f] = 1;
  }
  if (abs(c) < eps) {
    if (v1 < v0)
      e = CEdge(v1, v0);
    else
      e = CEdge(v0, v1);
    auto itr = edge2Face.find(e);
    if (itr == edge2Face.end()) {
      std::cout << "The edge is not found in function GetMaxAngleViaEdgeConnection!" << std::endl;
    }
    std::vector<int> faces = itr->second;
    for(auto f : faces)
      fids[f] = 1;
  }

  double maxV = -1e10;
  Eigen::RowVector3d ave_normal(0.0, 0.0, 0.0);

  for(auto i=fids.begin(); i!=fids.end(); i++) {
      Eigen::RowVector3d n = face_normals.row(i->first);
      ave_normal += n;
  }

  ave_normal = ave_normal / double(fids.size());
  ave_normal.normalize();
  maxV = ave_normal.dot(input_dir);
  return maxV;
}

void ComputeVert2FacesAndEdge2Faces(
  const Eigen::MatrixXi& faces,
  std::map<std::pair<int, int>, std::vector<int>>& edge2Face,
  std::map<int, std::vector<int>>& vert2Face) {
  for (int i = 0; i < faces.rows(); ++i) {
    int v0 = faces.row(i)[0], v1 = faces.row(i)[1], v2 = faces.row(i)[2];
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

void BuildPqp(PQP_Model* pqp_model, const Eigen::MatrixXd& verts, const Eigen::MatrixXi& faces) {
  pqp_model->BeginModel();
  PQP_REAL p1[3], p2[3], p3[3];	
  for (int i = 0; i < faces.rows(); i++) {
    Eigen::Vector3i vertIdx = faces.row(i);
    int vid0 = vertIdx[0], vid1 = vertIdx[1], vid2 = vertIdx[2];
    p1[0] = (PQP_REAL)(verts.row(vid0)[0]);
    p1[1] = (PQP_REAL)(verts.row(vid0)[1]);
    p1[2] = (PQP_REAL)(verts.row(vid0)[2]);
    
    p2[0] = (PQP_REAL)(verts.row(vid1)[0]);
    p2[1] = (PQP_REAL)(verts.row(vid1)[1]);
    p2[2] = (PQP_REAL)(verts.row(vid1)[2]);
    
    p3[0] = (PQP_REAL)(verts.row(vid2)[0]);
    p3[1] = (PQP_REAL)(verts.row(vid2)[1]);
    p3[2] = (PQP_REAL)(verts.row(vid2)[2]);
    pqp_model->AddTri(p1, p2, p3, i);
  }
  pqp_model->EndModel();
}

double PqpAbsDist(PQP_Model* pqp_model, Eigen::Vector3d query_pnt,
                  Eigen::Vector3d& nearest_pnt, int& closest_face_id) {
  // Perform PQP query
  PQP_DistanceResult result;
  result.last_tri = pqp_model->last_tri;
  PQP_REAL p[3];
  p[0] = query_pnt[0];
  p[1] = query_pnt[1];
  p[2] = query_pnt[2];
  PQP_Distance(&result, pqp_model, p, 0.0, 0.0);

  // Retrieve results
  closest_face_id = result.last_tri->id;
  nearest_pnt = Eigen::Vector3d(result.p1[0], result.p1[1], result.p1[2]);

  return result.Distance();
}

std::vector<double> Compute3psdfPerCell(
    const Eigen::MatrixXd& verts, const Eigen::MatrixXi& faces,
    const std::vector<Eigen::Vector3d>& points,
    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& cell,
    const int cell_id) {
  int face_num = faces.rows();
  Eigen::Vector3d cell_min = cell.first;
  Eigen::Vector3d cell_size = cell.second;
  std::vector<int> included_faces;  // store all faces that intersect with the current cell

  // collect intersected face ids
  for (int fid = 0; fid < face_num; fid++) {
    double cell_center[3] = {cell_min[0], cell_min[1], cell_min[2]};
    double half_size[3] = {0.5 * cell_size[0], 0.5 * cell_size[1], 0.5 * cell_size[2]};
    Eigen::Vector3i f = faces.row(fid);
    double tri_verts[3][3] = {{verts.row(f[0])[0], verts.row(f[0])[1], verts.row(f[0])[2]},
                              {verts.row(f[1])[0], verts.row(f[1])[1], verts.row(f[1])[2]},
                              {verts.row(f[2])[0], verts.row(f[2])[1], verts.row(f[2])[2]}};

    if (triBoxOverlap(cell_center, half_size, tri_verts)) {
      included_faces.push_back(fid);
    }
  }

  std::vector<double> output_dists;  // output

  if (included_faces.size() == 0) {
    for (int x = 0; x < points.size(); x++) {
      output_dists.push_back(NAN);
    }
    return output_dists;
  }

  // reorder the intersected/included trianlge faces
  Eigen::MatrixXi part_faces;
  Eigen::MatrixXd part_verts;
  ReorderMeshIndices(verts, faces, included_faces, part_verts, part_faces);

  // initialize PQP model
  PQP_Model* m_pqp_model = new PQP_Model();
  BuildPqp(m_pqp_model, part_verts, part_faces);

  std::map<std::pair<int, int>, std::vector<int>> part_edge2Face;
  std::map<int, std::vector<int>> part_vert2Face;
  ComputeVert2FacesAndEdge2Faces(part_faces, part_edge2Face, part_vert2Face);

  Eigen::MatrixXd face_normals;
  igl::per_face_normals(part_verts, part_faces, Eigen::Vector3d(1, 1, 1).normalized(), face_normals);

  // iterate over cell corners - compute its distance to the included faces
  // #pragma omp parallel for
  for (int k = 0; k < points.size(); k++) {
    const Eigen::Vector3d& pnt = points[k];
    Eigen::Vector3d nearest_pnt;
    int closest_tri_id;
    double dist = PqpAbsDist(m_pqp_model, pnt, nearest_pnt, closest_tri_id);
    Eigen::RowVector3d n = face_normals.row(closest_tri_id);
    Eigen::Vector3d dir_org = pnt - nearest_pnt;
    Eigen::Vector3d dir = dir_org.normalized();

    if (dist < 1e-10 || isnan(dir[0])) {
      dist = 0.0;
      output_dists.push_back(dist);
      continue;
    }
    double eps = 1e-4;
    double dotprod = dir.dot(n);
    if (abs(dotprod - 1.0) < eps)
      dotprod = 1.0;
    if (abs(dotprod + 1.0) < eps)
      dotprod = -1.0;
    double angle = acos(dotprod) * 180.0 / PI;

    if (abs(dotprod - 1.0) < eps) {
      // in this case, the nearest point lies inside certain triangle
      output_dists.push_back(dist);
      continue;
    } else if (abs(dotprod + 1.0) < eps) {
      output_dists.push_back(-dist);
      continue;
    } else {
      // barycentric coordinates
      double a = -1.0;
      double b = -1.0;
      double c = -1.0;

      int vid0 = part_faces.row(closest_tri_id)[0], vid1 = part_faces.row(closest_tri_id)[1], vid2 = part_faces.row(closest_tri_id)[2];
      Eigen::Vector3d v0 = part_verts.row(vid0);
      Eigen::Vector3d v1 = part_verts.row(vid1);
      Eigen::Vector3d v2 = part_verts.row(vid2);
      Barycentric(nearest_pnt, v0, v1, v2, a, b, c);
      float eps = 1e-4;

      if ( abs(a) < eps || abs(b) < eps || abs(c) < eps) {
        double prod = -10;
        if (abs(a - 1.0) < eps || abs(b - 1.0) < eps ||  abs(c - 1.0) < eps) {
          // closest point lies on the edge
          int vid = vid0;
          if (abs(b-1.0) < eps)
              vid = vid1;
          if (abs(c-1.0) < eps)
              vid = vid2;
          prod = GetAngleViaVertConnection(vid, dir, face_normals, part_vert2Face);
          // prod = GetAngleViaPseudonormal(vid, V, F, dir, face_normals, part_vert2Face);
        }else {
          // closest point lies on the vertex
          prod = GetAngleViaEdgeConnection(a, b, c, closest_tri_id, dir, part_faces, face_normals, part_edge2Face);
        }
        if (abs(prod - 1.0) < eps)
          prod = 1.0;
        if (abs(prod + 1.0) < eps)
          prod = -1.0;
        double final_angle = acos(prod) * 180.0 / PI;
        double final_dist;
        if (final_angle < 90.0) {
          final_dist = dist;
        } else {
          final_dist = -dist;
        }
        output_dists.push_back(final_dist);
      } else {
        // do not lie on the edge and the angle is not smaller than 90 degree
        output_dists.push_back(dist);
      }
    }
  }

  delete m_pqp_model;
  return output_dists;
}
