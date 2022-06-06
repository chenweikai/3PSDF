/*
	Octree definition and basic functionalities
*/

#ifndef OCTREE_H_
#define OCTREE_H_

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <vector>
#include <list>
#include <set>
#include <map>
#include <Eigen/Core>
#include <queue>
#include "Intersection.h"

using namespace std;
using namespace Eigen;

class Grid_Index
{
public:
	Grid_Index() {}
	Grid_Index(int x, int y, int z)
		: id(x, y, z)
	{
	}
	bool operator<(const Grid_Index &ind) const
	{
		int i = 0;
		while (i < 3 && id[i] == ind.id[i])
			i++;
		return (i < 3 && id[i] < ind.id[i]);
	}
	Grid_Index operator+(const Grid_Index &ind) const
	{
		Grid_Index grid(*this);
		grid.id += ind.id;
		return grid;
	}
	Grid_Index operator/(int x) const
	{
		return Grid_Index(id[0] / x, id[1] / x, id[2] / x);
	}
	glm::ivec3 id;
};

class Octree
{
public:
	Octree()
	{
		memset(children, 0, sizeof(Octree *) * 8);
		memset(connection, 0, sizeof(Octree *) * 6);
		memset(empty_connection, 0, sizeof(Octree *) * 6);
		level = 0;
		number = 1;
		occupied = 1;
		exterior = 0;
	}
	Octree(glm::dvec3 &min_c, glm::dvec3 &max_c, vector<glm::ivec3> &faces, float thickness)
	{
		memset(children, 0, sizeof(Octree *) * 8);
		memset(connection, 0, sizeof(Octree *) * 6);
		memset(empty_connection, 0, sizeof(Octree *) * 6);
		level = 0;
		number = 1;
		min_corner = min_c;
		length = max_c - min_corner;
		
		// find the longest dimension of the bounding box
    // Weikai - this is used to make sure the octree cell are always regular cube
    // I will comment it for now
    // 03/20/2021
    // int ind = 0;
		// for (int i = 1; i < 3; ++i)
		// 	if (length[i] > length[ind])
		// 		ind = i;
		// for (int i = 0; i < 3; ++i)
		// {
		// 	min_corner[i] -= (length[ind] - length[i]) * 0.5 + thickness * 0.5;
		// }
		// length = glm::dvec3(length[ind] + thickness, length[ind] + thickness, length[ind] + thickness);

		face_indices = faces;
		face_ind.resize(faces.size());
		for (int i = 0; i < (int)faces.size(); ++i)
			face_ind[i] = i;
		occupied = 1;
		exterior = 0;
	}

	Octree(glm::dvec3 &min_c, glm::dvec3 &length_)
	{
		memset(children, 0, sizeof(Octree *) * 8);
		memset(connection, 0, sizeof(Octree *) * 6);
		memset(empty_connection, 0, sizeof(Octree *) * 6);
		level = 0;
		number = 1;
		min_corner = min_c;
		length = length_;
		occupied = 1;
		exterior = 0;
	}

	~Octree()
	{
		for (int i = 0; i < 8; ++i)
		{
			if (children[i])
				delete children[i];
		}
	}

	bool Is_Exterior(const glm::dvec3 &p)
	{
		for (int i = 0; i < 3; ++i)
			if (p[i] < min_corner[i] || p[i] > min_corner[i] + length[i])
				return true;
		if (!occupied)
			return exterior;
		if (level == 0)
			return false;
		int index = 0;
		for (int i = 0; i < 3; ++i)
		{
			index *= 2;
			if (p[i] > min_corner[i] + length[i] / 2)
				index += 1;
		}
		return children[index]->Is_Exterior(p);
	}

	bool Intersection(int face_index, glm::dvec3 &min_corner, glm::dvec3 &length, vector<glm::dvec3> &vertices)
	{
		double boxcenter[3];
		double boxhalfsize[3];
		double triverts[3][3];
		for (int i = 0; i < 3; ++i)
		{
			boxhalfsize[i] = length[i] * 0.5;
			boxcenter[i] = min_corner[i] + boxhalfsize[i];
		}
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				triverts[i][j] = vertices[face_indices[face_index][i]][j];
			}
		}
		return triBoxOverlap(boxcenter, boxhalfsize, triverts);
	}

	// get the sign (inside or outside of the input mesh) of query points
	// outside: -1; inside: +1
	vector<double> getSign(const MatrixXd &pts, const set<Octree *> &outsideCellNodes)
	{
		vector<double> outputSigns;
		for (int i = 0; i < pts.rows(); ++i)
		{
			glm::dvec3 curP(pts.row(i)[0], pts.row(i)[1], pts.row(i)[2]);
			Octree *node = getLeafNodeContainP(curP);
			auto iter = outsideCellNodes.find(node);
			if (iter != outsideCellNodes.end())
			{
				// cout << "Outside!" << endl;
				outputSigns.push_back(-1);
			}
			else
			{
				// cout << "Inside!!" << endl;
				outputSigns.push_back(1);
			}
		}
		return outputSigns;
	}

	void splitUntilAllPointsContainedByEmptyNode(const MatrixXd &pts, vector<glm::dvec3> &vertices)
	{	cout << "start splitting the Octree ... ";
		if (vertices.empty())
		{
			cerr << "Vertices are empty! Pleasd load points first!" << endl;
		}

		for (int i = 0; i < pts.rows(); ++i)
		{
			Octree *parent = this;

			glm::dvec3 curP(pts.row(i)[0], pts.row(i)[1], pts.row(i)[2]);
			// cout << "Processing point (" << curP.x << ", " << curP.y
			// 	<< ", " << curP.z << ")" << endl;
			while (1)
			{
				Octree *node = parent->getLeafNodeContainP(curP);
				if (!node)
				{
					assert(node != NULL);
				}
				// contained by an empty leaf node
				if (!node->occupied)
					break;
				node->Split(vertices);
				parent = node;
			}
		}
	}

	Octree *getLeafNodeContainP(const glm::dvec3 &p)
	{
		if (!ifPointInside(p) && level == 0)
		{
			cerr << "Input point is not even in the root node!" << endl;
			return NULL;
		}
		queue<Octree *> rally;
		rally.push(this);
		Octree *output = NULL;
		while (!rally.empty())
		{
			Octree *cur = rally.front();
			rally.pop();
			if (!cur->ifPointInside(p))
				continue;
			if (cur->level == 0)
			{
				return cur;
			}
			for (int i = 0; i < 8; ++i)
				rally.push(cur->children[i]);
		}
		return output;
	}

	// check if the query point is inside the cell
	bool ifPointInside(const glm::dvec3 &p)
	{
		for (int i = 0; i < 3; ++i)
			if (p[i] < min_corner[i] || p[i] > min_corner[i] + length[i])
				return false;

		return true;
	}

	bool ifPointInside(const Vector3d &p)
	{
		for (int i = 0; i < 3; ++i)
			if (p[i] < min_corner[i] || p[i] > min_corner[i] + length[i])
				return false;

		return true;
	}

	bool ifPointsInside(const MatrixXd &pts)
	{
		bool anyInside = false;
		for (int j = 0; j < pts.rows(); j++)
		{
			auto p = pts.row(j);
			bool thisPointInside = ifPointInside(p);
			if (thisPointInside)
				anyInside = true;
		}
		return anyInside;
	}

	void SplitUntilAllPtsInEmptyNode(const glm::vec3 &p, vector<glm::dvec3> &vertices)
	{
		level += 1;
		number = 0;
		if (level > 1)
		{
			for (int i = 0; i < 2; ++i)
			{
				for (int j = 0; j < 2; ++j)
				{
					for (int k = 0; k < 2; ++k)
					{
						int ind = i * 4 + j * 2 + k;
						if (children[ind] &&
							children[ind]->ifPointInside(p) && children[ind]->occupied)
						{
							children[ind]->SplitUntilAllPtsInEmptyNode(p, vertices);
							number += children[ind]->number;
						}
					}
				}
			}
			face_indices.clear();
			face_ind.clear();
			return;
		}
		glm::dvec3 halfsize = length * glm::dvec3(0.5, 0.5, 0.5);
		for (int i = 0; i < 2; ++i)
		{
			for (int j = 0; j < 2; ++j)
			{
				for (int k = 0; k < 2; ++k)
				{
					int ind = i * 4 + j * 2 + k;

					glm::dvec3 startpoint = min_corner;
					startpoint[0] = startpoint[0] + i * halfsize[0];
					startpoint[1] = startpoint[1] + j * halfsize[1];
					startpoint[2] = startpoint[2] + k * halfsize[2];

					children[ind] = new Octree(startpoint, halfsize);
					children[ind]->occupied = 0;
					children[ind]->number = 0;

					for (int face = 0; face < (int)face_indices.size(); ++face)
					{
						if (Intersection(face, startpoint, halfsize, vertices))
						{
							children[ind]->face_indices.push_back(face_indices[face]);
							children[ind]->face_ind.push_back(face_ind[face]);
							if (children[ind]->occupied == 0)
							{
								children[ind]->occupied = 1;
								number += 1;
								children[ind]->number = 1;
							}
						}
					}
				}
			}
		}
		face_indices.clear();
		face_ind.clear();
	}

	void Split(vector<glm::dvec3> &vertices)
	{
		level += 1;
		number = 0;
		if (level > 1)
		{
			for (int i = 0; i < 2; ++i)
			{
				for (int j = 0; j < 2; ++j)
				{
					for (int k = 0; k < 2; ++k)
					{
						int ind = i * 4 + j * 2 + k;
            // for debug only -- comment out for now
						if (children[ind] && children[ind]->occupied)
						{
							children[ind]->Split(vertices);
							number += children[ind]->number;
						}
					}
				}
			}
			face_indices.clear();
			face_ind.clear();
			return;
		}
		glm::dvec3 halfsize = length * glm::dvec3(0.5, 0.5, 0.5);
		for (int i = 0; i < 2; ++i)
		{
			for (int j = 0; j < 2; ++j)
			{
				for (int k = 0; k < 2; ++k)
				{
					int ind = i * 4 + j * 2 + k;

					glm::dvec3 startpoint = min_corner;
					startpoint[0] = startpoint[0] + i * halfsize[0];
					startpoint[1] = startpoint[1] + j * halfsize[1];
					startpoint[2] = startpoint[2] + k * halfsize[2];

					children[ind] = new Octree(startpoint, halfsize);
					children[ind]->occupied = 0;
					children[ind]->number = 0;

					for (int face = 0; face < (int)face_indices.size(); ++face)
					{
						if (Intersection(face, startpoint, halfsize, vertices))
						{
							children[ind]->face_indices.push_back(face_indices[face]);
							children[ind]->face_ind.push_back(face_ind[face]);
							if (children[ind]->occupied == 0)
							{
								children[ind]->occupied = 1;
								number += 1;
								children[ind]->number = 1;
							}
						}
					}
				}
			}
		}
		face_indices.clear();
		face_ind.clear();
	}

	void BuildConnection(const MatrixXd &V, const MatrixXi &F)
	{
		if (level == 0)
			return;
		for (int i = 0; i < 8; ++i)
		{
			if (children[i])
			{
				children[i]->BuildConnection(V, F);
			}
		}
		int y_index[] = {0, 1, 4, 5};
		for (int i = 0; i < 4; ++i)
		{
			if (children[i * 2] && children[i * 2 + 1])
			{
				ConnectTree(children[i * 2], children[i * 2 + 1], 2); // dim = 2 is z-axis direction
			}
			if (children[y_index[i]] && children[y_index[i] + 2])
			{
				ConnectTree(children[y_index[i]], children[y_index[i] + 2], 1); // dim = 1 is y-axis direction
			}
			if (children[i] && children[i + 4])
			{
				ConnectTree(children[i], children[i + 4], 0); // dim = 0 is x-axis direction
			}
		}
	}

	void BuildConnection()
	{
		if (level == 0)
			return;
		for (int i = 0; i < 8; ++i)
		{
			if (children[i])
			{
				children[i]->BuildConnection();
			}
		}
		int y_index[] = {0, 1, 4, 5};
		for (int i = 0; i < 4; ++i)
		{
			if (children[i * 2] && children[i * 2 + 1])
			{
				ConnectTree(children[i * 2], children[i * 2 + 1], 2);
			}
			if (children[y_index[i]] && children[y_index[i] + 2])
			{
				ConnectTree(children[y_index[i]], children[y_index[i] + 2], 1);
			}
			if (children[i] && children[i + 4])
			{
				ConnectTree(children[i], children[i + 4], 0);
			}
		}
	}

	void ConnectTree(Octree *l, Octree *r, int dim)
	{
		int y_index[] = {0, 1, 4, 5};
		if (dim == 2)
		{
			l->connection[2] = r;
			r->connection[5] = l;
			for (int i = 0; i < 4; ++i)
			{
				if (l->children[i * 2 + 1] && r->children[i * 2])
				{
					ConnectTree(l->children[i * 2 + 1], r->children[i * 2], dim);
				}
			}
		}
		else if (dim == 1)
		{
			l->connection[1] = r;
			r->connection[4] = l;
			for (int i = 0; i < 4; ++i)
			{
				if (l->children[y_index[i] + 2] && r->children[y_index[i]])
				{
					ConnectTree(l->children[y_index[i] + 2], r->children[y_index[i]], dim);
				}
			}
		}
		else if (dim == 0)
		{
			l->connection[0] = r;
			r->connection[3] = l;
			for (int i = 0; i < 4; ++i)
			{
				if (l->children[i + 4] && r->children[i])
				{
					ConnectTree(l->children[i + 4], r->children[i], dim);
				}
			}
		}
	}

	void 
	ExpandEmpty(list<Octree *> &empty_list, set<Octree *> &empty_set, int dim)
	{
		if (!occupied)
		{
			if (empty_set.find(this) == empty_set.end())
			{
				empty_set.insert(this);
				empty_list.push_back(this);
			}
			return;
		}
		if (level == 0)
			return;
		int y_index[] = {0, 1, 4, 5};
		if (dim == 2 || dim == 5)
		{
			for (int i = 0; i < 4; ++i)
			{
				children[i * 2 + (dim == 5)]->ExpandEmpty(empty_list, empty_set, dim);
			}
			return;
		}
		if (dim == 1 || dim == 4)
		{
			for (int i = 0; i < 4; ++i)
			{
				children[y_index[i] + 2 * (dim == 4)]->ExpandEmpty(empty_list, empty_set, dim);
			}
			return;
		}
		for (int i = 0; i < 4; ++i)
		{
			children[i + 4 * (dim == 3)]->ExpandEmpty(empty_list, empty_set, dim);
		}
	}
	// build connection between all exterior empty cells
	void BuildExteriorEmptyConnection()
	{
		if (level == 0)
			return;

		for (int i = 0; i < 8; ++i)
		{
			if (children[i]->occupied)
			{
				children[i]->BuildExteriorEmptyConnection();
			}
		}
		int pair_x[] = {0, 2, 4, 6, 0, 1, 4, 5, 0, 1, 2, 3};
		int pair_y[] = {1, 3, 5, 7, 2, 3, 6, 7, 4, 5, 6, 7};
		int dim[] = {2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0};
		for (int i = 0; i < 12; ++i)
		{
			ConnectEmptyTree(children[pair_x[i]], children[pair_y[i]], dim[i]);
		}
	}

	void BuildExteriorEmptyConnection(const MatrixXd& V, const MatrixXi& F, const vector<Vector3d>& debugPts)
	{
		if (level == 0)
			return;

		for (int i = 0; i < 8; ++i)
		{
			if (children[i]->occupied)
			{
				children[i]->BuildExteriorEmptyConnection(V, F, debugPts);
			}
		}
		int pair_x[] = {0, 2, 4, 6, 0, 1, 4, 5, 0, 1, 2, 3};
		int pair_y[] = {1, 3, 5, 7, 2, 3, 6, 7, 4, 5, 6, 7};
		int dim[] = {2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0};
		for (int i = 0; i < 12; ++i)
		{
			ConnectEmptyTree(children[pair_x[i]], children[pair_y[i]], dim[i], V, F, debugPts);
		}
	}

	void ConnectEmptyTree(Octree *l, Octree *r, int dim, const MatrixXd& V, const MatrixXi& F, const vector<Vector3d>& debugPts)
	{
		int y_index[] = {0, 1, 4, 5};
		if (l->occupied && r->occupied)
		{
			// if (l->level == 0)
			// Weikai
			if (l->level == 0 || r->level == 0)
				return;
			if (dim == 2)
			{
				for (int i = 0; i < 4; ++i)
				{
					ConnectEmptyTree(l->children[i * 2 + 1], r->children[i * 2], dim, V, F, debugPts);
				}
			}
			else if (dim == 1)
			{
				for (int i = 0; i < 4; ++i)
				{
					ConnectEmptyTree(l->children[y_index[i] + 2], r->children[y_index[i]], dim, V, F, debugPts);
				}
			}
			else if (dim == 0)
			{
				for (int i = 0; i < 4; ++i)
				{
					ConnectEmptyTree(l->children[i + 4], r->children[i], dim, V, F, debugPts);
				}
			}
			return;
		}
		if (!(l->occupied || r->occupied))
		{
			l->empty_neighbors.push_back(r);
			r->empty_neighbors.push_back(l); // maybe empty counted as occupied

			// for debug only
			bool bugHere = false;
			Vector3d containedPt;
			for(int j=0; j < debugPts.size(); j++)
			{
				if (l->ifPointInside(debugPts[j]) || r->ifPointInside(debugPts[j]))
				{
					bugHere = true;
					containedPt = debugPts[j];
					break;
				}
			}
			if(bugHere)
			{
				cout << "fucking bug here!!!" << endl;
				cout << "Containing grd point: " << containedPt << endl;
			}

			return;
		}
		if (!l->occupied)
		{
			if (dim == 2)
			{
				r->empty_connection[5] = l;
				if (r->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l, r->children[i * 2], dim, V, F, debugPts);
					}
				}
			}
			else if (dim == 1)
			{
				r->empty_connection[4] = l;
				if (r->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l, r->children[y_index[i]], dim, V, F, debugPts);
					}
				}
			}
			else if (dim == 0)
			{
				r->empty_connection[3] = l;
				if (r->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l, r->children[i], dim, V, F, debugPts);
					}
				}
			}
			return;
		}
		if (!r->occupied)
		{
			if (dim == 2)
			{
				l->empty_connection[2] = r;
				if (l->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l->children[i * 2 + 1], r, dim, V, F, debugPts);
					}
				}
			}
			else if (dim == 1)
			{
				l->empty_connection[1] = r;
				if (l->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l->children[y_index[i] + 2], r, dim, V, F, debugPts);
					}
				}
			}
			else if (dim == 0)
			{
				l->empty_connection[0] = r;
				if (l->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l->children[i + 4], r, dim, V, F, debugPts);
					}
				}
			}
		}
	}



	void ConnectEmptyTree(Octree *l, Octree *r, int dim)
	{
		int y_index[] = {0, 1, 4, 5};
		if (l->occupied && r->occupied)
		{
			// if (l->level == 0)
			// Weikai
			if (l->level == 0 || r->level == 0)
				return;
			if (dim == 2)
			{
				for (int i = 0; i < 4; ++i)
				{
					ConnectEmptyTree(l->children[i * 2 + 1], r->children[i * 2], dim);
				}
			}
			else if (dim == 1)
			{
				for (int i = 0; i < 4; ++i)
				{
					ConnectEmptyTree(l->children[y_index[i] + 2], r->children[y_index[i]], dim);
				}
			}
			else if (dim == 0)
			{
				for (int i = 0; i < 4; ++i)
				{
					ConnectEmptyTree(l->children[i + 4], r->children[i], dim);
				}
			}
			return;
		}
		if (!(l->occupied || r->occupied))
		{
			l->empty_neighbors.push_back(r);
			r->empty_neighbors.push_back(l); // maybe empty counted as occupied
			return;
		}
		if (!l->occupied)
		{
			if (dim == 2)
			{
				r->empty_connection[5] = l;
				if (r->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l, r->children[i * 2], dim);
					}
				}
			}
			else if (dim == 1)
			{
				r->empty_connection[4] = l;
				if (r->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l, r->children[y_index[i]], dim);
					}
				}
			}
			else if (dim == 0)
			{
				r->empty_connection[3] = l;
				if (r->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l, r->children[i], dim);
					}
				}
			}
			return;
		}
		if (!r->occupied)
		{
			if (dim == 2)
			{
				l->empty_connection[2] = r;
				if (l->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l->children[i * 2 + 1], r, dim);
					}
				}
			}
			else if (dim == 1)
			{
				l->empty_connection[1] = r;
				if (l->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l->children[y_index[i] + 2], r, dim);
					}
				}
			}
			else if (dim == 0)
			{
				l->empty_connection[0] = r;
				if (l->level > 0)
				{
					for (int i = 0; i < 4; ++i)
					{
						ConnectEmptyTree(l->children[i + 4], r, dim);
					}
				}
			}
		}
	}

	void traverse(vector<pair<Vector3d, Vector3d>> &cells, int &count)
	{
		// cout << "level: " << level << endl;
		if (level == 0)
		{
			count++;
			cells.push_back(make_pair(Vector3d(min_corner[0], min_corner[1], min_corner[2]), Vector3d(length[0], length[1], length[2])));
			// cout << "There is no children!" << endl;
			// cout << "min corner: " << min_corner[0] << " " << min_corner[1] << " " << min_corner[2] << endl;
			// cout << "length: " << length[0] << " " << length[1] << " " << length[2] << endl;
		}
		else
		{
			// cout << "Checking children!" << endl;
			for (int i = 0; i < 8; ++i)
			{
				children[i]->traverse(cells, count);
			}
		}
	}

  // return the cells that contain the query point
  void getBoundingCellsForQueryPnt(const RowVector3d& p, vector<pair<Vector3d, Vector3d>>& output)
  {
    if (level == 0)
    {
      if (ifPointInside(Vector3d(p)))
        output.push_back(make_pair(Vector3d(min_corner[0], min_corner[1], min_corner[2]), Vector3d(length[0], length[1], length[2])));
    } else {
      for (int i = 0; i < 8; ++i) {
				children[i]->getBoundingCellsForQueryPnt(p, output);
			}
    }
  }

	void traverseOccupiedCells(vector<pair<Vector3d, Vector3d>> &cells, int &count)
	{
		// cout << "level: " << level << endl;
		if (level == 0)
		{
			count++;
			if (occupied)
				cells.push_back(make_pair(Vector3d(min_corner[0], min_corner[1], min_corner[2]), Vector3d(length[0], length[1], length[2])));
			// cout << "There is no children!" << endl;
			// cout << "min corner: " << min_corner[0] << " " << min_corner[1] << " " << min_corner[2] << endl;
			// cout << "length: " << length[0] << " " << length[1] << " " << length[2] << endl;
		}
		else
		{
			// cout << "Checking children!" << endl;
			for (int i = 0; i < 8; ++i)
			{
				children[i]->traverseOccupiedCells(cells, count);
			}
		}
	}

	void traverseEmptyCells(vector<pair<Vector3d, Vector3d>> &cells, int &count)
	{
		// cout << "level: " << level << endl;
		if (level == 0)
		{
			count++;
			if (!occupied)
				cells.push_back(make_pair(Vector3d(min_corner[0], min_corner[1], min_corner[2]), Vector3d(length[0], length[1], length[2])));
		}
		else
		{
			// cout << "Checking children!" << endl;
			for (int i = 0; i < 8; ++i)
			{
				children[i]->traverseEmptyCells(cells, count);
			}
		}
	}

	void ConstructFace(map<Grid_Index, int> &vcolor, const glm::ivec3 &start, vector<glm::dvec3> &vertices, vector<glm::ivec4> &faces, vector<set<int>> &v_faces)
	{
		if (level == 0)
		{
			if (!occupied)
				return;
			glm::ivec3 offset[6][4] = {{glm::ivec3(1, 0, 0), glm::ivec3(1, 0, 1), glm::ivec3(1, 1, 1), glm::ivec3(1, 1, 0)},
									   {glm::ivec3(0, 1, 0), glm::ivec3(1, 1, 0), glm::ivec3(1, 1, 1), glm::ivec3(0, 1, 1)},
									   {glm::ivec3(0, 0, 1), glm::ivec3(0, 1, 1), glm::ivec3(1, 1, 1), glm::ivec3(1, 0, 1)},
									   {glm::ivec3(0, 0, 0), glm::ivec3(0, 1, 0), glm::ivec3(0, 1, 1), glm::ivec3(0, 0, 1)},
									   {glm::ivec3(0, 0, 0), glm::ivec3(0, 0, 1), glm::ivec3(1, 0, 1), glm::ivec3(1, 0, 0)},
									   {glm::ivec3(0, 0, 0), glm::ivec3(1, 0, 0), glm::ivec3(1, 1, 0), glm::ivec3(0, 1, 0)}};
			for (int i = 0; i < 6; ++i)
			{
				if (empty_connection[i] && empty_connection[i]->exterior)
				{
					if (connection[i] && connection[i]->occupied)
					{
						cout << "Error!\n";
						exit(0);
					}
					int id[4];
					for (int j = 0; j < 4; ++j)
					{
						glm::ivec3 vind = start + offset[i][j];
						Grid_Index v_id;
						v_id.id = vind * 2;
						map<Grid_Index, int>::iterator it = vcolor.find(v_id);
						if (it == vcolor.end())
						{
							glm::dvec3 d = min_corner;
							for (int k = 0; k < 3; ++k)
								d[k] += offset[i][j][k] * length[k];
							vcolor.insert(make_pair(v_id, vertices.size()));
							id[j] = vertices.size();
							vertices.push_back(d);
							v_faces.push_back(set<int>());
							for (vector<int>::iterator it1 = face_ind.begin();
								 it1 != face_ind.end(); ++it1)
								v_faces[id[j]].insert(*it1);
						}
						else
						{
							id[j] = it->second;
							for (vector<int>::iterator it1 = face_ind.begin();
								 it1 != face_ind.end(); ++it1)
								v_faces[it->second].insert(*it1);
						}
					}
					faces.push_back(glm::ivec4(id[0], id[1], id[2], id[3]));
				}
			}
		}
		else
		{
			for (int i = 0; i < 8; ++i)
			{
				if (children[i] && children[i]->occupied)
				{
					int x = i / 4;
					int y = (i - x * 4) / 2;
					int z = i - x * 4 - y * 2;
					glm::ivec3 nstart = start * 2 + glm::ivec3(x, y, z);
					children[i]->ConstructFace(vcolor, nstart, vertices, faces, v_faces);
				}
			}
		}
	}

	glm::dvec3 min_corner, length;
	int level;
	int number;
	int occupied;
	int exterior;
	Octree *children[8];
	Octree *connection[6];
	Octree *empty_connection[6];
	list<Octree *> empty_neighbors; // only empty cell will have empty_neighbors

	vector<glm::ivec3> face_indices;
	vector<int> face_ind;
};

#endif
