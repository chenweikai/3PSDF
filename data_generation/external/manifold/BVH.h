#ifndef BVH_H_
#define BVH_H_

#include <vector>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
using namespace std;

class BV {
public:
    BV() 
        : tris(0)
    {}
    BV(glm::dvec3& d1, glm::dvec3& d2, glm::dvec3& d3)
    {
        tris = new glm::dvec3[3];
        tris[0] = d1;
        tris[1] = d2;
        tris[2] = d3;
        min_corner = tris[0];
        max_corner = tris[0];
        for (int i = 1; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                if (tris[i][j] < min_corner[j])
                    min_corner[j] = tris[i][j];
                if (tris[i][j] > max_corner[j])
                    max_corner[j] = tris[i][j];
            }
        }
    }
    void Include(std::vector<BV*>& array, int l, int r)
    {
        min_corner = glm::dvec3(1e30,1e30,1e30);
        max_corner = -min_corner;
        for (int i = l; i <= r; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                if (array[i]->min_corner[j] < min_corner[j])
                    min_corner[j] = array[i]->min_corner[j];
                if (array[i]->max_corner[j] > max_corner[j])
                    max_corner[j] = array[i]->max_corner[j];
            }
        }
    }

    bool HitBox(glm::dvec3& o, glm::dvec3& d)
    {
        double min_x = 1e30, max_x = -1e30;
        for (int i = 0; i < 3; ++i)
        {
            double t1 = (min_corner[i] - o[i]) / d[i];
            double t2 = (max_corner[i] - o[i]) / d[i];
            if (t1 > t2)
            {
                double temp = t1;
                t1 = t2;
                t2 = temp;
            }
            if (t1 < min_x)
                min_x = t1;
            if (t2 > max_x)
                max_x = t2;
        }
        return min_x <= max_x && max_x >= 0;
    }
    
    pair<glm::dvec3,bool> rayIntersectsTriangle(glm::dvec3& p, glm::dvec3& d) {
        glm::dvec3 e1, e2, h, s, q;

        double a,f,u,v;
        e1 = tris[1] - tris[0];
        e2 = tris[2] - tris[0];

        h = glm::cross(d, e2);
        a = glm::dot(e1, h);

        if (a > -0.00001 && a < 0.00001)
            return make_pair(glm::dvec3(),false);

        f = 1/a;
        s = p - tris[0];
        u = f * glm::dot(s,h);

        if (u < 0.0 || u > 1.0)
            return make_pair(glm::dvec3(),false);

        q = glm::cross(s,e1);
        v = f * glm::dot(d,q);

        if (v < 0.0 || u + v > 1.0)
            return make_pair(glm::dvec3(),false);

        // at this stage we can compute t to find out where
        // the intersection point is on the line
        double t = f * glm::dot(e2,q);

        if (t > 0.00001) // ray intersection
            return make_pair(tris[0] + u * e1 + v * e2, true);

        else // this means that there is a line intersection
             // but not a ray intersection
            return make_pair(glm::dvec3(),false);
    }

    double operator()(int dim)
    {
        return (min_corner[dim] + max_corner[dim]) * 0.5;
    }

    glm::dvec3 min_corner, max_corner;
    glm::dvec3* tris;
};

class BVH {
public:
    BVH()
    : left(0), right(0), bv(0)
    {}
    ~BVH() {
        clear();
    }
    void clear() {
        if (left)
            delete left;
        if (right)
            delete right;
        delete bv;
        left = 0;
        right = 0;
        bv = 0;
    }
    void updateBVH(std::vector<BV*>& bvs, int dim, int l, int r);
    pair<glm::dvec3,bool> rayIntersect(glm::dvec3& o, glm::dvec3& d);
    int axis;
    BVH *left, *right;
    BV* bv;
    int num;
};
    
#endif