/* 
    Implement OpenGL camera class
    Weikai Chen
*/
#pragma once
#include <iostream>
#include <vector>
#include <string>
#ifndef IGL_NO_EIGEN
    #include <Eigen/Core>
#endif
using namespace std;
using namespace Eigen;


class OpenGLCamera{
public:
    OpenGLCamera();
    OpenGLCamera(float left, float right, float bottom, float top, float near, float far,
                    Vector3d _eye, Vector3d _center, Vector3d _up, int img_height, int img_width);

    // orthogonal projection mode
    MatrixXd computeOrthoProjMatrix();
    MatrixXd computeOrthoModelViewMatrix();
    MatrixXd computeOrthoNDC2ScreenMatrix();
    vector<Vector2i> proj3DPointsToScreen(const vector<Vector3d>& points);

private:
    MatrixXd proj;          // projection matrix
    MatrixXd modelView;     // model view matrix
    MatrixXd NDC2Screen;    // normalized device coordinate to screen coordinate transform matrix

public:
    float l;    // left
    float r;    // right
    float b;    // bottom
    float t;    // top
    float n;    // near
    float f;    // far
    Vector3d eye;                           // eye position
    Vector3d center = Vector3d(1, 0, 0);    // look at center
    Vector3d up = Vector3d(1, 0, 0);        // up vector
    Vector2i leftUpCorner = Vector2i(0, 0); // left up corner position when converting NDC to screen coordinate
    int img_height=500;
    int img_width=500;
};