#include "OpenGLCamera.h"
#include <Eigen/Geometry>

OpenGLCamera::OpenGLCamera(){}

OpenGLCamera::OpenGLCamera(float left, float right, float bottom, float top, float near, float far,
                Vector3d _eye, Vector3d _center, Vector3d _up, int _img_height, int _img_width): 
                l(left), r(right), b(bottom), t(top), n(near), f(far), eye(_eye), center(_center), up(_up),
                img_height(_img_height), img_width(_img_width){}


MatrixXd OpenGLCamera::computeOrthoProjMatrix()
{
    proj = MatrixXd::Zero(4,4);
    proj(0, 0) = 2.0 / (r-l);
    proj(0, 3) = - (r+l) / (r-l);
    proj(1, 1) = 2.0 / (t-b);
    proj(1, 3) = -(t+b) / (t-b);
    proj(2, 2) = -2.0 / (f-n);
    proj(2, 3) = -(f+n) / (f-n);
    proj(3, 3) = 1;

    return proj;   
}

MatrixXd OpenGLCamera::computeOrthoModelViewMatrix()
{
    Vector3d forward = eye - center;
    
    forward.normalize(); // in-place normalization
    Vector3d left = up.cross(forward);
    left.normalize();
    up = forward.cross(left);

    // set rotation part
    modelView = MatrixXd::Identity(4,4);
    modelView.block(0,0,1,3) = left.transpose();
    modelView.block(1,0,1,3) = up.transpose();
    modelView.block(2,0,1,3) = forward.transpose();

    // cout << "left: " << left << endl;
    // cout << "up: " << up << endl;
    // cout << "forward: " << endl << forward << endl;
    // cout << "eye: " << endl << eye << endl;
    // set translation part
    modelView(0, 3) = -left[0] * eye[0] - left[1] * eye[1] - left[2] * eye[2];
    modelView(1, 3) = -up[0] * eye[0] - up[1] * eye[1] - up[2] * eye[2];
    modelView(2, 3) = -forward[0] * eye[0] - forward[1] * eye[1] - forward[2] * eye[2];

    return modelView;
}

MatrixXd OpenGLCamera::computeOrthoNDC2ScreenMatrix()
{
    NDC2Screen = MatrixXd::Identity(4,4);
    NDC2Screen(0,0) = img_width / 2;
    NDC2Screen(0,3) = leftUpCorner[0] + img_width / 2;
    NDC2Screen(1,1) = img_height / 2;
    NDC2Screen(1,3) = leftUpCorner[1] + img_height / 2;
    NDC2Screen(2,2) = (f - n) / 2;
    NDC2Screen(2,3) = (f + n) / 2;

    return NDC2Screen;
}

vector<Vector2i> OpenGLCamera::proj3DPointsToScreen(const vector<Vector3d>& points)
{
    vector<Vector2i> output;
    
    return output;
}