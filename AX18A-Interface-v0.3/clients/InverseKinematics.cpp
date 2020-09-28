//
// Created by 16076710 on 18/04/18.
//
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <tcp/AX18ARemoteInterface.h>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <math.h>
#include "tcp/AX-18A-Comm.h"

using namespace std;
using namespace Eigen;


//joint[alpha i, ai, theita i, di];
double joint1DHIK[4] = {3.14/2, 0.052, 0, 0.148};
double joint2DHIK[4] = {0, 0.173, 3.14, 0};
double joint3DHIK[4] = {-3.14/2, 0.025, -3.14/2, 0};
double joint4DHIK[4] = {0, 0, 0, 0.23};

//dh convention
Eigen::MatrixXd setDH2(double dhinfo[], double angle){
    Eigen::MatrixXd dhFormat(4, 4);
    dhFormat(0,0) = cos(angle + dhinfo[2]);
    dhFormat(0,1) = -sin(angle + dhinfo[2])*cos(dhinfo[0]);
    dhFormat(0,2) = sin(angle + dhinfo[2])*sin(dhinfo[0]);
    dhFormat(0,3) = dhinfo[1]*cos(angle + dhinfo[2]);
    dhFormat(1,0) = sin(angle + dhinfo[2]);
    dhFormat(1,1) = cos(angle + dhinfo[2])*cos(dhinfo[0]);
    dhFormat(1,2) = -cos(angle + dhinfo[2])*sin(dhinfo[0]);
    dhFormat(1,3) = dhinfo[1]*sin(angle + dhinfo[2]);
    dhFormat(2,0) = 0;
    dhFormat(2,1) = sin(dhinfo[0]);
    dhFormat(2,2) = cos(dhinfo[0]);
    dhFormat(2,3) = dhinfo[3];
    dhFormat(3,0) = 0;
    dhFormat(3,1) = 0;
    dhFormat(3,2) = 0;
    dhFormat(3,3) = 1;
    return dhFormat;
};

Eigen::Vector3d getPositionFK2(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd c, Eigen::MatrixXd d){
    Eigen::MatrixXd final = a*b*c*d;
    Eigen::VectorXd zeroZeroZeroOne(4);
    zeroZeroZeroOne[0] = 0;
    zeroZeroZeroOne[1] = 0;
    zeroZeroZeroOne[2] = 0;
    zeroZeroZeroOne[3] = 1;
    Eigen::Vector3d x = (final * zeroZeroZeroOne).block(0,0,3,1);
    return x;
}

Vector3d getZ(MatrixXd a){
    Vector3d z;
    z[0] = a(0, 2);
    z[1] = a(1, 2);
    z[2] = a(2, 2);
    return z;
}

Vector3d getP(MatrixXd a){
    Vector3d p;
    p[0] = a(0, 3);
    p[1] = a(1, 3);
    p[2] = a(2, 3);
    return p;
}



//rgb xyz
Vector4d getNextAngleIK(Vector4d currentAngle, Vector3d xFinal, double k) {

    //Jacobian matrix
    Eigen::MatrixXd jointJ1 = setDH2(joint1DHIK, currentAngle[0]);
    Eigen::MatrixXd jointJ2 = setDH2(joint2DHIK, currentAngle[1]);
    Eigen::MatrixXd jointJ3 = setDH2(joint3DHIK, currentAngle[2]);
    Eigen::MatrixXd jointJ4 = setDH2(joint4DHIK, currentAngle[3]);

    //Creating Transfer matrices
    MatrixXd t2 = jointJ1 * jointJ2;
    MatrixXd t3 = jointJ1 * jointJ2 * jointJ3;
    MatrixXd t4 = jointJ1 * jointJ2 * jointJ3 * jointJ4;

    //Creating Zi
    Vector3d z0(3);
    z0[0] = 0;
    z0[1] = 0;
    z0[2] = 1;
    Vector3d z1 = getZ(jointJ1);
    Vector3d z2 = getZ(t2);
    Vector3d z3 = getZ(t3);

    //Creating pi
    Vector3d p0;
    p0[0] = 0;
    p0[1] = 0;
    p0[2] = 0;
    Vector3d p1 = getP(jointJ1);
    Vector3d p2 = getP(t2);
    Vector3d p3 = getP(t3);
    Vector3d p = getP(t4);




    //Creating The Jacobian Matrix
    /*Matrix<Vector3d, Dynamic, Dynamic> J(2,4);
    J(0,0) = z0.cross(vector3Subtraction(p, p0));
    J(0,1) = z1.cross(vector3Subtraction(p, p1));
    J(0,2) = z2.cross(vector3Subtraction(p, p2));
    J(0,3) = z3.cross(vector3Subtraction(p, p3));
    J(1,0) = z0;
    J(1,1) = z1;
    J(1,2) = z2;
    J(1,3) = z3;*/

    MatrixXd j(6,4);
    //upper
    j(0,0) = z0.cross(p - p0)[0];
    j(1,0) = z0.cross(p - p0)[1];
    j(2,0) = z0.cross(p - p0)[2];
    j(0,1) = z1.cross(p - p1)[0];
    j(1,1) = z1.cross(p - p1)[1];
    j(2,1) = z1.cross(p - p1)[2];
    j(0,2) = z2.cross(p - p2)[0];
    j(1,2) = z2.cross(p - p2)[1];
    j(2,2) = z2.cross(p - p2)[2];
    j(0,3) = z3.cross(p - p3)[0];
    j(1,3) = z3.cross(p - p3)[1];
    j(2,3) = z3.cross(p - p3)[2];

    //lower
    j(3,0) = z0[0];
    j(4,0) = z0[1];
    j(5,0) = z0[2];
    j(3,1) = z1[0];
    j(4,1) = z1[1];
    j(5,1) = z1[2];
    j(3,2) = z2[0];
    j(4,2) = z2[1];
    j(5,2) = z2[2];
    j(3,3) = z3[0];
    j(4,3) = z3[1];
    j(5,3) = z3[2];



    //Define the final position and the new desired effector motion
    Vector3d xCurrent = getPositionFK2(jointJ1, jointJ2, jointJ3, jointJ4);



    Vector3d xNext = k* (xFinal - xCurrent);


    //Finding the J.inverse()
    MatrixXd jTranslation = j.block(0,0,3,4);
    MatrixXd jT = jTranslation.transpose();
    MatrixXd jInversed = jT * (jTranslation * jT).inverse();

    //Finding the new angles
    Vector4d qNewDifference = jInversed * xNext;

    Vector4d qNew = currentAngle + qNewDifference;

    return qNew;

}


Vector4d getNextAngleIKPointing(Vector4d currentAngle, Vector3d xFinal) {

    //Jacobian matrix
    Eigen::MatrixXd jointJ1 = setDH2(joint1DHIK, currentAngle[0]);
    Eigen::MatrixXd jointJ2 = setDH2(joint2DHIK, currentAngle[1]);
    Eigen::MatrixXd jointJ3 = setDH2(joint3DHIK, currentAngle[2]);
    Eigen::MatrixXd jointJ4 = setDH2(joint4DHIK, currentAngle[3]);

    //Creating Transfer matrices
    MatrixXd t2 = jointJ1 * jointJ2;
    MatrixXd t3 = jointJ1 * jointJ2 * jointJ3;
    MatrixXd t4 = jointJ1 * jointJ2 * jointJ3 * jointJ4;

    //Creating Zi
    Vector3d z0(3);
    z0[0] = 0;
    z0[1] = 0;
    z0[2] = 1;
    Vector3d z1 = getZ(jointJ1);
    Vector3d z2 = getZ(t2);
    Vector3d z3 = getZ(t3);

    //Creating pi
    Vector3d p0;
    p0[0] = 0;
    p0[1] = 0;
    p0[2] = 0;
    Vector3d p1 = getP(jointJ1);
    Vector3d p2 = getP(t2);
    Vector3d p3 = getP(t3);
    Vector3d p = getP(t4);




    //Creating The Jacobian Matrix
    /*Matrix<Vector3d, Dynamic, Dynamic> J(2,4);
    J(0,0) = z0.cross(vector3Subtraction(p, p0));
    J(0,1) = z1.cross(vector3Subtraction(p, p1));
    J(0,2) = z2.cross(vector3Subtraction(p, p2));
    J(0,3) = z3.cross(vector3Subtraction(p, p3));
    J(1,0) = z0;
    J(1,1) = z1;
    J(1,2) = z2;
    J(1,3) = z3;*/

    MatrixXd j(6,4);
    //upper
    j(0,0) = z0.cross(p - p0)[0];
    j(1,0) = z0.cross(p - p0)[1];
    j(2,0) = z0.cross(p - p0)[2];
    j(0,1) = z1.cross(p - p1)[0];
    j(1,1) = z1.cross(p - p1)[1];
    j(2,1) = z1.cross(p - p1)[2];
    j(0,2) = z2.cross(p - p2)[0];
    j(1,2) = z2.cross(p - p2)[1];
    j(2,2) = z2.cross(p - p2)[2];
    j(0,3) = z3.cross(p - p3)[0];
    j(1,3) = z3.cross(p - p3)[1];
    j(2,3) = z3.cross(p - p3)[2];

    //lower
    j(3,0) = z0[0];
    j(4,0) = z0[1];
    j(5,0) = z0[2];
    j(3,1) = z1[0];
    j(4,1) = z1[1];
    j(5,1) = z1[2];
    j(3,2) = z2[0];
    j(4,2) = z2[1];
    j(5,2) = z2[2];
    j(3,3) = z3[0];
    j(4,3) = z3[1];
    j(5,3) = z3[2];


    //getting jacobianPoint
    MatrixXd r14 = t4.block(0,0,3,3);
    //MatrixXd jw(2,4) = r14 * j.block(4,0,5,4);
    MatrixXd jw(3,4);
    jw(0,0) = j(3,0);
    jw(0,1) = j(3,1);
    jw(0,2) = j(3,2);
    jw(0,3) = j(3,3);
    jw(1,0) = j(4,0);
    jw(1,1) = j(4,1);
    jw(1,2) = j(4,2);
    jw(1,3) = j(4,3);
    jw(2,0) = j(5,0);
    jw(2,1) = j(5,1);
    jw(2,2) = j(5,2);
    jw = r14.inverse() * jw;
    jw = jw.block(0,0,2,4);

    Vector4d xFinal4;
    xFinal4[0] = xFinal[0];
    xFinal4[1] = xFinal[1];
    xFinal4[2] = xFinal[2];
    xFinal4[3] = 1;

    //Define the final angular velocity and the new desired effector motion
    double k = 0.1;
    Vector4d targetEEFrame = t4.inverse() * xFinal4;
    cout << targetEEFrame << endl;
    Vector2d angularVelocity(2);
    angularVelocity(0) = -k * targetEEFrame[1];
    angularVelocity(1) = k * targetEEFrame[0];

    //Finding the J.inverse()
    MatrixXd jT = jw.transpose();
    MatrixXd jInversed = jT * (jw * jT).inverse();


    //Finding the new angles
    Vector4d qNewDifference = jInversed * angularVelocity;

    Vector4d qNew = currentAngle + qNewDifference;

    return qNew;

}

/*int main(int argc, char* argv[]) {

    Vector4d currentAns;
    currentAns[0] = 0;
    currentAns[1] = 0;
    currentAns[2] = 0;
    currentAns[3] = 0;

    Vector3d final;
    final[0] = 0;
    final[1] = 0;
    final[2] = 0.5;
    cout << getNextAngleIK(currentAns, final) << endl;

}*/