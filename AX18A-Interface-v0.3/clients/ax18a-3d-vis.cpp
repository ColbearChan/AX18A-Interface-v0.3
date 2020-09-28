#include <qapplication.h>
#include <iostream>
#include <tcp/AX18ARemoteInterface.h>

#include "../detail/opengl/SceneViewer.h"
#include "Eigen/Eigen"


using namespace std;




void renderArrow() {
    double length = 0.06;
    double width = 0.002;
    GLUquadric *obj = gluNewQuadric();
    gluCylinder(obj, width, width, length, 6, 6);
    glPushMatrix();
    glTranslated(0.0, 0.0, length);
    gluCylinder(obj, width * 2, 0.0, width * 3, 6, 6);
    glPopMatrix();
    gluDeleteQuadric(obj);
}

void renderSphere(){
    GLUquadric* obj = gluNewQuadric();
    double r=0.01;
    double slices=12;
    double stacks=12;
    gluSphere(obj,r,slices,stacks);
    gluDeleteQuadric(obj);
}

void renderTargetSphere(){
    GLUquadric* obj = gluNewQuadric();
    glColor3d(0,1,0);
    double r=0.01;
    double slices=12;
    double stacks=12;
    gluSphere(obj,r,slices,stacks);
    gluDeleteQuadric(obj);
}


void renderCoordinates(){
    glPushAttrib(GL_CURRENT_BIT);
    glColor3d(0,0,1);
    renderArrow(); // z axis

    glPushMatrix();
    glRotated(-90, 1.0, 0.0, 0.0);
    glColor3d(0,1,0);
    renderArrow(); // y axis
    glPopMatrix();

    glPushMatrix();
    glRotated(90, 0.0, 1.0, 0.0);
    glColor3d(1,0,0);
    renderArrow(); // x axis
    glPopMatrix();

    glPopAttrib();
}


class AX18Drawable : public Drawable {
public:
    AX18Drawable(std::shared_ptr<AX18ARobotInterface> robotP){
        obj = gluNewQuadric();
        robot = robotP;
    }
    ~AX18Drawable(){
        gluDeleteQuadric(obj);
    }
    virtual void draw(QGLViewer* viewer){
        Eigen::VectorXd jointAngles = robot->getCurrentJointAngles();
        // TODO
        const unsigned slices = 12;
        const unsigned stacks = 12;
        const double radius = 0.05;
        const double height = 0.5;

        renderCoordinates();
        glColor3d(0.5,0.5,0.5);



        glPushMatrix();
        glTranslated(-0.6,-0.6,0.6);
        glColor3d(1,0,0);
        renderTargetSphere();
        glPopMatrix();

        glPushMatrix();
        glTranslated(-0.6,-0.6,-0.6);
        glColor3d(1,0,0);
        renderTargetSphere();
        glPopMatrix();



        //reaching
        glPushMatrix();
        glTranslated(0.2,0.2,0.25);
        renderTargetSphere();
        glPopMatrix();

        glPushMatrix();
        glTranslated(-0.05,-0.05,0.4);
        renderTargetSphere();
        glPopMatrix();

        glPushMatrix();
        glTranslated(0.1,-0.05,0.3);
        renderTargetSphere();
        glPopMatrix();

        glPushMatrix();
        glTranslated(-0.2,0.1,0.2);
        renderTargetSphere();
        glPopMatrix();




        //gluCylinder(obj, radius, radius, height, slices, stacks);

        glPushMatrix();

        //glTranslated(0,0,height);

        double angle = jointAngles[0];
        //double angle = 3.14/4.0 ;


        double rot1[16] = {cos(angle), sin(angle), 0.0, 0.0,
                           -sin(angle)*cos(3.14/2), cos(angle)*cos(3.14/2), sin(3.14/2), 0.0,
                           sin(angle)*sin(3.14/2), -cos(angle)*sin(3.14/2), cos(3.14/2), 0.0,
                           0.052*cos(angle), 0.052*sin(angle), 0.148, 1.0};




        glMultMatrixd(rot1);
        (obj,1,0.5,0.5);

        renderCoordinates();
        renderSphere();
        glPushMatrix();
        glRotated(90,1,0,0);
        glRotated(341,0,1,0);
        gluCylinder(obj,0.007,0.007,0.15,6,6);
        glPopMatrix();


        glColor3d(0.5,0.5,0.5);
        //gluSphere(obj, radius, slices, stacks);
        //gluCylinder(obj, radius, radius, height, slices, stacks);


        glPushMatrix();
        double angle2 = jointAngles[1]+3.14;
        double rot2[16] = {cos(angle2), sin(angle2), 0.0, 0.0,
                           -sin(angle2)*cos(0), cos(angle2)*cos(0), sin(0), 0.0,
                           sin(angle2)*sin(0), -cos(angle2)*sin(0), cos(0), 0.0,
                           0.173*cos(angle2), 0.173*sin(angle2), 0.0, 1.0};
        glMultMatrixd(rot2);
        glPushMatrix();
        glRotated(270,0,1,0);
        gluCylinder(obj,0.007,0.007,0.17,6,6);
        glPopMatrix();
        renderCoordinates();

        glPushMatrix();

        double angle3 = jointAngles[2]-3.14/2   ;
        double rot3[16] = {cos(angle3), sin(angle3), 0.0, 0.0,
                           -sin(angle3)*cos(-3.14/2), cos(angle3)*cos(-3.14/2), sin(-3.14/2), 0.0,
                           sin(angle3)*sin(-3.14/2), -cos(angle3)*sin(-3.14/2), cos(-3.14/2), 0.0,
                           0.025*cos(angle3), 0.025*sin(angle3), 0.0, 1.0};

        glMultMatrixd(rot3);
        renderCoordinates();
        renderSphere();
        glPushMatrix();
        glRotated(270,0,1,0);
        gluCylinder(obj,0.007,0.007,0.02,6,6);
        glPopMatrix();

        glPushMatrix();

        double angle4 = jointAngles[3];
        double rot4[16] = {cos(angle4), sin(angle4), 0.0, 0.0,
                           -sin(angle4)*cos(0), cos(angle4)*cos(0), sin(0), 0.0,
                           sin(angle4)*sin(0), -cos(angle4)*sin(0), cos(0), 0.0,
                           0*cos(angle4), 0*sin(angle4), 0.23, 1.0};

        glMultMatrixd(rot4);
        renderCoordinates();
        renderSphere();

        glPushMatrix();
        glRotated(180,1,0,0);
        gluCylinder(obj,0.007,0.007,0.22,6,6);
        glPopMatrix();

        glPopMatrix();


        glPopMatrix();
        renderSphere();


        glPopMatrix();



        glPopMatrix();
        renderSphere();

    }
private:
    GLUquadric* obj;
    int t;
    std::shared_ptr<AX18ARobotInterface> robot;
};



int main(int argc, char** argv){
    QApplication application(argc,argv);

    string remoteHost = "localhost";
    if(argc==2){
        remoteHost = argv[1];
    }

    boost::shared_ptr<SceneViewer> viewer = SceneViewer::create();

    viewer->addDrawable(new AX18Drawable(AX18ARemoteInterface::create(remoteHost)));

    viewer->setSceneRadius(5.0);
    viewer->show();
    viewer->restoreStateFromFile();

    return application.exec();
}