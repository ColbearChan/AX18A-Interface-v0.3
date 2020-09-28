/* ============================================================
 *
 * This file is a part of the CoR-Lab Continuum Kinematics project
 *
 * Copyright (C) 2011-2012 by Matthias Rolf <mrolf at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *   CIT-EC, Center of Excellence Cognitive Interaction Technology
 *     Bielefeld University
 *
 * ============================================================ */


#include "SceneViewer.h"

#include <QGLViewer/vec.h>
#include <algorithm>

#include <QKeyEvent>

using namespace std;
using namespace qglviewer;
using namespace boost;



void SceneViewer::refresh() {
    updateGL();
}

boost::shared_ptr<SceneViewer> SceneViewer::create(QWidget* parent, const QGLWidget* share){
    boost::shared_ptr<SceneViewer> viewer(
            new SceneViewer(
                    parent,
                    share
            )
    );

    viewer->startAnimation();

    return viewer;
}

SceneViewer::~SceneViewer() {}

void SceneViewer::init() {
    setStateFileName("QglSceneViewer.xml");
// 	restoreStateFromFile();

    setSceneRadius(1.5);
    camera()->showEntireScene();
    setGridIsDrawn(true);

    //scene->initscene(this);

// 	setBackgroundColor(Qt::black);
    setBackgroundColor(Qt::white);

    setKeyDescription(Qt::Key_O,"Opens the OPTIONS dialog");
    setKeyDescription(Qt::Key_U,"Changes the view such that the horizon of the view is horizontal");
    setKeyDescription(Qt::Key_G,"Toggle Grid");
    setKeyDescription(Qt::Key_C,"Toggle coordinates");
//	connect(scene,SIGNAL(updateView()),this,SLOT(refresh()));
}

void SceneViewer::horizontalHorizon() {
    camera()->setUpVector(Vec(0,0,1),false);
    updateGL();
}

void SceneViewer::draw() {
    // anti-aliasing
// 	glEnable (GL_POINT_SMOOTH);
// 	glEnable (GL_LINE_SMOOTH);
// // 	glEnable (GL_POLYGON_SMOOTH);
// 	glEnable(GL_MULTISAMPLE);=


    mutex.lock();
    std::vector<Drawable*> drawablesCopy = drawables;
    mutex.unlock();

    for(unsigned int i=0; i<drawablesCopy.size(); i++){
        glPushMatrix();
        drawablesCopy[i]->draw(this);
        glPopMatrix();
    }
}

void SceneViewer::animate() {
//     if (viewnr==1) {
//         scene->animate();
//     }
}

QString SceneViewer::helpString() const
{
    QString text("<h2>Q g l S c e n e  V i e w e r</h2>");
    //text += "An application for visualization of kinematic chains ";
    return text;
}

void SceneViewer::keyPressEvent(QKeyEvent* e) {
    if( e->key()==Qt::Key_U && e->modifiers()==Qt::ControlModifier ) {
        horizontalHorizon();
    }
    else if( e->key()==Qt::Key_G && e->modifiers()==Qt::ControlModifier ) {
        toggleGridIsDrawn();
    }
    else if( e->key()==Qt::Key_A && e->modifiers()==Qt::ControlModifier ) {
        toggleAxisIsDrawn();
    }
    else if( e->key()==Qt::Key_F && e->modifiers()==Qt::ControlModifier ) {
        toggleFPSIsDisplayed();
    }
    else {
        QGLViewer::keyPressEvent(e);
        if(customKeyHandle){
            customKeyHandle(e);
        }
    }
}

void SceneViewer::addDrawable(Drawable *drawable){
    mutex.lock();
    this->drawables.push_back(drawable);
    mutex.unlock();
}
void SceneViewer::removeDrawable(Drawable *drawable){
    mutex.lock();
    this->drawables.erase( std::find(drawables.begin(), drawables.end(), drawable) );
    mutex.unlock();
}

void SceneViewer::setCustomKeyHandle( void (*handleFunc)(QKeyEvent*) ){
    this->customKeyHandle = handleFunc;
}

SceneViewer::SceneViewer(QWidget* parentP, const QGLWidget* shareP):
        QGLViewer(parentP,shareP), mutex(), drawables(), customKeyHandle(NULL)
{
}



