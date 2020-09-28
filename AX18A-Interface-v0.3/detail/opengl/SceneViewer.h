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


#ifndef _QGL_SCENE_VIEWER_H_
#define _QGL_SCENE_VIEWER_H_

#include <vector>

#include <QGLViewer/qglviewer.h>

#include <QMutex>

#include <boost/shared_ptr.hpp>





class Drawable {
public:
    virtual void draw(QGLViewer* viewer) = 0;
    virtual ~Drawable(){}
};




/**
 * QGLViewer based environment to use QglDrawables.
 * Simply add them using addDrawable() and they will be rendered inside the viewer
 *
 * The user interface is responsive to the following hot-keys:
 * - Ctrl-U : Align horizont
 * - Ctrl-G : Toggle rendering of the grid in the x-y-plane
 * - Ctrl-A : Toggle rendering of world coordinate axes in the origin
 * - Ctrl-F : Toggle display of the current FPS
 */
class SceneViewer : public QGLViewer
{
    Q_OBJECT

public:
    virtual void refresh();
public :

    /**
     * Create a new instance QglSceneViewer.
     * The instance can be embedded in a complex QtLayout by setting
     * the parent argument.
     * If it is not set the viewer will be displayed in a separate window.
     */
    static boost::shared_ptr<SceneViewer> create(QWidget* parent=0, const QGLWidget* share = NULL);

    virtual ~SceneViewer();

    /**
     * Add a new drawable that will be rendered inside the viewer afterwards.
     *
     * The QglSceneViewer does NOT take the responsibility to manage the memory
     * of the QglDrawable instance! Make sure it's not deleted while still in use.
     */
    virtual void addDrawable(Drawable *drawable);

    /**
     * Removes the drawable from the rendering queue.
     */
    virtual void removeDrawable(Drawable *drawable);

    /**
     * Allows to specify a callback function that will receive any key-event
     * delivered to the viewer (except the four keys used by the viewer itself).
     * There can only be ONE callback function. When a callback is already registered
     * and a new one is set the old one is not active anymore.
     */
    void setCustomKeyHandle( void (*handleFunc)(QKeyEvent*) );

protected :
    virtual void init();
    virtual void horizontalHorizon();
    virtual void draw();
    virtual void animate();
    virtual QString helpString() const;
    virtual void keyPressEvent(QKeyEvent* e);

private:
    SceneViewer(QWidget* parent=0, const QGLWidget* share = NULL);

    QMutex mutex;
    std::vector<Drawable*> drawables;

    void (*customKeyHandle)(QKeyEvent*);
};


#endif
