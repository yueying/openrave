// -*- coding: utf-8 -*-
// Copyright (C) 2012-2016 Rosen Diankov, Gustavo Puche, OpenGrasp Team
//
// OpenRAVE Qt/OpenSceneGraph Viewer is licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef OPENRAVE_QTOSG_VIEWERCONTEXT_H
#define OPENRAVE_QTOSG_VIEWERCONTEXT_H

#include "qtosg.h"
#include "osgrenderitem.h"
#include "osgpick.h"
#include "osgskybox.h"

#include <QTime>
#include <QtCore/QTimer>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLayout>
#include <QtWidgets/QOpenGLWidget>

#include <osg/AnimationPath>
#include <osgManipulator/Dragger>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/PositionAttitudeTransform>
#include <osgGA/NodeTrackerManipulator>
#include <iostream>

namespace qtosgrave {

using namespace OpenRAVE;

class OpenRAVETracker;
class OpenRAVETrackball;

/// \brief  Class of the openscene graph 3d viewer
class QOSGViewerWidget : public QOpenGLWidget
{
public:

    QOSGViewerWidget(EnvironmentBasePtr penv, const std::string &userdatakey,
                     const boost::function<bool(int)> &onKeyDown = boost::function<bool(int)>(),
                     double metersinunit = 1, QWidget *parent = 0);

    virtual ~QOSGViewerWidget();

    /// \brief Draws bounding box around actual kinbody
    void DrawBoundingBox(bool pressed);

    /// \brief Active selection
    void ActivateSelection(bool active);

    /// \brief possible names include TrackballDragger, TranslateAxisDragger
    void SetDraggerMode(const std::string &draggerName);

    /// \brief sets up a dragger selection for a robot or kinbody item
    void SelectItem(KinBodyItemPtr item, KinBody::JointPtr joint = KinBody::JointPtr());

    void SelectItemFromName(const std::string &name);

    /// \brief  Sets scene data node in all viewers
    void SetSceneData();

    /// \brief  Reset viewer to original position
    void ResetViewToHome();

    /// \brief Reset viewer to original position
    void SetHome();

    /// \brief Light button
    void SetLight(bool enabled);

    //  Cull face
    void SetFacesMode(bool enabled);

    /// \brief Sets poligon mode (SMOOTH, FLAT or WIRED)
    void SetPolygonMode(int mode);

    /// \brief set the viewport to perspective or orthogonal
    void SetViewType(int isorthogonal);

    /// \brief sets the near plane for the camera
    void SetNearPlane(double nearplane);

    /// \brief Rotates the camera around the current focal point in the direction of the screen x vector (in world coordinates). The argument thetaX is in radians -pi < thetaX < pi.
    virtual void RotateCameraXDirection(float thetaX);

    /// \brief Rotates the camera around the current focal point in the direction of the screen y vector (in world coordinates). The argument thetaY is in radians -pi < thetaY < pi.
    virtual void RotateCameraYDirection(float thetaY);

    /// \brief Pans the camera in the direction of the screen x vector, parallel to screen plane. The argument dx is in normalized coordinates 0 < dx < 1, where 1 means canvas width.
    virtual void PanCameraXDirection(float dx);

    /// \brief Pans the camera in the direction of the screen y vector, parallel to screen plane. The argument dy is in normalized coordinates 0 < dy < 1, where 1 means canvas height.
    virtual void PanCameraYDirection(float dy);


    /// \param axis, the axis to align camera to. It will translate the camera so the whole scene can be visible, and new center of view will be the scene's bounding box center.
    void MoveCameraPointOfView(const std::string& axis);

    /// \param factor > 1.0 = zoom in. < 1.0 = zoom out
    /// \param isPan, if true, then focal distance will not change, but rather camera position will move along with focal point
    /// \param panDelta, if true, then focal distance will not change, but rather camera position will move along with focal point
    void MoveCameraZoom(float factor, bool isPan, float panDelta);

    /// \brief changes current focal distance (if in perspective mode) or the current projection plane size (if in ortho mode) in order
    /// to zoom in/out towards/from focal point (if factor < 1). This function never changes de focal point position.
    void Zoom(float factor);

    /// \brief set the cubemap for skybox
    void SetTextureCubeMap(const std::string &posx,
                           const std::string &negx,
                           const std::string &posy,
                           const std::string &negy,
                           const std::string &posz,
                           const std::string &negz);

    /// \brief returns the near plane set on the camera
    double GetCameraNearPlane();

    /// \brief called when the qt window size changes
    void SetViewport(int width, int height);

    /// \brief sets user-controlled hud text
    void SetUserHUDText(const std::string &text);

    /// \brief Set wire view to a node
    void SetWire(OSGNodePtr node);

    OSGGroupPtr GetSceneRoot() const {
        return _osgSceneRoot;
    }

    OSGGroupPtr GetFigureRoot() const {
        return _osgFigureRoot;
    }

    /// \brief called when the mouse is over a specified point
    ///
    void HandleRayPick(const osgUtil::LineSegmentIntersector::Intersection &intersection, int buttonPressed, int modkeymask = 0);

    /// \brief handle case when link is selected
    void SelectOSGLink(OSGNodePtr node, int modkeymask);

    /// \brief activate and configure trackmode manipulator to track given OSG node
    /// \brief trackInfoText is the text to display in canvas about the current element being tracked
    void TrackNode(OSGNodePtr node, const std::string& trackInfoText, const osg::Vec3d& offset, double trackDistance);
    void StopTrackNode();

    osg::Camera *GetCamera();
    bool IsInOrthoMode();

    osg::ref_ptr<osgGA::CameraManipulator> GetCurrentCameraManipulator();
    void SetCurrentCameraManipulator(osgGA::CameraManipulator* manipulator);
    void SetCameraDistanceToFocus(double distance);
    double GetCameraDistanceToFocus();

    void RestoreDefaultManipulator();
    bool IsUsingDefaultCameraManipulator();
    osg::ref_ptr<osgGA::TrackballManipulator> GetDefaultCameraManipulator();
    osg::ref_ptr<osgGA::NodeTrackerManipulator> GetTrackModeManipulator();

    OSGMatrixTransformPtr GetCameraHUD();

    /// \brief Updates any changes in OSG to to OpenRAVE core.
    void UpdateFromOSG();

    /// \brief Find node of Robot for the link picked
    KinBodyItemPtr FindKinBodyItemFromOSGNode(OSGNodePtr node);

    /// \brief Find KinBodyItem from a kinbody name
    KinBodyItemPtr GetItemFromName(const std::string &name);

    /// \brief Find KinBodyItem from a kinbody instance
    KinBodyItemPtr GetItemFromKinBody(KinBodyPtr kinBody);

    /// \brief restores cursor to what it was originally set to
    void RestoreCursor();

    /// \brief Get osg composite viewer
    osg::ref_ptr<osgViewer::CompositeViewer> GetViewer();

    /// \brief
    void SetKeyboardModifiers(QInputEvent* event);

    /// \brief Get osg viewer camera control mode for single finger gesture
    const char* GetCameraMoveMode() {
        return _bSwitchMouseLeftMiddleButton ? "Pan" : "Rot";
    }

    /// \brief Toggle camera move mode between pan and rotate
    void ToggleCameraMoveMode() {
        _bSwitchMouseLeftMiddleButton = !_bSwitchMouseLeftMiddleButton;
    }

protected:
    /// \brief handles a key press and looks at the modifier keys
    bool HandleOSGKeyDown(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);

    /// \brief Clear dragger from the viewer
    void _ClearDragger();

    /// \brief gather all the necessary text and updates it on the HUD control
    void _UpdateHUDText();

    /// \brief Set up cameras
    void _SetupCamera(osg::ref_ptr<osg::Camera> camera, osg::ref_ptr<osgViewer::View> view,
                      osg::ref_ptr<osg::Camera> hudcamera, osg::ref_ptr<osgViewer::View> hudview);

    /// \brief Retrieves RAVE environment world up unitary vector
    void _GetRAVEEnvironmentUpVector(osg::Vec3d& upVector);

    /// \brief Create Open GL Context
    osg::ref_ptr<osg::Camera> _CreateCamera(int x, int y, int w, int h, double metersinunit);

    osg::ref_ptr<osg::Camera> _CreateHUDCamera(int x, int y, int w, int h, double metersinunit);


    /// \brief Find joint into OpenRAVE core
    KinBody::JointPtr _FindJoint(KinBodyItemPtr pitem, KinBody::LinkPtr link);

    //  Lighting Stuff //
    osg::ref_ptr<osg::Material> _CreateSimpleMaterial(osg::Vec4 color);
    osg::ref_ptr<osg::Light> _CreateLight(osg::Vec4 color, int lightid);
    osg::ref_ptr<osg::Light> _CreateAmbientLight(osg::Vec4 color, int lightid);

    /// \brief Initialize lighting
    void _InitializeLights(int nlights);

    /// \brief Stores matrix transform
    void _StoreMatrixTransform();

    /// \brief Loads the stored matrix transform to the camera
    void _LoadMatrixTransform();

    /// \brief update hud display axis from current manipulator transform
    void _UpdateHUDAxisTransform(int width, int height);

    /// \brief set camera projection matrix as orthogonal using the given projection plane size
    void _SetCameraViewOrthoProjectionPlaneSize(double size);

    /// \brief will move the camera to sceneBoudingBox.center + axis * distance, and set view center to camerapos - axis, where distance is some distance that provide good visualization of the whole scene
    /// \param axis is an arbitrary *unitary* axis vector
    void _MoveCameraPointOfView(const osg::Vec3d& axis);

    /// \brief performs a rotation of the camera over the current focal point (see GetCameraDistanceToFocus()).
    /// Camera will keep looking ate the focal point after performing the rotation.
    /// \param angle in radians, -pi < angle < pi, camSpaceRotationOverDirection is the direction in camera space over which to rotate to.
    /// \param useCameraUpDirection in radians, -pi < angle < pi, camSpaceRotationOverDirection is the direction, in camera, space over which to rotate to.
    void _RotateCameraOverDirection(double angle, const osg::Vec3d& camSpaceRotationOverDirection, bool useCameraUpDirection=true);

    /// \brief performs a a pan translation of both camera and focal point position.
    /// \param camSpacePanDirection is the pan direction in camera space to apply to camera and focal point.
    /// \param delta is how much to translate in worlds units (see _metersinunit and QOSGViewerWidget() constructor)
    void _PanCameraTowardsDirection(double delta, const osg::Vec3d& camSpacePanDirection);

    /// \brief Create a dragger with a name given
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > _CreateDragger(const std::string &name);

    /// \brief Create a manipulator over a render item
    ///
    /// \param draggerName the type of dragger to create
    /// \param joint if not empty, the joint to create the dragger over (ie for moving the joint value)
    OSGNodePtr _AddDraggerToObject(const std::string &draggerName, KinBodyItemPtr item, KinBody::JointPtr joint);

    virtual void initializeGL();

    virtual void paintGL();

    virtual void resizeGL(int width, int height);

    virtual void mouseMoveEvent(QMouseEvent *event);

    virtual void mousePressEvent(QMouseEvent *event);

    virtual void mouseReleaseEvent(QMouseEvent *event);

    virtual void mouseDoubleClickEvent(QMouseEvent *event);

    virtual void wheelEvent(QWheelEvent *event);

    virtual void keyPressEvent(QKeyEvent *event);

    virtual void keyReleaseEvent(QKeyEvent *event);

    virtual bool event(QEvent *event);

    OSGGroupPtr _osgSceneRoot; ///< root scene node
    OSGGroupPtr _osgFigureRoot; ///< the node that all the figures are drawn into
    OSGMatrixTransformPtr _osgWorldAxis; ///< the node that draws the rgb axes on the lower right corner

    std::string _userdatakey; ///< the key to use for KinBody::GetUserData and KinBody::SetUserData
    OSGGroupPtr _osgLightsGroup; ///< Scene Node with lights
    OSGGroupPtr _osgLightsGroupData; ///< Scene Data to romove after each repaint
    OSGGroupPtr _osgDraggerRoot; ///< Parent of dragger and selection
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > _draggers; ///< There is only one dragger at the same time
    OSGMatrixTransformPtr _draggerMatrix; ///< Transform applied by dragger
    OSGGroupPtr _osgSelectedNodeByDragger; ///< Object selected by dragger
    OSGMatrixTransformPtr _osgCameraHUD; ///< MatrixTransform node that gets displayed in the heads up display

    KinBodyItemPtr _selectedItem; ///< render item selected
    std::string _draggerName; ///< Actual dragger selected

    osg::ref_ptr<OSGPickHandler> _picker; ///<  Pick handler for joint selection
    osg::ref_ptr<osgGA::GUIEventHandler> _keyhandler; ///<  Pick handler for joint selection
    osg::Matrixf _viewCameraMatrix; ///< stored matrix transform

    std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > _vLightTransform;
    osg::ref_ptr<osg::StateSet> _lightStateSet;
    osg::ref_ptr<osgViewer::CompositeViewer> _osgviewer;
    osgViewer::GraphicsWindowEmbedded* _osgGraphicWindow;
    osg::ref_ptr<osgViewer::View> _osgview;
    osg::ref_ptr<osgViewer::View> _osghudview;
    osg::ref_ptr<OpenRAVETrackball> _osgDefaultManipulator; //< default manipulator
    osg::ref_ptr<OpenRAVETracker> _osgTrackModeManipulator; //< manipulator used by TrackLink and TrackManip commands

    osg::ref_ptr<osgText::Text> _osgHudText; ///< the HUD text in the upper left corner
    std::string _strUserText, _strSelectedItemText, _strRayInfoText, _strTrackInfoText;; ///< the user hud text

    osg::ref_ptr<Skybox> _osgSkybox;  ///< the skybox moving together with camera

    QTimer _timer; ///< Timer for repaint
    EnvironmentBasePtr _penv;
    double _metersinunit; //< current meter unit to be used in all transformations and calculations
    boost::function<bool(int)> _onKeyDown; ///< call whenever key press is detected
    bool _bSwitchMouseLeftMiddleButton;  ///< whether to switch mouse left button and middle button (camera control mode)
    bool _bLightOn; ///< whether lights are on or not
    bool _bIsSelectiveActive; ///< if true, then can select a new
    double _zNear; ///< In OSG, znear and zfar are updated by CullVisitor, which
                   ///  causing getProjectionMatrixAsXXX to return negative
                   ///  values. Therefore, we manage zNear ourselves
    double _currentOrthoFrustumSize; ///< coordinate for the right vertical clipping plane 

    void GetSwitchedButtonValue(unsigned int &button);
};

class QtOSGKeyEventTranslator
{
public:
    QtOSGKeyEventTranslator()
    {
        keyMap[Qt::Key_Escape    ] = osgGA::GUIEventAdapter::KEY_Escape;
        keyMap[Qt::Key_Delete    ] = osgGA::GUIEventAdapter::KEY_Delete;
        keyMap[Qt::Key_Home      ] = osgGA::GUIEventAdapter::KEY_Home;
        keyMap[Qt::Key_Enter     ] = osgGA::GUIEventAdapter::KEY_KP_Enter;
        keyMap[Qt::Key_End       ] = osgGA::GUIEventAdapter::KEY_End;
        keyMap[Qt::Key_Return    ] = osgGA::GUIEventAdapter::KEY_Return;
        keyMap[Qt::Key_PageUp    ] = osgGA::GUIEventAdapter::KEY_Page_Up;
        keyMap[Qt::Key_PageDown  ] = osgGA::GUIEventAdapter::KEY_Page_Down;
        keyMap[Qt::Key_Left      ] = osgGA::GUIEventAdapter::KEY_Left;
        keyMap[Qt::Key_Right     ] = osgGA::GUIEventAdapter::KEY_Right;
        keyMap[Qt::Key_Up        ] = osgGA::GUIEventAdapter::KEY_Up;
        keyMap[Qt::Key_Down      ] = osgGA::GUIEventAdapter::KEY_Down;
        keyMap[Qt::Key_Backspace ] = osgGA::GUIEventAdapter::KEY_BackSpace;
        keyMap[Qt::Key_Tab       ] = osgGA::GUIEventAdapter::KEY_Tab;
        keyMap[Qt::Key_Space     ] = osgGA::GUIEventAdapter::KEY_Space;
        keyMap[Qt::Key_Delete    ] = osgGA::GUIEventAdapter::KEY_Delete;
        keyMap[Qt::Key_Alt       ] = osgGA::GUIEventAdapter::KEY_Alt_L;
        keyMap[Qt::Key_Shift     ] = osgGA::GUIEventAdapter::KEY_Shift_L;
        keyMap[Qt::Key_Control   ] = osgGA::GUIEventAdapter::KEY_Control_L;
        keyMap[Qt::Key_Meta      ] = osgGA::GUIEventAdapter::KEY_Meta_L;
    }

    ~QtOSGKeyEventTranslator() {
    };

    int GetOSGKeyValue(QKeyEvent* event)
    {
        std::map<int, unsigned int>::const_iterator itmap = keyMap.find(event->key());

        if (itmap == keyMap.end()) {
            return int(*(event->text().toLatin1().data()));
        } else {
            return itmap->second;
        }
    }

    unsigned int GetOSGButtonValue(QMouseEvent* event)
    {
        unsigned int button = 0;
        switch (event->button()) {
        case Qt::LeftButton:
            button = 1;
            break;
        case Qt::MiddleButton:
            button = 2;
            break;
        case Qt::RightButton:
            button = 3;
            break;
        default:
            break;
        }
        return button;
    }

private:
    std::map<int, unsigned int> keyMap;
};

static QtOSGKeyEventTranslator qtOSGKeyEventTranslator;
}

#endif
