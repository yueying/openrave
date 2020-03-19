// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "libopenrave.h"
#include <openrave/openravejson.h>

#define CHECK_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(hierarchy_computed_ == 2, "robot %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetName()%hierarchy_computed_, ORE_NotInitialized);

namespace OpenRAVE {

void RobotBase::AttachedSensorInfo::SerializeJSON(rapidjson::Value &value,
	rapidjson::Document::AllocatorType& allocator, dReal unit_scale, int options) const
{
    openravejson::SetJsonValueByKey(value, "name", name_, allocator);
	openravejson::SetJsonValueByKey(value, "linkName", link_name_, allocator);
	openravejson::SetJsonValueByKey(value, "transform", relative_transform_, allocator);
	openravejson::SetJsonValueByKey(value, "type", sensor_name_, allocator);

    if(!!sensor_geometry_)
    {
        rapidjson::Value sensorGeometryValue;
        sensor_geometry_->SerializeJSON(sensorGeometryValue, allocator, unit_scale, options);
		openravejson::SetJsonValueByKey(value, "sensorGeometry", sensorGeometryValue, allocator);
    }
}

void RobotBase::AttachedSensorInfo::DeserializeJSON(const rapidjson::Value& value, dReal unit_scale)
{
	openravejson::LoadJsonValueByKey(value, "name", name_);
	openravejson::LoadJsonValueByKey(value, "linkName", link_name_);
	openravejson::LoadJsonValueByKey(value, "transform", relative_transform_);
	openravejson::LoadJsonValueByKey(value, "type", sensor_name_);

    if (value.HasMember("sensorGeometry")) {
        BaseJSONReaderPtr pReader = RaveCallJSONReader(PT_Sensor, sensor_name_, InterfaceBasePtr(), AttributesList());
        if (!!pReader) {
            pReader->DeserializeJSON(value["sensorGeometry"], unit_scale);
            JSONReadablePtr pReadable = pReader->GetReadable();
            if (!!pReadable) {
                sensor_geometry_ = std::dynamic_pointer_cast<SensorBase::SensorGeometry>(pReadable);
            }
        } else {
            RAVELOG_WARN_FORMAT("failed to get json reader for sensor type \"%s\"", sensor_name_);
        }
    }
}

RobotBase::AttachedSensor::AttachedSensor(RobotBasePtr probot) : _probot(probot)
{
}

RobotBase::AttachedSensor::AttachedSensor(RobotBasePtr probot, const AttachedSensor& sensor,int cloningoptions)
{
    *this = sensor;
    _probot = probot;
    sensor_.reset();
    pdata.reset();
    pattachedlink.reset();
    if( (cloningoptions&Clone_Sensors) && !!sensor.sensor_ ) {
        sensor_ = RaveCreateSensor(probot->GetEnv(), sensor.sensor_->GetXMLId());
        if( !!sensor_ ) {
            sensor_->SetName(str(boost::format("%s:%s")%probot->GetName()%info_.name_)); // need a unique targettable name
            sensor_->Clone(sensor.sensor_,cloningoptions);
            if( !!sensor_ ) {
                pdata = sensor_->CreateSensorData();
            }
        }
    }
    int index = LinkPtr(sensor.pattachedlink)->GetIndex();
    if((index >= 0)&&(index < (int)probot->GetLinks().size())) {
        pattachedlink = probot->GetLinks().at(index);
    }
}

RobotBase::AttachedSensor::AttachedSensor(RobotBasePtr probot, const RobotBase::AttachedSensorInfo& info)
{
    info_ = info;
    _probot = probot;
    pattachedlink = probot->GetLink(info_.link_name_);
    if( !!probot ) {
        sensor_ = RaveCreateSensor(probot->GetEnv(), info_.sensor_name_);
        if( !!sensor_ ) {
            sensor_->SetName(str(boost::format("%s:%s")%probot->GetName()%info_.name_)); // need a unique targettable name
            if(!!info_.sensor_geometry_) {
                sensor_->SetSensorGeometry(info_.sensor_geometry_);
            }
            pdata = sensor_->CreateSensorData();
        }
    }
}

RobotBase::AttachedSensor::~AttachedSensor()
{
}



//void RobotBase::AttachedSensor::_ComputeInternalInformation()
//{
//    RobotBasePtr probot = _probot.lock();
//    sensor_.reset();
//    pdata.reset();
//    if( !!probot ) {
//        sensor_ = RaveCreateSensor(probot->GetEnv(), info_.sensor_name_);
//        if( !!sensor_ ) {
//            sensor_->SetName(str(boost::format("%s:%s")%probot->GetName()%info_.name_)); // need a unique targettable name
//            if(!!info_.sensor_geometry_) {
//                sensor_->SetSensorGeometry(info_.sensor_geometry_);
//            }
//            pdata = sensor_->CreateSensorData();
//        }
//    }
//}

SensorBase::SensorDataPtr RobotBase::AttachedSensor::GetData() const
{
    if( !!sensor_ && sensor_->GetSensorData(pdata) ) {
        return pdata;
    }
    return SensorBase::SensorDataPtr();
}

void RobotBase::AttachedSensor::SetRelativeTransform(const Transform& t)
{
    info_.relative_transform_ = t;
    GetRobot()->_PostprocessChangedParameters(Prop_SensorPlacement);
}

void RobotBase::AttachedSensor::UpdateInfo(SensorBase::SensorType type)
{
    if( !!sensor_ ) {
        info_.sensor_name_ = sensor_->GetXMLId();
        // TODO try to get the sensor geometry...?
        info_.sensor_geometry_ = std::const_pointer_cast<SensorBase::SensorGeometry>(sensor_->GetSensorGeometry(type));
        //info_.sensor_geometry_
    }
    LinkPtr prealattachedlink = pattachedlink.lock();
    if( !!prealattachedlink ) {
        info_.link_name_ = prealattachedlink->GetName();
    }
}

void RobotBase::AttachedSensor::serialize(std::ostream& o, int options) const
{
    o << (pattachedlink.expired() ? -1 : LinkPtr(pattachedlink)->GetIndex()) << " ";
    SerializeRound(o,info_.relative_transform_);
    o << (!pdata ? -1 : pdata->GetType()) << " ";
    // it is also important to serialize some of the geom parameters for the sensor (in case models are cached to it)
    if( !!sensor_ ) {
        SensorBase::SensorGeometryConstPtr prawgeom = sensor_->GetSensorGeometry();
        if( !!prawgeom ) {
            switch(prawgeom->GetType()) {
            case SensorBase::ST_Laser: {
                SensorBase::LaserGeomDataConstPtr pgeom = std::static_pointer_cast<SensorBase::LaserGeomData const>(prawgeom);
                o << pgeom->min_angle[0] << " " << pgeom->max_angle[0] << " " << pgeom->resolution[0] << " " << pgeom->max_range << " ";
                break;
            }
            case SensorBase::ST_Camera: {
                SensorBase::CameraGeomDataConstPtr pgeom = std::static_pointer_cast<SensorBase::CameraGeomData const>(prawgeom);
                o << pgeom->KK.fx << " " << pgeom->KK.fy << " " << pgeom->KK.cx << " " << pgeom->KK.cy << " " << pgeom->width << " " << pgeom->height << " ";
                break;
            }
            default:
                // don't support yet
                break;
            }
        }
    }
}

const std::string& RobotBase::AttachedSensor::GetStructureHash() const
{
    if( hash_structure_.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_RobotSensors);
        hash_structure_ = utils::GetMD5HashString(ss.str());
    }
    return hash_structure_;
}

RobotBase::RobotStateSaver::RobotStateSaver(RobotBasePtr probot, int options) : KinBodyStateSaver(probot, options), _probot(probot)
{
    if( options_ & Save_ActiveDOF ) {
        vactivedofs = _probot->GetActiveDOFIndices();
        affinedofs = _probot->GetAffineDOF();
        rotationaxis = _probot->GetAffineRotationAxis();
    }
    if( options_ & Save_ActiveManipulator ) {
        _pManipActive = _probot->GetActiveManipulator();
    }
    if( options_ & Save_ActiveManipulatorToolTransform ) {
        _pManipActive = _probot->GetActiveManipulator();
        if( !!_pManipActive ) {
            _tActiveManipLocalTool = _pManipActive->GetLocalToolTransform();
            _vActiveManipLocalDirection = _pManipActive->GetLocalToolDirection();
            _pActiveManipIkSolver = _pManipActive->GetIkSolver();
        }
    }
    if( options_ & Save_ManipulatorsToolTransform ) {
        std::vector<RobotBase::ManipulatorPtr> vmanips = probot->GetManipulators();
        _vtManipsLocalTool.resize(vmanips.size());
        _vvManipsLocalDirection.resize(vmanips.size());
        _vpManipsIkSolver.resize(vmanips.size());
        for(int imanip = 0; imanip < (int)vmanips.size(); ++imanip) {
            RobotBase::ManipulatorPtr pmanip = vmanips[imanip];
            if( !!pmanip ) {
                _vtManipsLocalTool[imanip] = pmanip->GetLocalToolTransform();
                _vvManipsLocalDirection[imanip] = pmanip->GetLocalToolDirection();
                _vpManipsIkSolver[imanip] = pmanip->GetIkSolver();
            }
        }
    }

    _probot->GetConnectedBodyActiveStates(_vConnectedBodyActiveStates);
}

RobotBase::RobotStateSaver::~RobotStateSaver()
{
    if( _bRestoreOnDestructor && !!_probot && _probot->GetEnvironmentId() != 0 ) {
        _RestoreRobot(_probot);
    }
}

void RobotBase::RobotStateSaver::Restore(std::shared_ptr<RobotBase> robot)
{
    _RestoreRobot(!robot ? _probot : robot);
    KinBodyStateSaver::Restore(!robot ? KinBodyPtr(_probot) : KinBodyPtr(robot));
}

void RobotBase::RobotStateSaver::Release()
{
    _probot.reset();
    KinBodyStateSaver::Release();
}

///\brief removes the robot from the environment temporarily while in scope
class EnvironmentRobotRemover
{
public:

    EnvironmentRobotRemover(RobotBasePtr pRobot) : _bRemoved(false), _pRobot(pRobot), _pEnv(pRobot->GetEnv()) {
        _pEnv->Remove(_pRobot);
        _bRemoved = true;
    }

    ~EnvironmentRobotRemover() {
        if (_bRemoved) {
            _pEnv->Add(_pRobot, false);
            _bRemoved = false;
        }
    }

private:

    bool _bRemoved;
    RobotBasePtr _pRobot;
    EnvironmentBasePtr _pEnv;
};

void RobotBase::RobotStateSaver::_RestoreRobot(std::shared_ptr<RobotBase> probot)
{
    if( !probot ) {
        return;
    }
    if( probot->GetEnvironmentId() == 0 ) {
        RAVELOG_WARN(str(boost::format("robot %s not added to environment, skipping restore")%probot->GetName()));
        return;
    }

    if( _vConnectedBodyActiveStates.size() == probot->connected_bodies_vector_.size() ) {
        bool bchanged = false;
        for(size_t iconnectedbody = 0; iconnectedbody < probot->connected_bodies_vector_.size(); ++iconnectedbody) {
            if( probot->connected_bodies_vector_[iconnectedbody]->IsActive() != (!!_vConnectedBodyActiveStates[iconnectedbody]) ) {
                bchanged = true;
                break;
            }
        }

        if( bchanged ) {
            EnvironmentRobotRemover robotremover(probot);
            // need to restore active connected bodies
            // but first check whether anything changed
            probot->SetConnectedBodyActiveStates(_vConnectedBodyActiveStates);
        }
    }

    if( options_ & Save_ActiveDOF ) {
        probot->SetActiveDOFs(vactivedofs, affinedofs, rotationaxis);
    }
    if( options_ & Save_ActiveManipulator ) {
        if( probot == _probot ) {
            probot->SetActiveManipulator(_pManipActive);
        }
        else {
            if( !_pManipActive ) {
                probot->SetActiveManipulator(ManipulatorPtr());
            }
            else {
                probot->SetActiveManipulator(_pManipActive->GetName());
            }
        }
    }
    if( options_ & Save_ActiveManipulatorToolTransform ) {
        if( !!_pManipActive ) {
            if( probot == _probot ) {
                RobotBase::ManipulatorPtr pmanip = probot->GetManipulator(_pManipActive->GetName()); // manipulator pointers might have changed, it is always safer to re-request the current manip
                if( !!pmanip ) {
                    pmanip->SetLocalToolTransform(_tActiveManipLocalTool);
                    pmanip->SetLocalToolDirection(_vActiveManipLocalDirection);
                    pmanip->SetIkSolver(_pActiveManipIkSolver);
                }
                else {
                    RAVELOG_VERBOSE_FORMAT("failed to restore active manipulator %s coordinate system", _pManipActive->GetName());
                }
            }
            else {
                RobotBase::ManipulatorPtr pmanip = probot->GetManipulator(_pManipActive->GetName());
                if( !!pmanip ) {
                    pmanip->SetLocalToolTransform(_tActiveManipLocalTool);
                    pmanip->SetLocalToolDirection(_vActiveManipLocalDirection);
                    if( !!_pActiveManipIkSolver ) {
                        IkSolverBasePtr pnewsolver = RaveCreateIkSolver(probot->GetEnv(), _pActiveManipIkSolver->GetXMLId());
                        pnewsolver->Clone(_pActiveManipIkSolver, 0);
                        pmanip->SetIkSolver(pnewsolver);
                    }
                    else {
                        pmanip->SetIkSolver(IkSolverBasePtr());
                    }
                }
            }
        }
    }
    if( options_ & Save_ManipulatorsToolTransform ) {
        if( probot == _probot ) {
            std::vector<RobotBase::ManipulatorPtr> vmanips = probot->GetManipulators();
            if(vmanips.size() == _vtManipsLocalTool.size()) {
                for(int imanip = 0; imanip < (int)vmanips.size(); ++imanip) {
                    RobotBase::ManipulatorPtr pmanip = vmanips[imanip];
                    if( !!pmanip ) {
                        pmanip->SetLocalToolTransform(_vtManipsLocalTool.at(imanip));
                        pmanip->SetLocalToolDirection(_vvManipsLocalDirection.at(imanip));
                        pmanip->SetIkSolver(_vpManipsIkSolver.at(imanip));
                    }
                }
            }
            else {
                RAVELOG_WARN(str(boost::format("failed to restore manipulators tool transform because the number of saved manipulators %i is different from the number of current manipulators %i\n")%_vtManipsLocalTool.size()%vmanips.size()));
            }
        }
    }
}

RobotBase::RobotBase(EnvironmentBasePtr penv) : KinBody(PT_Robot, penv)
{
    _nAffineDOFs = 0;
    _nActiveDOF = -1;
    vActvAffineRotationAxis = Vector(0,0,1);

    //set limits for the affine DOFs
    _vTranslationLowerLimits = Vector(-100,-100,-100);
    _vTranslationUpperLimits = Vector(100,100,100);
    _vTranslationMaxVels = Vector(1.0f,1.0f,1.0f);
    _vTranslationResolutions = Vector(0.001f,0.001f,0.001f);
    _vTranslationWeights = Vector(2.0f,2.0f,2.0f);

    // rotation axis has infinite movement, so make sure the limits are big
    _vRotationAxisLowerLimits = Vector(-10000,-10000,-10000,10000);
    _vRotationAxisUpperLimits = Vector(10000,10000,10000,10000);

    _vRotationAxisMaxVels = Vector(0.4f,0.4f,0.4f,0.4f);
    _vRotationAxisResolutions = Vector(0.01f,0.01f,0.01f,0.01f);
    _vRotationAxisWeights = Vector(2.0f,2.0f,2.0f,2.0f);

    _vRotation3DLowerLimits = Vector(-10000,-10000,-10000);
    _vRotation3DUpperLimits = Vector(10000,10000,10000);
    _vRotation3DMaxVels = Vector(0.07f,0.07f,0.07f);
    _vRotation3DResolutions = Vector(0.01f,0.01f,0.01f);
    _vRotation3DWeights = Vector(1.0f,1.0f,1.0f);

    _vRotationQuatLimitStart = Vector(1,0,0,0);
    _fQuatLimitMaxAngle = PI;
    _fQuatMaxAngleVelocity = 1.0;
    _fQuatAngleResolution = 0.01f;
    _fQuatAngleWeight = 0.4f;
}

RobotBase::~RobotBase()
{
    Destroy();
}

void RobotBase::Destroy()
{
    manipulator_active_.reset();
    manipulators_vector_.clear();
    attached_sensors_vector_.clear();
    connected_bodies_vector_.clear();
    _nActiveDOF = 0;
    active_dof_indices_vector_.resize(0);
    all_dof_indices_vector_.resize(0);
    SetController(ControllerBasePtr(),std::vector<int>(),0);

    KinBody::Destroy();
}

bool RobotBase::Init(const std::vector<KinBody::LinkInfoConstPtr>& linkinfos, const std::vector<KinBody::JointInfoConstPtr>& jointinfos, const std::vector<RobotBase::ManipulatorInfoConstPtr>& manipinfos, const std::vector<RobotBase::AttachedSensorInfoConstPtr>& attachedsensorinfos, const std::vector<ConnectedBodyInfoConstPtr>& connectedbodyinfos, const std::string& uri)
{
    if( !KinBody::Init(linkinfos, jointinfos, uri) ) {
        return false;
    }
    manipulators_vector_.resize(0);
    FOREACHC(itmanipinfo, manipinfos) {
        ManipulatorPtr newmanip(new Manipulator(shared_robot(),**itmanipinfo));
        manipulators_vector_.push_back(newmanip);
        hash_robot_structure_.resize(0);
    }
    attached_sensors_vector_.clear();
    FOREACHC(itattachedsensorinfo, attachedsensorinfos) {
        AttachedSensorPtr newattachedsensor(new AttachedSensor(shared_robot(),**itattachedsensorinfo));
        attached_sensors_vector_.push_back(newattachedsensor);
        newattachedsensor->UpdateInfo(); // just in case
        hash_robot_structure_.resize(0);
    }
    connected_bodies_vector_.clear();
    FOREACHC(itconnectedbodyinfo, connectedbodyinfos) {
        ConnectedBodyPtr newconnectedbody(new ConnectedBody(shared_robot(),**itconnectedbodyinfo));
        connected_bodies_vector_.push_back(newconnectedbody);
    }
    return true;
}

bool RobotBase::SetController(ControllerBasePtr controller, const std::vector<int>& jointindices, int control_transformation)
{
    RAVELOG_DEBUG_FORMAT("env=%d, default robot doesn't not support setting controllers (try GenericRobot)", GetEnv()->GetId());
    return false;
}

void RobotBase::SetName(const std::string& newname)
{
    if( name_ != newname ) {
        // have to replace the 2nd word of all the groups with the robot name
        FOREACH(itgroup, active_spec_.groups_vector_) {
            stringstream ss(itgroup->name);
            string grouptype, oldname;
            ss >> grouptype >> oldname;
            stringbuf buf;
            ss.get(buf,0);
            itgroup->name = str(boost::format("%s %s %s")%grouptype%newname%buf.str());
        }

        // have to rename any attached sensors with robotname:attachedname!!
        FOREACH(itattached, attached_sensors_vector_) {
            AttachedSensorPtr pattached = *itattached;
            if( !!pattached->sensor_ ) {
                pattached->sensor_->SetName(str(boost::format("%s:%s")%newname%pattached->info_.name_)); // need a unique targettable name
            }
            else {

            }
        }

        KinBody::SetName(newname);
    }
}

void RobotBase::SetDOFValues(const std::vector<dReal>& vJointValues, uint32_t bCheckLimits, const std::vector<int>& dofindices)
{
    KinBody::SetDOFValues(vJointValues, bCheckLimits,dofindices);
    _UpdateAttachedSensors();
}

void RobotBase::SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& transbase, uint32_t bCheckLimits)
{
    KinBody::SetDOFValues(vJointValues, transbase, bCheckLimits); // should call RobotBase::SetDOFValues, so no need to upgrade grabbed bodies, attached sensors
}

void RobotBase::SetLinkTransformations(const std::vector<Transform>& transforms)
{
    KinBody::SetLinkTransformations(transforms);
    _UpdateAttachedSensors();
}

void RobotBase::SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<dReal>& doflastsetvalues)
{
    KinBody::SetLinkTransformations(transforms,doflastsetvalues);
    _UpdateAttachedSensors();
}

void RobotBase::SetTransform(const Transform& trans)
{
    KinBody::SetTransform(trans);
    _UpdateAttachedSensors();
}

bool RobotBase::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    if( !KinBody::SetVelocity(linearvel,angularvel) ) {
        return false;
    }
    return true;
}

void RobotBase::SetDOFVelocities(const std::vector<dReal>& dofvelocities, const Vector& linearvel, const Vector& angularvel,uint32_t checklimits)
{
    KinBody::SetDOFVelocities(dofvelocities,linearvel,angularvel,checklimits);
    // do sensors need to have their velocities updated?
}

void RobotBase::SetDOFVelocities(const std::vector<dReal>& dofvelocities, uint32_t checklimits, const std::vector<int>& dofindices)
{
    KinBody::SetDOFVelocities(dofvelocities,checklimits, dofindices); // RobotBase::SetDOFVelocities should be called internally
}

void RobotBase::_UpdateAttachedSensors()
{
    FOREACH(itsensor, attached_sensors_vector_) {
        if( !!(*itsensor)->GetSensor() && !(*itsensor)->pattachedlink.expired() ) {
            (*itsensor)->GetSensor()->SetTransform(LinkPtr((*itsensor)->pattachedlink)->GetTransform()*(*itsensor)->GetRelativeTransform());
        }
    }
}

void RobotBase::SetAffineTranslationLimits(const Vector& lower, const Vector& upper)
{
    _vTranslationLowerLimits = lower;
    _vTranslationUpperLimits = upper;
}

void RobotBase::SetAffineRotationAxisLimits(const Vector& lower, const Vector& upper)
{
    _vRotationAxisLowerLimits = lower;
    _vRotationAxisUpperLimits = upper;
}

void RobotBase::SetAffineRotation3DLimits(const Vector& lower, const Vector& upper)
{
    _vRotation3DLowerLimits = lower;
    _vRotation3DUpperLimits = upper;
}

void RobotBase::SetAffineRotationQuatLimits(const Vector& quatangle)
{
    _fQuatLimitMaxAngle = RaveSqrt(quatangle.lengthsqr4());
    if( _fQuatLimitMaxAngle > 0 ) {
        _vRotationQuatLimitStart = quatangle * (1/_fQuatLimitMaxAngle);
    }
    else {
        _vRotationQuatLimitStart = GetTransform().rot;
    }
}

void RobotBase::SetAffineTranslationMaxVels(const Vector& vels)
{
    _vTranslationMaxVels = vels;
}

void RobotBase::SetAffineRotationAxisMaxVels(const Vector& vels)
{
    _vRotationAxisMaxVels = vels;
}

void RobotBase::SetAffineRotation3DMaxVels(const Vector& vels)
{
    _vRotation3DMaxVels = vels;
}

void RobotBase::SetAffineRotationQuatMaxVels(dReal anglevelocity)
{
    _fQuatMaxAngleVelocity = anglevelocity;
}

void RobotBase::SetAffineTranslationResolution(const Vector& resolution)
{
    _vTranslationResolutions = resolution;
}

void RobotBase::SetAffineRotationAxisResolution(const Vector& resolution)
{
    _vRotationAxisResolutions = resolution;
}

void RobotBase::SetAffineRotation3DResolution(const Vector& resolution)
{
    _vRotation3DResolutions = resolution;
}

void RobotBase::SetAffineRotationQuatResolution(dReal angleresolution)
{
    _fQuatAngleResolution = angleresolution;
}

void RobotBase::SetAffineTranslationWeights(const Vector& weights)
{
    _vTranslationWeights = weights;
}

void RobotBase::SetAffineRotationAxisWeights(const Vector& weights)
{
    _vRotationAxisWeights = weights;
}

void RobotBase::SetAffineRotation3DWeights(const Vector& weights)
{
    _vRotation3DWeights = weights;
}

void RobotBase::SetAffineRotationQuatWeights(dReal angleweight)
{
    _fQuatAngleWeight = angleweight;
}

void RobotBase::GetAffineTranslationLimits(Vector& lower, Vector& upper) const
{
    lower = _vTranslationLowerLimits;
    upper = _vTranslationUpperLimits;
}

void RobotBase::GetAffineRotationAxisLimits(Vector& lower, Vector& upper) const
{
    lower = _vRotationAxisLowerLimits;
    upper = _vRotationAxisUpperLimits;
}

void RobotBase::GetAffineRotation3DLimits(Vector& lower, Vector& upper) const
{
    lower = _vRotation3DLowerLimits;
    upper = _vRotation3DUpperLimits;
}

void RobotBase::SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOFBitmask, const Vector& vRotationAxis)
{
    vActvAffineRotationAxis = vRotationAxis;
    SetActiveDOFs(vJointIndices,nAffineDOFBitmask);
}

void RobotBase::SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOFBitmask)
{
    FOREACHC(itj, vJointIndices) {
        OPENRAVE_ASSERT_FORMAT(*itj>=0 && *itj<GetDOF(), "bad index %d (dof=%d)",*itj%GetDOF(),ORE_InvalidArguments);
    }
    // only reset the cache if the dof values are different
    if( active_dof_indices_vector_.size() != vJointIndices.size() ) {
        non_adjacent_link_cache_ &= ~AO_ActiveDOFs;
    }
    else {
        for(size_t i = 0; i < vJointIndices.size(); ++i) {
            if( active_dof_indices_vector_[i] != vJointIndices[i] ) {
                non_adjacent_link_cache_ &= ~AO_ActiveDOFs;
                break;
            }
        }
    }

    bool bactivedofchanged = false;
    if( active_dof_indices_vector_.size() != vJointIndices.size() ) {
        bactivedofchanged = true;
    }
    else {
        // same size, check to see if the values and order is the same
        for(size_t i = 0; i < active_dof_indices_vector_.size(); ++i) {
            if( active_dof_indices_vector_[i] != vJointIndices[i] ) {
                bactivedofchanged = true;
                break;
            }
        }
    }
    if( bactivedofchanged ) {
        active_dof_indices_vector_ = vJointIndices;
    }

    if( _nAffineDOFs != nAffineDOFBitmask ) {
        bactivedofchanged = true;
        _nAffineDOFs = nAffineDOFBitmask;
    }

    _nActiveDOF = vJointIndices.size() + RaveGetAffineDOF(_nAffineDOFs);

    if( bactivedofchanged ) {
        // do not initialize interpolation, since it implies a motion sampling strategy
        int offset = 0;
        active_spec_.groups_vector_.resize(0);
        if( GetActiveDOFIndices().size() > 0 ) {
            ConfigurationSpecification::Group group;
            stringstream ss;
            ss << "joint_values " << GetName();
            FOREACHC(it,GetActiveDOFIndices()) {
                ss << " " << *it;
            }
            group.name = ss.str();
            group.dof = (int)GetActiveDOFIndices().size();
            group.offset = offset;
            offset += group.dof;
            active_spec_.groups_vector_.push_back(group);
        }
        if( GetAffineDOF() > 0 ) {
            ConfigurationSpecification::Group group;
            group.name = str(boost::format("affine_transform %s %d")%GetName()%GetAffineDOF());
            group.offset = offset;
            group.dof = RaveGetAffineDOF(GetAffineDOF());
            active_spec_.groups_vector_.push_back(group);
        }

        _PostprocessChangedParameters(Prop_RobotActiveDOFs);
    }
}

void RobotBase::SetActiveDOFValues(const std::vector<dReal>& values, uint32_t bCheckLimits)
{
    if(_nActiveDOF < 0) {
        SetDOFValues(values,bCheckLimits);
        return;
    }
    OPENRAVE_ASSERT_OP_FORMAT((int)values.size(),>=,GetActiveDOF(), "not enough values %d<%d",values.size()%GetActiveDOF(),ORE_InvalidArguments);

    Transform t;
    if( (int)active_dof_indices_vector_.size() < _nActiveDOF ) {
        t = GetTransform();
        RaveGetTransformFromAffineDOFValues(t, values.begin()+active_dof_indices_vector_.size(),_nAffineDOFs,vActvAffineRotationAxis);
        if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
            t.rot = quatMultiply(_vRotationQuatLimitStart, t.rot);
        }
        if( active_dof_indices_vector_.size() == 0 ) {
            SetTransform(t);
        }
    }

    if( active_dof_indices_vector_.size() > 0 ) {
        GetDOFValues(_vTempRobotJoints);
        for(size_t i = 0; i < active_dof_indices_vector_.size(); ++i) {
            _vTempRobotJoints[active_dof_indices_vector_[i]] = values[i];
        }
        if( (int)active_dof_indices_vector_.size() < _nActiveDOF ) {
            SetDOFValues(_vTempRobotJoints, t, bCheckLimits);
        }
        else {
            SetDOFValues(_vTempRobotJoints, bCheckLimits);
        }
    }
}

void RobotBase::GetActiveDOFValues(std::vector<dReal>& values) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFValues(values);
        return;
    }

    values.resize(GetActiveDOF());
    if( values.size() == 0 ) {
        return;
    }
    vector<dReal>::iterator itvalues = values.begin();
    if( active_dof_indices_vector_.size() != 0 ) {
        GetDOFValues(_vTempRobotJoints);
        FOREACHC(it, active_dof_indices_vector_) {
            *itvalues++ = _vTempRobotJoints[*it];
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    Transform t = GetTransform();
    if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        t.rot = quatMultiply(quatInverse(_vRotationQuatLimitStart), t.rot);
    }
    RaveGetAffineDOFValuesFromTransform(itvalues,t,_nAffineDOFs,vActvAffineRotationAxis);
}

void RobotBase::SetActiveDOFVelocities(const std::vector<dReal>& velocities, uint32_t bCheckLimits)
{
    if(_nActiveDOF < 0) {
        SetDOFVelocities(velocities,true);
        return;
    }
    OPENRAVE_ASSERT_OP_FORMAT((int)velocities.size(),>=,GetActiveDOF(), "not enough values %d<%d",velocities.size()%GetActiveDOF(),ORE_InvalidArguments);

    Vector linearvel, angularvel;
    if( (int)active_dof_indices_vector_.size() < _nActiveDOF ) {
        // first set the affine transformation of the first link before setting joints
        const dReal* pAffineValues = &velocities[active_dof_indices_vector_.size()];

        links_vector_.at(0)->GetVelocity(linearvel, angularvel);

        if( _nAffineDOFs & OpenRAVE::DOF_X ) linearvel.x = *pAffineValues++;
        if( _nAffineDOFs & OpenRAVE::DOF_Y ) linearvel.y = *pAffineValues++;
        if( _nAffineDOFs & OpenRAVE::DOF_Z ) linearvel.z = *pAffineValues++;
        if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
            angularvel = vActvAffineRotationAxis * *pAffineValues++;
        }
        else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
            angularvel.x = *pAffineValues++;
            angularvel.y = *pAffineValues++;
            angularvel.z = *pAffineValues++;
        }
        else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_tr("quaternions not supported"),ORE_InvalidArguments);
        }

        if( active_dof_indices_vector_.size() == 0 ) {
            SetVelocity(linearvel, angularvel);
        }
    }

    if( active_dof_indices_vector_.size() > 0 ) {
        GetDOFVelocities(_vTempRobotJoints);
        std::vector<dReal>::const_iterator itvel = velocities.begin();
        FOREACHC(it, active_dof_indices_vector_) {
            _vTempRobotJoints[*it] = *itvel++;
        }
        if( (int)active_dof_indices_vector_.size() < _nActiveDOF ) {
            SetDOFVelocities(_vTempRobotJoints,linearvel,angularvel,bCheckLimits);
        }
        else {
            SetDOFVelocities(_vTempRobotJoints,bCheckLimits);
        }
    }
}

void RobotBase::GetActiveDOFVelocities(std::vector<dReal>& velocities) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFVelocities(velocities);
        return;
    }

    velocities.resize(GetActiveDOF());
    if( velocities.size() == 0 )
        return;
    dReal* pVelocities = &velocities[0];
    if( active_dof_indices_vector_.size() != 0 ) {
        GetDOFVelocities(_vTempRobotJoints);
        FOREACHC(it, active_dof_indices_vector_) {
            *pVelocities++ = _vTempRobotJoints[*it];
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    Vector linearvel, angularvel;
    links_vector_.at(0)->GetVelocity(linearvel, angularvel);

    if( _nAffineDOFs & OpenRAVE::DOF_X ) *pVelocities++ = linearvel.x;
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) *pVelocities++ = linearvel.y;
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) *pVelocities++ = linearvel.z;
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {

        *pVelocities++ = vActvAffineRotationAxis.dot3(angularvel);
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pVelocities++ = angularvel.x;
        *pVelocities++ = angularvel.y;
        *pVelocities++ = angularvel.z;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_tr("quaternions not supported"),ORE_InvalidArguments);
    }
}

void RobotBase::GetActiveDOFLimits(std::vector<dReal>& lower, std::vector<dReal>& upper) const
{
    lower.resize(GetActiveDOF());
    upper.resize(GetActiveDOF());
    if( GetActiveDOF() == 0 ) {
        return;
    }
    dReal* pLowerLimit = &lower[0];
    dReal* pUpperLimit = &upper[0];
    vector<dReal> alllower,allupper;

    if( _nAffineDOFs == 0 ) {
        if( _nActiveDOF < 0 ) {
            GetDOFLimits(lower,upper);
            return;
        }
        else {
            GetDOFLimits(alllower,allupper);
            FOREACHC(it, active_dof_indices_vector_) {
                *pLowerLimit++ = alllower.at(*it);
                *pUpperLimit++ = allupper.at(*it);
            }
        }
    }
    else {
        if( active_dof_indices_vector_.size() > 0 ) {
            GetDOFLimits(alllower,allupper);
            FOREACHC(it, active_dof_indices_vector_) {
                *pLowerLimit++ = alllower.at(*it);
                *pUpperLimit++ = allupper.at(*it);
            }
        }

        if( _nAffineDOFs & OpenRAVE::DOF_X ) {
            *pLowerLimit++ = _vTranslationLowerLimits.x;
            *pUpperLimit++ = _vTranslationUpperLimits.x;
        }
        if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
            *pLowerLimit++ = _vTranslationLowerLimits.y;
            *pUpperLimit++ = _vTranslationUpperLimits.y;
        }
        if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
            *pLowerLimit++ = _vTranslationLowerLimits.z;
            *pUpperLimit++ = _vTranslationUpperLimits.z;
        }

        if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
            *pLowerLimit++ = _vRotationAxisLowerLimits.x;
            *pUpperLimit++ = _vRotationAxisUpperLimits.x;
        }
        else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
            *pLowerLimit++ = _vRotation3DLowerLimits.x;
            *pLowerLimit++ = _vRotation3DLowerLimits.y;
            *pLowerLimit++ = _vRotation3DLowerLimits.z;
            *pUpperLimit++ = _vRotation3DUpperLimits.x;
            *pUpperLimit++ = _vRotation3DUpperLimits.y;
            *pUpperLimit++ = _vRotation3DUpperLimits.z;
        }
        else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
            // this is actually difficult to do correctly...
            dReal fsin = RaveSin(_fQuatLimitMaxAngle);
            *pLowerLimit++ = RaveCos(_fQuatLimitMaxAngle);
            *pLowerLimit++ = -fsin;
            *pLowerLimit++ = -fsin;
            *pLowerLimit++ = -fsin;
            *pUpperLimit++ = 1;
            *pUpperLimit++ = fsin;
            *pUpperLimit++ = fsin;
            *pUpperLimit++ = fsin;
        }
    }
}

void RobotBase::GetActiveDOFResolutions(std::vector<dReal>& resolution) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFResolutions(resolution);
        return;
    }

    resolution.resize(GetActiveDOF());
    if( resolution.size() == 0 ) {
        return;
    }
    dReal* pResolution = &resolution[0];

    GetDOFResolutions(_vTempRobotJoints);
    FOREACHC(it, active_dof_indices_vector_) {
        *pResolution++ = _vTempRobotJoints[*it];
    }
    // set some default limits
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        *pResolution++ = _vTranslationResolutions.x;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        *pResolution++ = _vTranslationResolutions.y;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) { *pResolution++ = _vTranslationResolutions.z; }

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        *pResolution++ = _vRotationAxisResolutions.x;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pResolution++ = _vRotation3DResolutions.x;
        *pResolution++ = _vRotation3DResolutions.y;
        *pResolution++ = _vRotation3DResolutions.z;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        *pResolution++ = _fQuatLimitMaxAngle;
        *pResolution++ = _fQuatLimitMaxAngle;
        *pResolution++ = _fQuatLimitMaxAngle;
        *pResolution++ = _fQuatLimitMaxAngle;
    }
}

void RobotBase::GetActiveDOFWeights(std::vector<dReal>& weights) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFWeights(weights);
        return;
    }

    weights.resize(GetActiveDOF());
    if( weights.size() == 0 ) {
        return;
    }
    dReal* pweight = &weights[0];

    GetDOFWeights(_vTempRobotJoints);
    FOREACHC(it, active_dof_indices_vector_) {
        *pweight++ = _vTempRobotJoints[*it];
    }
    // set some default limits
    if( _nAffineDOFs & OpenRAVE::DOF_X ) { *pweight++ = _vTranslationWeights.x; }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) { *pweight++ = _vTranslationWeights.y; }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) { *pweight++ = _vTranslationWeights.z; }

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) { *pweight++ = _vRotationAxisWeights.x; }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pweight++ = _vRotation3DWeights.x;
        *pweight++ = _vRotation3DWeights.y;
        *pweight++ = _vRotation3DWeights.z;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        *pweight++ = _fQuatAngleWeight;
        *pweight++ = _fQuatAngleWeight;
        *pweight++ = _fQuatAngleWeight;
        *pweight++ = _fQuatAngleWeight;
    }
}

void RobotBase::GetActiveDOFVelocityLimits(std::vector<dReal>& maxvel) const
{
    std::vector<dReal> dummy;
    if( _nActiveDOF < 0 ) {
        GetDOFVelocityLimits(dummy,maxvel);
        return;
    }
    maxvel.resize(GetActiveDOF());
    if( maxvel.size() == 0 ) {
        return;
    }
    dReal* pMaxVel = &maxvel[0];

    GetDOFVelocityLimits(dummy,_vTempRobotJoints);
    FOREACHC(it, active_dof_indices_vector_) {
        *pMaxVel++ = _vTempRobotJoints[*it];
    }
    if( _nAffineDOFs & OpenRAVE::DOF_X ) { *pMaxVel++ = _vTranslationMaxVels.x; }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) { *pMaxVel++ = _vTranslationMaxVels.y; }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) { *pMaxVel++ = _vTranslationMaxVels.z; }

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) { *pMaxVel++ = _vRotationAxisMaxVels.x; }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pMaxVel++ = _vRotation3DMaxVels.x;
        *pMaxVel++ = _vRotation3DMaxVels.y;
        *pMaxVel++ = _vRotation3DMaxVels.z;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        *pMaxVel++ = _fQuatMaxAngleVelocity;
        *pMaxVel++ = _fQuatMaxAngleVelocity;
        *pMaxVel++ = _fQuatMaxAngleVelocity;
        *pMaxVel++ = _fQuatMaxAngleVelocity;
    }
}

void RobotBase::GetActiveDOFAccelerationLimits(std::vector<dReal>& maxaccel) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFAccelerationLimits(maxaccel);
        return;
    }
    maxaccel.resize(GetActiveDOF());
    if( maxaccel.size() == 0 ) {
        return;
    }
    dReal* pMaxAccel = &maxaccel[0];

    GetDOFAccelerationLimits(_vTempRobotJoints);
    FOREACHC(it, active_dof_indices_vector_) {
        *pMaxAccel++ = _vTempRobotJoints[*it];
    }
    if( _nAffineDOFs & OpenRAVE::DOF_X ) { *pMaxAccel++ = _vTranslationMaxVels.x; } // wrong
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) { *pMaxAccel++ = _vTranslationMaxVels.y; } // wrong
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) { *pMaxAccel++ = _vTranslationMaxVels.z; } // wrong

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) { *pMaxAccel++ = _vRotationAxisMaxVels.x; } // wrong
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        *pMaxAccel++ = _vRotation3DMaxVels.x; // wrong
        *pMaxAccel++ = _vRotation3DMaxVels.y; // wrong
        *pMaxAccel++ = _vRotation3DMaxVels.z; // wrong
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
    }
}

void RobotBase::GetActiveDOFJerkLimits(std::vector<dReal>& maxjerk) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFJerkLimits(maxjerk);
        return;
    }
    maxjerk.resize(GetActiveDOF());
    if( maxjerk.size() == 0 ) {
        return;
    }
    dReal* pMaxJerk = &maxjerk[0];

    GetDOFJerkLimits(_vTempRobotJoints);
    FOREACHC(it, active_dof_indices_vector_) {
        *pMaxJerk++ = _vTempRobotJoints[*it];
    }
}

void RobotBase::GetActiveDOFHardVelocityLimits(std::vector<dReal>& maxvel) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFHardVelocityLimits(maxvel);
        return;
    }
    maxvel.resize(GetActiveDOF());
    if( maxvel.size() == 0 ) {
        return;
    }
    dReal* pMaxVel = &maxvel[0];

    GetDOFHardVelocityLimits(_vTempRobotJoints);
    FOREACHC(it, active_dof_indices_vector_) {
        *pMaxVel++ = _vTempRobotJoints[*it];
    }
}

void RobotBase::GetActiveDOFHardAccelerationLimits(std::vector<dReal>& maxaccel) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFHardAccelerationLimits(maxaccel);
        return;
    }
    maxaccel.resize(GetActiveDOF());
    if( maxaccel.size() == 0 ) {
        return;
    }
    dReal* pMaxAccel = &maxaccel[0];

    GetDOFHardAccelerationLimits(_vTempRobotJoints);
    FOREACHC(it, active_dof_indices_vector_) {
        *pMaxAccel++ = _vTempRobotJoints[*it];
    }
}

void RobotBase::GetActiveDOFHardJerkLimits(std::vector<dReal>& maxjerk) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFHardJerkLimits(maxjerk);
        return;
    }
    maxjerk.resize(GetActiveDOF());
    if( maxjerk.size() == 0 ) {
        return;
    }
    dReal* pMaxJerk = &maxjerk[0];

    GetDOFHardJerkLimits(_vTempRobotJoints);
    FOREACHC(it, active_dof_indices_vector_) {
        *pMaxJerk++ = _vTempRobotJoints[*it];
    }
}

void RobotBase::SubtractActiveDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const
{
    if( _nActiveDOF < 0 ) {
        SubtractDOFValues(q1,q2);
        return;
    }

    OPENRAVE_ASSERT_OP(q1.size(),==,q2.size());
    OPENRAVE_ASSERT_OP(q1.size(), >=, active_dof_indices_vector_.size());
    size_t index = 0;
    if (is_all_joints_1dof_and_no_circular_) {
        for (size_t i = 0; i < active_dof_indices_vector_.size(); ++i) {
            q1[i] -= q2[i];
        }
        index = active_dof_indices_vector_.size();
    }
    else {
        // go through all active joints
        for(; index < active_dof_indices_vector_.size(); ++index) {
            // We already did range check above
            JointConstPtr pjoint = GetJointFromDOFIndex(active_dof_indices_vector_[index]);
            q1[index] = pjoint->SubtractValue(q1[index],q2[index],active_dof_indices_vector_[index]-pjoint->GetDOFIndex());
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }

    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        q1.at(index) -= q2.at(index);
        index++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        q1.at(index) -= q2.at(index);
        index++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        q1.at(index) -= q2.at(index);
        index++;
    }

    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        q1.at(index) = utils::SubtractCircularAngle(q1.at(index),q2.at(index));
        index++;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        // would like to do q2^-1 q1, but that might break rest of planners...?
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
    }
}

const std::vector<int>& RobotBase::GetActiveDOFIndices() const
{
    return _nActiveDOF < 0 ? all_dof_indices_vector_ : active_dof_indices_vector_;
}

ConfigurationSpecification RobotBase::GetActiveConfigurationSpecification(const std::string& interpolation) const
{
    if( interpolation.size() == 0 ) {
        return active_spec_;
    }
    ConfigurationSpecification spec = active_spec_;
    FOREACH(itgroup,spec.groups_vector_) {
        itgroup->interpolation=interpolation;
    }
    return spec;
}

void RobotBase::CalculateActiveJacobian(int index, const Vector& offset, vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateJacobian(index, offset, vjacobian);
        return;
    }

    int dofstride = GetActiveDOF();
    vjacobian.resize(3*dofstride);
    if( active_dof_indices_vector_.size() != 0 ) {
        if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
            ComputeJacobianTranslation(index, offset, vjacobian, active_dof_indices_vector_);
            return;
        }
        // have to copy
        std::vector<dReal> vjacobianjoints;
        ComputeJacobianTranslation(index, offset, vjacobianjoints, active_dof_indices_vector_);
        for(size_t i = 0; i < 3; ++i) {
            std::copy(vjacobianjoints.begin()+i*active_dof_indices_vector_.size(),vjacobianjoints.begin()+(i+1)*active_dof_indices_vector_.size(),vjacobian.begin()+i*dofstride);
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    size_t ind = active_dof_indices_vector_.size();
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        vjacobian[ind] = 1;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 1;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 1;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        Vector vj = vActvAffineRotationAxis.cross(offset-GetTransform().trans);
        vjacobian[ind] = vj.x;
        vjacobian[dofstride+ind] = vj.y;
        vjacobian[2*dofstride+ind] = vj.z;
        ind++;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        // have to take the partial derivative dT/dA of the axis*angle representation with respect to the transformation it induces
        // can introduce converting to quaternions in the middle, then by chain rule,  dT/dA = dT/tQ * dQ/dA
        // for questions on derivation email rdiankov@cs.cmu.edu
        Transform t = GetTransform();
        dReal Qx = t.rot.x, Qy = t.rot.y, Qz = t.rot.z, Qw = t.rot.w;
        dReal Tx = offset.x-t.trans.x, Ty = offset.y-t.trans.y, Tz = offset.z-t.trans.z;

        // after some math, the dT/dQ looks like:
        dReal dRQ[12] = { 2*Qy*Ty+2*Qz*Tz,         -4*Qy*Tx+2*Qx*Ty+2*Qw*Tz,   -4*Qz*Tx-2*Qw*Ty+2*Qx*Tz,   -2*Qz*Ty+2*Qy*Tz,
                          2*Qy*Tx-4*Qx*Ty-2*Qw*Tz, 2*Qx*Tx+2*Qz*Tz,            2*Qw*Tx-4*Qz*Ty+2*Qy*Tz,    2*Qz*Tx-2*Qx*Tz,
                          2*Qz*Tx+2*Qw*Ty-4*Qx*Tz, -2*Qw*Tx+2*Qz*Ty-4*Qy*Tz,   2*Qx*Tx+2*Qy*Ty,            -2*Qy*Tx+2*Qx*Ty };

        // calc dQ/dA
        dReal fsin = sqrt(t.rot.y * t.rot.y + t.rot.z * t.rot.z + t.rot.w * t.rot.w);
        dReal fcos = t.rot.x;
        dReal fangle = 2 * atan2(fsin, fcos);
        dReal normalizer = fangle / fsin;
        dReal Ax = normalizer * t.rot.y;
        dReal Ay = normalizer * t.rot.z;
        dReal Az = normalizer * t.rot.w;

        if( RaveFabs(fangle) < 1e-8f )
            fangle = 1e-8f;

        dReal fangle2 = fangle*fangle;
        dReal fiangle2 = 1/fangle2;
        dReal inormalizer = normalizer > 0 ? 1/normalizer : 0;
        dReal fconst = inormalizer*fiangle2;
        dReal fconst2 = fcos*fiangle2;
        dReal dQA[12] = { -0.5f*Ax*inormalizer,                     -0.5f*Ay*inormalizer,                       -0.5f*Az*inormalizer,
                          inormalizer+0.5f*Ax*Ax*(fconst2-fconst),  0.5f*Ax*fconst2*Ay-Ax*fconst*Ay,            0.5f*Ax*fconst2*Az-Ax*fconst*Az,
                          0.5f*Ax*fconst2*Ay-Ax*fconst*Ay,          inormalizer+0.5f*Ay*Ay*(fconst2-fconst),    0.5f*Ay*fconst2*Az-Ay*fconst*Az,
                          0.5f*Ax*fconst2*Az-Ax*fconst*Az,          0.5f*Ay*fconst2*Az-Ay*fconst*Az,            inormalizer+0.5f*Az*Az*(fconst2-fconst)};

        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                vjacobian[i*dofstride+ind+j] = dRQ[4*i+0]*dQA[3*0+j] + dRQ[4*i+1]*dQA[3*1+j] + dRQ[4*i+2]*dQA[3*2+j] + dRQ[4*i+3]*dQA[3*3+j];
            }
        }
        ind += 3;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        Transform t; t.identity(); t.rot = quatInverse(_vRotationQuatLimitStart);
        t = t * GetTransform();
        // note: qw, qx, qy, qz here follow the standard quaternion convention, not the openrave one
        dReal qw = t.rot[0], qx = t.rot[1], qy = t.rot[2], qz = t.rot[3];
        Vector offset_local = t.inverse() * offset;
        dReal x = offset_local.x, y = offset_local.y, z = offset_local.z;

        dReal dRQ[12] = {-2*qz*y + 2*qw*x + 2*qy*z,2*qx*x + 2*qy*y + 2*qz*z,-2*qy*x + 2*qw*z + 2*qx*y,-2*qw*y - 2*qz*x + 2*qx*z,-2*qx*z + 2*qw*y + 2*qz*x,-2*qw*z - 2*qx*y + 2*qy*x,2*qx*x + 2*qy*y + 2*qz*z,-2*qz*y + 2*qw*x + 2*qy*z,-2*qy*x + 2*qw*z + 2*qx*y,-2*qx*z + 2*qw*y + 2*qz*x,-2*qw*x - 2*qy*z + 2*qz*y,2*qx*x + 2*qy*y + 2*qz*z};
        for (int i=0; i < 3; ++i) {
            double qdotrow = dRQ[4*i]*qw + dRQ[4*i+1]*qx + dRQ[4*i+2]*qy + dRQ[4*i+3]*qz;
            dRQ[4*i] -= qdotrow*qw;
            dRQ[4*i+1] -= qdotrow*qx;
            dRQ[4*i+2] -= qdotrow*qy;
            dRQ[4*i+3] -= qdotrow*qz;
        }

        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 4; ++j) {
                vjacobian[i*dofstride+ind + j] = dRQ[4*i+j];
            }
        }
        ind += 4;
    }
}

void RobotBase::CalculateActiveJacobian(int linkindex, const Vector& offset, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateJacobian(linkindex, offset, mjacobian);
        return;
    }
    std::vector<dReal> vjacobian;
    RobotBase::CalculateActiveJacobian(linkindex,offset,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetActiveDOF());
    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetActiveDOF(),itdst->begin());
        itsrc += GetActiveDOF();
    }
}

void RobotBase::CalculateActiveRotationJacobian(int index, const Vector& q, std::vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateRotationJacobian(index, q, vjacobian);
        return;
    }
    int dofstride = GetActiveDOF();
    vjacobian.resize(4*dofstride);
    if( active_dof_indices_vector_.size() != 0 ) {
        std::vector<dReal> vjacobianjoints;
        CalculateRotationJacobian(index, q, vjacobianjoints);
        for(size_t i = 0; i < active_dof_indices_vector_.size(); ++i) {
            vjacobian[i] = vjacobianjoints[active_dof_indices_vector_[i]];
            vjacobian[dofstride+i] = vjacobianjoints[GetDOF()+active_dof_indices_vector_[i]];
            vjacobian[2*dofstride+i] = vjacobianjoints[2*GetDOF()+active_dof_indices_vector_[i]];
            vjacobian[3*dofstride+i] = vjacobianjoints[3*GetDOF()+active_dof_indices_vector_[i]];
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }

    size_t ind = active_dof_indices_vector_.size();
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        vjacobian[3*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        vjacobian[3*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        vjacobian[3*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        vjacobian[ind] = dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
        vjacobian[dofstride+ind] = dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
        vjacobian[2*dofstride+ind] = dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
        vjacobian[3*dofstride+ind] = dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
        ind++;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("robot %s rotation 3d not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);
        ind += 3;
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("robot %s quaternion not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);
        ind += 4;
    }
}

void RobotBase::CalculateActiveRotationJacobian(int linkindex, const Vector& q, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateRotationJacobian(linkindex, q, mjacobian);
        return;
    }
    std::vector<dReal> vjacobian;
    RobotBase::CalculateActiveRotationJacobian(linkindex,q,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,4*GetActiveDOF());
    mjacobian.resize(boost::extents[4][GetActiveDOF()]);
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetActiveDOF(),itdst->begin());
        itsrc += GetActiveDOF();
    }
}

void RobotBase::CalculateActiveAngularVelocityJacobian(int index, std::vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        ComputeJacobianAxisAngle(index, vjacobian);
        return;
    }

    int dofstride = GetActiveDOF();
    vjacobian.resize(3*dofstride);
    if( active_dof_indices_vector_.size() != 0 ) {
        if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
            ComputeJacobianAxisAngle(index, vjacobian, active_dof_indices_vector_);
            return;
        }
        // have to copy
        std::vector<dReal> vjacobianjoints;
        ComputeJacobianAxisAngle(index, vjacobianjoints, active_dof_indices_vector_);
        for(size_t i = 0; i < 3; ++i) {
            std::copy(vjacobianjoints.begin()+i*active_dof_indices_vector_.size(),vjacobianjoints.begin()+(i+1)*active_dof_indices_vector_.size(),vjacobian.begin()+i*dofstride);
        }
    }

    if( _nAffineDOFs == OpenRAVE::DOF_NoTransform ) {
        return;
    }
    size_t ind = active_dof_indices_vector_.size();
    if( _nAffineDOFs & OpenRAVE::DOF_X ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Y ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_Z ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & OpenRAVE::DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        vjacobian[ind] = v.x;
        vjacobian[dofstride+ind] = v.y;
        vjacobian[2*dofstride+ind] = v.z;

    }
    else if( _nAffineDOFs & OpenRAVE::DOF_Rotation3D ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("robot %s rotation 3d not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);
    }
    else if( _nAffineDOFs & OpenRAVE::DOF_RotationQuat ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("robot %s quaternion not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);

        // most likely wrong
        Transform t; t.rot = quatInverse(_vRotationQuatLimitStart);
        t = t * GetTransform();
        dReal fnorm = t.rot.y*t.rot.y+t.rot.z*t.rot.z+t.rot.w*t.rot.w;
        if( fnorm > 0 ) {
            fnorm = dReal(1)/RaveSqrt(fnorm);
            vjacobian[ind] = t.rot.y*fnorm;
            vjacobian[dofstride+ind] = t.rot.z*fnorm;
            vjacobian[2*dofstride+ind] = t.rot.w*fnorm;
        }
        else {
            vjacobian[ind] = 0;
            vjacobian[dofstride+ind] = 0;
            vjacobian[2*dofstride+ind] = 0;
        }

        ++ind;
    }
}

void RobotBase::CalculateActiveAngularVelocityJacobian(int linkindex, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateAngularVelocityJacobian(linkindex, mjacobian);
        return;
    }
    std::vector<dReal> vjacobian;
    CalculateActiveAngularVelocityJacobian(linkindex,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetActiveDOF());
    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetActiveDOF(),itdst->begin());
        itsrc += GetActiveDOF();
    }
}

bool CompareNonAdjacentFarthest(int pair0, int pair1); // defined in kinbody.cpp

const std::vector<int>& RobotBase::GetNonAdjacentLinks(int adjacentoptions) const
{
    KinBody::GetNonAdjacentLinks(0); // need to call to set the cache
    if( (non_adjacent_link_cache_&adjacentoptions) != adjacentoptions ) {
        int requestedoptions = (~non_adjacent_link_cache_)&adjacentoptions;
        // find out what needs to computed
        std::array<uint8_t,4> compute={ { 0,0,0,0}};
        if( requestedoptions & AO_Enabled ) {
            for(size_t i = 0; i < compute.size(); ++i) {
                if( i & AO_Enabled ) {
                    compute[i] = 1;
                }
            }
        }
        if( requestedoptions & AO_ActiveDOFs ) {
            for(size_t i = 0; i < compute.size(); ++i) {
                if( i & AO_ActiveDOFs ) {
                    compute[i] = 1;
                }
            }
        }
        if( requestedoptions & ~(AO_Enabled|AO_ActiveDOFs) ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("does not support adjacentoptions %d"),adjacentoptions,ORE_InvalidArguments);
        }

        // compute it
        if( compute.at(AO_Enabled) ) {
            non_adjacent_links_vector_.at(AO_Enabled).resize(0);
            FOREACHC(itset, non_adjacent_links_vector_[0]) {
                KinBody::LinkConstPtr plink1(links_vector_.at(*itset&0xffff)), plink2(links_vector_.at(*itset>>16));
                if( plink1->IsEnabled() && plink2->IsEnabled() ) {
                    non_adjacent_links_vector_[AO_Enabled].push_back(*itset);
                }
            }
            std::sort(non_adjacent_links_vector_[AO_Enabled].begin(), non_adjacent_links_vector_[AO_Enabled].end(), CompareNonAdjacentFarthest);
        }
        if( compute.at(AO_ActiveDOFs) ) {
            non_adjacent_links_vector_.at(AO_ActiveDOFs).resize(0);
            FOREACHC(itset, non_adjacent_links_vector_[0]) {
                FOREACHC(it, GetActiveDOFIndices()) {
                    if( IsDOFInChain(*itset&0xffff,*itset>>16,*it) ) {
                        non_adjacent_links_vector_[AO_ActiveDOFs].push_back(*itset);
                        break;
                    }
                }
            }
            std::sort(non_adjacent_links_vector_[AO_ActiveDOFs].begin(), non_adjacent_links_vector_[AO_ActiveDOFs].end(), CompareNonAdjacentFarthest);
        }
        if( compute.at(AO_Enabled|AO_ActiveDOFs) ) {
            non_adjacent_links_vector_.at(AO_Enabled|AO_ActiveDOFs).resize(0);
            FOREACHC(itset, non_adjacent_links_vector_[AO_ActiveDOFs]) {
                KinBody::LinkConstPtr plink1(links_vector_.at(*itset&0xffff)), plink2(links_vector_.at(*itset>>16));
                if( plink1->IsEnabled() && plink2->IsEnabled() ) {
                    non_adjacent_links_vector_[AO_Enabled|AO_ActiveDOFs].push_back(*itset);
                }
            }
            std::sort(non_adjacent_links_vector_[AO_Enabled|AO_ActiveDOFs].begin(), non_adjacent_links_vector_[AO_Enabled|AO_ActiveDOFs].end(), CompareNonAdjacentFarthest);
        }
        non_adjacent_link_cache_ |= requestedoptions;
    }
    return non_adjacent_links_vector_.at(adjacentoptions);
}

void RobotBase::SetNonCollidingConfiguration()
{
    KinBody::SetNonCollidingConfiguration();
    RegrabAll();
}

bool RobotBase::Grab(KinBodyPtr pbody)
{
    ManipulatorPtr pmanip = GetActiveManipulator();
    if( !pmanip ) {
        return false;
    }
    return Grab(pbody, pmanip->GetEndEffector());
}

bool RobotBase::Grab(KinBodyPtr pbody, const std::set<int>& setRobotLinksToIgnore)
{
    ManipulatorPtr pmanip = GetActiveManipulator();
    if( !pmanip ) {
        return false;
    }
    return Grab(pbody, pmanip->GetEndEffector(), setRobotLinksToIgnore);
}

bool RobotBase::Grab(KinBodyPtr body, LinkPtr pRobotLinkToGrabWith)
{
    return KinBody::Grab(body, pRobotLinkToGrabWith);
}

bool RobotBase::Grab(KinBodyPtr body, LinkPtr pRobotLinkToGrabWith, const std::set<int>& setRobotLinksToIgnore)
{
    return KinBody::Grab(body, pRobotLinkToGrabWith, setRobotLinksToIgnore);
}

void RobotBase::SetActiveManipulator(ManipulatorConstPtr pmanip)
{
    if( !pmanip ) {
        manipulator_active_.reset();
    }
    else {
        FOREACH(itmanip,manipulators_vector_) {
            if( *itmanip == pmanip ) {
                manipulator_active_ = *itmanip;
                return;
            }
        }
        // manipulator might have been recoreded, search for the same name
        FOREACH(itmanip,manipulators_vector_) {
            if( (*itmanip)->GetName() == pmanip->GetName() ) {
                manipulator_active_ = *itmanip;
                return;
            }
        }

        manipulator_active_.reset();
        RAVELOG_WARN_FORMAT("failed to find manipulator with name %s, most likely removed", pmanip->GetName());
    }
}

RobotBase::ManipulatorPtr RobotBase::SetActiveManipulator(const std::string& manipname)
{
    if( manipname.size() > 0 ) {
        FOREACH(itmanip,manipulators_vector_) {
            if( (*itmanip)->GetName() == manipname ) {
                manipulator_active_ = *itmanip;
                return manipulator_active_;
            }
        }
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to find manipulator with name: %s"), manipname, ORE_InvalidArguments);
    }
    manipulator_active_.reset();
    return manipulator_active_;
}

RobotBase::ManipulatorPtr RobotBase::GetActiveManipulator()
{
    return manipulator_active_;
}

RobotBase::ManipulatorConstPtr RobotBase::GetActiveManipulator() const
{
    return manipulator_active_;
}

RobotBase::ManipulatorPtr RobotBase::AddManipulator(const RobotBase::ManipulatorInfo& manipinfo, bool removeduplicate)
{
    OPENRAVE_ASSERT_OP(manipinfo.name_.size(),>,0);
    int iremoveindex = -1;
    for(int imanip = 0; imanip < (int)manipulators_vector_.size(); ++imanip) {
        if( manipulators_vector_[imanip]->GetName() == manipinfo.name_ ) {
            if( removeduplicate ) {
                iremoveindex = imanip;
                break;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("manipulator with name %s already exists"),manipinfo.name_,ORE_InvalidArguments);
            }
        }
    }
    ManipulatorPtr newmanip(new Manipulator(shared_robot(),manipinfo));
    newmanip->_ComputeInternalInformation();
    if( iremoveindex >= 0 ) {
        // replace the old one
        manipulators_vector_[iremoveindex] = newmanip;
    }
    else {
        manipulators_vector_.push_back(newmanip);
    }
    hash_robot_structure_.resize(0);
    return newmanip;
}

bool RobotBase::RemoveManipulator(ManipulatorPtr manip)
{
    if( manipulator_active_ == manip ) {
        manipulator_active_.reset();
    }
    FOREACH(itmanip,manipulators_vector_) {
        if( *itmanip == manip ) {
            manipulators_vector_.erase(itmanip);
            hash_robot_structure_.resize(0);
            return true;
        }
    }
    return false;
}

RobotBase::AttachedSensorPtr RobotBase::AddAttachedSensor(const RobotBase::AttachedSensorInfo& attachedsensorinfo, bool removeduplicate)
{
    OPENRAVE_ASSERT_OP(attachedsensorinfo.name_.size(),>,0);
    int iremoveindex = -1;
    for(int iasensor = 0; iasensor < (int)attached_sensors_vector_.size(); ++iasensor) {
        if( attached_sensors_vector_[iasensor]->GetName() == attachedsensorinfo.name_ ) {
            if( removeduplicate ) {
                iremoveindex = iasensor;
                break;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("attached sensor with name %s already exists"),attachedsensorinfo.name_,ORE_InvalidArguments);
            }
        }
    }
    AttachedSensorPtr newattachedsensor(new AttachedSensor(shared_robot(),attachedsensorinfo));
//    if( hierarchy_computed_ ) {
//        newattachedsensor->_ComputeInternalInformation();
//    }
    if( iremoveindex >= 0 ) {
        // replace the old one
        attached_sensors_vector_[iremoveindex] = newattachedsensor;
    }
    else {
        attached_sensors_vector_.push_back(newattachedsensor);
    }
    newattachedsensor->UpdateInfo(); // just in case
    hash_robot_structure_.resize(0);
    return newattachedsensor;
}

RobotBase::AttachedSensorPtr RobotBase::GetAttachedSensor(const std::string& name) const
{
    FOREACHC(itsensor, attached_sensors_vector_) {
        if( (*itsensor)->GetName() == name ) {
            return *itsensor;
        }
    }
    return RobotBase::AttachedSensorPtr();
}

bool RobotBase::RemoveAttachedSensor(RobotBase::AttachedSensor &attsensor)
{
    FOREACH(itattsensor,attached_sensors_vector_) {
        if( itattsensor->get() == &attsensor ) {
            attached_sensors_vector_.erase(itattsensor);
            hash_robot_structure_.resize(0);
            return true;
        }
    }
    return false;
}

void RobotBase::SimulationStep(dReal fElapsedTime)
{
    KinBody::SimulationStep(fElapsedTime);
    _UpdateAttachedSensors();
}

void RobotBase::_ComputeInternalInformation()
{
    _ComputeConnectedBodiesInformation(); // should process the connected bodies in order to get the real resolved links, joints, etc

    KinBody::_ComputeInternalInformation();
    all_dof_indices_vector_.resize(GetDOF());
    for(int i = 0; i < GetDOF(); ++i) {
        all_dof_indices_vector_[i] = i;
    }

    active_spec_.groups_vector_.reserve(2);
    active_spec_.groups_vector_.resize(0);
    if( all_dof_indices_vector_.size() > 0 ) {
        ConfigurationSpecification::Group group;
        stringstream ss;
        ss << "joint_values " << GetName();
        if( _nActiveDOF >= 0 ) {
            // use active_dof_indices_vector_
            FOREACHC(it,active_dof_indices_vector_) {
                ss << " " << *it;
            }
            group.dof = (int)active_dof_indices_vector_.size();
        }
        else {
            FOREACHC(it,all_dof_indices_vector_) {
                ss << " " << *it;
            }
            group.dof = (int)all_dof_indices_vector_.size();
        }
        group.name = ss.str();
        group.offset = 0;
        // do not initialize interpolation, since it implies a motion sampling strategy
        active_spec_.groups_vector_.push_back(group);
    }

    int manipindex=0;
    FOREACH(itmanip,manipulators_vector_) {
        if( (*itmanip)->info_.name_.size() == 0 ) {
            stringstream ss;
            ss << "manip" << manipindex;
            RAVELOG_WARN(str(boost::format("robot %s has a manipulator with no name, setting to %s\n")%GetName()%ss.str()));
            (*itmanip)->info_.name_ = ss.str();
        }
        (*itmanip)->_ComputeInternalInformation();
        vector<ManipulatorPtr>::iterator itmanip2 = itmanip; ++itmanip2;
        for(; itmanip2 != manipulators_vector_.end(); ++itmanip2) {
            if( (*itmanip)->GetName() == (*itmanip2)->GetName() ) {
                RAVELOG_WARN(str(boost::format("robot %s has two manipulators with the same name: %s!\n")%GetName()%(*itmanip)->GetName()));
            }
        }
        manipindex++;
    }
    // set active manipulator to first manipulator
    if( manipulators_vector_.size() > 0 ) {
        // preserve active manip when robot is removed and added back to the env
        if (!!manipulator_active_) {
            bool bmanipfound = false;
            FOREACHC(itmanip, manipulators_vector_) {
                if (*itmanip == manipulator_active_) {
                    bmanipfound = true;
                    break;
                }
            }
            if (!bmanipfound) {
                manipulator_active_.reset();
            }
        }
        if (!manipulator_active_) {
            manipulator_active_ = manipulators_vector_.at(0);
        }
    }
    else {
        manipulator_active_.reset();
    }

    int sensorindex=0;
    FOREACH(itsensor,attached_sensors_vector_) {
        if( (*itsensor)->GetName().size() == 0 ) {
            stringstream ss;
            ss << "sensor" << sensorindex;
            RAVELOG_WARN(str(boost::format("robot %s has a sensor with no name, setting to %s\n")%GetName()%ss.str()));
            (*itsensor)->info_.name_ = ss.str();
        }
        else if( !utils::IsValidName((*itsensor)->GetName()) ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("sensor name \"%s\" is not valid"), (*itsensor)->GetName(), ORE_Failed);
        }
        //(*itsensor)->_ComputeInternalInformation();
        sensorindex++;
    }

    {
        hash_robot_structure_.resize(0);
        FOREACH(itsensor,attached_sensors_vector_) {
            (*itsensor)->hash_structure_.resize(0);
        }
    }

    if( ComputeAABB().extents.lengthsqr3() > 900.0f ) {
        RAVELOG_WARN(str(boost::format("Robot %s span is greater than 30 meaning that it is most likely defined in a unit other than meters. It is highly encouraged to define all OpenRAVE robots in meters since many metrics, database models, and solvers have been specifically optimized for this unit\n")%GetName()));
    }

    if( !GetController() ) {
        RAVELOG_VERBOSE(str(boost::format("no default controller set on robot %s\n")%GetName()));
        std::vector<int> dofindices;
        for(int i = 0; i < GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        SetController(RaveCreateController(GetEnv(), "IdealController"),dofindices,1);
    }

    // reset the power on the sensors
    FOREACH(itsensor,attached_sensors_vector_) {
        SensorBasePtr psensor = (*itsensor)->GetSensor();
        if( !!psensor ) {
            int ispower = psensor->Configure(SensorBase::CC_PowerCheck);
            psensor->Configure(ispower ? SensorBase::CC_PowerOn : SensorBase::CC_PowerOff);
        }
    }
}

void RobotBase::_DeinitializeInternalInformation()
{
    KinBody::_DeinitializeInternalInformation();
    _DeinitializeConnectedBodiesInformation();
}

void RobotBase::_PostprocessChangedParameters(uint32_t parameters)
{
    if( parameters & (Prop_Sensors|Prop_SensorPlacement) ) {
        FOREACH(itsensor,attached_sensors_vector_) {
            (*itsensor)->hash_structure_.resize(0);
        }
    }
    if( parameters & Prop_RobotManipulatorTool ) {
        FOREACH(itmanip,manipulators_vector_) {
            (*itmanip)->hash_structure_.resize(0);
            (*itmanip)->hash_kinematics_structure_.resize(0);
        }
    }
    KinBody::_PostprocessChangedParameters(parameters);
}

const std::vector<RobotBase::ManipulatorPtr>& RobotBase::GetManipulators() const
{
    return manipulators_vector_;
}

RobotBase::ManipulatorPtr RobotBase::GetManipulator(const std::string& name) const
{
    FOREACHC(itmanip, manipulators_vector_) {
        if( (*itmanip)->GetName() == name ) {
            return *itmanip;
        }
    }
    return RobotBase::ManipulatorPtr();
}

void RobotBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    KinBody::Clone(preference,cloningoptions);
    RobotBaseConstPtr r = RaveInterfaceConstCast<RobotBase>(preference);
    self_collision_checker_.reset();
    if( !!r->self_collision_checker_ ) {
        // TODO clone the self collision checker?
    }
    hash_robot_structure_ = r->hash_robot_structure_;
    manipulators_vector_.clear();
    manipulator_active_.reset();
    FOREACHC(itmanip, r->manipulators_vector_) {
        ManipulatorPtr pmanip(new Manipulator(shared_robot(),*itmanip));
        manipulators_vector_.push_back(pmanip);
        if( !!r->GetActiveManipulator() && r->GetActiveManipulator()->GetName() == (*itmanip)->GetName() ) {
            manipulator_active_ = pmanip;
        }
    }

    connected_bodies_vector_.clear();
    FOREACHC(itConnectedBody, r->connected_bodies_vector_) {
        ConnectedBodyPtr pConnectedBody(new ConnectedBody(shared_robot(),**itConnectedBody,cloningoptions));
        connected_bodies_vector_.push_back(pConnectedBody);
    }

    attached_sensors_vector_.clear();
    FOREACHC(itsensor, r->attached_sensors_vector_) {
        attached_sensors_vector_.push_back(AttachedSensorPtr(new AttachedSensor(shared_robot(),**itsensor,cloningoptions)));
    }
    _UpdateAttachedSensors();

    active_dof_indices_vector_ = r->active_dof_indices_vector_;
    active_spec_ = r->active_spec_;
    all_dof_indices_vector_ = r->all_dof_indices_vector_;
    vActvAffineRotationAxis = r->vActvAffineRotationAxis;
    _nActiveDOF = r->_nActiveDOF;
    _nAffineDOFs = r->_nAffineDOFs;

    _vTranslationLowerLimits = r->_vTranslationLowerLimits;
    _vTranslationUpperLimits = r->_vTranslationUpperLimits;
    _vTranslationMaxVels = r->_vTranslationMaxVels;
    _vTranslationResolutions = r->_vTranslationResolutions;
    _vRotationAxisLowerLimits = r->_vRotationAxisLowerLimits;
    _vRotationAxisUpperLimits = r->_vRotationAxisUpperLimits;
    _vRotationAxisMaxVels = r->_vRotationAxisMaxVels;
    _vRotationAxisResolutions = r->_vRotationAxisResolutions;
    _vRotation3DLowerLimits = r->_vRotation3DLowerLimits;
    _vRotation3DUpperLimits = r->_vRotation3DUpperLimits;
    _vRotation3DMaxVels = r->_vRotation3DMaxVels;
    _vRotation3DResolutions = r->_vRotation3DResolutions;
    _vRotationQuatLimitStart = r->_vRotationQuatLimitStart;
    _fQuatLimitMaxAngle = r->_fQuatLimitMaxAngle;
    _fQuatMaxAngleVelocity = r->_fQuatMaxAngleVelocity;
    _fQuatAngleResolution = r->_fQuatAngleResolution;
    _fQuatAngleWeight = r->_fQuatAngleWeight;

    // clone the controller
    if( (cloningoptions&Clone_RealControllers) && !!r->GetController() ) {
        if( !SetController(RaveCreateController(GetEnv(), r->GetController()->GetXMLId()),r->GetController()->GetControlDOFIndices(),r->GetController()->IsControlTransformation()) ) {
            RAVELOG_WARN(str(boost::format("failed to set %s controller for robot %s\n")%r->GetController()->GetXMLId()%GetName()));
        }
    }

    if( !GetController() ) {
        std::vector<int> dofindices;
        for(int i = 0; i < GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        if( !SetController(RaveCreateController(GetEnv(), "IdealController"),dofindices, 1) ) {
            RAVELOG_WARN("failed to set IdealController\n");
        }
    }
}

void RobotBase::serialize(std::ostream& o, int options) const
{
    KinBody::serialize(o,options);
    if( options & SO_RobotManipulators ) {
        FOREACHC(itmanip,manipulators_vector_) {
            (*itmanip)->serialize(o,options);
        }
    }
    if( options & SO_RobotSensors ) {
        FOREACHC(itsensor,attached_sensors_vector_) {
            (*itsensor)->serialize(o,options);
        }
    }
}

const std::string& RobotBase::GetRobotStructureHash() const
{
    CHECK_INTERNAL_COMPUTATION;
    if( hash_robot_structure_.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_Kinematics|SO_Geometry|SO_RobotManipulators|SO_RobotSensors);
        hash_robot_structure_ = utils::GetMD5HashString(ss.str());
    }
    return hash_robot_structure_;
}

} // end namespace OpenRAVE
