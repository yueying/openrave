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
#include <algorithm>
#include <boost/algorithm/string.hpp> // boost::trim
#include <boost/lexical_cast.hpp>
#include <openrave/kinbody.h>
#include <openrave/openrave_macros.h>
#include "fparsermulti.h"


namespace OpenRAVE 
{


KinBody::JointInfo::JointControlInfo_RobotController::JointControlInfo_RobotController() : robotId(-1)
{
    robotControllerDOFIndex[0] = robotControllerDOFIndex[1] = robotControllerDOFIndex[2] = -1;
}

KinBody::JointInfo::JointControlInfo_IO::JointControlInfo_IO() : deviceId(-1)
{
}

KinBody::JointInfo::JointControlInfo_ExternalDevice::JointControlInfo_ExternalDevice()
{
}

KinBody::JointInfo::JointInfo() 
	: type_(JointNone), 
	is_active_(true),
	control_mode_(JCM_None) 
{
    for(size_t i = 0; i < axes_vector_.size(); ++i)
	{
        axes_vector_[i] = Vector(0,0,1);
    }
    std::fill(resolution_vector_.begin(), resolution_vector_.end(), 0.02);
    std::fill(max_velocity_vector_.begin(), max_velocity_vector_.end(), 10);
    std::fill(hard_max_velocity_vector_.begin(), hard_max_velocity_vector_.end(), 0); // Default hard limits is 0. if 0, the user should not use the hard limit value.
    std::fill(max_accelerate_vector_.begin(), max_accelerate_vector_.end(), 50);
    std::fill(hard_max_accelerate_vector_.begin(), hard_max_accelerate_vector_.end(), 0); // Default hard limits is 0. if 0, the user should not use the hard limit value.
    std::fill(max_jerk_vector_.begin(), max_jerk_vector_.end(), 50*1000); // Set negligibly large jerk by default which can change acceleration between min and max within a typical time step.
    std::fill(hard_max_jerk_vector_.begin(), hard_max_jerk_vector_.end(), 0); // Default hard limits is 0. if 0, the user should not use the hard limit value.
    std::fill(max_torque_vector_.begin(), max_torque_vector_.end(), 0); // set max torque to 0 to notify the system that dynamics parameters might not be valid.
    std::fill(max_inertia_vector_.begin(), max_inertia_vector_.end(), 0);
    std::fill(weights_vector_.begin(), weights_vector_.end(), 1);
    std::fill(offsets_vector_.begin(), offsets_vector_.end(), 0);
    std::fill(lower_limit_vector_.begin(), lower_limit_vector_.end(), 0);
    std::fill(upper_limit_vector_.begin(), upper_limit_vector_.end(), 0);
    std::fill(is_circular_.begin(), is_circular_.end(), 0);
}

KinBody::JointInfo::JointInfo(const JointInfo& other)
{
    *this = other;
}

int KinBody::JointInfo::GetDOF() const
{
    if(type_ & KinBody::JointSpecialBit)
	{
        switch(type_) 
		{
        case KinBody::JointHinge2:
        case KinBody::JointUniversal: return 2;
        case KinBody::JointSpherical: return 3;
        case KinBody::JointTrajectory: return 1;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("invalid joint type 0x%x"), type_, ORE_Failed);
        }
    }
    return int(type_ & 0xf);
}

void KinBody::JointInfo::SerializeJSON(rapidjson::Value& value,
	rapidjson::Document::AllocatorType& allocator, dReal unit_scale, int options) const
{
    int dof = GetDOF();

    switch (type_) 
	{
    case JointRevolute:
        openravejson::SetJsonValueByKey(value, "type", "revolute", allocator);
        break;
    case JointPrismatic:
        openravejson::SetJsonValueByKey(value, "type", "prismatic", allocator);
        break;
    case JointNone:
        break;
    default:
        openravejson::SetJsonValueByKey(value, "type", static_cast<int>(type_), allocator);
        break;
    }

    dReal fjointmult = unit_scale;
    if(type_ == JointRevolute)
    {
        fjointmult = 1;
    }
    else if(type_ == JointPrismatic)
    {
        fjointmult = unit_scale;
    }

    openravejson::SetJsonValueByKey(value, "name", name_, allocator);
    openravejson::SetJsonValueByKey(value, "anchors", anchor_, allocator);
    openravejson::SetJsonValueByKey(value, "parentLinkName", link_name0_, allocator);
    openravejson::SetJsonValueByKey(value, "childLinkName", link_name1_, allocator);
    openravejson::SetJsonValueByKey(value, "axes", axes_vector_, allocator);
    openravejson::SetJsonValueByKey(value, "currentValues", current_values_vector_, allocator);
    openravejson::SetJsonValueByKey(value, "resolutions", resolution_vector_, allocator, dof);

    std::array<dReal, 3> newvmaxvel = max_velocity_vector_;
    std::array<dReal, 3> newvmaxaccel = max_accelerate_vector_;
    std::array<dReal, 3> newvlowerlimit = lower_limit_vector_;
    std::array<dReal, 3> newvupperlimit = upper_limit_vector_;
    for(size_t i = 0; i < 3; i++) 
	{
        newvmaxvel[i] *= fjointmult;
        newvmaxaccel[i] *= fjointmult;
        newvlowerlimit[i] *= fjointmult;
        newvupperlimit[i] *= fjointmult;
    }
    openravejson::SetJsonValueByKey(value, "maxVel", newvmaxvel, allocator, dof);
    openravejson::SetJsonValueByKey(value, "hardMaxVel", hard_max_velocity_vector_, allocator, dof);
    openravejson::SetJsonValueByKey(value, "maxAccel", newvmaxaccel, allocator, dof);
    openravejson::SetJsonValueByKey(value, "hardMaxAccel", hard_max_accelerate_vector_, allocator, dof);
    openravejson::SetJsonValueByKey(value, "maxJerk", max_jerk_vector_, allocator, dof);
    openravejson::SetJsonValueByKey(value, "hardMaxJerk", hard_max_jerk_vector_, allocator, dof);
    openravejson::SetJsonValueByKey(value, "maxTorque", max_torque_vector_, allocator, dof);
    openravejson::SetJsonValueByKey(value, "maxInertia", max_inertia_vector_, allocator, dof);
    openravejson::SetJsonValueByKey(value, "weights", weights_vector_, allocator, dof);
    openravejson::SetJsonValueByKey(value, "offsets", offsets_vector_, allocator, dof);
    openravejson::SetJsonValueByKey(value, "lowerLimit", newvlowerlimit, allocator, dof);
    openravejson::SetJsonValueByKey(value, "upperLimit", newvupperlimit, allocator, dof);
    // TODO: openravejson::SetJsonValueByKey(value, allocator, "trajfollow", _trajfollow);

    if (_vmimic.size() > 0) {
        bool bfound = false;
        for (size_t i = 0; i < _vmimic.size() && i < (size_t)dof; ++i) {
            if (!!_vmimic[i]) {
                bfound = true;
                break;
            }
        }
        if (bfound) {
            rapidjson::Value mimics;
            mimics.SetArray();
            for (size_t i = 0; i < _vmimic.size() && i < (size_t)dof; ++i) {
                rapidjson::Value mimicValue;
                _vmimic[i]->SerializeJSON(mimicValue, allocator, unit_scale);
                mimics.PushBack(mimicValue, allocator);
            }
            value.AddMember("mimics", mimics, allocator);
        }
    }

    if(float_parameters_map_.size() > 0)
    {
        openravejson::SetJsonValueByKey(value, "floatParameters", float_parameters_map_, allocator);
    }
    if(int_parameters_map_.size() > 0)
    {
        openravejson::SetJsonValueByKey(value, "intParameters", int_parameters_map_, allocator);
    }
    if(string_parameters_map_.size() > 0)
    {
        openravejson::SetJsonValueByKey(value, "stringParameters", string_parameters_map_, allocator);
    }

    if (!!electric_motor_info_) {
        rapidjson::Value electricMotorInfoValue;
        electricMotorInfoValue.SetObject();
        electric_motor_info_->SerializeJSON(electricMotorInfoValue, allocator, unit_scale, options);
        value.AddMember("electricMotorActuator", electricMotorInfoValue, allocator);
    }

    openravejson::SetJsonValueByKey(value, "isCircular", is_circular_, allocator, dof);
    openravejson::SetJsonValueByKey(value, "isActive", is_active_, allocator);

}

void KinBody::JointInfo::DeserializeJSON(const rapidjson::Value& value, dReal unit_scale)
{
    std::string typestr;
    openravejson::LoadJsonValueByKey(value, "type", typestr);

    if (typestr == "revolute")
    {
        type_ = JointType::JointRevolute;
    }
    else if (typestr == "prismatic")
    {
        type_ = JointType::JointPrismatic;
    }
    else
    {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to deserialize json, unsupported joint type \"%s\"", typestr, ORE_InvalidArguments);
    }

    openravejson::LoadJsonValueByKey(value, "name", name_);
    openravejson::LoadJsonValueByKey(value, "parentLinkName", link_name0_);
    openravejson::LoadJsonValueByKey(value, "anchors", anchor_);
    openravejson::LoadJsonValueByKey(value, "childLinkName", link_name1_);
    openravejson::LoadJsonValueByKey(value, "axes", axes_vector_);
    openravejson::LoadJsonValueByKey(value, "currentValues", current_values_vector_);
    openravejson::LoadJsonValueByKey(value, "resolutions", resolution_vector_);
    openravejson::LoadJsonValueByKey(value, "maxVel", max_velocity_vector_);
    openravejson::LoadJsonValueByKey(value, "hardMaxVel", hard_max_velocity_vector_);
    openravejson::LoadJsonValueByKey(value, "maxAccel", max_accelerate_vector_);
    openravejson::LoadJsonValueByKey(value, "hardMaxAccel", hard_max_accelerate_vector_);
    openravejson::LoadJsonValueByKey(value, "maxJerk", max_jerk_vector_);
    openravejson::LoadJsonValueByKey(value, "hardMaxJerk", hard_max_jerk_vector_);
    openravejson::LoadJsonValueByKey(value, "maxTorque", max_torque_vector_);
    openravejson::LoadJsonValueByKey(value, "maxInertia", max_inertia_vector_);
    openravejson::LoadJsonValueByKey(value, "weights", weights_vector_);
    openravejson::LoadJsonValueByKey(value, "offsets", offsets_vector_);
    openravejson::LoadJsonValueByKey(value, "lowerLimit", lower_limit_vector_);
    openravejson::LoadJsonValueByKey(value, "upperLimit", upper_limit_vector_);
    openravejson::LoadJsonValueByKey(value, "isCircular", is_circular_);
    openravejson::LoadJsonValueByKey(value, "isActive", is_active_);

    // multiply unit_scale on maxVel, maxAccel, lowerLimit, upperLimit
    dReal fjointmult = unit_scale;
    if(type_ == JointRevolute)
    {
        fjointmult = 1;
    }
    else if(type_ == JointPrismatic)
    {
        fjointmult = unit_scale;
    }
    for(size_t ic = 0; ic < axes_vector_.size(); ic++)
    {
        max_velocity_vector_[ic] *= fjointmult;
        max_accelerate_vector_[ic] *= fjointmult;
        lower_limit_vector_[ic] *= fjointmult;
        upper_limit_vector_[ic] *= fjointmult;
    }

    std::array<MimicInfoPtr, 3> newmimic;
    if (value.HasMember("mimics"))
    {
        for (rapidjson::SizeType i = 0; i < value["mimics"].Size(); ++i) {
            MimicInfoPtr mimicinfo(new MimicInfo());
            mimicinfo->DeserializeJSON(value["mimics"][i], unit_scale);
            newmimic[i] = mimicinfo;
        }
    }
    _vmimic = newmimic;

    openravejson::LoadJsonValueByKey(value, "floatParameters", float_parameters_map_);
    openravejson::LoadJsonValueByKey(value, "intParameters", int_parameters_map_);
    openravejson::LoadJsonValueByKey(value, "stringParameters", string_parameters_map_);

    if (value.HasMember("electricMotorActuator")) {
        ElectricMotorActuatorInfoPtr info(new ElectricMotorActuatorInfo());
        info->DeserializeJSON(value["electricMotorActuator"], unit_scale);
        electric_motor_info_ = info;
    }
}

KinBody::JointInfo& KinBody::JointInfo::operator=(const KinBody::JointInfo& other)
{
    type_ = other.type_;
    name_ = other.name_;
    link_name0_ = other.link_name0_;
    link_name1_ = other.link_name1_;
    anchor_ = other.anchor_;
    axes_vector_ = other.axes_vector_;
    current_values_vector_ = other.current_values_vector_;
    resolution_vector_ = other.resolution_vector_;
    max_velocity_vector_ = other.max_velocity_vector_;
    hard_max_velocity_vector_ = other.hard_max_velocity_vector_;
    max_accelerate_vector_ = other.max_accelerate_vector_;
    hard_max_accelerate_vector_ = other.hard_max_accelerate_vector_;
    max_jerk_vector_ = other.max_jerk_vector_;
    hard_max_jerk_vector_ = other.hard_max_jerk_vector_;
    max_torque_vector_ = other.max_torque_vector_;
    max_inertia_vector_ = other.max_inertia_vector_;
    weights_vector_ = other.weights_vector_;
    offsets_vector_ = other.offsets_vector_;
    lower_limit_vector_ = other.lower_limit_vector_;
    upper_limit_vector_ = other.upper_limit_vector_;

    if( !other._trajfollow ) {
        _trajfollow.reset();
    }
    else {
        _trajfollow = RaveClone<TrajectoryBase>(other._trajfollow, Clone_All);
    }

    for( size_t i = 0; i < _vmimic.size(); ++i ) {
        if( !other._vmimic[i] ) {
            _vmimic[i].reset();
        }
        else {
            _vmimic[i].reset(new MimicInfo(*(other._vmimic[i])));
        }
    }

    float_parameters_map_ = other.float_parameters_map_;
    int_parameters_map_ = other.int_parameters_map_;
    string_parameters_map_ = other.string_parameters_map_;

    if( !other.electric_motor_info_ ) {
        electric_motor_info_.reset();
    }
    else {
        electric_motor_info_.reset(new ElectricMotorActuatorInfo(*other.electric_motor_info_));
    }

    is_circular_ = other.is_circular_;
    is_active_ = other.is_active_;

    control_mode_ = other.control_mode_;
    if( control_mode_ == KinBody::JCM_RobotController ) {
        if( !other._jci_robotcontroller ) {
            _jci_robotcontroller.reset();
        }
        else {
            _jci_robotcontroller.reset(new JointControlInfo_RobotController(*other._jci_robotcontroller));
        }
    }
    else if( control_mode_ == KinBody::JCM_IO ) {
        if( !other._jci_io ) {
            _jci_io.reset();
        }
        else {
            _jci_io.reset(new JointControlInfo_IO(*other._jci_io));
        }
    }
    else if( control_mode_ == KinBody::JCM_ExternalDevice ) {
        if( !other._jci_externaldevice ) {
            _jci_externaldevice.reset();
        }
        else {
            _jci_externaldevice.reset(new JointControlInfo_ExternalDevice(*other._jci_externaldevice));
        }
    }
    return *this;
}




static void fparser_polyroots2(std::vector<dReal>& rawroots, const std::vector<dReal>& rawcoeffs)
{
    BOOST_ASSERT(rawcoeffs.size()==3);
    int numroots=0;
    rawroots.resize(2);
    polyroots2<dReal>(&rawcoeffs[0],&rawroots[0],numroots);
    rawroots.resize(numroots);
}

template <int D>
static void fparser_polyroots(std::vector<dReal>& rawroots, const std::vector<dReal>& rawcoeffs)
{
    BOOST_ASSERT(rawcoeffs.size()==D+1);
    int numroots=0;
    rawroots.resize(D);
    polyroots<dReal,D>(&rawcoeffs[0],&rawroots[0],numroots);
    rawroots.resize(numroots);
}

// take triangle 3 sides and compute the angle opposite the first side
static void fparser_sssa(std::vector<dReal>& res, const vector<dReal>& coeffs)
{
    dReal a = coeffs.at(0), b = coeffs.at(1), c = coeffs.at(2);
    dReal f = (a*a+b*b-c*c)/(2*b);
    res.resize(1);
    res[0] = RaveAtan2(RaveSqrt(a*a-f*f),b-f);
}

/// take triangle 2 sides and an angle and compute the missing angle
static void fparser_sasa(std::vector<dReal>& res, const vector<dReal>& coeffs)
{
    dReal a = coeffs[0], gamma = coeffs[1], b = coeffs[2];
    res.resize(1);
    res[0] = RaveAtan2(a*RaveSin(gamma),b-a*RaveCos(gamma));
}

/// take triangle 2 sides and an angle and compute the missing side
static void fparser_sass(std::vector<dReal>& res, const vector<dReal>& coeffs)
{
    dReal a = coeffs[0], gamma = coeffs[1], b = coeffs[2];
    res.resize(1);
    res[0] = RaveSqrt(a*a+b*b-2*a*b*RaveCos(gamma));
}

OpenRAVEFunctionParserRealPtr CreateJointFunctionParser()
{
    OpenRAVEFunctionParserRealPtr parser(new OpenRAVEFunctionParserReal());
#ifdef OPENRAVE_FPARSER_SETEPSILON
    parser->setEpsilon(g_fEpsilonLinear);
#endif
    // register commonly used functions
    parser->AddBoostFunction("polyroots2",fparser_polyroots2,3);
    parser->AddBoostFunction("polyroots3",fparser_polyroots<3>,4);
    parser->AddBoostFunction("polyroots4",fparser_polyroots<4>,5);
    parser->AddBoostFunction("polyroots5",fparser_polyroots<5>,6);
    parser->AddBoostFunction("polyroots6",fparser_polyroots<6>,7);
    parser->AddBoostFunction("polyroots7",fparser_polyroots<7>,8);
    parser->AddBoostFunction("polyroots8",fparser_polyroots<8>,9);
    parser->AddBoostFunction("SSSA",fparser_sssa,3);
    parser->AddBoostFunction("SASA",fparser_sasa,3);
    parser->AddBoostFunction("SASS",fparser_sass,3);
    return parser;
}

KinBody::Joint::Joint(KinBodyPtr parent, KinBody::JointType type)
{
    _parent = parent;
    FOREACH(it,_doflastsetvalues)
	{
        *it = 0;
    }
    for(size_t i = 0; i < _vaxes.size(); ++i) {
        _vaxes[i] = Vector(0,0,1);
    }
    joint_index_=-1;
    dof_index_ = -1; // invalid index
    is_initialized_ = false;
    info_.type_ = type;
    info_.control_mode_ = JCM_None;
}

KinBody::Joint::~Joint()
{
}

int KinBody::Joint::GetDOF() const
{
    return info_.GetDOF();
}

bool KinBody::Joint::IsCircular() const
{
    return info_.is_circular_[0] || info_.is_circular_[1] || info_.is_circular_[2];
}

bool KinBody::Joint::IsCircular(int iaxis) const
{
    return static_cast<bool>(info_.is_circular_.at(iaxis));
}

bool KinBody::Joint::IsRevolute(int iaxis) const
{
    if( info_.type_ & KinBody::JointSpecialBit ) 
	{
        return info_.type_ == KinBody::JointHinge2 || info_.type_ == KinBody::JointUniversal;
    }
    return !(info_.type_&(1<<(4+iaxis)));
}

bool KinBody::Joint::IsPrismatic(int iaxis) const
{
    if( info_.type_ & KinBody::JointSpecialBit ) 
	{
        return false;
    }
    return !!(info_.type_&(1<<(4+iaxis)));
}

bool KinBody::Joint::IsStatic() const
{
    if( IsMimic() ) 
	{
        bool is_static = true;
        KinBodyConstPtr parent(_parent);
        for(int i = 0; i < GetDOF(); ++i) {
            if( !!mimic_array_.at(i) ) {
                FOREACHC(it, mimic_array_.at(i)->mimic_dofs_vector_) {
                    if( !parent->GetJointFromDOFIndex(it->dofindex)->IsStatic() ) {
                        is_static = false;
                        break;
                    }
                }
                if( !is_static ) {
                    break;
                }
            }
        }
        if( is_static ) {
            return true;
        }
    }
    for(int i = 0; i < GetDOF(); ++i) {
        if( IsCircular(i) ) {
            return false;
        }
        if( info_.lower_limit_vector_.at(i) < info_.upper_limit_vector_.at(i) ) {
            return false;
        }
    }
    return true;
}

void KinBody::Joint::GetValues(vector<dReal>& pValues, bool is_append) const
{
    OPENRAVE_ASSERT_FORMAT0(is_initialized_, "joint not initialized",ORE_NotInitialized);
    if( !is_append ) {
        pValues.resize(0);
    }
    if( GetDOF() == 1 ) {
        pValues.push_back(GetValue(0));
        return;
    }
    dReal f;
    Transform tjoint = _tinvLeft * attached_bodies_array_[0]->GetTransform().inverse() * attached_bodies_array_[1]->GetTransform() * _tinvRight;
    if( info_.type_ & KinBody::JointSpecialBit ) {
        switch(info_.type_) {
        case KinBody::JointHinge2: {
            Vector axis1cur = tjoint.rotate(_vaxes[0]), axis2cur = tjoint.rotate(_vaxes[1]);
            Vector vec1, vec2, vec3;
            vec1 = (_vaxes[1] - _vaxes[0].dot3(_vaxes[1])*_vaxes[0]).normalize();
            vec2 = (axis2cur - _vaxes[0].dot3(axis2cur)*_vaxes[0]).normalize();
            vec3 = _vaxes[0].cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot3(vec2), vec1.dot3(vec2));
            pValues.push_back(GetClosestValueAlongCircle(info_.offsets_vector_[0]+f, _doflastsetvalues[0]));
            vec1 = (_vaxes[0] - axis2cur.dot(_vaxes[0])*axis2cur).normalize();
            vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
            vec3 = axis2cur.cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
            if( f < -PI ) {
                f += 2*PI;
            }
            else if( f > PI ) {
                f -= 2*PI;
            }
            pValues.push_back(GetClosestValueAlongCircle(info_.offsets_vector_[1]+f, _doflastsetvalues[1]));
            break;
        }
        case KinBody::JointSpherical: {
            dReal fsinang2 = tjoint.rot.y*tjoint.rot.y+tjoint.rot.z*tjoint.rot.z+tjoint.rot.w*tjoint.rot.w;
            if( fsinang2 > 1e-10f ) {
                dReal fsinang = RaveSqrt(fsinang2);
                dReal fmult = 2*RaveAtan2(fsinang,tjoint.rot.x)/fsinang;
                pValues.push_back(tjoint.rot.y*fmult);
                pValues.push_back(tjoint.rot.z*fmult);
                pValues.push_back(tjoint.rot.w*fmult);
            }
            else {
                pValues.push_back(0);
                pValues.push_back(0);
                pValues.push_back(0);
            }
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("unknown joint type 0x%x"), info_.type_, ORE_Failed);
        }
    }
    else {
        // chain of revolute and prismatic joints
        for(int i = 0; i < GetDOF(); ++i) {
            Vector vaxis = _vaxes.at(i);
            if( IsRevolute(i) ) {
                if( i+1 < GetDOF() ) {
                    std::pair<dReal, Vector > res = normalizeAxisRotation(vaxis,tjoint.rot);
                    tjoint.rot = res.second;
                    if( res.first != 0 ) {
                        // could speed up by checking if trans is ever needed after this
                        tjoint.trans = quatRotate(quatFromAxisAngle(vaxis,res.first),tjoint.trans);
                    }
                    f = -res.first;
                }
                else {
                    f = 2.0f*RaveAtan2(tjoint.rot.y*vaxis.x+tjoint.rot.z*vaxis.y+tjoint.rot.w*vaxis.z, tjoint.rot.x);
                }
                // expect values to be within -PI to PI range
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                pValues.push_back(GetClosestValueAlongCircle(info_.offsets_vector_[i]+f, _doflastsetvalues[i]));
            }
            else { // prismatic
                f = tjoint.trans.x*vaxis.x+tjoint.trans.y*vaxis.y+tjoint.trans.z*vaxis.z;
                pValues.push_back(info_.offsets_vector_[i]+f);
                if( i+1 < GetDOF() ) {
                    tjoint.trans -= vaxis*f;
                }
            }
        }
    }
}

dReal KinBody::Joint::GetValue(int iaxis) const
{
    OPENRAVE_ASSERT_FORMAT0(is_initialized_, "joint not initialized",ORE_NotInitialized);
    dReal f;
    Transform tjoint = _tinvLeft * attached_bodies_array_[0]->GetTransform().inverse() * attached_bodies_array_[1]->GetTransform() * _tinvRight;
    if( info_.type_ & KinBody::JointSpecialBit ) {
        switch(info_.type_) {
        case KinBody::JointHinge2: {
            Vector axis1cur = tjoint.rotate(_vaxes[0]), axis2cur = tjoint.rotate(_vaxes[1]);
            Vector vec1, vec2, vec3;
            if( iaxis == 0 ) {
                vec1 = (_vaxes[1] - _vaxes[0].dot3(_vaxes[1])*_vaxes[0]).normalize();
                vec2 = (axis2cur - _vaxes[0].dot3(axis2cur)*_vaxes[0]).normalize();
                vec3 = _vaxes[0].cross(vec1);
                f = 2.0*RaveAtan2(vec3.dot3(vec2), vec1.dot3(vec2));
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                return GetClosestValueAlongCircle(info_.offsets_vector_[0]+f, _doflastsetvalues[0]);
            }
            else if( iaxis == 1 ) {
                vec1 = (_vaxes[0] - axis2cur.dot(_vaxes[0])*axis2cur).normalize();
                vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
                vec3 = axis2cur.cross(vec1);
                f = 2.0*RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                return GetClosestValueAlongCircle(info_.offsets_vector_[1]+f, _doflastsetvalues[1]);
            }
            break;
        }
        case KinBody::JointSpherical: {
            dReal fsinang2 = tjoint.rot.y*tjoint.rot.y+tjoint.rot.z*tjoint.rot.z+tjoint.rot.w*tjoint.rot.w;
            if( fsinang2 > 1e-10f ) {
                dReal fsinang = RaveSqrt(fsinang2);
                dReal fmult = 2*RaveAtan2(fsinang,tjoint.rot.x)/fsinang;
                if( iaxis == 0 ) {
                    return tjoint.rot.y*fmult;
                }
                else if( iaxis == 1 ) {
                    return tjoint.rot.z*fmult;
                }
                else if( iaxis == 2 ) {
                    return tjoint.rot.w*fmult;
                }
            }
            else {
                if((iaxis >= 0)&&(iaxis < 3)) {
                    return 0;
                }
            }
            break;
        }
        case KinBody::JointTrajectory: {
            //uint64_t starttime = utils::GetMicroTime();
            vector<dReal> vsampledata;
            dReal splitpercentage = 0.01;
            dReal precision(1e-6);
            dReal timemin = 0, timemax = info_._trajfollow->GetDuration();
            Transform tbest, ttest;
            int totalcalls = 0;
            while(timemin+precision < timemax) {
                dReal timestep = (timemax-timemin)*splitpercentage;
                dReal timeclosest = timemin;
                dReal bestdist = 1e30, besttime=0;
                for(; timeclosest < timemax; timeclosest += timestep ) {
                    if( timeclosest > timemax ) {
                        timeclosest = timemax;
                    }
                    totalcalls += 1;
                    info_._trajfollow->Sample(vsampledata,timeclosest);
                    if( info_._trajfollow->GetConfigurationSpecification().ExtractTransform(ttest,vsampledata.begin(),KinBodyConstPtr()) ) {
                        dReal fdist = TransformDistanceFast(ttest,tjoint,0.3);
                        if( bestdist > fdist ) {
                            besttime = timeclosest;
                            bestdist = fdist;
                            tbest = ttest;
                        }
                    }
                }
                OPENRAVE_ASSERT_OP_FORMAT(bestdist, <, 1e30, "failed to compute trajectory value for joint %s\n",GetName(),ORE_Assert);
                timemin = max(timemin,besttime-timestep);
                timemax = min(timemax, besttime+timestep);
                splitpercentage = 0.1f;
                //RAVELOG_INFO(str(boost::format("calls: %d time: %f")%totalcalls%((utils::GetMicroTime()-starttime)*1e-6)));
            }
            return 0.5*(timemin+timemax);
        }
        default:
            break;
        }
    }
    else {
        if( info_.type_ == KinBody::JointPrismatic ) {
            return info_.offsets_vector_[0]+(tjoint.trans.x*_vaxes[0].x+tjoint.trans.y*_vaxes[0].y+tjoint.trans.z*_vaxes[0].z);
        }
        else if( info_.type_ == KinBody::JointRevolute ) {
            f = 2.0f*RaveAtan2(tjoint.rot.y*_vaxes[0].x+tjoint.rot.z*_vaxes[0].y+tjoint.rot.w*_vaxes[0].z, tjoint.rot.x);
            // expect values to be within -PI to PI range
            if( f < -PI ) {
                f += 2*PI;
            }
            else if( f > PI ) {
                f -= 2*PI;
            }
            return GetClosestValueAlongCircle(info_.offsets_vector_[0]+f, _doflastsetvalues[0]);
        }

        // chain of revolute and prismatic joints
        for(int i = 0; i < GetDOF(); ++i) {
            Vector vaxis = _vaxes.at(i);
            if( IsRevolute(i) ) {
                if( i+1 < GetDOF() ) {
                    std::pair<dReal, Vector > res = normalizeAxisRotation(vaxis,tjoint.rot);
                    tjoint.rot = res.second;
                    if( res.first != 0 ) {
                        // could speed up by checking if trans is ever needed after this
                        tjoint.trans = quatRotate(quatFromAxisAngle(vaxis,res.first),tjoint.trans);
                    }
                    f = -res.first;
                }
                else {
                    f = 2.0f*RaveAtan2(tjoint.rot.y*vaxis.x+tjoint.rot.z*vaxis.y+tjoint.rot.w*vaxis.z, tjoint.rot.x);
                }
                // expect values to be within -PI to PI range
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                if( i == iaxis ) {
                    return GetClosestValueAlongCircle(info_.offsets_vector_[i]+f, _doflastsetvalues[i]);
                }
            }
            else { // prismatic
                f = tjoint.trans.x*vaxis.x+tjoint.trans.y*vaxis.y+tjoint.trans.z*vaxis.z;
                if( i == iaxis ) {
                    return info_.offsets_vector_[i]+f;
                }
                if( i+1 < GetDOF() ) {
                    tjoint.trans -= vaxis*f;
                }
            }
        }
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_tr("unknown joint type 0x%x axis %d\n"), info_.type_%iaxis, ORE_Failed);
}

void KinBody::Joint::GetVelocities(std::vector<dReal>& pVelocities, bool is_append) const
{
    OPENRAVE_ASSERT_FORMAT0(is_initialized_, "joint not initialized",ORE_NotInitialized);
    if( !is_append ) {
        pVelocities.resize(0);
    }
    if( GetDOF() == 1 ) {
        pVelocities.push_back(GetVelocity(0));
        return;
    }
    _GetVelocities(pVelocities,is_append,attached_bodies_array_[0]->GetVelocity(), attached_bodies_array_[1]->GetVelocity());
};

dReal KinBody::Joint::GetVelocity(int axis) const
{
    OPENRAVE_ASSERT_FORMAT0(is_initialized_, "joint not initialized",ORE_NotInitialized);
    return _GetVelocity(axis,attached_bodies_array_[0]->GetVelocity(), attached_bodies_array_[1]->GetVelocity());
}

void KinBody::Joint::_GetVelocities(std::vector<dReal>& pVelocities, bool is_append, const std::pair<Vector,Vector>& linkparentvelocity, const std::pair<Vector,Vector>& linkchildvelocity) const
{
    if( !is_append ) {
        pVelocities.resize(0);
    }
    if( GetDOF() == 1 ) {
        pVelocities.push_back(GetVelocity(0));
        return;
    }
    const Transform& linkparenttransform = attached_bodies_array_[0]->info_.transform_;
    const Transform& linkchildtransform = attached_bodies_array_[1]->info_.transform_;
    Vector quatdelta = quatMultiply(linkparenttransform.rot,_tLeft.rot);
    Vector quatdeltainv = quatInverse(quatdelta);
    if( info_.type_ & KinBody::JointSpecialBit ) {
        switch(info_.type_) {
        case KinBody::JointSpherical: {
            Vector v = quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second);
            pVelocities.push_back(v.x);
            pVelocities.push_back(v.y);
            pVelocities.push_back(v.z);
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("unknown joint type 0x%x"), info_.type_, ORE_InvalidArguments);
        }
    }
    else {
        // chain of revolute and prismatic joints
        Vector angvelocitycovered, linvelocitycovered;
        for(int i = 0; i < GetDOF(); ++i) {
            if( IsRevolute(i) ) {
                pVelocities.push_back(_vaxes[i].dot3(quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second-angvelocitycovered)));
                angvelocitycovered += quatRotate(quatdelta,_vaxes[i]*pVelocities.back());
            }
            else { // prismatic
                pVelocities.push_back(_vaxes[i].dot3(quatRotate(quatdeltainv,linkchildvelocity.first-linkparentvelocity.first-(linkparentvelocity.second-angvelocitycovered).cross(linkchildtransform.trans-linkparenttransform.trans)-linvelocitycovered)));
                linvelocitycovered += quatRotate(quatdelta,_vaxes[i]*pVelocities.back());
            }
        }
    }
}

dReal KinBody::Joint::_GetVelocity(int axis, const std::pair<Vector,Vector>&linkparentvelocity, const std::pair<Vector,Vector>&linkchildvelocity) const
{
    const Transform& linkparenttransform = attached_bodies_array_[0]->info_.transform_;
    const Transform& linkchildtransform = attached_bodies_array_[1]->info_.transform_;
    Vector quatdelta = quatMultiply(linkparenttransform.rot,_tLeft.rot);
    Vector quatdeltainv = quatInverse(quatdelta);
    if( info_.type_ & KinBody::JointSpecialBit ) {
        switch(info_.type_) {
        case KinBody::JointSpherical: {
            Vector v = quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second);
            return v[axis];
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("unknown joint type 0x%x"), info_.type_, ORE_InvalidArguments);
        }
    }
    else {
        if( info_.type_ == KinBody::JointPrismatic ) {
            return _vaxes[0].dot3(quatRotate(quatdeltainv,linkchildvelocity.first-linkparentvelocity.first-linkparentvelocity.second.cross(linkchildtransform.trans-linkparenttransform.trans)));
        }
        else if( info_.type_ == KinBody::JointRevolute ) {
            return _vaxes[0].dot3(quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second));
        }
        else {
            // chain of revolute and prismatic joints
            Vector angvelocitycovered, linvelocitycovered;
            for(int i = 0; i < GetDOF(); ++i) {
                if( IsRevolute(i) ) {
                    dReal fvelocity = _vaxes[i].dot3(quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second-angvelocitycovered));
                    if( i == axis ) {
                        return fvelocity;
                    }
                    angvelocitycovered += quatRotate(quatdelta,_vaxes[i]*fvelocity);
                }
                else { // prismatic
                    dReal fvelocity = _vaxes[i].dot3(quatRotate(quatdeltainv,linkchildvelocity.first-linkparentvelocity.first-(linkparentvelocity.second-angvelocitycovered).cross(linkparenttransform.trans-linkchildtransform.trans)-linvelocitycovered));
                    if( i == axis ) {
                        return fvelocity;
                    }
                    linvelocitycovered += quatRotate(quatdelta,_vaxes[i]*fvelocity);
                }
            }
        }
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_tr("unsupported joint type 0x%x"), info_.type_, ORE_InvalidArguments);
}

Vector KinBody::Joint::GetAnchor() const
{
    return attached_bodies_array_[0]->GetTransform() * _tLeft.trans;
}

Vector KinBody::Joint::GetAxis(int iaxis) const
{
    return attached_bodies_array_[0]->GetTransform().rotate(_tLeft.rotate(_vaxes.at(iaxis)));
}

void KinBody::Joint::_ComputeInternalInformation(LinkPtr plink0, LinkPtr plink1, const Vector& vanchorraw, const std::vector<Vector>& vaxes, const std::vector<dReal>& vcurrentvalues)
{
    OPENRAVE_ASSERT_OP_FORMAT(!!plink0,&&,!!plink1, "one or more attached attached_bodies_array_ are invalid for joint %s", GetName(),ORE_InvalidArguments);
    for(int i = 0; i < GetDOF(); ++i) {
        OPENRAVE_ASSERT_OP_FORMAT(info_.max_velocity_vector_[i], >=, 0, "joint %s[%d] max velocity is invalid",info_.name_%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(info_.max_accelerate_vector_[i], >=, 0, "joint %s[%d] max acceleration is invalid",info_.name_%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(info_.max_jerk_vector_[i], >=, 0, "joint %s[%d] max jerk is invalid",info_.name_%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(info_.max_torque_vector_[i], >=, 0, "joint %s[%d] max torque is invalid",info_.name_%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(info_.max_inertia_vector_[i], >=, 0, "joint %s[%d] max inertia is invalid",info_.name_%i, ORE_InvalidArguments);
    }

    KinBodyPtr parent(_parent);
    is_initialized_ = false;
    attached_bodies_array_[0] = plink0;
    attached_bodies_array_[1] = plink1;
    Transform trel, tbody0, tbody1;
    Vector vanchor=vanchorraw;
    for(size_t i = 0; i < vaxes.size(); ++i) {
        _vaxes[i] = vaxes[i];
    }
    // make sure first body is always closer to the root, unless the second body is static and the first body is not the root link
    if( attached_bodies_array_[1]->IsStatic() && attached_bodies_array_[0]->GetIndex() > 0) {
        if( !attached_bodies_array_[0]->IsStatic() ) {
            Transform tswap = plink1->GetTransform().inverse() * plink0->GetTransform();
            for(int i = 0; i < GetDOF(); ++i) {
                _vaxes[i] = -tswap.rotate(_vaxes[i]);
            }
            vanchor = tswap*vanchor;
            swap(attached_bodies_array_[0],attached_bodies_array_[1]);
        }
    }

    // update info_
    for(size_t i = 0; i < vaxes.size(); ++i) {
        info_.axes_vector_[i] = _vaxes[i];
    }
    info_.anchor_ = vanchor;

    tbody0 = attached_bodies_array_[0]->GetTransform();
    tbody1 = attached_bodies_array_[1]->GetTransform();
    trel = tbody0.inverse() * tbody1;
    _tLeft = Transform();
    _tLeftNoOffset = Transform();
    _tRight = Transform();
    _tRightNoOffset = Transform();

    if( info_.type_ & KinBody::JointSpecialBit ) {
        switch(info_.type_) {
        case KinBody::JointUniversal:
            _tLeft.trans = vanchor;
            _tRight.trans = -vanchor;
            _tRight = _tRight * trel;
            OPENRAVE_ASSERT_OP((int)vaxes.size(),==,2);
            break;
        case KinBody::JointHinge2:
            _tLeft.trans = vanchor;
            _tRight.trans = -vanchor;
            _tRight = _tRight * trel;
            OPENRAVE_ASSERT_OP((int)vaxes.size(),==,2);
            break;
        case KinBody::JointSpherical:
            _tLeft.trans = vanchor;
            _tRight.trans = -vanchor;
            _tRight = _tRight * trel;
            break;
        case KinBody::JointTrajectory:
            if( !info_._trajfollow ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_tr("trajectory joint requires Joint::_trajfollow to be initialized"),ORE_InvalidState);
            }
            _tRight = _tRight * trel;
            break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("unrecognized joint type 0x%x"), info_.type_, ORE_InvalidArguments);
        }
        _tLeftNoOffset = _tLeft;
        _tRightNoOffset = _tRight;
    }
    else {
        OPENRAVE_ASSERT_OP((int)vaxes.size(),==,GetDOF());
        _tLeftNoOffset.trans = vanchor;
        _tRightNoOffset.trans = -vanchor;
        _tRightNoOffset = _tRightNoOffset * trel;
        if( GetDOF() == 1 ) {
            // in the case of one axis, create a new coordinate system such that the axis rotates about (0,0,1)
            // this is necessary in order to simplify the rotation matrices (for future symbolic computation)
            // and suppress any floating-point error. The data structures are only setup for this to work in 1 DOF.
            Transform trot; trot.rot = quatRotateDirection(_vaxes[0],Vector(0,0,1));
            _tLeftNoOffset = _tLeftNoOffset * trot.inverse();
            _tRightNoOffset = trot*_tRightNoOffset;
            _vaxes[0] = Vector(0,0,1);
        }

        Transform toffset;
        if( IsRevolute(0) ) {
            toffset.rot = quatFromAxisAngle(_vaxes[0], info_.offsets_vector_[0]);
        }
        else {
            toffset.trans = _vaxes[0]*info_.offsets_vector_[0];
        }
        _tLeft = _tLeftNoOffset * toffset;
        _tRight = _tRightNoOffset;
        if( GetDOF() > 1 ) {
            // right multiply by the offset of the last axis, might be buggy?
            if( IsRevolute(GetDOF()-1) ) {
                _tRight = matrixFromAxisAngle(_vaxes[GetDOF()-1], info_.offsets_vector_[GetDOF()-1]) * _tRight;
            }
            else {
                _tRight.trans += _vaxes[GetDOF()-1]*info_.offsets_vector_[GetDOF()-1];
            }
        }
    }

    if( vcurrentvalues.size() > 0 ) {
        // see if any joints have offsets
        Transform toffset;
        if( info_.type_ == KinBody::JointTrajectory ) {
            vector<dReal> vsampledata;
            Transform t0, t1;
            info_._trajfollow->Sample(vsampledata,0);
            if( !info_._trajfollow->GetConfigurationSpecification().ExtractTransform(t0,vsampledata.begin(),KinBodyConstPtr()) ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to sample trajectory for joint %s"),GetName(),ORE_Assert);
            }
            info_._trajfollow->Sample(vsampledata,vcurrentvalues.at(0));
            if( !info_._trajfollow->GetConfigurationSpecification().ExtractTransform(t1,vsampledata.begin(),KinBodyConstPtr()) ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to sample trajectory for joint %s"),GetName(),ORE_Assert);
            }
            toffset = t0*t1.inverse();
        }
        else if( !(info_.type_&KinBody::JointSpecialBit) || info_.type_ == KinBody::JointUniversal || info_.type_ == KinBody::JointHinge2 ) {
            if( IsRevolute(0) ) {
                toffset.rot = quatFromAxisAngle(_vaxes[0], -vcurrentvalues[0]);
            }
            else {
                toffset.trans = -_vaxes[0]*vcurrentvalues[0];
            }
        }
        _tLeftNoOffset *= toffset;
        _tLeft *= toffset;
        if( vcurrentvalues.size() > 1 ) {
            if( IsRevolute(GetDOF()-1) ) {
                toffset.rot = quatFromAxisAngle(_vaxes[GetDOF()-1], -vcurrentvalues.at(GetDOF()-1));
            }
            else {
                toffset.trans = -_vaxes[GetDOF()-1]*vcurrentvalues.at(GetDOF()-1);
            }
            _tRightNoOffset = toffset * _tRightNoOffset;
            _tRight = toffset * _tRight;
        }
    }
    _tinvRight = _tRight.inverse();
    _tinvLeft = _tLeft.inverse();

    circular_lower_limit_ = info_.lower_limit_vector_;
    circular_upper_limit_ = info_.upper_limit_vector_;
    for(int i = 0; i < GetDOF(); ++i) {
        if( IsCircular(i) ) {
            // can rotate forever, so don't limit it. Unfortunately if numbers are too big precision will start getting lost
            info_.lower_limit_vector_.at(i) = -1e4;
            info_.upper_limit_vector_.at(i) = 1e4;
        }
    }

    if( !!attached_bodies_array_[0] ) {
        info_.link_name0_ = attached_bodies_array_[0]->GetName();
    }
    else {
        info_.link_name0_.clear();
    }
    if( !!attached_bodies_array_[1] ) {
        info_.link_name1_ = attached_bodies_array_[1]->GetName();
    }
    else {
        info_.link_name1_.clear();
    }
    info_.current_values_vector_ = vcurrentvalues;

    is_initialized_ = true;

    if( attached_bodies_array_[1]->IsStatic() && !IsStatic() ) {
        RAVELOG_WARN(str(boost::format("joint %s: all attached links are static, but joint is not!\n")%GetName()));
    }
}

KinBody::LinkPtr KinBody::Joint::GetHierarchyParentLink() const
{
    return attached_bodies_array_[0];
}

KinBody::LinkPtr KinBody::Joint::GetHierarchyChildLink() const
{
    return attached_bodies_array_[1];
}

const Vector& KinBody::Joint::GetInternalHierarchyAxis(int iaxis) const
{
    return _vaxes.at(iaxis);
}

const Transform& KinBody::Joint::GetInternalHierarchyLeftTransform() const
{
    OPENRAVE_ASSERT_FORMAT0(is_initialized_, "joint not initialized",ORE_NotInitialized);
    return _tLeftNoOffset;
}

const Transform& KinBody::Joint::GetInternalHierarchyRightTransform() const
{
    OPENRAVE_ASSERT_FORMAT0(is_initialized_, "joint not initialized",ORE_NotInitialized);
    return _tRightNoOffset;
}

void KinBody::Joint::GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, bool is_append) const
{
    if( !is_append ) {
        vLowerLimit.resize(0);
        vUpperLimit.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vLowerLimit.push_back(info_.lower_limit_vector_[i]);
        vUpperLimit.push_back(info_.upper_limit_vector_[i]);
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetLimit(int iaxis) const
{
    return make_pair(info_.lower_limit_vector_.at(iaxis),info_.upper_limit_vector_.at(iaxis));
}

void KinBody::Joint::SetLimits(const std::vector<dReal>& vLowerLimit, const std::vector<dReal>& vUpperLimit)
{
    bool bChanged = false;
    for(int i = 0; i < GetDOF(); ++i) {
        if( info_.lower_limit_vector_[i] != vLowerLimit.at(i) || info_.upper_limit_vector_[i] != vUpperLimit.at(i) ) {
            bChanged = true;
            info_.lower_limit_vector_[i] = vLowerLimit.at(i);
            info_.upper_limit_vector_[i] = vUpperLimit.at(i);
            if( IsRevolute(i) && !IsCircular(i) ) {
                // TODO, necessary to set wrap?
                if( info_.lower_limit_vector_[i] < -PI || info_.upper_limit_vector_[i] > PI) {
                    SetWrapOffset(0.5f * (info_.lower_limit_vector_.at(i) + info_.upper_limit_vector_.at(i)),i);
                }
                else {
                    SetWrapOffset(0,i);
                }
            }
        }
    }
    if( bChanged ) {
        GetParent()->_PostprocessChangedParameters(Prop_JointLimits);
    }
}

void KinBody::Joint::GetVelocityLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper, bool is_append) const
{
    if( !is_append ) {
        vlower.resize(0);
        vupper.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vlower.push_back(-info_.max_velocity_vector_[i]);
        vupper.push_back(info_.max_velocity_vector_[i]);
    }
}

void KinBody::Joint::GetVelocityLimits(std::vector<dReal>& vmax, bool is_append) const
{
    if( !is_append ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(info_.max_velocity_vector_[i]);
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetVelocityLimit(int iaxis) const
{
    return make_pair(-info_.max_velocity_vector_.at(iaxis), info_.max_velocity_vector_.at(iaxis));
}

void KinBody::Joint::SetVelocityLimits(const std::vector<dReal>& vmaxvel)
{
    for(int i = 0; i < GetDOF(); ++i) {
        info_.max_velocity_vector_[i] = vmaxvel.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetAccelerationLimits(std::vector<dReal>& vmax, bool is_append) const
{
    if( !is_append ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(info_.max_accelerate_vector_[i]);
    }
}

dReal KinBody::Joint::GetAccelerationLimit(int iaxis) const
{
    return info_.max_accelerate_vector_.at(iaxis);
}

void KinBody::Joint::SetAccelerationLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        info_.max_accelerate_vector_[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetJerkLimits(std::vector<dReal>& vmax, bool is_append) const
{
    if( !is_append ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(info_.max_jerk_vector_[i]);
    }
}

dReal KinBody::Joint::GetJerkLimit(int iaxis) const
{
    return info_.max_jerk_vector_.at(iaxis);
}

void KinBody::Joint::SetJerkLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        info_.max_jerk_vector_[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetHardVelocityLimits(std::vector<dReal>& vmax, bool is_append) const
{
    if( !is_append ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(info_.hard_max_velocity_vector_[i]);
    }
}

dReal KinBody::Joint::GetHardVelocityLimit(int iaxis) const
{
    return info_.hard_max_velocity_vector_.at(iaxis);
}

void KinBody::Joint::SetHardVelocityLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        info_.hard_max_velocity_vector_[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetHardAccelerationLimits(std::vector<dReal>& vmax, bool is_append) const
{
    if( !is_append ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(info_.hard_max_accelerate_vector_[i]);
    }
}

dReal KinBody::Joint::GetHardAccelerationLimit(int iaxis) const
{
    return info_.hard_max_accelerate_vector_.at(iaxis);
}

void KinBody::Joint::SetHardAccelerationLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        info_.hard_max_accelerate_vector_[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetHardJerkLimits(std::vector<dReal>& vmax, bool is_append) const
{
    if( !is_append ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(info_.hard_max_jerk_vector_[i]);
    }
}

dReal KinBody::Joint::GetHardJerkLimit(int iaxis) const
{
    return info_.hard_max_jerk_vector_.at(iaxis);
}

void KinBody::Joint::SetHardJerkLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        info_.hard_max_jerk_vector_[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetTorqueLimits(std::vector<dReal>& vmax, bool is_append) const
{
    if( !is_append ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(info_.max_torque_vector_[i]);
    }
}

void KinBody::Joint::SetTorqueLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        info_.max_torque_vector_[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetInertiaLimits(std::vector<dReal>& vmax, bool is_append) const
{
    if( !is_append ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(info_.max_inertia_vector_[i]);
    }
}

void KinBody::Joint::SetInertiaLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        info_.max_inertia_vector_[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::SetWrapOffset(dReal newoffset, int iaxis)
{
    if( info_.offsets_vector_.at(iaxis) != newoffset ) {
        info_.offsets_vector_.at(iaxis) = newoffset;
        if( iaxis == 0 ) {
            Transform toffset;
            if( IsRevolute(0) ) {
                toffset.rot = quatFromAxisAngle(_vaxes[0], newoffset);
            }
            else {
                toffset.trans = _vaxes[0]*newoffset;
            }
            _tLeft = _tLeftNoOffset * toffset;
            _tinvLeft = _tLeft.inverse();
        }
        if(GetDOF() > 1 && iaxis==GetDOF()-1 ) {
            _tRight = _tRightNoOffset;
            // right multiply by the offset of the last axis, might be buggy?
            if( IsRevolute(GetDOF()-1) ) {
                _tRight = matrixFromAxisAngle(_vaxes[GetDOF()-1], newoffset) * _tRight;
            }
            else {
                _tRight.trans += _vaxes[GetDOF()-1]*newoffset;
            }
            _tinvRight = _tRight.inverse();
        }
        GetParent()->_PostprocessChangedParameters(Prop_JointOffset);
    }
}

void KinBody::Joint::GetResolutions(std::vector<dReal>& resolutions, bool is_append) const
{
    if( !is_append ) {
        resolutions.resize(GetDOF());
    }
    for(int i = 0; i < GetDOF(); ++i) {
        resolutions.push_back(info_.resolution_vector_[i]);
    }
}

dReal KinBody::Joint::GetResolution(int iaxis) const
{
    return info_.resolution_vector_.at(iaxis);
}

void KinBody::Joint::SetResolution(dReal resolution, int iaxis)
{
    info_.resolution_vector_.at(iaxis) = resolution;
    GetParent()->_PostprocessChangedParameters(Prop_JointProperties);
}

void KinBody::Joint::GetWeights(std::vector<dReal>& weights, bool is_append) const
{
    if( !is_append ) {
        weights.resize(GetDOF());
    }
    for(int i = 0; i < GetDOF(); ++i) {
        weights.push_back(info_.weights_vector_[i]);
    }
}

dReal KinBody::Joint::GetWeight(int iaxis) const
{
    return info_.weights_vector_.at(iaxis);
}

void KinBody::Joint::SetWeights(const std::vector<dReal>& vweights)
{
    for(int i = 0; i < GetDOF(); ++i) {
        OPENRAVE_ASSERT_OP(vweights.at(i),>,0);
        info_.weights_vector_[i] = vweights.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointProperties);
}

void KinBody::Joint::SubtractValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const
{
    for(int i = 0; i < GetDOF(); ++i) {
        if( IsCircular(i) ) {
            q1.at(i) = utils::NormalizeCircularAngle(q1.at(i)-q2.at(i),circular_lower_limit_.at(i),circular_upper_limit_.at(i));
        }
        else {
            q1.at(i) -= q2.at(i);
        }
    }
}

dReal KinBody::Joint::SubtractValue(dReal value1, dReal value2, int iaxis) const
{
    if( IsCircular(iaxis) ) 
	{
        return utils::NormalizeCircularAngle(value1-value2,circular_lower_limit_.at(iaxis),circular_upper_limit_.at(iaxis));
    }
    else 
	{
        return value1-value2;
    }
}

void KinBody::Joint::AddTorque(const std::vector<dReal>& pTorques)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->AddJointTorque(shared_from_this(), pTorques);
}

dReal KinBody::Joint::GetMaxTorque(int iaxis) const
{
    if( !info_.electric_motor_info_ ) {
        return info_.max_torque_vector_.at(iaxis);
    }
    else {
        if( info_.electric_motor_info_->max_speed_torque_points.size() > 0 ) {
            if( info_.electric_motor_info_->max_speed_torque_points.size() == 1 ) {
                // doesn't matter what the velocity is
                return info_.electric_motor_info_->max_speed_torque_points.at(0).second*info_.electric_motor_info_->gear_ratio;
            }

            dReal velocity = RaveFabs(GetVelocity(iaxis));
            dReal revolutionsPerSecond = info_.electric_motor_info_->gear_ratio * velocity;
            if( IsRevolute(iaxis) ) {
                revolutionsPerSecond /= 2*M_PI;
            }

            if( revolutionsPerSecond <= info_.electric_motor_info_->max_speed_torque_points.at(0).first ) {
                return info_.electric_motor_info_->max_speed_torque_points.at(0).second*info_.electric_motor_info_->gear_ratio;
            }

            for(size_t i = 1; i < info_.electric_motor_info_->max_speed_torque_points.size(); ++i) {
                if( revolutionsPerSecond <= info_.electric_motor_info_->max_speed_torque_points.at(i).first ) {
                    // linearly interpolate to get the desired torque
                    dReal rps0 = info_.electric_motor_info_->max_speed_torque_points.at(i-1).first;
                    dReal torque0 = info_.electric_motor_info_->max_speed_torque_points.at(i-1).second;
                    dReal rps1 = info_.electric_motor_info_->max_speed_torque_points.at(i).first;
                    dReal torque1 = info_.electric_motor_info_->max_speed_torque_points.at(i).second;
                    if( rps1 - rps0 <= g_fEpsilonLinear ) {
                        return torque1*info_.electric_motor_info_->gear_ratio;
                    }

                    return ((revolutionsPerSecond - rps0)/(rps1 - rps0)*(torque1-torque0) + torque0)*info_.electric_motor_info_->gear_ratio;
                }
            }

            // revolutionsPerSecond is huge, return the last point
            return info_.electric_motor_info_->max_speed_torque_points.back().second*info_.electric_motor_info_->gear_ratio;
        }
        else {
            return info_.electric_motor_info_->max_instantaneous_torque*info_.electric_motor_info_->gear_ratio;
        }
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetInstantaneousTorqueLimits(int iaxis) const
{
    if( !info_.electric_motor_info_ ) {
        return std::make_pair(-info_.max_torque_vector_.at(iaxis), info_.max_torque_vector_.at(iaxis));
    }
    else {
        if( info_.electric_motor_info_->max_speed_torque_points.size() > 0 ) {
            dReal fMaxTorqueAtZeroSpeed = info_.electric_motor_info_->max_speed_torque_points.at(0).second*info_.electric_motor_info_->gear_ratio;
            if( info_.electric_motor_info_->max_speed_torque_points.size() == 1 ) {
                // doesn't matter what the velocity is
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            dReal rawvelocity = GetVelocity(iaxis);
            dReal velocity = RaveFabs(rawvelocity);
            dReal revolutionsPerSecond = info_.electric_motor_info_->gear_ratio * velocity;
            if( IsRevolute(iaxis) ) {
                revolutionsPerSecond /= 2*M_PI;
            }

            if( revolutionsPerSecond <= info_.electric_motor_info_->max_speed_torque_points.at(0).first ) {
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            for(size_t i = 1; i < info_.electric_motor_info_->max_speed_torque_points.size(); ++i) {
                if( revolutionsPerSecond <= info_.electric_motor_info_->max_speed_torque_points.at(i).first ) {
                    // linearly interpolate to get the desired torque
                    dReal rps0 = info_.electric_motor_info_->max_speed_torque_points.at(i-1).first;
                    dReal torque0 = info_.electric_motor_info_->max_speed_torque_points.at(i-1).second;
                    dReal rps1 = info_.electric_motor_info_->max_speed_torque_points.at(i).first;
                    dReal torque1 = info_.electric_motor_info_->max_speed_torque_points.at(i).second;

                    dReal finterpolatedtorque;
                    if( rps1 - rps0 <= g_fEpsilonLinear ) {
                        finterpolatedtorque = torque1*info_.electric_motor_info_->gear_ratio;
                    }
                    else {
                        finterpolatedtorque = ((revolutionsPerSecond - rps0)/(rps1 - rps0)*(torque1-torque0) + torque0)*info_.electric_motor_info_->gear_ratio;
                    }

                    // due to back emf, the deceleration magnitude is less than acceleration?
                    if (abs(rawvelocity) < 1.0/360) {
                        return std::make_pair(-finterpolatedtorque, finterpolatedtorque);
                    }
                    else if( rawvelocity > 0 ) {
                        return std::make_pair(-0.9*finterpolatedtorque, finterpolatedtorque);
                    }
                    else {
                        return std::make_pair(-finterpolatedtorque, 0.9*finterpolatedtorque);
                    }
                }
            }

            // due to back emf, the deceleration magnitude is less than acceleration?
            // revolutionsPerSecond is huge, return the last point
            dReal f = info_.electric_motor_info_->max_speed_torque_points.back().second*info_.electric_motor_info_->gear_ratio;
            if (abs(rawvelocity) < 1.0/360) {
                return std::make_pair(-f, f);
            }
            else if( rawvelocity > 0 ) {
                return std::make_pair(-0.9*f, f);
            }
            else {
                return std::make_pair(-f, 0.9*f);
            }
        }
        else {
            dReal f = info_.electric_motor_info_->max_instantaneous_torque*info_.electric_motor_info_->gear_ratio;
            return std::make_pair(-f, f);
        }
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetNominalTorqueLimits(int iaxis) const
{
    if( !info_.electric_motor_info_ ) {
        return std::make_pair(-info_.max_torque_vector_.at(iaxis), info_.max_torque_vector_.at(iaxis));
    }
    else {
        if( info_.electric_motor_info_->nominal_speed_torque_points.size() > 0 ) {
            dReal fMaxTorqueAtZeroSpeed = info_.electric_motor_info_->nominal_speed_torque_points.at(0).second*info_.electric_motor_info_->gear_ratio;
            if( info_.electric_motor_info_->nominal_speed_torque_points.size() == 1 ) {
                // doesn't matter what the velocity is
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            dReal rawvelocity = GetVelocity(iaxis);
            dReal velocity = RaveFabs(rawvelocity);
            dReal revolutionsPerSecond = info_.electric_motor_info_->gear_ratio * velocity;
            if( IsRevolute(iaxis) ) {
                revolutionsPerSecond /= 2*M_PI;
            }

            if( revolutionsPerSecond <= info_.electric_motor_info_->nominal_speed_torque_points.at(0).first ) {
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            for(size_t i = 1; i < info_.electric_motor_info_->nominal_speed_torque_points.size(); ++i) {
                if( revolutionsPerSecond <= info_.electric_motor_info_->nominal_speed_torque_points.at(i).first ) {
                    // linearly interpolate to get the desired torque
                    dReal rps0 = info_.electric_motor_info_->nominal_speed_torque_points.at(i-1).first;
                    dReal torque0 = info_.electric_motor_info_->nominal_speed_torque_points.at(i-1).second;
                    dReal rps1 = info_.electric_motor_info_->nominal_speed_torque_points.at(i).first;
                    dReal torque1 = info_.electric_motor_info_->nominal_speed_torque_points.at(i).second;

                    dReal finterpolatedtorque;
                    if( rps1 - rps0 <= g_fEpsilonLinear ) {
                        finterpolatedtorque = torque1*info_.electric_motor_info_->gear_ratio;
                    }
                    else {
                        finterpolatedtorque = ((revolutionsPerSecond - rps0)/(rps1 - rps0)*(torque1-torque0) + torque0)*info_.electric_motor_info_->gear_ratio;
                    }

                    // due to back emf, the deceleration magnitude is less than acceleration?
                    if (abs(rawvelocity) < 1.0/360) {
                        return std::make_pair(-finterpolatedtorque, finterpolatedtorque);
                    }
                    else if( rawvelocity > 0 ) {
                        return std::make_pair(-0.9*finterpolatedtorque, finterpolatedtorque);
                    }
                    else {
                        return std::make_pair(-finterpolatedtorque, 0.9*finterpolatedtorque);
                    }
                }
            }

            // due to back emf, the deceleration magnitude is less than acceleration?
            // revolutionsPerSecond is huge, return the last point
            dReal f = info_.electric_motor_info_->nominal_speed_torque_points.back().second*info_.electric_motor_info_->gear_ratio;
            if (abs(rawvelocity) < 1.0/360) {
                return std::make_pair(-f, f);
            }
            else if( rawvelocity > 0 ) {
                return std::make_pair(-0.9*f, f);
            }
            else {
                return std::make_pair(-f, 0.9*f);
            }
        }
        else {
            dReal f = info_.electric_motor_info_->nominal_torque*info_.electric_motor_info_->gear_ratio;
            return std::make_pair(-f, f);
        }
    }
}

bool KinBody::Joint::IsMimic(int iaxis) const
{
    if( iaxis >= 0 )
	{
        return !!mimic_array_.at(iaxis);
    }
    for(int i = 0; i < GetDOF(); ++i) 
	{
        if( !!mimic_array_.at(i) ) {
            return true;
        }
    }
    return false;
}

std::string KinBody::Joint::GetMimicEquation(int iaxis, int itype, const std::string& format) const
{
    if( !mimic_array_.at(iaxis) ) {
        return "";
    }
    if((format.size() == 0)||(format == "fparser")) {
        return mimic_array_.at(iaxis)->equations_.at(itype);
    }
    else if( format == "mathml" ) {
        boost::format mathfmt("<math xmlns=\"http://www.w3.org/1998/Math/MathML\">\n%s</math>\n");
        std::vector<std::string> Vars;
        std::string sout;
        KinBodyConstPtr parent(_parent);
        FOREACHC(itdofformat, mimic_array_.at(iaxis)->dof_format_vector_) {
            JointConstPtr pjoint = itdofformat->GetJoint(*parent);
            if( pjoint->GetDOF() > 1 ) {
                Vars.push_back(str(boost::format("<csymbol>%s_%d</csymbol>")%pjoint->GetName()%(int)itdofformat->axis));
            }
            else {
                Vars.push_back(str(boost::format("<csymbol>%s</csymbol>")%pjoint->GetName()));
            }
        }
        if( itype == 0 ) {
            mimic_array_.at(iaxis)->_posfn->toMathML(sout,Vars);
            if((sout.size() > 9)&&(sout.substr(0,9) == "<csymbol>")) {
                // due to a bug in ROS robot_model, have to return with <apply> (remove this in 2012).
                sout = boost::str(boost::format("<apply>\n  <plus/><cn type=\"real\">0</cn>\n  %s\n  </apply>")%sout);
            }
            sout = str(mathfmt%sout);
        }
        else if( itype == 1 ) {
            std::string stemp;
            FOREACHC(itfn, mimic_array_.at(iaxis)->_velfns) {
                (*itfn)->toMathML(stemp,Vars);
                sout += str(mathfmt%stemp);
            }
        }
        else if( itype == 2 ) {
            std::string stemp;
            FOREACHC(itfn, mimic_array_.at(iaxis)->_accelfns) {
                (*itfn)->toMathML(stemp,Vars);
                sout += str(mathfmt%stemp);
            }
        }
        return sout;
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_tr("unsupported math format %s"), format, ORE_InvalidArguments);
}

void KinBody::Joint::GetMimicDOFIndices(std::vector<int>& vmimicdofs, int iaxis) const
{
    OPENRAVE_ASSERT_FORMAT(!!mimic_array_.at(iaxis), "joint %s axis %d is not mimic", GetName()%iaxis,ORE_InvalidArguments);
    vmimicdofs.resize(0);
    FOREACHC(it, mimic_array_.at(iaxis)->mimic_dofs_vector_) {
        std::vector<int>::iterator itinsert = std::lower_bound(vmimicdofs.begin(),vmimicdofs.end(), it->dofindex);
        if((itinsert == vmimicdofs.end())||(*itinsert != it->dofindex)) {
            vmimicdofs.insert(itinsert,it->dofindex);
        }
    }
}

void KinBody::Joint::SetMimicEquations(int iaxis, const std::string& poseq,
	const std::string& veleq, const std::string& acceleq)
{
    mimic_array_.at(iaxis).reset();
    if( poseq.size() == 0 ) {
        return;
    }
    KinBodyPtr parent(_parent);
    std::vector<std::string> resultVars;
    MimicPtr mimic(new Mimic());
    mimic->equations_.at(0) = poseq;
    mimic->equations_.at(1) = veleq;
    mimic->equations_.at(2) = acceleq;

    // copy equations into the info
    if( !info_._vmimic.at(iaxis) ) {
        info_._vmimic.at(iaxis).reset(new MimicInfo());
    }
    info_._vmimic.at(iaxis)->equations_ = mimic->equations_;

    OpenRAVEFunctionParserRealPtr posfn = CreateJointFunctionParser();
    mimic->_posfn = posfn;
    // because openrave joint names can hold symbols like '-' and '.' can affect the equation, so first do a search and replace
    std::vector< std::pair<std::string, std::string> > jointnamepairs; jointnamepairs.reserve(parent->GetJoints().size());
    FOREACHC(itjoint,parent->GetJoints()) {
        if( (*itjoint)->GetName().size() > 0 ) {
            std::string newname = str(boost::format("joint%d")%(*itjoint)->GetJointIndex());
            jointnamepairs.emplace_back((*itjoint)->GetName(), newname);
        }
    }
    size_t index = parent->GetJoints().size();
    FOREACHC(itjoint,parent->GetPassiveJoints()) {
        if( (*itjoint)->GetName().size() > 0 ) {
            std::string newname = str(boost::format("joint%d")%index);
            jointnamepairs.emplace_back((*itjoint)->GetName(), newname);
        }
        ++index;
    }

    std::map<std::string,std::string> mapinvnames;
    FOREACH(itpair,jointnamepairs) {
        mapinvnames[itpair->second] = itpair->first;
    }

    std::string eq;
    int ret = posfn->ParseAndDeduceVariables(utils::SearchAndReplace(eq,mimic->equations_[0],jointnamepairs),resultVars);
    if( ret >= 0 ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to set equation '%s' on %s:%s, at %d. Error is %s\n"), mimic->equations_[0]%parent->GetName()%GetName()%ret%posfn->ErrorMsg(),ORE_InvalidArguments);
    }
    // process the variables
    FOREACH(itvar,resultVars) {
        OPENRAVE_ASSERT_FORMAT(itvar->find("joint") == 0, "equation '%s' uses unknown variable", mimic->equations_[0], ORE_InvalidArguments);
		Mimic::DOFFormat dofformat;
        size_t axisindex = itvar->find('_');
        if( axisindex != std::string::npos ) {
            dofformat.jointindex = boost::lexical_cast<uint16_t>(itvar->substr(5,axisindex-5));
            dofformat.axis = boost::lexical_cast<uint8_t>(itvar->substr(axisindex+1));
        }
        else {
            dofformat.jointindex = boost::lexical_cast<uint16_t>(itvar->substr(5));
            dofformat.axis = 0;
        }
        dofformat.dofindex = -1;
        JointPtr pjoint = dofformat.GetJoint(*parent);
        if((pjoint->GetDOFIndex() >= 0)&& !pjoint->IsMimic(dofformat.axis) ) {
            dofformat.dofindex = pjoint->GetDOFIndex()+dofformat.axis;
			Mimic::DOFHierarchy h;
            h.dofindex = dofformat.dofindex;
            h.dofformatindex = mimic->dof_format_vector_.size();
            mimic->mimic_dofs_vector_.push_back(h);
        }
        mimic->dof_format_vector_.push_back(dofformat);
    }

    // need to set sVars to resultVars since that's what the user will be feeding with the input
    stringstream sVars;
    if( !resultVars.empty() ) {
        sVars << resultVars.at(0);
        for(size_t i = 1; i < resultVars.size(); ++i) {
            sVars << "," << resultVars[i];
        }
    }

    for(int itype = 1; itype < 3; ++itype) {
        if((itype == 2)&&(mimic->equations_[itype].size() == 0)) {
            continue;
        }

        std::vector<OpenRAVEFunctionParserRealPtr> vfns(resultVars.size());
        // extract the equations
        utils::SearchAndReplace(eq,mimic->equations_[itype],jointnamepairs);
        size_t index = eq.find('|');
        while(index != std::string::npos) {
            size_t startindex = index+1;
            index = eq.find('|',startindex);
            string sequation;
            if( index != std::string::npos) {
                sequation = eq.substr(startindex,index-startindex);
            }
            else {
                sequation = eq.substr(startindex);
            }
            boost::trim(sequation);
            size_t nameendindex = sequation.find(' ');
            string varname;
            if( nameendindex == std::string::npos ) {
                RAVELOG_WARN(str(boost::format("invalid equation syntax '%s' for joint %s")%sequation%info_.name_));
                varname = sequation;
                sequation = "0";
            }
            else {
                varname = sequation.substr(0,nameendindex);
                sequation = sequation.substr(nameendindex);
            }
            vector<string>::iterator itnameindex = find(resultVars.begin(),resultVars.end(),varname);
            OPENRAVE_ASSERT_FORMAT(itnameindex != resultVars.end(), "variable %s from velocity equation is not referenced in the position, skipping...", mapinvnames[varname],ORE_InvalidArguments);

            OpenRAVEFunctionParserRealPtr fn = CreateJointFunctionParser();
            ret = fn->Parse(sequation,sVars.str());
            if( ret >= 0 ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to set equation '%s' on %s:%s, at %d. Error is %s"), sequation%parent->GetName()%GetName()%ret%fn->ErrorMsg(),ORE_InvalidArguments);
            }
            vfns.at(itnameindex-resultVars.begin()) = fn;
        }
        // check if anything is missing
        for(size_t j = 0; j < resultVars.size(); ++j) {
            if( !vfns[j] ) {
                // print a message instead of throwing an exception since it might be common for only position equations to be specified
                RAVELOG_WARN(str(boost::format("SetMimicEquations: missing variable %s from partial derivatives of joint %s!")%mapinvnames[resultVars[j]]%info_.name_));
                vfns[j] = CreateJointFunctionParser();
                vfns[j]->Parse("0","");
            }
        }

        if( itype == 1 ) {
            mimic->_velfns.swap(vfns);
        }
        else {
            mimic->_accelfns.swap(vfns);
        }
    }
    mimic_array_.at(iaxis) = mimic;
    parent->_PostprocessChangedParameters(Prop_JointMimic);
}

void KinBody::Joint::_ComputePartialVelocities(std::vector<std::pair<int,dReal> >& vpartials,
	int iaxis, std::map< std::pair<Mimic::DOFFormat, int>, dReal >& mapcachedpartials) const
{
    vpartials.resize(0);
    if( dof_index_ >= 0 ) {
        vpartials.emplace_back(dof_index_+iaxis, 1.0);
        return;
    }
    OPENRAVE_ASSERT_FORMAT(!!mimic_array_.at(iaxis), "cannot compute partial velocities of joint %s", info_.name_, ORE_Failed);
    KinBodyConstPtr parent(_parent);
	Mimic::DOFFormat thisdofformat;
    thisdofformat.dofindex = -1; // always -1 since it is mimiced
    thisdofformat.axis = iaxis;
    thisdofformat.jointindex = joint_index_;
    if( joint_index_ < 0 ) {
        // this is a weird computation... have to figure out the passive joint index given where it is in parent->GetPassiveJoints()
        thisdofformat.jointindex = parent->GetJoints().size() + (find(parent->GetPassiveJoints().begin(),parent->GetPassiveJoints().end(),shared_from_this()) - parent->GetPassiveJoints().begin());
    }
    std::vector<std::pair<int,dReal> > vtemppartials;
    vector<dReal> vtempvalues;
    FOREACHC(itmimicdof, mimic_array_[iaxis]->mimic_dofs_vector_) {
        std::pair<Mimic::DOFFormat, int> key = make_pair(thisdofformat,itmimicdof->dofindex);
        std::map< std::pair<Mimic::DOFFormat, int>, dReal >::iterator it = mapcachedpartials.find(key);
        if( it == mapcachedpartials.end() ) {
            // not in the cache so compute using the chain rule
            if( vtempvalues.empty() ) {
                FOREACHC(itdofformat, mimic_array_[iaxis]->dof_format_vector_) {
                    vtempvalues.push_back(itdofformat->GetJoint(*parent)->GetValue(itdofformat->axis));
                }
            }
            dReal fvel = mimic_array_[iaxis]->_velfns.at(itmimicdof->dofformatindex)->Eval(vtempvalues.empty() ? NULL : &vtempvalues[0]);
            const Mimic::DOFFormat& dofformat = mimic_array_[iaxis]->dof_format_vector_.at(itmimicdof->dofformatindex);
            if( dofformat.GetJoint(*parent)->IsMimic(dofformat.axis) ) {
                dofformat.GetJoint(*parent)->_ComputePartialVelocities(vtemppartials,dofformat.axis,mapcachedpartials);
                dReal fpartial = 0;
                FOREACHC(itpartial,vtemppartials) {
                    if( itpartial->first == itmimicdof->dofindex ) {
                        fpartial += itpartial->second;
                    }
                }
                fvel *= fpartial;
            }
            // before pushing back, check for repetition
            bool badd = true;
            FOREACH(itpartial,vpartials) {
                if( itpartial->first == itmimicdof->dofindex ) {
                    itpartial->second += fvel;
                    badd = false;
                    break;
                }
            }
            if( badd ) {
                vpartials.emplace_back(itmimicdof->dofindex,  fvel);
            }
        }
        else {
            bool badd = true;
            FOREACH(itpartial,vpartials) {
                if( itpartial->first == itmimicdof->dofindex ) {
                    badd = false;
                    break;
                }
            }
            if( badd ) {
                vpartials.emplace_back(itmimicdof->dofindex,  it->second);
            }
        }
    }
}

int KinBody::Joint::_Eval(int axis, uint32_t timederiv, const std::vector<dReal>& vdependentvalues, std::vector<dReal>& voutput)
{
    if( timederiv == 0 ) {
        mimic_array_.at(axis)->_posfn->EvalMulti(voutput, vdependentvalues.empty() ? NULL : &vdependentvalues[0]);
        return mimic_array_.at(axis)->_posfn->EvalError();
    }
    else if( timederiv == 1 ) {
        voutput.resize(mimic_array_.at(axis)->_velfns.size());
        for(size_t i = 0; i < voutput.size(); ++i) {
            voutput[i] = mimic_array_.at(axis)->_velfns.at(i)->Eval(vdependentvalues.empty() ? NULL : &vdependentvalues[0]);
            int err = mimic_array_.at(axis)->_velfns.at(i)->EvalError();
            if( err ) {
                return err;
            }
        }
    }
    else if( timederiv == 2 ) {
        voutput.resize(mimic_array_.at(axis)->_accelfns.size());
        for(size_t i = 0; i < voutput.size(); ++i) {
            voutput[i] = mimic_array_.at(axis)->_accelfns.at(i)->Eval(vdependentvalues.empty() ? NULL : &vdependentvalues[0]);
            int err = mimic_array_.at(axis)->_accelfns.at(i)->EvalError();
            if( err ) {
                return err;
            }
        }
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("timederiv %d not supported"),timederiv,ORE_InvalidArguments);
    }
    return 0;
}

bool KinBody::Mimic::DOFFormat::operator <(const KinBody::Mimic::DOFFormat& r) const
{
    return jointindex < r.jointindex || (jointindex == r.jointindex && (dofindex < r.dofindex || (dofindex == r.dofindex && axis < r.axis)));
}

bool KinBody::Mimic::DOFFormat::operator ==(const KinBody::Mimic::DOFFormat& r) const
{
    return jointindex == r.jointindex && dofindex == r.dofindex && axis == r.axis;
}

KinBody::JointPtr KinBody::Mimic::DOFFormat::GetJoint(KinBody &parent) const
{
    int numjoints = (int)parent.GetJoints().size();
    return jointindex < numjoints ? parent.GetJoints().at(jointindex) : parent.GetPassiveJoints().at(jointindex-numjoints);
}

KinBody::JointConstPtr KinBody::Mimic::DOFFormat::GetJoint(const KinBody &parent) const
{
    int numjoints = (int)parent.GetJoints().size();
    return jointindex < numjoints ? parent.GetJoints().at(jointindex) : parent.GetPassiveJoints().at(jointindex-numjoints);
}

void KinBody::Joint::SetFloatParameters(const std::string& key, const std::vector<dReal>& parameters)
{
    if( parameters.size() > 0 ) 
	{
        info_.float_parameters_map_[key] = parameters;
    }
    else 
	{
        info_.float_parameters_map_.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointCustomParameters);
}

void KinBody::Joint::SetIntParameters(const std::string& key, const std::vector<int>& parameters)
{
    if( parameters.size() > 0 )
	{
        info_.int_parameters_map_[key] = parameters;
    }
    else 
	{
        info_.int_parameters_map_.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointCustomParameters);
}

void KinBody::Joint::SetStringParameters(const std::string& key, const std::string& value)
{
    if( value.size() > 0 ) 
	{
        info_.string_parameters_map_[key] = value;
    }
    else 
	{
        info_.string_parameters_map_.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointCustomParameters);
}

void KinBody::Joint::UpdateInfo()
{
    info_.current_values_vector_.resize(0);
    GetValues(info_.current_values_vector_);
}

void KinBody::Joint::serialize(std::ostream& o, int options) const
{
    if( options & SO_Kinematics ) {
        o << dof_index_ << " " << joint_index_ << " " << info_.type_ << " ";
        SerializeRound(o,_tRightNoOffset);
        SerializeRound(o,_tLeftNoOffset);
        for(int i = 0; i < GetDOF(); ++i) {
            SerializeRound3(o,_vaxes[i]);
            if( !!mimic_array_.at(i) ) {
                FOREACHC(iteq,mimic_array_.at(i)->equations_) {
                    o << *iteq << " ";
                }
            }
        }
        o << (!attached_bodies_array_[0] ? -1 : attached_bodies_array_[0]->GetIndex()) << " " << (attached_bodies_array_[1]->GetIndex()) << " ";
    }
    // in the past was including saving limits as part of SO_Dynamics, but given that limits change a lot when planning, should *not* include them as part of dynamics.
    if( options & SO_JointLimits ) {
        for(int i = 0; i < GetDOF(); ++i) {
            SerializeRound(o,info_.max_velocity_vector_[i]);
            SerializeRound(o,info_.max_accelerate_vector_[i]);
            SerializeRound(o,info_.max_jerk_vector_[i]);
            SerializeRound(o,info_.max_torque_vector_[i]);
            SerializeRound(o,info_.max_inertia_vector_[i]);
            SerializeRound(o,info_.lower_limit_vector_[i]);
            SerializeRound(o,info_.upper_limit_vector_[i]);
        }
    }
}

void KinBody::MimicInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal unit_scale, int options) const
{
    openravejson::SetJsonValueByKey(value, "equations", equations_, allocator);
}

void KinBody::MimicInfo::DeserializeJSON(const rapidjson::Value& value, dReal unit_scale)
{
    openravejson::LoadJsonValueByKey(value, "equations", equations_);
}

}
