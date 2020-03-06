// -*- coding: utf-8 -*-
// Copyright (C) 2019
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
#include <boost/make_shared.hpp>

namespace OpenRAVE {

RobotBase::ConnectedBodyInfo::ConnectedBodyInfo() : _bIsActive(false)
{
}

void RobotBase::ConnectedBodyInfo::InitInfoFromBody(RobotBase& robot)
{
    _vLinkInfos.clear();
    _vJointInfos.clear();
    _vManipulatorInfos.clear();
    _vAttachedSensorInfos.clear();

    // have to set to the identity before extracting info
    KinBody::KinBodyStateSaverRef statesaver(robot, Save_LinkTransformation);
    std::vector<dReal> vzeros(robot.GetDOF());
    robot.SetDOFValues(vzeros, Transform());

    FOREACH(itlink, robot._veclinks) {
        _vLinkInfos.push_back(std::make_shared<KinBody::LinkInfo>((*itlink)->UpdateAndGetInfo()));
    }

    FOREACH(itjoint, robot._vecjoints) {
        _vJointInfos.push_back(std::make_shared<KinBody::JointInfo>((*itjoint)->UpdateAndGetInfo()));
    }
    FOREACH(itjoint, robot._vPassiveJoints) {
        _vJointInfos.push_back(std::make_shared<KinBody::JointInfo>((*itjoint)->UpdateAndGetInfo()));
    }

    FOREACH(itmanip, robot.GetManipulators()) {
        _vManipulatorInfos.push_back(std::make_shared<RobotBase::ManipulatorInfo>((*itmanip)->GetInfo()));
    }

    FOREACH(itattachedsensor, robot.GetAttachedSensors()) {
        _vAttachedSensorInfos.push_back(std::make_shared<RobotBase::AttachedSensorInfo>((*itattachedsensor)->UpdateAndGetInfo()));
    }
}
void RobotBase::ConnectedBodyInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal unit_scale, int options) const
{
    openravejson::SetJsonValueByKey(value, "name", name_, allocator);
    openravejson::SetJsonValueByKey(value, "linkName", _linkname, allocator);
    openravejson::SetJsonValueByKey(value, "uri", _uri, allocator);
    openravejson::SetJsonValueByKey(value, "transform", _trelative, allocator);

    // rapidjson::Value linkInfosValue;
    // linkInfosValue.SetArray();
    // FOREACH(it, _vLinkInfos)
    // {
    //     rapidjson::Value info;
    //     (*it)->SerializeJSON(info, allocator, options);
    //     linkInfosValue.PushBack(info, allocator);
    // }
    // value.AddMember("links", linkInfosValue, allocator);

    // rapidjson::Value jointInfosValue;
    // jointInfosValue.SetArray();
    // FOREACH(it, _vJointInfos)
    // {
    //     rapidjson::Value v;
    //     (*it)->SerializeJSON(v, allocator, options);
    //     jointInfosValue.PushBack(v, allocator);
    // }
    // value.AddMember("joints", jointInfosValue, allocator);

    // rapidjson::Value manipulatorInfosValue;
    // manipulatorInfosValue.SetArray();
    // FOREACH(it, _vManipulatorInfos)
    // {
    //     rapidjson::Value info;
    //     (*it)->SerializeJSON(info, allocator, options);
    //     manipulatorInfosValue.PushBack(info, allocator);
    // }
    // value.AddMember("manipulators", manipulatorInfosValue, allocator);

    // rapidjson::Value attachedSensorInfosValue;
    // attachedSensorInfosValue.SetArray();
    // FOREACH(it, _vAttachedSensorInfos)
    // {
    //     rapidjson::Value info;
    //     (*it)->SerializeJSON(info, allocator, options);
    //     attachedSensorInfosValue.PushBack(info, allocator);
    // }
    // value.AddMember("attachedSensors", attachedSensorInfosValue, allocator);

    openravejson::SetJsonValueByKey(value, "isActive", _bIsActive, allocator);
}

void RobotBase::ConnectedBodyInfo::DeserializeJSON(const rapidjson::Value &value, dReal unit_scale)
{
    openravejson::LoadJsonValueByKey(value, "name", name_);
    openravejson::LoadJsonValueByKey(value, "linkName", _linkname);
    openravejson::LoadJsonValueByKey(value, "uri", _uri);
    openravejson::LoadJsonValueByKey(value, "transform", _trelative);

    // if(value.HasMember("links"))
    // {
    //     _vLinkInfos.resize(0);
    //     _vLinkInfos.reserve(value["links"].Size());
    //     for (size_t i = 0; i < value["links"].Size(); ++i) {
    //         LinkInfoPtr linkinfo(new LinkInfo());
    //         linkinfo->DeserializeJSON(value["links"][i]);
    //         _vLinkInfos.push_back(linkinfo);
    //     }
    // }

    // if(value.HasMember("joints"))
    // {
    //     _vJointInfos.resize(0);
    //     _vJointInfos.reserve(value["joints"].Size());
    //     for (size_t i = 0; i < value["joints"].Size(); ++i) {
    //         JointInfoPtr jointinfo(new JointInfo());
    //         jointinfo->DeserializeJSON(value["joints"][i]);
    //         _vJointInfos.push_back(jointinfo);
    //     }
    // }

    // if(value.HasMember("manipulators"))
    // {
    //     _vManipulatorInfos.resize(0);
    //     _vManipulatorInfos.reserve(value["manipulators"].Size());
    //     for (size_t i = 0; i < value["manipulators"].Size(); ++i) {
    //         ManipulatorInfoPtr manipulatorinfo(new ManipulatorInfo());
    //         manipulatorinfo->DeserializeJSON(value["manipulators"][i]);
    //         _vManipulatorInfos.push_back(manipulatorinfo);
    //     }
    // }

    // if(value.HasMember("attachedSensors"))
    // {
    //     _vAttachedSensorInfos.resize(0);
    //     _vAttachedSensorInfos.reserve(value["attachedSensors"].Size());
    //     for (size_t i = 0; i < value["attachedSensors"].Size(); ++i) {
    //         AttachedSensorInfoPtr attachedsensorinfo(new AttachedSensorInfo());
    //         attachedsensorinfo->DeserializeJSON(value["attachedSensors"][i]);
    //         _vAttachedSensorInfos.push_back(attachedsensorinfo);
    //     }
    // }

    openravejson::LoadJsonValueByKey(value, "isActive", _bIsActive);
}



RobotBase::ConnectedBody::ConnectedBody(OpenRAVE::RobotBasePtr probot) : _pattachedrobot(probot)
{
}

RobotBase::ConnectedBody::ConnectedBody(OpenRAVE::RobotBasePtr probot, const OpenRAVE::RobotBase::ConnectedBodyInfo &info)
    : info_(info), _pattachedrobot(probot)
{
    if (!!probot) {
        LinkPtr attachedLink = probot->GetLink(info_._linkname);
        if( !attachedLink ) {
            throw OPENRAVE_EXCEPTION_FORMAT("Link \"%s\" to which ConnectedBody %s is attached does not exist in robot %s", info._linkname%GetName()%probot->GetName(), ORE_InvalidArguments);
        }
        _pattachedlink = attachedLink;
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT("Valid robot is not given for ConnectedBody %s", GetName(), ORE_InvalidArguments);
    }
}


RobotBase::ConnectedBody::ConnectedBody(OpenRAVE::RobotBasePtr probot, const ConnectedBody &connectedBody, int cloningoptions)
{
    *this = connectedBody;
    _pDummyJointCache = probot->GetJoint(_dummyPassiveJointName);
    FOREACH(itinfo, _vResolvedLinkNames) {
        itinfo->second = probot->GetLink(itinfo->first);
    }
    FOREACH(itinfo, _vResolvedJointNames) {
        itinfo->second = probot->GetJoint(itinfo->first);
    }
    FOREACH(itinfo, _vResolvedManipulatorNames) {
        itinfo->second = probot->GetManipulator(itinfo->first);
    }
    FOREACH(itinfo, _vResolvedAttachedSensorNames) {
        itinfo->second = probot->GetAttachedSensor(itinfo->first);
    }
    _pattachedrobot = probot;
    _pattachedlink = probot->GetLink(LinkPtr(connectedBody._pattachedlink)->GetName());
}

RobotBase::ConnectedBody::~ConnectedBody()
{
}

bool RobotBase::ConnectedBody::SetActive(bool active)
{
    if (info_._bIsActive == active) {
        return false;
    }

    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        if( pattachedrobot->_nHierarchyComputed != 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT("Cannot set ConnectedBody %s active to %s since robot %s is still in the environment", info_.name_%active%pattachedrobot->GetName(), ORE_InvalidState);
        }
    }
    info_._bIsActive = active;
    return true; // changed
}

bool RobotBase::ConnectedBody::IsActive()
{
    return info_._bIsActive;
}

void RobotBase::ConnectedBody::SetLinkEnable(bool benable)
{
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        std::vector<uint8_t> enablestates;
        pattachedrobot->GetLinkEnableStates(enablestates);
        bool bchanged = false;

        FOREACH(itlinkname, _vResolvedLinkNames) {
            KinBody::LinkPtr plink = pattachedrobot->GetLink(itlinkname->first);
            if( !!plink ) {
                if( enablestates.at(plink->GetIndex()) != benable ) {
                    enablestates.at(plink->GetIndex()) = benable;
                    bchanged = true;
                }
            }
        }

        if( bchanged ) {
            pattachedrobot->SetLinkEnableStates(enablestates);
        }
    }
}

void RobotBase::ConnectedBody::SetLinkVisible(bool bvisible)
{
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        FOREACH(itlinkname, _vResolvedLinkNames) {
            KinBody::LinkPtr plink = pattachedrobot->GetLink(itlinkname->first);
            if( !!plink ) {
                plink->SetVisible(bvisible);
            }
        }
    }
}

void RobotBase::ConnectedBody::GetResolvedLinks(std::vector<KinBody::LinkPtr>& links)
{
    links.resize(_vResolvedLinkNames.size());
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        for(size_t ilink = 0; ilink < _vResolvedLinkNames.size(); ++ilink) {
            links[ilink] = pattachedrobot->GetLink(_vResolvedLinkNames[ilink].first);
        }
    }
    else {
        FOREACH(itlink, links) {
            itlink->reset();
        }
    }
}

void RobotBase::ConnectedBody::GetResolvedJoints(std::vector<KinBody::JointPtr>& joints)
{
    joints.resize(_vResolvedJointNames.size());
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        for(size_t ijoint = 0; ijoint < _vResolvedJointNames.size(); ++ijoint) {
            joints[ijoint] = pattachedrobot->GetJoint(_vResolvedJointNames[ijoint].first);
        }
    }
    else {
        FOREACH(itjoint, joints) {
            itjoint->reset();
        }
    }
}

void RobotBase::ConnectedBody::GetResolvedManipulators(std::vector<RobotBase::ManipulatorPtr>& manipulators)
{
    manipulators.resize(_vResolvedManipulatorNames.size());
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        for(size_t imanipulator = 0; imanipulator < _vResolvedManipulatorNames.size(); ++imanipulator) {
            manipulators[imanipulator] = pattachedrobot->GetManipulator(_vResolvedManipulatorNames[imanipulator].first);
        }
    }
    else {
        FOREACH(itmanipulator, manipulators) {
            itmanipulator->reset();
        }
    }
}

void RobotBase::ConnectedBody::GetResolvedAttachedSensors(std::vector<RobotBase::AttachedSensorPtr>& attachedSensors)
{
    attachedSensors.resize(_vResolvedAttachedSensorNames.size());
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        for(size_t iattachedSensor = 0; iattachedSensor < _vResolvedAttachedSensorNames.size(); ++iattachedSensor) {
            attachedSensors[iattachedSensor] = pattachedrobot->GetAttachedSensor(_vResolvedAttachedSensorNames[iattachedSensor].first);
        }
    }
    else {
        FOREACH(itattachedSensor, attachedSensors) {
            itattachedSensor->reset();
        }
    }
}

RobotBase::ConnectedBodyPtr RobotBase::AddConnectedBody(const RobotBase::ConnectedBodyInfo& connectedBodyInfo, bool removeduplicate)
{
    if( _nHierarchyComputed != 0 ) {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot add connected body while robot %s is added to the environment", GetName(), ORE_InvalidState);
    }

    OPENRAVE_ASSERT_OP(connectedBodyInfo.name_.size(),>,0);
    int iremoveindex = -1;
    for(int iconnectedbody = 0; iconnectedbody < (int)_vecConnectedBodies.size(); ++iconnectedbody) {
        if( _vecConnectedBodies[iconnectedbody]->GetName() == connectedBodyInfo.name_ ) {
            if( removeduplicate ) {
                iremoveindex = iconnectedbody;
                break;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("attached sensor with name %s already exists"),connectedBodyInfo.name_,ORE_InvalidArguments);
            }
        }
    }
    ConnectedBodyPtr newConnectedBody(new ConnectedBody(shared_robot(),connectedBodyInfo));
//    if( _nHierarchyComputed ) {
//        newConnectedBody->_ComputeInternalInformation();
//    }
    if( iremoveindex >= 0 ) {
        // replace the old one
        _vecConnectedBodies[iremoveindex] = newConnectedBody;
    }
    else {
        _vecConnectedBodies.push_back(newConnectedBody);
    }
    //newConnectedBody->UpdateInfo(); // just in case
    __hashrobotstructure.resize(0);
    return newConnectedBody;
}

RobotBase::ConnectedBodyPtr RobotBase::GetConnectedBody(const std::string& name) const
{
    FOREACHC(itconnectedbody, _vecConnectedBodies) {
        if( (*itconnectedbody)->GetName() == name ) {
            return *itconnectedbody;
        }
    }
    return RobotBase::ConnectedBodyPtr();
}

bool RobotBase::RemoveConnectedBody(RobotBase::ConnectedBody &connectedBody)
{
    FOREACH(itconnectedBody,_vecConnectedBodies) {
        if( itconnectedBody->get() == &connectedBody ) {
            _vecConnectedBodies.erase(itconnectedBody);
            __hashrobotstructure.clear();
            return true;
        }
    }
    return false;
}

void RobotBase::_ComputeConnectedBodiesInformation()
{
    // resolve duplicate names for links and joints in connected body info
    // reinitialize robot with combined infos
    if (_vecConnectedBodies.empty()) {
        return;
    }

    // should have already done adding the necessary link etc
    // during cloning, we should not add links and joints again
    if (_nHierarchyComputed != 0) {
        return;
    }

    FOREACH(itconnectedBody, _vecConnectedBodies) {
        ConnectedBody& connectedBody = **itconnectedBody;
        const ConnectedBodyInfo& connectedBodyInfo = connectedBody.info_;

        if( !connectedBody.GetAttachingLink() ) {
            throw OPENRAVE_EXCEPTION_FORMAT("ConnectedBody %s for robot %s does not have a valid pointer to link %s", connectedBody.GetName()%GetName()%connectedBodyInfo._linkname, ORE_InvalidArguments);
        }

        Transform tBaseLinkInWorld = connectedBody.GetTransform(); // transform all links and joints by this

        if( connectedBody.GetName().size() == 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT("ConnectedBody %s attached to link %s has no name initialized", connectedBodyInfo._uri%connectedBodyInfo._linkname, ORE_InvalidArguments);
        }

        vector<ConnectedBodyPtr>::iterator itconnectedBody2 = itconnectedBody; ++itconnectedBody2;
        for(; itconnectedBody2 != _vecConnectedBodies.end(); ++itconnectedBody2) {
            if( connectedBody.GetName() == (*itconnectedBody2)->GetName() ) {
                throw OPENRAVE_EXCEPTION_FORMAT("robot %s has two ConnectedBody with the same name %s!", GetName()%connectedBody.GetName(), ORE_InvalidArguments);
            }
        }

        if( !connectedBody.IsActive() ) {
            // skip
            continue;
        }

        if( connectedBodyInfo._vLinkInfos.size() == 0 ) {
            RAVELOG_WARN_FORMAT("ConnectedBody %s for robot %s has no link infos, so cannot add anything", connectedBody.GetName()%GetName());
            continue;
        }

        // check if connectedBodyInfo._linkname exists
        bool bExists = false;
        FOREACH(ittestlink, _veclinks) {
            if( connectedBodyInfo._linkname == (*ittestlink)->GetName() ) {
                bExists = true;
                break;
            }
        }
        if( !bExists ) {
            throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, the attaching link '%s' on robot does not exist!", connectedBody.GetName()%GetName()%connectedBodyInfo._linkname, ORE_InvalidArguments);
        }

        connectedBody._nameprefix = connectedBody.GetName() + "_";

        // Links
        connectedBody._vResolvedLinkNames.resize(connectedBodyInfo._vLinkInfos.size());
        for(int ilink = 0; ilink < (int)connectedBodyInfo._vLinkInfos.size(); ++ilink) {
            KinBody::LinkPtr& plink = connectedBody._vResolvedLinkNames[ilink].second;
            if( !plink ) {
                plink.reset(new KinBody::Link(shared_kinbody()));
            }
            plink->info_ = *connectedBodyInfo._vLinkInfos[ilink]; // copy
            plink->info_.name_ = connectedBody._nameprefix + plink->info_.name_;
            plink->info_._t = tBaseLinkInWorld * plink->info_._t;
            _InitAndAddLink(plink);
            connectedBody._vResolvedLinkNames[ilink].first = plink->info_.name_;
        }

        // Joints
        std::vector<KinBody::JointPtr> vNewJointsToAdd;
        std::vector<std::pair<std::string, std::string> > jointNamePairs;
        connectedBody._vResolvedJointNames.resize(connectedBodyInfo._vJointInfos.size());
        for(int ijoint = 0; ijoint < (int)connectedBodyInfo._vJointInfos.size(); ++ijoint) {
            KinBody::JointPtr& pjoint = connectedBody._vResolvedJointNames[ijoint].second;
            if( !pjoint ) {
                pjoint.reset(new KinBody::Joint(shared_kinbody()));
            }
            pjoint->info_ = *connectedBodyInfo._vJointInfos[ijoint]; // copy
            pjoint->info_.name_ = connectedBody._nameprefix + pjoint->info_.name_;

            // search for the correct resolved _linkname0 and _linkname1
            bool bfoundlink0 = false, bfoundlink1 = false;
            for(size_t ilink = 0; ilink < connectedBodyInfo._vLinkInfos.size(); ++ilink) {
                if( pjoint->info_._linkname0 == connectedBodyInfo._vLinkInfos[ilink]->name_ ) {
                    pjoint->info_._linkname0 = connectedBody._vResolvedLinkNames.at(ilink).first;
                    bfoundlink0 = true;
                }
                if( pjoint->info_._linkname1 == connectedBodyInfo._vLinkInfos[ilink]->name_ ) {
                    pjoint->info_._linkname1 = connectedBody._vResolvedLinkNames.at(ilink).first;
                    bfoundlink1 = true;
                }
            }

            if( !bfoundlink0 ) {
                throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for joint %s, could not find linkname0 %s in connected body link infos!", connectedBody.GetName()%GetName()%pjoint->info_.name_%pjoint->info_._linkname0, ORE_InvalidArguments);
            }
            if( !bfoundlink1 ) {
                throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for joint %s, could not find linkname1 %s in connected body link infos!", connectedBody.GetName()%GetName()%pjoint->info_.name_%pjoint->info_._linkname1, ORE_InvalidArguments);
            }
            jointNamePairs.emplace_back(connectedBodyInfo._vJointInfos[ijoint]->name_,  pjoint->info_.name_);
            vNewJointsToAdd.push_back(pjoint);
            connectedBody._vResolvedJointNames[ijoint].first = pjoint->info_.name_;
        }

        FOREACH(itnewjoint, vNewJointsToAdd) {
            KinBody::JointInfo& jointinfo = (*itnewjoint)->info_;
            FOREACH(itmimic, jointinfo._vmimic) {
                if (!(*itmimic)) {
                    continue;
                }
                for (std::size_t iequation = 0; iequation < (*itmimic)->_equations.size(); ++iequation) {
                    std::string eq;
                    utils::SearchAndReplace(eq, (*itmimic)->_equations[iequation], jointNamePairs);
                    (*itmimic)->_equations[iequation] = eq;
                }
            }

            _InitAndAddJoint(*itnewjoint);
        }

        // Manipulators
        connectedBody._vResolvedManipulatorNames.resize(connectedBodyInfo._vManipulatorInfos.size());
        for(int imanipulator = 0; imanipulator < (int)connectedBodyInfo._vManipulatorInfos.size(); ++imanipulator) {
            RobotBase::ManipulatorPtr& pnewmanipulator = connectedBody._vResolvedManipulatorNames[imanipulator].second;
            if( !pnewmanipulator ) {
                pnewmanipulator.reset(new RobotBase::Manipulator(shared_robot(), *connectedBodyInfo._vManipulatorInfos[imanipulator]));
            }
            else {
                pnewmanipulator->info_ = *connectedBodyInfo._vManipulatorInfos[imanipulator];
            }
            pnewmanipulator->info_.name_ = connectedBody._nameprefix + pnewmanipulator->info_.name_;

            FOREACH(ittestmanipulator, _vecManipulators) {
                if( pnewmanipulator->info_.name_ == (*ittestmanipulator)->GetName() ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, got resolved manipulator with same name %s!", connectedBody.GetName()%GetName()%pnewmanipulator->info_.name_, ORE_InvalidArguments);
                }
            }

            {
                LinkPtr pArmBaseLink = !GetLinks().empty() ? GetLinks()[0] : LinkPtr();
                if( !pArmBaseLink ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for manipulator %s, could not find a base link of the robot.", connectedBody.GetName()%GetName()%pnewmanipulator->info_.name_, ORE_InvalidArguments);
                }
                pnewmanipulator->info_._sBaseLinkName = pArmBaseLink->info_.name_;
            }

            // search for the correct resolved _sEffectorLinkName
            bool bFoundEffectorLink = false;
            for(size_t ilink = 0; ilink < connectedBodyInfo._vLinkInfos.size(); ++ilink) {
                if( pnewmanipulator->info_._sEffectorLinkName == connectedBodyInfo._vLinkInfos[ilink]->name_ ) {
                    pnewmanipulator->info_._sEffectorLinkName = connectedBody._vResolvedLinkNames.at(ilink).first;
                    bFoundEffectorLink = true;
                }
            }

            if( !bFoundEffectorLink ) {
                throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for manipulator %s, could not find linkname1 %s in connected body link infos!", connectedBody.GetName()%GetName()%pnewmanipulator->info_.name_%pnewmanipulator->info_._sEffectorLinkName, ORE_InvalidArguments);
            }

            _vecManipulators.push_back(pnewmanipulator);
            connectedBody._vResolvedManipulatorNames[imanipulator].first = pnewmanipulator->info_.name_;
        }

        // AttachedSensors
        connectedBody._vResolvedAttachedSensorNames.resize(connectedBodyInfo._vAttachedSensorInfos.size());
        for(int iattachedsensor = 0; iattachedsensor < (int)connectedBodyInfo._vAttachedSensorInfos.size(); ++iattachedsensor) {
            RobotBase::AttachedSensorPtr& pnewattachedSensor = connectedBody._vResolvedAttachedSensorNames[iattachedsensor].second;
            if( !pnewattachedSensor ) {
                pnewattachedSensor.reset(new RobotBase::AttachedSensor(shared_robot(), *connectedBodyInfo._vAttachedSensorInfos[iattachedsensor]));
            }
            else {
                pnewattachedSensor->info_ = *connectedBodyInfo._vAttachedSensorInfos[iattachedsensor];
            }
            pnewattachedSensor->info_.name_ = connectedBody._nameprefix + pnewattachedSensor->info_.name_;

            FOREACH(ittestattachedSensor, _vecAttachedSensors) {
                if( pnewattachedSensor->info_.name_ == (*ittestattachedSensor)->GetName() ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, got resolved attachedSensor with same name %s!", connectedBody.GetName()%GetName()%pnewattachedSensor->info_.name_, ORE_InvalidArguments);
                }
            }

            // search for the correct resolved _linkname and _sEffectorLinkName
            bool bFoundLink = false;
            for(size_t ilink = 0; ilink < connectedBodyInfo._vLinkInfos.size(); ++ilink) {
                if( pnewattachedSensor->info_._linkname == connectedBodyInfo._vLinkInfos[ilink]->name_ ) {
                    pnewattachedSensor->info_._linkname = connectedBody._vResolvedLinkNames.at(ilink).first;
                    bFoundLink = true;
                }
            }

            if( !bFoundLink ) {
                throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for attachedSensor %s, could not find linkname0 %s in connected body link infos!", connectedBody.GetName()%GetName()%pnewattachedSensor->info_.name_%pnewattachedSensor->info_._linkname, ORE_InvalidArguments);
            }

            _vecAttachedSensors.push_back(pnewattachedSensor);
            connectedBody._vResolvedAttachedSensorNames[iattachedsensor].first = pnewattachedSensor->info_.name_;
        }

        connectedBody._dummyPassiveJointName = connectedBody._nameprefix + "_dummyconnectedbody__";

        if( !connectedBody._pDummyJointCache ) {
            connectedBody._pDummyJointCache.reset(new KinBody::Joint(shared_kinbody()));
        }
        KinBody::JointInfo& dummyJointInfo = connectedBody._pDummyJointCache->info_;
        dummyJointInfo.name_ = connectedBody._dummyPassiveJointName;
        dummyJointInfo.is_active_ = false;
        dummyJointInfo.type_ = KinBody::JointType::JointPrismatic;
        dummyJointInfo.max_accelerate_vector_[0] = 0.0;
        dummyJointInfo.max_velocity_vector_[0] = 0.0;
        dummyJointInfo.upper_limit_vector_[0] = 0;

        dummyJointInfo._linkname0 = connectedBodyInfo._linkname;
        dummyJointInfo._linkname1 = connectedBody._vResolvedLinkNames.at(0).first;
        _InitAndAddJoint(connectedBody._pDummyJointCache);
    }
}

void RobotBase::_DeinitializeConnectedBodiesInformation()
{
    if (_vecConnectedBodies.empty()) {
        return;
    }

    std::vector<uint8_t> vConnectedLinks; vConnectedLinks.resize(_veclinks.size(),0);
    std::vector<uint8_t> vConnectedJoints; vConnectedJoints.resize(_vecjoints.size(),0);
    std::vector<uint8_t> vConnectedPassiveJoints; vConnectedPassiveJoints.resize(_vPassiveJoints.size(),0);

    // have to remove any of the added links
    FOREACH(itconnectedBody, _vecConnectedBodies) {
        ConnectedBody& connectedBody = **itconnectedBody;

        // unfortunately cannot save info as easily since have to restore origin and names
        for(int iresolvedlink = 0; iresolvedlink < (int)connectedBody._vResolvedLinkNames.size(); ++iresolvedlink) {
            LinkPtr presolvedlink = GetLink(connectedBody._vResolvedLinkNames[iresolvedlink].first);
            if( !!presolvedlink ) {
                vConnectedLinks.at(presolvedlink->GetIndex()) = 1;
            }
            connectedBody._vResolvedLinkNames[iresolvedlink].first.clear();
        }

        for(int iresolvedjoint = 0; iresolvedjoint < (int)connectedBody._vResolvedJointNames.size(); ++iresolvedjoint) {
            for(int ijointindex = 0; ijointindex < (int)_vecjoints.size(); ++ijointindex) {
                if( _vecjoints[ijointindex]->GetName() == connectedBody._vResolvedJointNames[iresolvedjoint].first ) {
                    vConnectedJoints[ijointindex] = 1;
                }
            }
            for(int ijointindex = 0; ijointindex < (int)_vPassiveJoints.size(); ++ijointindex) {
                if( _vPassiveJoints[ijointindex]->GetName() == connectedBody._vResolvedJointNames[iresolvedjoint].first ) {
                    vConnectedPassiveJoints[ijointindex] = 1;
                }
            }
            connectedBody._vResolvedJointNames[iresolvedjoint].first.clear();
        }

        for(int iresolvedmanipulator = 0; iresolvedmanipulator < (int)connectedBody._vResolvedManipulatorNames.size(); ++iresolvedmanipulator) {
            ManipulatorPtr presolvedmanipulator = GetManipulator(connectedBody._vResolvedManipulatorNames[iresolvedmanipulator].first);
            if( !!presolvedmanipulator ) {
                RemoveManipulator(presolvedmanipulator);
            }
            connectedBody._vResolvedManipulatorNames[iresolvedmanipulator].first.clear();
        }

        for(int iresolvedattachedSensor = 0; iresolvedattachedSensor < (int)connectedBody._vResolvedAttachedSensorNames.size(); ++iresolvedattachedSensor) {
            AttachedSensorPtr presolvedattachedSensor = GetAttachedSensor(connectedBody._vResolvedAttachedSensorNames[iresolvedattachedSensor].first);
            if( !!presolvedattachedSensor ) {
                RemoveAttachedSensor(*presolvedattachedSensor);
            }
            connectedBody._vResolvedAttachedSensorNames[iresolvedattachedSensor].first.clear();
        }

        for(int ijointindex = 0; ijointindex < (int)_vPassiveJoints.size(); ++ijointindex) {
            if( _vPassiveJoints[ijointindex]->GetName() == connectedBody._dummyPassiveJointName ) {
                vConnectedPassiveJoints[ijointindex] = 1;
            }
        }
        connectedBody._dummyPassiveJointName.clear();
    }

    int iwritelink = 0;
    for(int ireadlink = 0; ireadlink < (int)vConnectedLinks.size(); ++ireadlink) {
        if( !vConnectedLinks[ireadlink] ) {
            // preserve as original
            _veclinks[iwritelink++] = _veclinks[ireadlink];
        }
    }
    _veclinks.resize(iwritelink);

    int iwritejoint = 0;
    for(int ireadjoint = 0; ireadjoint < (int)vConnectedJoints.size(); ++ireadjoint) {
        if( !vConnectedJoints[ireadjoint] ) {
            // preserve as original
            _vecjoints[iwritejoint++] = _vecjoints[ireadjoint];
        }
    }
    _vecjoints.resize(iwritejoint);

    int iwritepassiveJoint = 0;
    for(int ireadpassiveJoint = 0; ireadpassiveJoint < (int)vConnectedPassiveJoints.size(); ++ireadpassiveJoint) {
        if( !vConnectedPassiveJoints[ireadpassiveJoint] ) {
            // preserve as original
            _vPassiveJoints[iwritepassiveJoint++] = _vPassiveJoints[ireadpassiveJoint];
        }
    }
    _vPassiveJoints.resize(iwritepassiveJoint);
}

void RobotBase::GetConnectedBodyActiveStates(std::vector<uint8_t>& activestates) const
{
    activestates.resize(_vecConnectedBodies.size());
    for(size_t iconnectedbody = 0; iconnectedbody < _vecConnectedBodies.size(); ++iconnectedbody) {
        activestates[iconnectedbody] = _vecConnectedBodies[iconnectedbody]->IsActive();
    }
}

void RobotBase::SetConnectedBodyActiveStates(const std::vector<uint8_t>& activestates)
{
    OPENRAVE_ASSERT_OP(activestates.size(),==,_vecConnectedBodies.size());
    for(size_t iconnectedbody = 0; iconnectedbody < _vecConnectedBodies.size(); ++iconnectedbody) {
        _vecConnectedBodies[iconnectedbody]->SetActive(!!activestates[iconnectedbody]);
    }
}

} // end namespace OpenRAVE
