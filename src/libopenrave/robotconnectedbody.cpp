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

	RobotBase::ConnectedBodyInfo::ConnectedBodyInfo() : is_active_(false)
	{
	}

	void RobotBase::ConnectedBodyInfo::InitInfoFromBody(RobotBase& robot)
	{
		link_infos_vector_.clear();
		joint_infos_vector_.clear();
		manipulator_infos_vector_.clear();
		attached_sensor_infos_vector_.clear();
		_vGripperInfos.clear();

		// have to set to the identity before extracting info
		KinBody::KinBodyStateSaverRef statesaver(robot, Save_LinkTransformation);
		std::vector<dReal> vzeros(robot.GetDOF());
		robot.SetDOFValues(vzeros, Transform());

		FOREACH(itlink, robot.links_vector_) {
			link_infos_vector_.push_back(std::make_shared<KinBody::LinkInfo>((*itlink)->UpdateAndGetInfo()));
		}

		FOREACH(itjoint, robot.joints_vector_) {
			joint_infos_vector_.push_back(std::make_shared<KinBody::JointInfo>((*itjoint)->UpdateAndGetInfo()));
		}
		FOREACH(itjoint, robot.passive_joints_vector_) {
			joint_infos_vector_.push_back(std::make_shared<KinBody::JointInfo>((*itjoint)->UpdateAndGetInfo()));
		}

		FOREACH(itmanip, robot.GetManipulators()) {
			manipulator_infos_vector_.push_back(std::make_shared<RobotBase::ManipulatorInfo>((*itmanip)->GetInfo()));
		}

		FOREACH(itattachedsensor, robot.GetAttachedSensors()) {
			attached_sensor_infos_vector_.push_back(std::make_shared<RobotBase::AttachedSensorInfo>((*itattachedsensor)->UpdateAndGetInfo()));
		}
		FOREACH(itGripperInfo, robot.GetGripperInfos()) {
			RobotBase::GripperInfoPtr pGripperInfo(new RobotBase::GripperInfo());
			*pGripperInfo = **itGripperInfo;
			_vGripperInfos.push_back(pGripperInfo);
		}
	}

	void RobotBase::ConnectedBodyInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal unit_scale, int options) const
	{
		openravejson::SetJsonValueByKey(value, "name", name_, allocator);
		openravejson::SetJsonValueByKey(value, "linkName", link_name_, allocator);
		openravejson::SetJsonValueByKey(value, "uri", uri_, allocator);
		openravejson::SetJsonValueByKey(value, "transform", relative_transform_, allocator);

		// rapidjson::Value linkInfosValue;
		// linkInfosValue.SetArray();
		// FOREACH(it, link_infos_vector_)
		// {
		//     rapidjson::Value info;
		//     (*it)->SerializeJSON(info, allocator, options);
		//     linkInfosValue.PushBack(info, allocator);
		// }
		// value.AddMember("links", linkInfosValue, allocator);

		// rapidjson::Value jointInfosValue;
		// jointInfosValue.SetArray();
		// FOREACH(it, joint_infos_vector_)
		// {
		//     rapidjson::Value v;
		//     (*it)->SerializeJSON(v, allocator, options);
		//     jointInfosValue.PushBack(v, allocator);
		// }
		// value.AddMember("joints", jointInfosValue, allocator);

		// rapidjson::Value manipulatorInfosValue;
		// manipulatorInfosValue.SetArray();
		// FOREACH(it, manipulator_infos_vector_)
		// {
		//     rapidjson::Value info;
		//     (*it)->SerializeJSON(info, allocator, options);
		//     manipulatorInfosValue.PushBack(info, allocator);
		// }
		// value.AddMember("manipulators", manipulatorInfosValue, allocator);

		// rapidjson::Value attachedSensorInfosValue;
		// attachedSensorInfosValue.SetArray();
		// FOREACH(it, attached_sensor_infos_vector_)
		// {
		//     rapidjson::Value info;
		//     (*it)->SerializeJSON(info, allocator, options);
		//     attachedSensorInfosValue.PushBack(info, allocator);
		// }
		// value.AddMember("attachedSensors", attachedSensorInfosValue, allocator);


		rapidjson::Value rGripperInfos;
		rGripperInfos.SetArray();
		FOREACH(it, _vGripperInfos)
		{
			rapidjson::Value info;
			(*it)->SerializeJSON(info, allocator, options);
			rGripperInfos.PushBack(info, allocator);
		}
		value.AddMember("gripperInfos", rGripperInfos, allocator);
		openravejson::SetJsonValueByKey(value, "isActive", is_active_, allocator);
	}

	void RobotBase::ConnectedBodyInfo::DeserializeJSON(const rapidjson::Value &value, dReal unit_scale)
	{
		openravejson::LoadJsonValueByKey(value, "name", name_);
		openravejson::LoadJsonValueByKey(value, "linkName", link_name_);
		openravejson::LoadJsonValueByKey(value, "uri", uri_);
		openravejson::LoadJsonValueByKey(value, "transform", relative_transform_);

		if (value.HasMember("links")) {
			link_infos_vector_.clear();
			link_infos_vector_.reserve(value["links"].Size());
			for (size_t i = 0; i < value["links"].Size(); ++i) {
				LinkInfoPtr linkinfo(new LinkInfo());
				linkinfo->DeserializeJSON(value["links"][i]);
				link_infos_vector_.push_back(linkinfo);
			}
		}

		if (value.HasMember("joints")) {
			joint_infos_vector_.clear();
			joint_infos_vector_.reserve(value["joints"].Size());
			for (size_t i = 0; i < value["joints"].Size(); ++i) {
				JointInfoPtr jointinfo(new JointInfo());
				jointinfo->DeserializeJSON(value["joints"][i]);
				joint_infos_vector_.push_back(jointinfo);
			}
		}

		if (value.HasMember("manipulators")) {
			manipulator_infos_vector_.clear();
			manipulator_infos_vector_.reserve(value["manipulators"].Size());
			for (size_t i = 0; i < value["manipulators"].Size(); ++i) {
				ManipulatorInfoPtr manipulatorinfo(new ManipulatorInfo());
				manipulatorinfo->DeserializeJSON(value["manipulators"][i]);
				manipulator_infos_vector_.push_back(manipulatorinfo);
			}
		}

		if (value.HasMember("attachedSensors")) {
			attached_sensor_infos_vector_.clear();
			attached_sensor_infos_vector_.reserve(value["attachedSensors"].Size());
			for (size_t i = 0; i < value["attachedSensors"].Size(); ++i) {
				AttachedSensorInfoPtr attachedsensorinfo(new AttachedSensorInfo());
				attachedsensorinfo->DeserializeJSON(value["attachedSensors"][i]);
				attached_sensor_infos_vector_.push_back(attachedsensorinfo);
			}
		}

		if (value.HasMember("gripperInfos")) {
			_vGripperInfos.clear();
			_vGripperInfos.reserve(value["gripperInfos"].Size());
			for (size_t igripperInfo = 0; igripperInfo < value["gripperInfos"].Size(); ++igripperInfo) {
				GripperInfoPtr gripperInfo(new GripperInfo());
				gripperInfo->DeserializeJSON(value["gripperInfos"][igripperInfo]);
				_vGripperInfos.push_back(gripperInfo);
			}
		}

		openravejson::LoadJsonValueByKey(value, "isActive", is_active_);
	}

	RobotBase::ConnectedBody::ConnectedBody(OpenRAVE::RobotBasePtr probot) : attached_robot_(probot)
	{
	}

	RobotBase::ConnectedBody::ConnectedBody(OpenRAVE::RobotBasePtr probot, const OpenRAVE::RobotBase::ConnectedBodyInfo &info)
		: info_(info), attached_robot_(probot)
	{
		if (!!probot) {
			LinkPtr attachedLink = probot->GetLink(info_.link_name_);
			if (!attachedLink) {
				throw OPENRAVE_EXCEPTION_FORMAT("Link \"%s\" to which ConnectedBody %s is attached does not exist in robot %s", info.link_name_%GetName() % probot->GetName(), ORE_InvalidArguments);
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
		FOREACH(itinfo, _vResolvedGripperInfoNames) {
			itinfo->second = probot->GetGripperInfo(itinfo->first);
		}
		attached_robot_ = probot;
		_pattachedlink = probot->GetLink(LinkPtr(connectedBody._pattachedlink)->GetName());
	}

	RobotBase::ConnectedBody::~ConnectedBody()
	{
	}

	bool RobotBase::ConnectedBody::SetActive(bool active)
	{
		if (info_.is_active_ == active) {
			return false;
		}

		RobotBasePtr pattachedrobot = attached_robot_.lock();
		if (!!pattachedrobot) {
			if (pattachedrobot->hierarchy_computed_ != 0) {
				throw OPENRAVE_EXCEPTION_FORMAT("Cannot set ConnectedBody %s active to %s since robot %s is still in the environment", info_.name_%active%pattachedrobot->GetName(), ORE_InvalidState);
			}
		}
		info_.is_active_ = active;
		return true; // changed
	}

	bool RobotBase::ConnectedBody::IsActive()
	{
		return info_.is_active_;
	}

	void RobotBase::ConnectedBody::SetLinkEnable(bool benable)
	{
		RobotBasePtr pattachedrobot = attached_robot_.lock();
		if (!!pattachedrobot) {
			std::vector<uint8_t> enablestates;
			pattachedrobot->GetLinkEnableStates(enablestates);
			bool bchanged = false;

			FOREACH(itlinkname, _vResolvedLinkNames) {
				KinBody::LinkPtr plink = pattachedrobot->GetLink(itlinkname->first);
				if (!!plink) {
					if (enablestates.at(plink->GetIndex()) != benable) {
						enablestates.at(plink->GetIndex()) = benable;
						bchanged = true;
					}
				}
			}

			if (bchanged) {
				pattachedrobot->SetLinkEnableStates(enablestates);
			}
		}
	}

	void RobotBase::ConnectedBody::SetLinkVisible(bool bvisible)
	{
		RobotBasePtr pattachedrobot = attached_robot_.lock();
		if (!!pattachedrobot) {
			FOREACH(itlinkname, _vResolvedLinkNames) {
				KinBody::LinkPtr plink = pattachedrobot->GetLink(itlinkname->first);
				if (!!plink) {
					plink->SetVisible(bvisible);
				}
			}
		}
	}

	void RobotBase::ConnectedBody::GetResolvedLinks(std::vector<KinBody::LinkPtr>& links)
	{
		links.resize(_vResolvedLinkNames.size());
		RobotBasePtr pattachedrobot = attached_robot_.lock();
		if (!!pattachedrobot) {
			for (size_t ilink = 0; ilink < _vResolvedLinkNames.size(); ++ilink) {
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
		RobotBasePtr pattachedrobot = attached_robot_.lock();
		if (!!pattachedrobot) {
			for (size_t ijoint = 0; ijoint < _vResolvedJointNames.size(); ++ijoint) {
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
		RobotBasePtr pattachedrobot = attached_robot_.lock();
		if (!!pattachedrobot) {
			for (size_t imanipulator = 0; imanipulator < _vResolvedManipulatorNames.size(); ++imanipulator) {
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
		RobotBasePtr pattachedrobot = attached_robot_.lock();
		if (!!pattachedrobot) {
			for (size_t iattachedSensor = 0; iattachedSensor < _vResolvedAttachedSensorNames.size(); ++iattachedSensor) {
				attachedSensors[iattachedSensor] = pattachedrobot->GetAttachedSensor(_vResolvedAttachedSensorNames[iattachedSensor].first);
			}
		}
		else {
			FOREACH(itattachedSensor, attachedSensors) {
				itattachedSensor->reset();
			}
		}
	}

	void RobotBase::ConnectedBody::GetResolvedGripperInfos(std::vector<RobotBase::GripperInfoPtr>& gripperInfos)
	{
		gripperInfos.resize(_vResolvedGripperInfoNames.size());
		RobotBasePtr pattachedrobot = attached_robot_.lock();
		if (!!pattachedrobot) {
			for (size_t igripperInfo = 0; igripperInfo < _vResolvedGripperInfoNames.size(); ++igripperInfo) {
				gripperInfos[igripperInfo] = pattachedrobot->GetGripperInfo(_vResolvedGripperInfoNames[igripperInfo].first);
			}
		}
		else {
			FOREACH(itgripperInfo, gripperInfos) {
				itgripperInfo->reset();
			}
		}
	}

	RobotBase::ConnectedBodyPtr RobotBase::AddConnectedBody(const RobotBase::ConnectedBodyInfo& connectedBodyInfo, bool removeduplicate)
	{
		if (hierarchy_computed_ != 0) {
			throw OPENRAVE_EXCEPTION_FORMAT("Cannot add connected body while robot %s is added to the environment", GetName(), ORE_InvalidState);
		}

		OPENRAVE_ASSERT_OP(connectedBodyInfo.name_.size(), > , 0);
		int iremoveindex = -1;
		for (int iconnectedbody = 0; iconnectedbody < (int)connected_bodies_vector_.size(); ++iconnectedbody) {
			if (connected_bodies_vector_[iconnectedbody]->GetName() == connectedBodyInfo.name_) {
				if (removeduplicate) {
					iremoveindex = iconnectedbody;
					break;
				}
				else {
					throw OPENRAVE_EXCEPTION_FORMAT(_tr("attached sensor with name %s already exists"), connectedBodyInfo.name_, ORE_InvalidArguments);
				}
			}
		}
		ConnectedBodyPtr newConnectedBody(new ConnectedBody(shared_robot(), connectedBodyInfo));
		//    if( hierarchy_computed_ ) {
		//        newConnectedBody->_ComputeInternalInformation();
		//    }
		if (iremoveindex >= 0) {
			// replace the old one
			connected_bodies_vector_[iremoveindex] = newConnectedBody;
		}
		else {
			connected_bodies_vector_.push_back(newConnectedBody);
		}
		//newConnectedBody->UpdateInfo(); // just in case
		hash_robot_structure_.resize(0);
		return newConnectedBody;
	}

	RobotBase::ConnectedBodyPtr RobotBase::GetConnectedBody(const std::string& name) const
	{
		FOREACHC(itconnectedbody, connected_bodies_vector_) {
			if ((*itconnectedbody)->GetName() == name) {
				return *itconnectedbody;
			}
		}
		return RobotBase::ConnectedBodyPtr();
	}

	bool RobotBase::RemoveConnectedBody(RobotBase::ConnectedBody &connectedBody)
	{
		FOREACH(itconnectedBody, connected_bodies_vector_) {
			if (itconnectedBody->get() == &connectedBody) {
				connected_bodies_vector_.erase(itconnectedBody);
				hash_robot_structure_.clear();
				return true;
			}
		}
		return false;
	}

	void RobotBase::_ComputeConnectedBodiesInformation()
	{
		// resolve duplicate names for links and joints in connected body info
		// reinitialize robot with combined infos
		if (connected_bodies_vector_.empty()) {
			return;
		}

		// should have already done adding the necessary link etc
		// during cloning, we should not add links and joints again
		if (hierarchy_computed_ != 0) {
			return;
		}

		FOREACH(itconnectedBody, connected_bodies_vector_) {
			ConnectedBody& connectedBody = **itconnectedBody;
			const ConnectedBodyInfo& connectedBodyInfo = connectedBody.info_;

			if (!connectedBody.GetAttachingLink()) {
				throw OPENRAVE_EXCEPTION_FORMAT("ConnectedBody %s for robot %s does not have a valid pointer to link %s", connectedBody.GetName() % GetName() % connectedBodyInfo.link_name_, ORE_InvalidArguments);
			}

			Transform tBaseLinkInWorld = connectedBody.GetTransform(); // transform all links and joints by this

			if (connectedBody.GetName().size() == 0) {
				throw OPENRAVE_EXCEPTION_FORMAT("ConnectedBody %s attached to link %s has no name initialized", connectedBodyInfo.uri_%connectedBodyInfo.link_name_, ORE_InvalidArguments);
			}

			vector<ConnectedBodyPtr>::iterator itconnectedBody2 = itconnectedBody; ++itconnectedBody2;
			for (; itconnectedBody2 != connected_bodies_vector_.end(); ++itconnectedBody2) {
				if (connectedBody.GetName() == (*itconnectedBody2)->GetName()) {
					throw OPENRAVE_EXCEPTION_FORMAT("robot %s has two ConnectedBody with the same name %s!", GetName() % connectedBody.GetName(), ORE_InvalidArguments);
				}
			}

			if (!connectedBody.IsActive()) {
				// skip
				continue;
			}

			if (connectedBodyInfo.link_infos_vector_.size() == 0) {
				RAVELOG_WARN_FORMAT("ConnectedBody %s for robot %s has no link infos, so cannot add anything", connectedBody.GetName() % GetName());
				continue;
			}

			// check if connectedBodyInfo.link_name_ exists
			bool is_exists = false;
			FOREACH(ittestlink, links_vector_) {
				if (connectedBodyInfo.link_name_ == (*ittestlink)->GetName()) {
					is_exists = true;
					break;
				}
			}
			if (!is_exists) {
				throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, the attaching link '%s' on robot does not exist!", connectedBody.GetName() % GetName() % connectedBodyInfo.link_name_, ORE_InvalidArguments);
			}

			connectedBody._nameprefix = connectedBody.GetName() + "_";

			// Links
			connectedBody._vResolvedLinkNames.resize(connectedBodyInfo.link_infos_vector_.size());
			for (int ilink = 0; ilink < (int)connectedBodyInfo.link_infos_vector_.size(); ++ilink) {
				KinBody::LinkPtr& plink = connectedBody._vResolvedLinkNames[ilink].second;
				if (!plink) {
					plink.reset(new KinBody::Link(shared_kinbody()));
				}
				plink->info_ = *connectedBodyInfo.link_infos_vector_[ilink]; // copy
				plink->info_.name_ = connectedBody._nameprefix + plink->info_.name_;
				plink->info_.transform_ = tBaseLinkInWorld * plink->info_.transform_;
				_InitAndAddLink(plink);
				connectedBody._vResolvedLinkNames[ilink].first = plink->info_.name_;
			}

			// Joints
			std::vector<KinBody::JointPtr> vNewJointsToAdd;
			std::vector<std::pair<std::string, std::string> > jointNamePairs;
			connectedBody._vResolvedJointNames.resize(connectedBodyInfo.joint_infos_vector_.size());
			for (int ijoint = 0; ijoint < (int)connectedBodyInfo.joint_infos_vector_.size(); ++ijoint) {
				KinBody::JointPtr& pjoint = connectedBody._vResolvedJointNames[ijoint].second;
				if (!pjoint) {
					pjoint.reset(new KinBody::Joint(shared_kinbody()));
				}
				pjoint->info_ = *connectedBodyInfo.joint_infos_vector_[ijoint]; // copy
				pjoint->info_.name_ = connectedBody._nameprefix + pjoint->info_.name_;

				// search for the correct resolved link_name0_ and link_name1_
				bool bfoundlink0 = false, bfoundlink1 = false;
				for (size_t ilink = 0; ilink < connectedBodyInfo.link_infos_vector_.size(); ++ilink) {
					if (pjoint->info_.link_name0_ == connectedBodyInfo.link_infos_vector_[ilink]->name_) {
						pjoint->info_.link_name0_ = connectedBody._vResolvedLinkNames.at(ilink).first;
						bfoundlink0 = true;
					}
					if (pjoint->info_.link_name1_ == connectedBodyInfo.link_infos_vector_[ilink]->name_) {
						pjoint->info_.link_name1_ = connectedBody._vResolvedLinkNames.at(ilink).first;
						bfoundlink1 = true;
					}
				}

				if (!bfoundlink0) {
					throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for joint %s, could not find linkname0 %s in connected body link infos!", connectedBody.GetName() % GetName() % pjoint->info_.name_%pjoint->info_.link_name0_, ORE_InvalidArguments);
				}
				if (!bfoundlink1) {
					throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for joint %s, could not find linkname1 %s in connected body link infos!", connectedBody.GetName() % GetName() % pjoint->info_.name_%pjoint->info_.link_name1_, ORE_InvalidArguments);
				}
				jointNamePairs.emplace_back(connectedBodyInfo.joint_infos_vector_[ijoint]->name_, pjoint->info_.name_);
				vNewJointsToAdd.push_back(pjoint);
				connectedBody._vResolvedJointNames[ijoint].first = pjoint->info_.name_;
			}

			FOREACH(itnewjoint, vNewJointsToAdd) {
				KinBody::JointInfo& jointinfo = (*itnewjoint)->info_;
				FOREACH(itmimic, jointinfo._vmimic) {
					if (!(*itmimic)) {
						continue;
					}
					for (std::size_t iequation = 0; iequation < (*itmimic)->equations_.size(); ++iequation) {
						std::string eq;
						utils::SearchAndReplace(eq, (*itmimic)->equations_[iequation], jointNamePairs);
						(*itmimic)->equations_[iequation] = eq;
					}
				}

				_InitAndAddJoint(*itnewjoint);
			}

			// Manipulators
			connectedBody._vResolvedManipulatorNames.resize(connectedBodyInfo.manipulator_infos_vector_.size());
			for (int imanipulator = 0; imanipulator < (int)connectedBodyInfo.manipulator_infos_vector_.size(); ++imanipulator) {
				RobotBase::ManipulatorPtr& pnewmanipulator = connectedBody._vResolvedManipulatorNames[imanipulator].second;
				if (!pnewmanipulator) {
					pnewmanipulator.reset(new RobotBase::Manipulator(shared_robot(), *connectedBodyInfo.manipulator_infos_vector_[imanipulator]));
				}
				else {
					pnewmanipulator->info_ = *connectedBodyInfo.manipulator_infos_vector_[imanipulator];
				}
				pnewmanipulator->info_.name_ = connectedBody._nameprefix + pnewmanipulator->info_.name_;

				FOREACH(ittestmanipulator, manipulators_vector_) {
					if (pnewmanipulator->info_.name_ == (*ittestmanipulator)->GetName()) {
						throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, got resolved manipulator with same name %s!", connectedBody.GetName() % GetName() % pnewmanipulator->info_.name_, ORE_InvalidArguments);
					}
				}

				{
					LinkPtr pArmBaseLink = !GetLinks().empty() ? GetLinks()[0] : LinkPtr();
					if (!pArmBaseLink) {
						throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for manipulator %s, could not find a base link of the robot.", connectedBody.GetName() % GetName() % pnewmanipulator->info_.name_, ORE_InvalidArguments);
					}
					pnewmanipulator->info_.base_link_name_ = pArmBaseLink->info_.name_;
				}

				// search for the correct resolved effector_link_name_
				bool bFoundEffectorLink = false;
				for (size_t ilink = 0; ilink < connectedBodyInfo.link_infos_vector_.size(); ++ilink) {
					if (pnewmanipulator->info_.effector_link_name_ == connectedBodyInfo.link_infos_vector_[ilink]->name_) {
						pnewmanipulator->info_.effector_link_name_ = connectedBody._vResolvedLinkNames.at(ilink).first;
						bFoundEffectorLink = true;
					}
				}

				if (!bFoundEffectorLink) {
					throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for manipulator %s, could not find linkname1 %s in connected body link infos!", connectedBody.GetName() % GetName() % pnewmanipulator->info_.name_%pnewmanipulator->info_.effector_link_name_, ORE_InvalidArguments);
				}

				// search for the correct resolved joint name
				for (size_t iGripperJoint = 0; iGripperJoint < pnewmanipulator->info_.gripper_joint_names_vector_.size(); ++iGripperJoint) {
					std::string& gripperJointName = pnewmanipulator->info_.gripper_joint_names_vector_[iGripperJoint];
					bool bFoundJoint = false;
					for (size_t ijoint = 0; ijoint < connectedBodyInfo.joint_infos_vector_.size(); ++ijoint) {
						if (gripperJointName == connectedBodyInfo.joint_infos_vector_[ijoint]->name_) {
							gripperJointName = connectedBody._vResolvedJointNames.at(ijoint).first;
							bFoundJoint = true;
						}
					}

					if (!bFoundJoint) {
						throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for Manipulator %s, could not find joint %s in connected body joint infos!",
							connectedBody.GetName() % GetName() % pnewmanipulator->info_.name_%gripperJointName, ORE_InvalidArguments);
					}
				}

				manipulators_vector_.push_back(pnewmanipulator);
				connectedBody._vResolvedManipulatorNames[imanipulator].first = pnewmanipulator->info_.name_;
			}

			// AttachedSensors
			connectedBody._vResolvedAttachedSensorNames.resize(connectedBodyInfo.attached_sensor_infos_vector_.size());
			for (int iattachedsensor = 0; iattachedsensor < (int)connectedBodyInfo.attached_sensor_infos_vector_.size(); ++iattachedsensor) {
				RobotBase::AttachedSensorPtr& pnewattachedSensor = connectedBody._vResolvedAttachedSensorNames[iattachedsensor].second;
				if (!pnewattachedSensor) {
					pnewattachedSensor.reset(new RobotBase::AttachedSensor(shared_robot(), *connectedBodyInfo.attached_sensor_infos_vector_[iattachedsensor]));
				}
				else {
					pnewattachedSensor->info_ = *connectedBodyInfo.attached_sensor_infos_vector_[iattachedsensor];
				}
				pnewattachedSensor->info_.name_ = connectedBody._nameprefix + pnewattachedSensor->info_.name_;

				FOREACH(ittestattachedSensor, attached_sensors_vector_) {
					if (pnewattachedSensor->info_.name_ == (*ittestattachedSensor)->GetName()) {
						throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, got resolved attachedSensor with same name %s!", connectedBody.GetName() % GetName() % pnewattachedSensor->info_.name_, ORE_InvalidArguments);
					}
				}

				// search for the correct resolved link_name_ and effector_link_name_
				bool bFoundLink = false;
				for (size_t ilink = 0; ilink < connectedBodyInfo.link_infos_vector_.size(); ++ilink) {
					if (pnewattachedSensor->info_.link_name_ == connectedBodyInfo.link_infos_vector_[ilink]->name_) {
						pnewattachedSensor->info_.link_name_ = connectedBody._vResolvedLinkNames.at(ilink).first;
						bFoundLink = true;
					}
				}

				if (!bFoundLink) {
					throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for attachedSensor %s, could not find linkname0 %s in connected body link infos!", connectedBody.GetName() % GetName() % pnewattachedSensor->info_.name_%pnewattachedSensor->info_.link_name_, ORE_InvalidArguments);
				}

				attached_sensors_vector_.push_back(pnewattachedSensor);
				connectedBody._vResolvedAttachedSensorNames[iattachedsensor].first = pnewattachedSensor->info_.name_;
			}

			// GripperInfos
			connectedBody._vResolvedGripperInfoNames.resize(connectedBodyInfo._vGripperInfos.size());
			for (int iGripperInfo = 0; iGripperInfo < (int)connectedBodyInfo._vGripperInfos.size(); ++iGripperInfo) {
				RobotBase::GripperInfoPtr& pnewgripperInfo = connectedBody._vResolvedGripperInfoNames[iGripperInfo].second;
				if (!pnewgripperInfo) {
					pnewgripperInfo.reset(new RobotBase::GripperInfo());
				}
				*pnewgripperInfo = *connectedBodyInfo._vGripperInfos[iGripperInfo];
				pnewgripperInfo->gripperid = connectedBody._nameprefix + pnewgripperInfo->gripperid;

				FOREACH(ittestgripperInfo, _vecGripperInfos) {
					if (pnewgripperInfo->gripperid == (*ittestgripperInfo)->gripperid) {
						throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, got resolved gripperInfo with same name %s!", connectedBody.GetName() % GetName() % pnewgripperInfo->gripperid, ORE_InvalidArguments);
					}
				}

				// search for the correct resolved joint name
				for (size_t iGripperJoint = 0; iGripperJoint < pnewgripperInfo->gripperJointNames.size(); ++iGripperJoint) {
					std::string& gripperJointName = pnewgripperInfo->gripperJointNames[iGripperJoint];
					bool bFoundJoint = false;
					for (size_t ijoint = 0; ijoint < connectedBodyInfo.joint_infos_vector_.size(); ++ijoint) {
						if (gripperJointName == connectedBodyInfo.joint_infos_vector_[ijoint]->name_) {
							gripperJointName = connectedBody._vResolvedJointNames.at(ijoint).first;
							bFoundJoint = true;
						}
					}

					if (!bFoundJoint) {
						throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for gripperInfo %s, could not find joint %s in connected body joint infos!", connectedBody.GetName() % GetName() % pnewgripperInfo->gripperid%gripperJointName, ORE_InvalidArguments);
					}
				}

				_vecGripperInfos.push_back(pnewgripperInfo);
				connectedBody._vResolvedGripperInfoNames[iGripperInfo].first = pnewgripperInfo->gripperid;
			}

			connectedBody._dummyPassiveJointName = connectedBody._nameprefix + "_dummyconnectedbody__";

			if (!connectedBody._pDummyJointCache) {
				connectedBody._pDummyJointCache.reset(new KinBody::Joint(shared_kinbody()));
			}
			KinBody::JointInfo& dummyJointInfo = connectedBody._pDummyJointCache->info_;
			dummyJointInfo.name_ = connectedBody._dummyPassiveJointName;
			dummyJointInfo.is_active_ = false;
			dummyJointInfo.type_ = KinBody::JointType::JointPrismatic;
			dummyJointInfo.max_accelerate_vector_[0] = 0.0;
			dummyJointInfo.max_velocity_vector_[0] = 0.0;
			dummyJointInfo.upper_limit_vector_[0] = 0;

			dummyJointInfo.link_name0_ = connectedBodyInfo.link_name_;
			dummyJointInfo.link_name1_ = connectedBody._vResolvedLinkNames.at(0).first;
			_InitAndAddJoint(connectedBody._pDummyJointCache);
		}
	}

	void RobotBase::_DeinitializeConnectedBodiesInformation()
	{
		if (connected_bodies_vector_.empty()) {
			return;
		}

		std::vector<uint8_t> vConnectedLinks; vConnectedLinks.resize(links_vector_.size(), 0);
		std::vector<uint8_t> vConnectedJoints; vConnectedJoints.resize(joints_vector_.size(), 0);
		std::vector<uint8_t> vConnectedPassiveJoints; vConnectedPassiveJoints.resize(passive_joints_vector_.size(), 0);

		// have to remove any of the added links
		FOREACH(itconnectedBody, connected_bodies_vector_) {
			ConnectedBody& connectedBody = **itconnectedBody;

			// unfortunately cannot save info as easily since have to restore origin and names
			for (int iresolvedlink = 0; iresolvedlink < (int)connectedBody._vResolvedLinkNames.size(); ++iresolvedlink) {
				LinkPtr presolvedlink = GetLink(connectedBody._vResolvedLinkNames[iresolvedlink].first);
				if (!!presolvedlink) {
					vConnectedLinks.at(presolvedlink->GetIndex()) = 1;
				}
				connectedBody._vResolvedLinkNames[iresolvedlink].first.clear();
			}

			for (int iresolvedjoint = 0; iresolvedjoint < (int)connectedBody._vResolvedJointNames.size(); ++iresolvedjoint) {
				for (int ijointindex = 0; ijointindex < (int)joints_vector_.size(); ++ijointindex) {
					if (joints_vector_[ijointindex]->GetName() == connectedBody._vResolvedJointNames[iresolvedjoint].first) {
						vConnectedJoints[ijointindex] = 1;
					}
				}
				for (int ijointindex = 0; ijointindex < (int)passive_joints_vector_.size(); ++ijointindex) {
					if (passive_joints_vector_[ijointindex]->GetName() == connectedBody._vResolvedJointNames[iresolvedjoint].first) {
						vConnectedPassiveJoints[ijointindex] = 1;
					}
				}
				connectedBody._vResolvedJointNames[iresolvedjoint].first.clear();
			}

			for (int iresolvedmanipulator = 0; iresolvedmanipulator < (int)connectedBody._vResolvedManipulatorNames.size(); ++iresolvedmanipulator) {
				ManipulatorPtr presolvedmanipulator = GetManipulator(connectedBody._vResolvedManipulatorNames[iresolvedmanipulator].first);
				if (!!presolvedmanipulator) {
					RemoveManipulator(presolvedmanipulator);
				}
				connectedBody._vResolvedManipulatorNames[iresolvedmanipulator].first.clear();
			}

			for (int iresolvedattachedSensor = 0; iresolvedattachedSensor < (int)connectedBody._vResolvedAttachedSensorNames.size(); ++iresolvedattachedSensor) {
				AttachedSensorPtr presolvedattachedSensor = GetAttachedSensor(connectedBody._vResolvedAttachedSensorNames[iresolvedattachedSensor].first);
				if (!!presolvedattachedSensor) {
					RemoveAttachedSensor(*presolvedattachedSensor);
				}
				connectedBody._vResolvedAttachedSensorNames[iresolvedattachedSensor].first.clear();
			}

			for (int iresolvedGripperInfo = 0; iresolvedGripperInfo < (int)connectedBody._vResolvedGripperInfoNames.size(); ++iresolvedGripperInfo) {
				GripperInfoPtr presolvedGripperInfo = GetGripperInfo(connectedBody._vResolvedGripperInfoNames[iresolvedGripperInfo].first);
				if (!!presolvedGripperInfo) {
					RemoveGripperInfo(presolvedGripperInfo->gripperid);
				}
				connectedBody._vResolvedGripperInfoNames[iresolvedGripperInfo].first.clear();
			}

			for (int ijointindex = 0; ijointindex < (int)passive_joints_vector_.size(); ++ijointindex) {
				if (passive_joints_vector_[ijointindex]->GetName() == connectedBody._dummyPassiveJointName) {
					vConnectedPassiveJoints[ijointindex] = 1;
				}
			}
			connectedBody._dummyPassiveJointName.clear();
		}

		int iwritelink = 0;
		for (int ireadlink = 0; ireadlink < (int)vConnectedLinks.size(); ++ireadlink) {
			if (!vConnectedLinks[ireadlink]) {
				// preserve as original
				links_vector_[iwritelink++] = links_vector_[ireadlink];
			}
		}
		links_vector_.resize(iwritelink);

		int iwritejoint = 0;
		for (int ireadjoint = 0; ireadjoint < (int)vConnectedJoints.size(); ++ireadjoint) {
			if (!vConnectedJoints[ireadjoint]) {
				// preserve as original
				joints_vector_[iwritejoint++] = joints_vector_[ireadjoint];
			}
		}
		joints_vector_.resize(iwritejoint);

		int iwritepassiveJoint = 0;
		for (int ireadpassiveJoint = 0; ireadpassiveJoint < (int)vConnectedPassiveJoints.size(); ++ireadpassiveJoint) {
			if (!vConnectedPassiveJoints[ireadpassiveJoint]) {
				// preserve as original
				passive_joints_vector_[iwritepassiveJoint++] = passive_joints_vector_[ireadpassiveJoint];
			}
		}
		passive_joints_vector_.resize(iwritepassiveJoint);
	}

	void RobotBase::GetConnectedBodyActiveStates(std::vector<uint8_t>& activestates) const
	{
		activestates.resize(connected_bodies_vector_.size());
		for (size_t iconnectedbody = 0; iconnectedbody < connected_bodies_vector_.size(); ++iconnectedbody) {
			activestates[iconnectedbody] = connected_bodies_vector_[iconnectedbody]->IsActive();
		}
	}

	void RobotBase::SetConnectedBodyActiveStates(const std::vector<uint8_t>& activestates)
	{
		OPENRAVE_ASSERT_OP(activestates.size(), == , connected_bodies_vector_.size());
		for (size_t iconnectedbody = 0; iconnectedbody < connected_bodies_vector_.size(); ++iconnectedbody) {
			connected_bodies_vector_[iconnectedbody]->SetActive(!!activestates[iconnectedbody]);
		}
	}

} // end namespace OpenRAVE
