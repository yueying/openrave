// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "environment-core.h"

//namespace OpenRAVE
//{
	Environment::Environment() : EnvironmentBase(),
		delta_simulation_time_(0.01f),
		cur_simulation_time_(0)
	{
		home_directory_ = RaveGetHomeDirectory();
		RAVELOG_DEBUG_FORMAT("setting openrave home directory to %s", home_directory_);

		bodies_modified_stamp_ = 0;
		environment_index_ = 1;

		simulation_start_time_ = utils::GetMicroTime();
		is_real_time_ = true;
		is_init_ = false;
		is_enable_simulation_ = true;     // need to start by default
		unit_ = std::make_pair("meter", 1.0); //default unit settings

		generic_robot_handle_ = RaveRegisterInterface(PT_Robot, "GenericRobot", 
			RaveGetInterfaceHash(PT_Robot), GetHash(), CreateGenericRobot);
		generic_trajectory_handle_ = RaveRegisterInterface(PT_Trajectory, "GenericTrajectory", 
			RaveGetInterfaceHash(PT_Trajectory), GetHash(), CreateGenericTrajectory);
		multi_controller_handle_ = RaveRegisterInterface(PT_Controller, "GenericMultiController", 
			RaveGetInterfaceHash(PT_Controller), GetHash(), CreateMultiController);
		generic_physics_engine_handle_ = RaveRegisterInterface(PT_PhysicsEngine, "GenericPhysicsEngine", 
			RaveGetInterfaceHash(PT_PhysicsEngine), GetHash(), CreateGenericPhysicsEngine);
		generic_collision_checker_handle_ = RaveRegisterInterface(PT_CollisionChecker, "GenericCollisionChecker",
			RaveGetInterfaceHash(PT_CollisionChecker), GetHash(), CreateGenericCollisionChecker);
	}

	Environment::~Environment()
	{
		Destroy();
	}

	void Environment::Init(bool is_start_simulation_thread)
	{
		boost::mutex::scoped_lock lockinit(_mutexInit);
		if (is_init_) 
		{
			RAVELOG_WARN("environment is already initialized, ignoring\n");
			return;
		}

		bodies_modified_stamp_ = 0;
		environment_index_ = 1;

		delta_simulation_time_ = 0.01f;
		cur_simulation_time_ = 0;
		simulation_start_time_ = utils::GetMicroTime();
		is_real_time_ = true;
		is_enable_simulation_ = true;     // need to start by default

		if (!current_collision_checker_)
		{
			current_collision_checker_ = RaveCreateCollisionChecker(shared_from_this(), "GenericCollisionChecker");
		}
		if (!physics_engine_) 
		{
			physics_engine_ = RaveCreatePhysicsEngine(shared_from_this(), "GenericPhysicsEngine");
			_SetDefaultGravity();
		}

		// try to set init as early as possible since will be calling into user code
		is_init_ = true;

		// set a collision checker, don't call EnvironmentBase::CreateCollisionChecker
		CollisionCheckerBasePtr localchecker;

		const char* pOPENRAVE_DEFAULT_COLLISIONCHECKER = std::getenv("OPENRAVE_DEFAULT_COLLISIONCHECKER");
		if (!!pOPENRAVE_DEFAULT_COLLISIONCHECKER && strlen(pOPENRAVE_DEFAULT_COLLISIONCHECKER) > 0) 
		{
			localchecker = RaveCreateCollisionChecker(shared_from_this(), std::string(pOPENRAVE_DEFAULT_COLLISIONCHECKER));
		}

		if (!localchecker)
		{
			std::array<string, 4> checker_prefs = { { "fcl_", "ode", "bullet", "pqp"} };     // ode takes priority since bullet has some bugs with deleting bodies
			for(auto& itchecker: checker_prefs)
			{
				localchecker = RaveCreateCollisionChecker(shared_from_this(), itchecker);
				if (!!localchecker) 
				{
					break;
				}
			}
		}

		if (!localchecker) // take any collision checker
		{     
			std::map<InterfaceType, std::vector<std::string> > interfacenames;
			RaveGetLoadedInterfaces(interfacenames);
			std::map<InterfaceType, std::vector<std::string> >::const_iterator itnames
				= interfacenames.find(PT_CollisionChecker);
			if (itnames != interfacenames.end()) 
			{
				for(auto& itname: itnames->second)
				{
					localchecker = RaveCreateCollisionChecker(shared_from_this(), itname);
					if (!!localchecker) 
					{
						break;
					}
				}
			}
		}

		if (!!localchecker)
		{
			RAVELOG_DEBUG("using %s collision checker\n", localchecker->GetXMLId().c_str());
			SetCollisionChecker(localchecker);
		}
		else 
		{
			RAVELOG_WARN("failed to find any collision checker.\n");
		}

		if (is_start_simulation_thread) 
		{
			_StartSimulationThread();
		}
	}

	void Environment::Destroy()
	{
		boost::mutex::scoped_lock lockdestroy(_mutexInit);
		if (!is_init_) 
		{
			RAVELOG_VERBOSE_FORMAT("env=%d is already destroyed", GetId());
			return;
		}

		// destruction order is *very* important, don't touch it without consultation
		is_init_ = false;

		RAVELOG_VERBOSE_FORMAT("env=%d destructor", GetId());
		_StopSimulationThread();

		// destroy the modules (their destructors could attempt to lock environment, so have to do it before global lock)
		// however, do not clear the modules_list_ yet
		RAVELOG_DEBUG_FORMAT("env=%d destroy module", GetId());
		list< pair<ModuleBasePtr, std::string> > listModules;
		list<ViewerBasePtr> listViewers = viewers_list_;
		{
			boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
			listModules = modules_list_;
			listViewers = viewers_list_;
		}
		FOREACH(itmodule, listModules) {
			itmodule->first->Destroy();
		}
		listModules.clear();

		FOREACH(itviewer, listViewers) {
			// don't reset the viewer since it can already be dead
			// todo: this call could lead into a deadlock if a SIGINT got called from the viewer thread
			RAVELOG_DEBUG_FORMAT("quitting viewer %s", (*itviewer)->GetXMLId());
			(*itviewer)->quitmainloop();
		}
		listViewers.clear();

		// lock the environment
		{
			EnvironmentMutex::scoped_lock lockenv(GetMutex());
			is_enable_simulation_ = false;
			if (!!physics_engine_) {
				physics_engine_->DestroyEnvironment();
			}
			if (!!current_collision_checker_) {
				current_collision_checker_->DestroyEnvironment();
			}

			// clear internal interface lists, have to Destroy all kinbodys without locking mutex_interfaces_ since some can hold BodyCallbackData, which requires to lock mutex_interfaces_
			std::vector<RobotBasePtr> vecrobots;
			std::vector<KinBodyPtr> vecbodies;
			list<SensorBasePtr> listSensors;
			{
				boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
				vecrobots.swap(robots_vector_);
				vecbodies.swap(bodies_vector_);
				listSensors.swap(sensors_list_);
				published_bodies_vector_.clear();
				bodies_modified_stamp_++;
				modules_list_.clear();
				viewers_list_.clear();
				_listOwnedInterfaces.clear();
			}

			// destroy the dangling pointers outside of mutex_interfaces_

			// release all grabbed
			FOREACH(itrobot, vecrobots) {
				(*itrobot)->ReleaseAllGrabbed();
			}
			FOREACH(itbody, vecbodies) {
				(*itbody)->Destroy();
			}
			vecbodies.clear();
			FOREACH(itrobot, vecrobots) {
				(*itrobot)->Destroy();
			}
			vecrobots.clear();

			FOREACH(itsensor, listSensors) {
				(*itsensor)->Configure(SensorBase::CC_PowerOff);
				(*itsensor)->Configure(SensorBase::CC_RenderGeometryOff);
			}
		}

		// release all other interfaces, not necessary to hold a mutex?
		current_collision_checker_.reset();
		physics_engine_.reset();
		RAVELOG_VERBOSE("Environment destroyed\n");
	}

	void Environment::Reset()
	{
		// destruction order is *very* important, don't touch it without consultation
		list<ViewerBasePtr> listViewers;
		GetViewers(listViewers);
		if (listViewers.size() > 0) {
			RAVELOG_DEBUG("resetting raveviewer\n");
			FOREACH(itviewer, listViewers) {
				(*itviewer)->Reset();
			}
		}

		EnvironmentMutex::scoped_lock lockenv(GetMutex());

		if (!!physics_engine_) {
			physics_engine_->DestroyEnvironment();
		}
		if (!!current_collision_checker_) {
			current_collision_checker_->DestroyEnvironment();
		}
		std::vector<KinBodyPtr> vcallbackbodies;
		{
			boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
			boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);

			FOREACH(itbody, bodies_vector_) {
				(*itbody)->environment_id_ = 0;
				(*itbody)->Destroy();
			}
			if (_listRegisteredBodyCallbacks.size() > 0) {
				vcallbackbodies.insert(vcallbackbodies.end(), bodies_vector_.begin(), bodies_vector_.end());
			}
			bodies_vector_.clear();
			FOREACH(itrobot, robots_vector_) {
				(*itrobot)->environment_id_ = 0;
				(*itrobot)->Destroy();
			}
			if (_listRegisteredBodyCallbacks.size() > 0) {
				vcallbackbodies.insert(vcallbackbodies.end(), robots_vector_.begin(), robots_vector_.end());
			}
			robots_vector_.clear();
			published_bodies_vector_.clear();
			bodies_modified_stamp_++;

			_mapBodies.clear();

			FOREACH(itsensor, sensors_list_) {
				(*itsensor)->Configure(SensorBase::CC_PowerOff);
				(*itsensor)->Configure(SensorBase::CC_RenderGeometryOff);
			}
			sensors_list_.clear();
		}
		if (vcallbackbodies.size() > 0) {
			FOREACH(itbody, vcallbackbodies) {
				_CallBodyCallbacks(*itbody, 0);
			}
			vcallbackbodies.clear();
		}

		list< pair<ModuleBasePtr, std::string> > listModules;
		{
			boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
			listModules = modules_list_;
		}

		FOREACH(itmodule, listModules) {
			itmodule->first->Reset();
		}
		listModules.clear();
		_listOwnedInterfaces.clear();

		if (!!current_collision_checker_) {
			current_collision_checker_->InitEnvironment();
		}
		if (!!physics_engine_) {
			physics_engine_->InitEnvironment();
		}
	}

	bool Environment::LoadURI(const std::string& uri, const AttributesList& atts)
	{
		if (_IsColladaURI(uri))
		{
			return RaveParseColladaURI(shared_from_this(), uri, atts);
		}
		else if (_IsJSONURI(uri))
		{
			return RaveParseJSONURI(shared_from_this(), uri, atts);
		}

		RAVELOG_WARN("load failed on uri %s\n", uri.c_str());
		return false;
	}

	bool Environment::Load(const std::string& filename, const AttributesList& atts)
	{
		EnvironmentMutex::scoped_lock lockenv(GetMutex());
		OpenRAVEXMLParser::GetXMLErrorCount() = 0;
		if (_IsColladaURI(filename))
		{
			if (RaveParseColladaURI(shared_from_this(), filename, atts))
			{
				UpdatePublishedBodies();
				return true;
			}
		}
		else if (_IsColladaFile(filename))
		{
			if (RaveParseColladaFile(shared_from_this(), filename, atts))
			{
				UpdatePublishedBodies();
				return true;
			}
		}
		else if (_IsJSONFile(filename))
		{
			if (RaveParseJSONFile(shared_from_this(), filename, atts)) 
			{
				return true;
			}
		}
		else if (_IsXFile(filename))
		{
			RobotBasePtr robot;
			if (RaveParseXFile(shared_from_this(), robot, filename, atts)) 
			{
				_AddRobot(robot, true);
				UpdatePublishedBodies();
				return true;
			}
		}
		else if (!_IsOpenRAVEFile(filename) && _IsRigidModelFile(filename))
		{
			KinBodyPtr pbody = ReadKinBodyURI(KinBodyPtr(), filename, atts);
			if (!!pbody)
			{
				_AddKinBody(pbody, true);
				UpdatePublishedBodies();
				return true;
			}
		}
		else
		{
			if (_ParseXMLFile(OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), atts, true), filename)) 
			{
				if (OpenRAVEXMLParser::GetXMLErrorCount() == 0) 
				{
					UpdatePublishedBodies();
					return true;
				}
			}
		}

		RAVELOG_WARN("load failed on file %s\n", filename.c_str());
		return false;
	}

	bool Environment::LoadData(const std::string& data, const AttributesList& atts)
	{
		EnvironmentMutex::scoped_lock lockenv(GetMutex());
		if (_IsColladaData(data))
		{
			return RaveParseColladaData(shared_from_this(), data, atts);
		}
		if (_IsJSONData(data)) 
		{
			return RaveParseJSONData(shared_from_this(), data, atts);
		}
		return _ParseXMLData(OpenRAVEXMLParser::CreateEnvironmentReader(shared_from_this(), atts), data);
	}

	bool Environment::LoadJSON(const rapidjson::Document& doc, const AttributesList& atts)
	{
		EnvironmentMutex::scoped_lock lockenv(GetMutex());
		return RaveParseJSON(shared_from_this(), doc, atts);
	}




//}