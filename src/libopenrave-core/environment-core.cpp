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
	Environment::Environment() : EnvironmentBase()
	{
		_homedirectory = RaveGetHomeDirectory();
		RAVELOG_DEBUG_FORMAT("setting openrave home directory to %s", _homedirectory);

		_nBodiesModifiedStamp = 0;
		_nEnvironmentIndex = 1;

		_fDeltaSimTime = 0.01f;
		_nCurSimTime = 0;
		_nSimStartTime = utils::GetMicroTime();
		_bRealTime = true;
		_bInit = false;
		_bEnableSimulation = true;     // need to start by default
		_unit = std::make_pair("meter", 1.0); //default unit settings

		_handlegenericrobot = RaveRegisterInterface(PT_Robot, "GenericRobot", 
			RaveGetInterfaceHash(PT_Robot), GetHash(), CreateGenericRobot);
		_handlegenerictrajectory = RaveRegisterInterface(PT_Trajectory, "GenericTrajectory", 
			RaveGetInterfaceHash(PT_Trajectory), GetHash(), CreateGenericTrajectory);
		_handlemulticontroller = RaveRegisterInterface(PT_Controller, "GenericMultiController", 
			RaveGetInterfaceHash(PT_Controller), GetHash(), CreateMultiController);
		_handlegenericphysicsengine = RaveRegisterInterface(PT_PhysicsEngine, "GenericPhysicsEngine", 
			RaveGetInterfaceHash(PT_PhysicsEngine), GetHash(), CreateGenericPhysicsEngine);
		_handlegenericcollisionchecker = RaveRegisterInterface(PT_CollisionChecker, "GenericCollisionChecker",
			RaveGetInterfaceHash(PT_CollisionChecker), GetHash(), CreateGenericCollisionChecker);
	}

	Environment::~Environment()
	{
		Destroy();
	}
//}