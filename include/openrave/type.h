// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_TYPE_H_
#define OPENRAVE_TYPE_H_

#include <openrave/config.h>

#if defined(__GNUC__)
#define RAVE_DEPRECATED __attribute__((deprecated))
#else
#define RAVE_DEPRECATED
#endif

namespace OpenRAVE
{
	class CollisionReport;
	class InterfaceBase;
	class IkSolverBase;
	class TrajectoryBase;
	class ControllerBase;
	class PlannerBase;
	class RobotBase;
	class ModuleBase;
	class EnvironmentBase;
	class KinBody;
	class SensorSystemBase;
	class PhysicsEngineBase;
	class SensorBase;
	class CollisionCheckerBase;
	class ViewerBase;
	class SpaceSamplerBase;
	class IkParameterization;
	class ConfigurationSpecification;
	class IkReturn;

	typedef std::shared_ptr<CollisionReport> CollisionReportPtr;
	typedef std::shared_ptr<CollisionReport const> CollisionReportConstPtr;
	typedef std::shared_ptr<InterfaceBase> InterfaceBasePtr;
	typedef std::shared_ptr<InterfaceBase const> InterfaceBaseConstPtr;
	typedef std::weak_ptr<InterfaceBase> InterfaceBaseWeakPtr;
	typedef std::shared_ptr<KinBody> KinBodyPtr;
	typedef std::shared_ptr<KinBody const> KinBodyConstPtr;
	typedef std::weak_ptr<KinBody> KinBodyWeakPtr;
	typedef std::shared_ptr<RobotBase> RobotBasePtr;
	typedef std::shared_ptr<RobotBase const> RobotBaseConstPtr;
	typedef std::weak_ptr<RobotBase> RobotBaseWeakPtr;
	typedef std::shared_ptr<CollisionCheckerBase> CollisionCheckerBasePtr;
	typedef std::shared_ptr<CollisionCheckerBase const> CollisionCheckerBaseConstPtr;
	typedef std::weak_ptr<CollisionCheckerBase> CollisionCheckerBaseWeakPtr;
	typedef std::shared_ptr<ControllerBase> ControllerBasePtr;
	typedef std::shared_ptr<ControllerBase const> ControllerBaseConstPtr;
	typedef std::weak_ptr<ControllerBase> ControllerBaseWeakPtr;
	typedef std::shared_ptr<IkSolverBase> IkSolverBasePtr;
	typedef std::shared_ptr<IkSolverBase const> IkSolverBaseConstPtr;
	typedef std::weak_ptr<IkSolverBase> IkSolverBaseWeakPtr;
	typedef std::shared_ptr<PhysicsEngineBase> PhysicsEngineBasePtr;
	typedef std::shared_ptr<PhysicsEngineBase const> PhysicsEngineBaseConstPtr;
	typedef std::weak_ptr<PhysicsEngineBase> PhysicsEngineBaseWeakPtr;
	typedef std::shared_ptr<PlannerBase> PlannerBasePtr;
	typedef std::shared_ptr<PlannerBase const> PlannerBaseConstPtr;
	typedef std::weak_ptr<PlannerBase> PlannerBaseWeakPtr;
	typedef std::shared_ptr<ModuleBase> ModuleBasePtr;
	typedef std::shared_ptr<ModuleBase const> ModuleBaseConstPtr;
	typedef std::weak_ptr<ModuleBase> ModuleBaseWeakPtr;
	typedef std::shared_ptr<SensorBase> SensorBasePtr;
	typedef std::shared_ptr<SensorBase const> SensorBaseConstPtr;
	typedef std::weak_ptr<SensorBase> SensorBaseWeakPtr;
	typedef std::shared_ptr<SensorSystemBase> SensorSystemBasePtr;
	typedef std::shared_ptr<SensorSystemBase const> SensorSystemBaseConstPtr;
	typedef std::weak_ptr<SensorSystemBase> SensorSystemBaseWeakPtr;
	typedef std::shared_ptr<TrajectoryBase> TrajectoryBasePtr;
	typedef std::shared_ptr<TrajectoryBase const> TrajectoryBaseConstPtr;
	typedef std::weak_ptr<TrajectoryBase> TrajectoryBaseWeakPtr;
	typedef std::shared_ptr<ViewerBase> ViewerBasePtr;
	typedef std::shared_ptr<ViewerBase const> ViewerBaseConstPtr;
	typedef std::weak_ptr<ViewerBase> ViewerBaseWeakPtr;
	typedef std::shared_ptr<SpaceSamplerBase> SpaceSamplerBasePtr;
	typedef std::shared_ptr<SpaceSamplerBase const> SpaceSamplerBaseConstPtr;
	typedef std::weak_ptr<SpaceSamplerBase> SpaceSamplerBaseWeakPtr;
	typedef std::shared_ptr<EnvironmentBase> EnvironmentBasePtr;
	typedef std::shared_ptr<EnvironmentBase const> EnvironmentBaseConstPtr;
	typedef std::weak_ptr<EnvironmentBase> EnvironmentBaseWeakPtr;

	typedef std::shared_ptr<IkReturn> IkReturnPtr;
	typedef std::shared_ptr<IkReturn const> IkReturnConstPtr;
	typedef std::weak_ptr<IkReturn> IkReturnWeakPtr;
}

#endif // OPENRAVE_TYPE_H_