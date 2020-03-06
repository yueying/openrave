// Copyright (C) 2020 fengbing <fengbing123@gmail.com>
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

#include <openrave/config.h>
#include <openrave/openrave_macros.h>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include <cstdint>


#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <exception>

#include <iomanip>
#include <fstream>
#include <sstream>
#include <memory>

// QTBUG-22829 alternative workaround
#ifndef Q_MOC_RUN

#include <boost/version.hpp>
#include <boost/function.hpp>

#include <boost/tuple/tuple.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/static_assert.hpp>
#include <boost/format.hpp>
#include <array>
#include <boost/multi_array.hpp>
//#include <boost/cstdint.hpp>

#endif

#if defined(__GNUC__)
#define RAVE_DEPRECATED __attribute__((deprecated))
#else
#define RAVE_DEPRECATED
#endif
namespace OpenRAVE
{
	OPENRAVE_CLASS_FORWARD(CollisionReport);
	OPENRAVE_CLASS_FORWARD(InterfaceBase);
	OPENRAVE_CLASS_FORWARD(IkSolverBase);
	OPENRAVE_CLASS_FORWARD(TrajectoryBase);
	OPENRAVE_CLASS_FORWARD(ControllerBase);
	OPENRAVE_CLASS_FORWARD(PlannerBase);
	OPENRAVE_CLASS_FORWARD(RobotBase);
	OPENRAVE_CLASS_FORWARD(ModuleBase);
	OPENRAVE_CLASS_FORWARD(EnvironmentBase);
	OPENRAVE_CLASS_FORWARD(KinBody);
	OPENRAVE_CLASS_FORWARD(SensorSystemBase);
	OPENRAVE_CLASS_FORWARD(PhysicsEngineBase);
	OPENRAVE_CLASS_FORWARD(SensorBase);
	OPENRAVE_CLASS_FORWARD(CollisionCheckerBase);
	OPENRAVE_CLASS_FORWARD(ViewerBase);
	OPENRAVE_CLASS_FORWARD(SpaceSamplerBase);
	OPENRAVE_CLASS_FORWARD(IkParameterization);
	OPENRAVE_CLASS_FORWARD(ConfigurationSpecification);
	OPENRAVE_CLASS_FORWARD(IkReturn);
}