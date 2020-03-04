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
#include <openrave/rave_global.h>

namespace OpenRAVE
{
	std::shared_ptr<RaveGlobal> RaveGlobal::global_state_;

	RaveGlobal::RaveGlobal()
	{
		// is this really necessary? just makes bugs hard to reproduce...
		//srand(GetMilliTime());
		//RaveInitRandomGeneration(GetMilliTime());
		debug_level_ = Level_Info;
		global_environment_id_ = 0;
		_nDataAccessOptions = 0;
#ifdef USE_CRLIBM
		_bcrlibmInit = false;
#endif

		_mapinterfacenames[PT_Planner] = "planner";
		_mapinterfacenames[PT_Robot] = "robot";
		_mapinterfacenames[PT_SensorSystem] = "sensorsystem";
		_mapinterfacenames[PT_Controller] = "controller";
		_mapinterfacenames[PT_Module] = "module";
		_mapinterfacenames[PT_IkSolver] = "iksolver";
		_mapinterfacenames[PT_KinBody] = "kinbody";
		_mapinterfacenames[PT_PhysicsEngine] = "physicsengine";
		_mapinterfacenames[PT_Sensor] = "sensor";
		_mapinterfacenames[PT_CollisionChecker] = "collisionchecker";
		_mapinterfacenames[PT_Trajectory] = "trajectory";
		_mapinterfacenames[PT_Viewer] = "viewer";
		_mapinterfacenames[PT_SpaceSampler] = "spacesampler";
		BOOST_ASSERT(_mapinterfacenames.size() == PT_NumberOfInterfaces);

		_mapikparameterization[IKP_Transform6D] = "Transform6D";
		_mapikparameterization[IKP_Rotation3D] = "Rotation3D";
		_mapikparameterization[IKP_Translation3D] = "Translation3D";
		_mapikparameterization[IKP_Direction3D] = "Direction3D";
		_mapikparameterization[IKP_Ray4D] = "Ray4D";
		_mapikparameterization[IKP_Lookat3D] = "Lookat3D";
		_mapikparameterization[IKP_TranslationDirection5D] = "TranslationDirection5D";
		_mapikparameterization[IKP_TranslationXY2D] = "TranslationXY2D";
		_mapikparameterization[IKP_TranslationXYOrientation3D] = "TranslationXYOrientation3D";
		_mapikparameterization[IKP_TranslationLocalGlobal6D] = "TranslationLocalGlobal6D";
		_mapikparameterization[IKP_TranslationXAxisAngle4D] = "TranslationXAxisAngle4D";
		_mapikparameterization[IKP_TranslationYAxisAngle4D] = "TranslationYAxisAngle4D";
		_mapikparameterization[IKP_TranslationZAxisAngle4D] = "TranslationZAxisAngle4D";
		_mapikparameterization[IKP_TranslationXAxisAngleZNorm4D] = "TranslationXAxisAngleZNorm4D";
		_mapikparameterization[IKP_TranslationYAxisAngleXNorm4D] = "TranslationYAxisAngleXNorm4D";
		_mapikparameterization[IKP_TranslationZAxisAngleYNorm4D] = "TranslationZAxisAngleYNorm4D";
		BOOST_ASSERT(_mapikparameterization.size() == IKP_NumberOfParameterizations);
		for (auto it : _mapikparameterization)
		{
			std::string name = it.second;
			std::transform(name.begin(), name.end(), name.begin(), ::tolower);
			_mapikparameterizationlower[it.first] = name;
		}
	}

	RaveGlobal::~RaveGlobal()
	{
		Destroy();
	}
}