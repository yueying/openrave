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
		data_access_options_ = 0;
#ifdef USE_CRLIBM
		is_crlibm_init_ = false;
#endif

		interface_names_map_[PT_Planner] = "planner";
		interface_names_map_[PT_Robot] = "robot";
		interface_names_map_[PT_SensorSystem] = "sensorsystem";
		interface_names_map_[PT_Controller] = "controller";
		interface_names_map_[PT_Module] = "module";
		interface_names_map_[PT_IkSolver] = "iksolver";
		interface_names_map_[PT_KinBody] = "kinbody";
		interface_names_map_[PT_PhysicsEngine] = "physicsengine";
		interface_names_map_[PT_Sensor] = "sensor";
		interface_names_map_[PT_CollisionChecker] = "collisionchecker";
		interface_names_map_[PT_Trajectory] = "trajectory";
		interface_names_map_[PT_Viewer] = "viewer";
		interface_names_map_[PT_SpaceSampler] = "spacesampler";
		BOOST_ASSERT(interface_names_map_.size() == PT_NumberOfInterfaces);

		ik_parameterization_map_[IKP_Transform6D] = "Transform6D";
		ik_parameterization_map_[IKP_Rotation3D] = "Rotation3D";
		ik_parameterization_map_[IKP_Translation3D] = "Translation3D";
		ik_parameterization_map_[IKP_Direction3D] = "Direction3D";
		ik_parameterization_map_[IKP_Ray4D] = "Ray4D";
		ik_parameterization_map_[IKP_Lookat3D] = "Lookat3D";
		ik_parameterization_map_[IKP_TranslationDirection5D] = "TranslationDirection5D";
		ik_parameterization_map_[IKP_TranslationXY2D] = "TranslationXY2D";
		ik_parameterization_map_[IKP_TranslationXYOrientation3D] = "TranslationXYOrientation3D";
		ik_parameterization_map_[IKP_TranslationLocalGlobal6D] = "TranslationLocalGlobal6D";
		ik_parameterization_map_[IKP_TranslationXAxisAngle4D] = "TranslationXAxisAngle4D";
		ik_parameterization_map_[IKP_TranslationYAxisAngle4D] = "TranslationYAxisAngle4D";
		ik_parameterization_map_[IKP_TranslationZAxisAngle4D] = "TranslationZAxisAngle4D";
		ik_parameterization_map_[IKP_TranslationXAxisAngleZNorm4D] = "TranslationXAxisAngleZNorm4D";
		ik_parameterization_map_[IKP_TranslationYAxisAngleXNorm4D] = "TranslationYAxisAngleXNorm4D";
		ik_parameterization_map_[IKP_TranslationZAxisAngleYNorm4D] = "TranslationZAxisAngleYNorm4D";
		BOOST_ASSERT(ik_parameterization_map_.size() == IKP_NumberOfParameterizations);
		for (auto& it : ik_parameterization_map_)
		{
			std::string name = it.second;
			std::transform(name.begin(), name.end(), name.begin(), ::tolower);
			ik_parameterization_lower_map_[it.first] = name;
		}
	}

	RaveGlobal::~RaveGlobal()
	{
		Destroy();
	}

	int RaveGlobal::Initialize(bool is_load_all_plugins, int level)
	{
		if (_IsInitialized())
		{
			return 0;     // already initialized
		}

		_InitializeLogging(level);

#ifdef USE_CRLIBM
		if (!is_crlibm_init_)
		{
			_crlibm_fpu_state = crlibm_init();
			is_crlibm_init_ = true;
		}
#endif
		try
		{
			// TODO: eventually we should remove this call to set global locale for the process
			// and imbue each stringstream with the correct locale.

			// set to the classic locale so that number serialization/hashing works correctly
			// std::locale::global(std::locale::classic());
			std::locale::global(std::locale(std::locale(""), std::locale::classic(), std::locale::numeric));
		}
		catch (const std::runtime_error& e)
		{
			RAVELOG_WARN_FORMAT("failed to set to C locale: %s\n", e.what());
		}

		plugin_database_.reset(new PluginDatabase());
		if (!plugin_database_->Init(is_load_all_plugins))
		{
			RAVELOG_FATAL("failed to create the openrave plugin database\n");
		}

		char* phomedir = getenv("OPENRAVE_HOME"); // getenv not thread-safe?
		if (phomedir == NULL)
		{
#ifndef _WIN32
			_homedirectory = std::string(getenv("HOME")) + std::string("/.openrave"); // getenv not thread-safe?
#else
			_homedirectory = std::string(getenv("HOMEDRIVE")) + std::string(getenv("HOMEPATH")) + std::string("\\.openrave"); // getenv not thread-safe?
#endif
		}
		else
		{
			_homedirectory = phomedir;
		}
#ifndef _WIN32
		mkdir(_homedirectory.c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH | S_IRWXU);
#else
		CreateDirectory(_homedirectory.c_str(), NULL);
#endif

#ifdef _WIN32
		const char* delim = ";";
#else
		const char* delim = ":";
#endif
		_vdbdirectories.clear();
		char* pOPENRAVE_PLUGINS = getenv("OPENRAVE_DATABASE"); // getenv not thread-safe?
		if (pOPENRAVE_PLUGINS != NULL) {
			utils::TokenizeString(pOPENRAVE_PLUGINS, delim, _vdbdirectories);
		}
		_vdbdirectories.push_back(_homedirectory);

		_defaultviewertype.clear();
		const char* pOPENRAVE_DEFAULT_VIEWER = std::getenv("OPENRAVE_DEFAULT_VIEWER");
		if (!!pOPENRAVE_DEFAULT_VIEWER && strlen(pOPENRAVE_DEFAULT_VIEWER) > 0) {
			_defaultviewertype = std::string(pOPENRAVE_DEFAULT_VIEWER);
		}

		_UpdateDataDirs();
		return 0;
	}

}