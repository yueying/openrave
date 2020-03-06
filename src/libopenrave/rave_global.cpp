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

		char* home_dir = getenv("OPENRAVE_HOME"); // getenv not thread-safe?
		if (home_dir == nullptr)
		{
#ifndef _WIN32
			home_directory_ = std::string(getenv("HOME")) + std::string("/.openrave"); // getenv not thread-safe?
#else
			home_directory_ = std::string(getenv("HOMEDRIVE")) + std::string(getenv("HOMEPATH")) + std::string("\\.openrave"); // getenv not thread-safe?
#endif
		}
		else
		{
			home_directory_ = home_dir;
		}
#ifndef _WIN32
		mkdir(home_directory_.c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH | S_IRWXU);
#else
		CreateDirectory(home_directory_.c_str(), NULL);
#endif

#ifdef _WIN32
		const char* delim = ";";
#else
		const char* delim = ":";
#endif
		database_directory_vector_.clear();
		char* plugin_dirs_str = getenv("OPENRAVE_DATABASE"); // getenv not thread-safe?
		if (plugin_dirs_str != NULL) 
		{
			utils::TokenizeString(plugin_dirs_str, delim, database_directory_vector_);
		}
		database_directory_vector_.push_back(home_directory_);

		default_viewer_type_.clear();
		const char* openrave_default_viewer = std::getenv("OPENRAVE_DEFAULT_VIEWER");
		if (!!openrave_default_viewer && strlen(openrave_default_viewer) > 0) 
		{
			default_viewer_type_ = std::string(openrave_default_viewer);
		}

		_UpdateDataDirs();
		return 0;
	}

	void RaveGlobal::Destroy()
	{
		if (!!plugin_database_) 
		{
			// notify all plugins that about to destroy
			plugin_database_->OnRavePreDestroy();
		}

		// don't use any log statements since global instance might be null
		// environments have to be destroyed carefully since their destructors can be called, which will attempt to unregister the environment
		std::map<int, EnvironmentBase*> mapenvironments;
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			mapenvironments = environments_map_;
		}
		FOREACH(itenv, mapenvironments) {
			// equire a shared pointer to prevent environment from getting deleted during Destroy loop
			EnvironmentBasePtr penv = itenv->second->shared_from_this();
			penv->Destroy();
		}
		mapenvironments.clear();
		environments_map_.clear();
		default_space_sampler_.reset();
		_mapxmlreaders.clear();
		_mapjsonreaders.clear();

		// process the callbacks
		std::list<boost::function<void()> > listDestroyCallbacks;
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			listDestroyCallbacks.swap(destroy_callbacks_list_);
		}
		FOREACH(itcallback, listDestroyCallbacks) {
			(*itcallback)();
		}
		listDestroyCallbacks.clear();

		if (!!plugin_database_) {
			// force destroy in case some one is holding a pointer to it
			plugin_database_->Destroy();
			plugin_database_.reset();
		}
#ifdef USE_CRLIBM

#ifdef HAS_FENV_H
		feclearexcept(-1); // clear any cached exceptions
#endif
		if (is_crlibm_init_) {
			crlibm_exit(_crlibm_fpu_state);
			is_crlibm_init_ = false;
		}
#endif

#if OPENRAVE_LOG4CXX
		_logger = 0;
#endif
	}



	void RaveGlobal::_UpdateDataDirs()
	{
		data_dirs_vector_.resize(0);

		bool is_exists = false;
#ifdef _WIN32
		const char* delim = ";";
#else
		const char* delim = ":";
#endif
		char* pOPENRAVE_DATA = getenv("OPENRAVE_DATA"); // getenv not thread-safe?
		if (pOPENRAVE_DATA != NULL) 
		{
			utils::TokenizeString(pOPENRAVE_DATA, delim, data_dirs_vector_);
		}
		std::string install_dir = OPENRAVE_DATA_INSTALL_DIR;
#ifdef HAVE_BOOST_FILESYSTEM
		if (!boost::filesystem::is_directory(boost::filesystem::path(install_dir))) {
#ifdef _WIN32
			HKEY hkey;
			if (RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\OpenRAVE\\" OPENRAVE_VERSION_STRING),
				0, KEY_QUERY_VALUE, &hkey) == ERROR_SUCCESS)
			{
				DWORD dwType = REG_SZ;
				CHAR szInstallRoot[4096];     // dont' take chances, it is windows
				DWORD dwSize = sizeof(szInstallRoot);
				RegQueryValueEx(hkey, TEXT("InstallRoot"), NULL, &dwType, (PBYTE)szInstallRoot, &dwSize);
				RegCloseKey(hkey);
				install_dir.assign(szInstallRoot);
				install_dir += str(boost::format("%cshare%copenrave-%d.%d") % s_filesep%s_filesep%OPENRAVE_VERSION_MAJOR%OPENRAVE_VERSION_MINOR);
				RAVELOG_VERBOSE(str(boost::format("window registry data dir '%s'") % install_dir));
			}
			else
#endif
			{
				RAVELOG_WARN(str(boost::format("%s doesn't exist") % install_dir));
			}
		}

		boost::filesystem::path data_file_name = boost::filesystem::absolute(boost::filesystem::path(install_dir));
		for(auto itname: data_dirs_vector_)
		{
			if (data_file_name == boost::filesystem::absolute(boost::filesystem::path(itname)))
			{
				is_exists = true;
				break;
			}
		}
#else
		std::string data_file_name = install_dir;
		for (auto itname : data_dirs_vector_)
		{
			if (itname == install_dir)
			{
				is_exists = true;
				break;
			}
		}
#endif
		if (!is_exists) 
		{
			data_dirs_vector_.push_back(install_dir);
		}
		for (auto itdir : data_dirs_vector_)
		{
			RAVELOG_VERBOSE(str(boost::format("data dir: %s") % itdir));
		}

#ifdef HAVE_BOOST_FILESYSTEM
		boost_data_dirs_vector_.resize(0);
		for(auto file_name: data_dirs_vector_) 
		{
			boost::filesystem::path full_file_name = boost::filesystem::absolute(boost::filesystem::path(file_name));
			_CustomNormalizePath(full_file_name);
			if (full_file_name.filename() == ".") 
			{
				// full_file_name ends in '/', so remove it
				full_file_name = full_file_name.parent_path();
			}
			boost_data_dirs_vector_.push_back(full_file_name);
		}
#endif
	}

}


