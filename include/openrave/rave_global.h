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
#include <openrave/numerical.h>
#include <openrave/user_data.h>
#include <boost/utility.hpp>
#include <mutex>
#include <map>
#include <openrave/openrave.h>
#include <openrave/plugin_database.h>
#include <openrave/utils.h>

namespace OpenRAVE
{
#ifdef _WIN32
	const char s_filesep = '\\';
#else
	const char s_filesep = '/';
#endif
	/// there is only once global openrave state. It is created when openrave
/// is first used, and destroyed when the program quits or RaveDestroy is called.
	class RaveGlobal : public std::enable_shared_from_this<RaveGlobal>, public UserData
	{
		typedef std::map<std::string, CreateXMLReaderFn, CaseInsensitiveCompare> XMLREADERSMAP;
		typedef std::map<std::string, CreateJSONReaderFn, CaseInsensitiveCompare> JSONREADERSMAP;

	public:
		virtual ~RaveGlobal();

		static std::shared_ptr<RaveGlobal>& instance()
		{
			static std::once_flag s_flag;
			std::call_once(s_flag, [&]() {
				global_state_.reset(new RaveGlobal);
			});

			return global_state_;
		}

		int Initialize(bool is_load_all_plugins, int level);

		void Destroy();

		void AddCallbackForDestroy(const boost::function<void()>& fn)
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			destroy_callbacks_list_.push_back(fn);
		}

		std::string GetHomeDirectory()
		{
			return home_directory_;
		}

		std::string FindDatabaseFile(const std::string& filename, bool bRead)
		{
			FOREACH(itdirectory, database_directory_vector_) {
#ifdef HAVE_BOOST_FILESYSTEM
				std::string fullfilename = boost::filesystem::absolute(boost::filesystem::path(*itdirectory) / filename).string();
#else
				std::string fullfilename = *itdirectory;
				fullfilename += s_filesep;
				fullfilename += filename;
#endif
				if (bRead) {
					if (!!std::ifstream(fullfilename.c_str())) {
						return fullfilename;
					}
				}
				else {
					return fullfilename;
				}
			}
			return "";
		}

#if OPENRAVE_LOG4CXX
		log4cxx::LoggerPtr GetLogger()
		{
			return _logger;
		}

		void SetDebugLevel(int level)
		{
			if (_logger != NULL) {
				log4cxx::LevelPtr levelptr = log4cxx::Level::getInfo();
				switch (level&Level_OutputMask) {
				case Level_Fatal: levelptr = log4cxx::Level::getFatal(); break;
				case Level_Error: levelptr = log4cxx::Level::getError(); break;
				case Level_Warn: levelptr = log4cxx::Level::getWarn(); break;
				case Level_Info: levelptr = log4cxx::Level::getInfo(); break;
				case Level_Debug: levelptr = log4cxx::Level::getDebug(); break;
				case Level_Verbose: levelptr = log4cxx::Level::getTrace(); break;
				}
				_logger->setLevel(levelptr);
			}
			debug_level_ = level;
		}

		int GetDebugLevel()
		{
			int level = debug_level_;
			if (_logger != NULL) {
				if (_logger->isEnabledFor(log4cxx::Level::getTrace())) {
					level = Level_Verbose;
				}
				else if (_logger->isEnabledFor(log4cxx::Level::getDebug())) {
					level = Level_Debug;
				}
				else if (_logger->isEnabledFor(log4cxx::Level::getInfo())) {
					level = Level_Info;
				}
				else if (_logger->isEnabledFor(log4cxx::Level::getWarn())) {
					level = Level_Warn;
				}
				else if (_logger->isEnabledFor(log4cxx::Level::getError())) {
					level = Level_Error;
				}
				else {
					level = Level_Fatal;
				}
			}
			return level | (debug_level_ & ~Level_OutputMask);
		}

#else
		void SetDebugLevel(int level)
		{
			debug_level_ = level;
		}

		int GetDebugLevel()
		{
			return debug_level_;
		}
#endif

		class XMLReaderFunctionData : public UserData
		{
		public:
			XMLReaderFunctionData(InterfaceType type, const std::string& xmltag,
				const CreateXMLReaderFn& fn, std::shared_ptr<RaveGlobal> global)
				: _global(global), _type(type), _xmltag(xmltag)
			{
				boost::mutex::scoped_lock lock(global->_mutexinternal);
				_oldfn = global->_mapxmlreaders[_type][_xmltag];
				global->_mapxmlreaders[_type][_xmltag] = fn;
			}
			virtual ~XMLReaderFunctionData()
			{
				std::shared_ptr<RaveGlobal> global = _global.lock();
				if (!!global) {
					boost::mutex::scoped_lock lock(global->_mutexinternal);
					global->_mapxmlreaders[_type][_xmltag] = _oldfn;
				}
			}
		protected:
			CreateXMLReaderFn _oldfn;
			std::weak_ptr<RaveGlobal> _global;
			InterfaceType _type;
			std::string _xmltag;
		};

		class JSONReaderFunctionData : public UserData
		{
		public:
			JSONReaderFunctionData(InterfaceType type, const std::string& id,
				const CreateJSONReaderFn& fn, std::shared_ptr<RaveGlobal> global)
				: _global(global), _type(type), _id(id)
			{
				boost::mutex::scoped_lock lock(global->_mutexinternal);
				_oldfn = global->_mapjsonreaders[_type][_id];
				global->_mapjsonreaders[_type][_id] = fn;
			}
			virtual ~JSONReaderFunctionData()
			{
				std::shared_ptr<RaveGlobal> global = _global.lock();
				if (!!global) {
					boost::mutex::scoped_lock lock(global->_mutexinternal);
					global->_mapjsonreaders[_type][_id] = _oldfn;
				}
			}
		protected:
			CreateJSONReaderFn _oldfn;
			std::weak_ptr<RaveGlobal> _global;
			InterfaceType _type;
			std::string _id;
		};

		UserDataPtr RegisterJSONReader(InterfaceType type, const std::string& id, const CreateJSONReaderFn& fn)
		{
			return UserDataPtr(new JSONReaderFunctionData(type, id, fn, shared_from_this()));
		}

		const BaseJSONReaderPtr CallJSONReader(InterfaceType type, const std::string& id,
			InterfaceBasePtr pinterface, const AttributesList& atts)
		{
			JSONREADERSMAP::iterator it = _mapjsonreaders[type].find(id);
			if (it == _mapjsonreaders[type].end()) {
				//throw openrave_exception(str(boost::format(_tr("No function registered for interface %s xml tag %s"))%GetInterfaceName(type)%id),ORE_InvalidArguments);
				return BaseJSONReaderPtr();
			}
			return it->second(pinterface, atts);
		}

		UserDataPtr RegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn)
		{
			return UserDataPtr(new XMLReaderFunctionData(type, xmltag, fn, shared_from_this()));
		}

		const BaseXMLReaderPtr CallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts)
		{
			XMLREADERSMAP::iterator it = _mapxmlreaders[type].find(xmltag);
			if (it == _mapxmlreaders[type].end()) {
				//throw OpenRAVEException(str(boost::format(_tr("No function registered for interface %s xml tag %s"))%GetInterfaceName(type)%xmltag),ORE_InvalidArguments);
				return BaseXMLReaderPtr();
			}
			return it->second(pinterface, atts);
		}

		std::shared_ptr<PluginDatabase> GetPluginDatabase() const
		{
			return plugin_database_;
		}

		const std::map<InterfaceType, std::string>& GetInterfaceNamesMap() const {
			return interface_names_map_;
		}
		const std::map<IkParameterizationType, std::string>& GetIkParameterizationMap(int alllowercase = 0) {
			if (alllowercase) {
				return ik_parameterization_lower_map_;
			}
			return ik_parameterization_map_;
		}

		const std::string& GetInterfaceName(InterfaceType type)
		{
			std::map<InterfaceType, std::string>::const_iterator it = interface_names_map_.find(type);
			if (it == interface_names_map_.end()) {
				throw OPENRAVE_EXCEPTION_FORMAT(_tr("Invalid type %d specified"), type, ORE_Failed);
			}
			return it->second;
		}

		// have to take in pointer instead of shared_ptr since method will be called in EnvironmentBase constructor
		int RegisterEnvironment(EnvironmentBase* penv)
		{
			BOOST_ASSERT(!!plugin_database_);
			boost::mutex::scoped_lock lock(_mutexinternal);
			environments_map_[++global_environment_id_] = penv;
			return global_environment_id_;
		}

		void UnregisterEnvironment(EnvironmentBase* penv)
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			FOREACH(it, environments_map_) {
				if (it->second == penv) {
					environments_map_.erase(it);
					break;
				}
			}
		}

		int GetEnvironmentId(EnvironmentBaseConstPtr penv)
		{
			return !!penv ? penv->GetId() : 0;
			//        boost::mutex::scoped_lock lock(_mutexinternal);
			//        FOREACH(it,environments_map_) {
			//            if( it->second == penv.get() ) {
			//                return it->first;
			//            }
			//        }
			//        return 0;
		}

		EnvironmentBasePtr GetEnvironment(int id)
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			std::map<int, EnvironmentBase*>::iterator it = environments_map_.find(id);
			if (it == environments_map_.end())
			{
				return EnvironmentBasePtr();
			}
			return it->second->shared_from_this();
		}

		void GetEnvironments(std::list<EnvironmentBasePtr>& listenvironments)
		{
			listenvironments.clear();
			boost::mutex::scoped_lock lock(_mutexinternal);
			for (auto it : environments_map_)
			{
				EnvironmentBasePtr penv = it.second->shared_from_this();
				if (!!penv)
				{
					listenvironments.push_back(penv);
				}
			}
		}

		SpaceSamplerBasePtr GetDefaultSampler()
		{
			if (!default_space_sampler_)
			{
				boost::mutex::scoped_lock lock(_mutexinternal);
				BOOST_ASSERT(environments_map_.size() > 0);
				default_space_sampler_ = GetPluginDatabase()->CreateSpaceSampler(environments_map_.begin()->second->shared_from_this(), "MT19937");
			}
			return default_space_sampler_;
		}

		std::string FindLocalFile(const std::string& _filename, const std::string& curdir)
		{
#ifndef HAVE_BOOST_FILESYSTEM
			throw OPENRAVE_EXCEPTION_FORMAT0(_tr("need to compile with boost::filesystem"), ORE_Assert);
#else
			if (_filename.size() == 0) {
				return std::string();
			}

			boost::mutex::scoped_lock lock(_mutexinternal);
			boost::filesystem::path fullfilename;
			boost::filesystem::path filename(_filename);

			if (filename.is_complete()) {
				fullfilename = filename;
			}
			else if (curdir.size() > 0) {
				fullfilename = boost::filesystem::absolute(boost::filesystem::path(curdir)) / filename;
			}
			else {
				fullfilename = boost::filesystem::current_path() / filename;
			}

			if (boost::filesystem::exists(fullfilename)) {
				if (!_ValidateFilename(fullfilename, boost::filesystem::path())) {
					RAVELOG_WARN(str(boost::format("acess denied to file %s\n") % fullfilename));
					return std::string();
				}
				return fullfilename.string();
			}

			// try the openrave directories
			FOREACHC(itdir, boost_data_dirs_vector_) {
				fullfilename = *itdir / filename;
				if (_ValidateFilename(fullfilename, boost::filesystem::path())) {
					return fullfilename.string();
				}
			}

			RAVELOG_INFO_FORMAT("could not find file %s", filename);
			return std::string();
#endif
		}

		bool InvertFileLookup(std::string& newfilename, const std::string& filename)
		{
#ifndef HAVE_BOOST_FILESYSTEM
			RAVELOG_WARN("need to compile with boost::filesystem\n");
#else
			// check if filename is within boost_data_dirs_vector_
			boost::filesystem::path fullfilename = boost::filesystem::absolute(filename);
			fullfilename.normalize();
			FOREACHC(itpath, boost_data_dirs_vector_) {
				std::list<boost::filesystem::path> listfilenameparts;
				boost::filesystem::path testfilename = fullfilename.parent_path();
				while (testfilename >= *itpath) {
					if (testfilename == *itpath) {
						boost::filesystem::path relpath;
						FOREACH(itpart, listfilenameparts) {
							relpath /= *itpart;
						}
						relpath /= fullfilename.filename();
						newfilename = relpath.string();
						return true;
					}
					listfilenameparts.push_front(testfilename.filename());
					testfilename = testfilename.parent_path();
				}
			}
#endif
			return false;
		}

		void SetDataAccess(int options)
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			data_access_options_ = options;
		}
		int GetDataAccess()
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			return data_access_options_;
		}

		std::string GetDefaultViewerType()
		{
			if (default_viewer_type_.size() > 0)
			{
				return default_viewer_type_;
			}

			// get the first viewer that can be loadable, with preferenace to qtosg, qtcoin
			std::shared_ptr<PluginDatabase> pdatabase = plugin_database_;
			if (!!pdatabase)
			{
				if (pdatabase->HasInterface(PT_Viewer, "qtosg"))
				{
					return std::string("qtosg");
				}

				if (pdatabase->HasInterface(PT_Viewer, "qtcoin"))
				{
					return std::string("qtcoin");
				}

				// search for the first viewer found
				std::map<InterfaceType, std::vector<std::string> > interfacenames;
				pdatabase->GetLoadedInterfaces(interfacenames);
				if (interfacenames.find(PT_Viewer) != interfacenames.end())
				{
					if (interfacenames[PT_Viewer].size() > 0)
					{
						return interfacenames[PT_Viewer].at(0);
					}
				}
			}

			return std::string();
		}

	protected:
		bool _IsInitialized() const
		{
			return !!plugin_database_;
		}

		void _UpdateDataDirs();

#ifdef HAVE_BOOST_FILESYSTEM

		bool _ValidateFilename(const boost::filesystem::path& filename, const boost::filesystem::path& curdir)
		{
			if (!boost::filesystem::exists(filename)) {
				return false;
			}

			if (data_access_options_ & 1) {
				// check if filename is within boost_data_dirs_vector_
				boost::filesystem::path fullfilename = boost::filesystem::absolute(filename, curdir.empty() ? boost::filesystem::current_path() : curdir);
				fullfilename.normalize();
				bool bfound = false;
				FOREACHC(itpath, boost_data_dirs_vector_) {
					boost::filesystem::path testfilename = fullfilename.parent_path();
					while (testfilename >= *itpath) {
						if (testfilename == *itpath) {
							bfound = true;
							break;
						}
						testfilename = testfilename.parent_path();
					}
					if (bfound) {
						break;
					}
				}
				if (!bfound) {
					return false;
				}
			}
			return true;
		}

#endif

		void _InitializeLogging(int level)
		{
#if OPENRAVE_LOG4CXX
			_logger = log4cxx::Logger::getLogger("openrave");

			// if root appenders have not been configured, configure a default console appender
			log4cxx::LoggerPtr root(log4cxx::Logger::getRootLogger());
			if (root->getAllAppenders().size() == 0) {
				log4cxx::LayoutPtr consolePatternLayout(new log4cxx::PatternLayout(LOG4CXX_STR("%d %c [%p] [%F:%L %M] %m%n")));
				log4cxx::LayoutPtr colorLayout(new log4cxx::ColorLayout(consolePatternLayout));
				log4cxx::AppenderPtr consoleAppender(new log4cxx::ConsoleAppender(colorLayout));
				root->setLevel(log4cxx::Level::getTrace());
				root->addAppender(consoleAppender);
			}
#endif
			SetDebugLevel(level);
		}
	private:
		RaveGlobal();
		RaveGlobal(const RaveGlobal&) = delete;
		RaveGlobal& operator=(const RaveGlobal&) = delete;

	private:
		static std::shared_ptr<RaveGlobal> global_state_; //!< state that is always present


		// state that is initialized/destroyed
		std::shared_ptr<PluginDatabase> plugin_database_;
		int debug_level_;
		boost::mutex _mutexinternal;
		std::map<InterfaceType, XMLREADERSMAP > _mapxmlreaders;
		std::map<InterfaceType, JSONREADERSMAP > _mapjsonreaders;
		std::map<InterfaceType, std::string> interface_names_map_;
		std::map<IkParameterizationType, std::string> ik_parameterization_map_, ik_parameterization_lower_map_;
		std::map<int, EnvironmentBase*> environments_map_;
		std::list<boost::function<void()> > destroy_callbacks_list_;
		std::string home_directory_;
		std::string default_viewer_type_; //!< the default viewer type from the environment variable OPENRAVE_DEFAULT_VIEWER
		std::vector<std::string> database_directory_vector_;
		int global_environment_id_;
		SpaceSamplerBasePtr default_space_sampler_;
#ifdef USE_CRLIBM
		long long _crlibm_fpu_state;
		bool is_crlibm_init_; //!< true if crlibm is initialized
#endif
		int data_access_options_;

		std::vector<std::string> data_dirs_vector_;
#ifdef HAVE_BOOST_FILESYSTEM
		std::vector<boost::filesystem::path> boost_data_dirs_vector_; //!< \brief returns absolute filenames of the data
#endif

#if OPENRAVE_LOG4CXX
		log4cxx::LoggerPtr _logger;
#endif

		friend void RaveInitializeFromState(UserDataPtr);
		friend UserDataPtr RaveGlobalState();
	};



		}