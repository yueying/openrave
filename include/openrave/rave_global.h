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

		void Destroy()
		{
			if (!!plugin_database_) {
				// notify all plugins that about to destroy
				plugin_database_->OnRavePreDestroy();
			}

			// don't use any log statements since global instance might be null
			// environments have to be destroyed carefully since their destructors can be called, which will attempt to unregister the environment
			std::map<int, EnvironmentBase*> mapenvironments;
			{
				boost::mutex::scoped_lock lock(_mutexinternal);
				mapenvironments = _mapenvironments;
			}
			FOREACH(itenv, mapenvironments) {
				// equire a shared pointer to prevent environment from getting deleted during Destroy loop
				EnvironmentBasePtr penv = itenv->second->shared_from_this();
				penv->Destroy();
			}
			mapenvironments.clear();
			_mapenvironments.clear();
			_pdefaultsampler.reset();
			_mapxmlreaders.clear();
			_mapjsonreaders.clear();

			// process the callbacks
			std::list<boost::function<void()> > listDestroyCallbacks;
			{
				boost::mutex::scoped_lock lock(_mutexinternal);
				listDestroyCallbacks.swap(_listDestroyCallbacks);
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

		void AddCallbackForDestroy(const boost::function<void()>& fn)
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			_listDestroyCallbacks.push_back(fn);
		}

		std::string GetHomeDirectory()
		{
			return _homedirectory;
		}

		std::string FindDatabaseFile(const std::string& filename, bool bRead)
		{
			FOREACH(itdirectory, _vdbdirectories) {
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
			XMLReaderFunctionData(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn, std::shared_ptr<RaveGlobal> global) : _global(global), _type(type), _xmltag(xmltag)
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
				const CreateJSONReaderFn& fn, std::shared_ptr<RaveGlobal> global) : _global(global), _type(type), _id(id)
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

		const BaseJSONReaderPtr CallJSONReader(InterfaceType type, const std::string& id, InterfaceBasePtr pinterface, const AttributesList& atts)
		{
			JSONREADERSMAP::iterator it = _mapjsonreaders[type].find(id);
			if (it == _mapjsonreaders[type].end()) {
				//throw openrave_exception(str(boost::format(_("No function registered for interface %s xml tag %s"))%GetInterfaceName(type)%id),ORE_InvalidArguments);
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
				//throw OpenRAVEException(str(boost::format(_("No function registered for interface %s xml tag %s"))%GetInterfaceName(type)%xmltag),ORE_InvalidArguments);
				return BaseXMLReaderPtr();
			}
			return it->second(pinterface, atts);
		}

		std::shared_ptr<PluginDatabase> GetDatabase() const {
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
				throw OPENRAVE_EXCEPTION_FORMAT(_("Invalid type %d specified"), type, ORE_Failed);
			}
			return it->second;
		}

		// have to take in pointer instead of shared_ptr since method will be called in EnvironmentBase constructor
		int RegisterEnvironment(EnvironmentBase* penv)
		{
			BOOST_ASSERT(!!plugin_database_);
			boost::mutex::scoped_lock lock(_mutexinternal);
			_mapenvironments[++global_environment_id_] = penv;
			return global_environment_id_;
		}

		void UnregisterEnvironment(EnvironmentBase* penv)
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			FOREACH(it, _mapenvironments) {
				if (it->second == penv) {
					_mapenvironments.erase(it);
					break;
				}
			}
		}

		int GetEnvironmentId(EnvironmentBaseConstPtr penv)
		{
			return !!penv ? penv->GetId() : 0;
			//        boost::mutex::scoped_lock lock(_mutexinternal);
			//        FOREACH(it,_mapenvironments) {
			//            if( it->second == penv.get() ) {
			//                return it->first;
			//            }
			//        }
			//        return 0;
		}

		EnvironmentBasePtr GetEnvironment(int id)
		{
			boost::mutex::scoped_lock lock(_mutexinternal);
			std::map<int, EnvironmentBase*>::iterator it = _mapenvironments.find(id);
			if (it == _mapenvironments.end()) {
				return EnvironmentBasePtr();
			}
			return it->second->shared_from_this();
		}

		void GetEnvironments(std::list<EnvironmentBasePtr>& listenvironments)
		{
			listenvironments.clear();
			boost::mutex::scoped_lock lock(_mutexinternal);
			FOREACH(it, _mapenvironments) {
				EnvironmentBasePtr penv = it->second->shared_from_this();
				if (!!penv) {
					listenvironments.push_back(penv);
				}
			}
		}

		SpaceSamplerBasePtr GetDefaultSampler()
		{
			if (!_pdefaultsampler) {
				boost::mutex::scoped_lock lock(_mutexinternal);
				BOOST_ASSERT(_mapenvironments.size() > 0);
				_pdefaultsampler = GetDatabase()->CreateSpaceSampler(_mapenvironments.begin()->second->shared_from_this(), "MT19937");
			}
			return _pdefaultsampler;
		}

		std::string FindLocalFile(const std::string& _filename, const std::string& curdir)
		{
#ifndef HAVE_BOOST_FILESYSTEM
			throw OPENRAVE_EXCEPTION_FORMAT0(_("need to compile with boost::filesystem"), ORE_Assert);
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
			FOREACHC(itdir, _vBoostDataDirs) {
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
			// check if filename is within _vBoostDataDirs
			boost::filesystem::path fullfilename = boost::filesystem::absolute(filename);
			_CustomNormalizePath(fullfilename);
			FOREACHC(itpath, _vBoostDataDirs) {
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

		std::string GetDefaultViewerType() {
			if (_defaultviewertype.size() > 0) {
				return _defaultviewertype;
			}

			// get the first viewer that can be loadable, with preferenace to qtosg, qtcoin
			std::shared_ptr<PluginDatabase> pdatabase = plugin_database_;
			if (!!pdatabase) {
				if (pdatabase->HasInterface(PT_Viewer, "qtosg")) {
					return std::string("qtosg");
				}

				if (pdatabase->HasInterface(PT_Viewer, "qtcoin")) {
					return std::string("qtcoin");
				}

				// search for the first viewer found
				std::map<InterfaceType, std::vector<std::string> > interfacenames;
				pdatabase->GetLoadedInterfaces(interfacenames);
				if (interfacenames.find(PT_Viewer) != interfacenames.end()) {
					if (interfacenames[PT_Viewer].size() > 0) {
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

		void _UpdateDataDirs()
		{
			_vdatadirs.resize(0);

			bool bExists = false;
#ifdef _WIN32
			const char* delim = ";";
#else
			const char* delim = ":";
#endif
			char* pOPENRAVE_DATA = getenv("OPENRAVE_DATA"); // getenv not thread-safe?
			if (pOPENRAVE_DATA != NULL) {
				utils::TokenizeString(pOPENRAVE_DATA, delim, _vdatadirs);
			}
			std::string installdir = OPENRAVE_DATA_INSTALL_DIR;
#ifdef HAVE_BOOST_FILESYSTEM
			if (!boost::filesystem::is_directory(boost::filesystem::path(installdir))) {
#ifdef _WIN32
				HKEY hkey;
				if (RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\OpenRAVE\\" OPENRAVE_VERSION_STRING), 0, KEY_QUERY_VALUE, &hkey) == ERROR_SUCCESS) {
					DWORD dwType = REG_SZ;
					CHAR szInstallRoot[4096];     // dont' take chances, it is windows
					DWORD dwSize = sizeof(szInstallRoot);
					RegQueryValueEx(hkey, TEXT("InstallRoot"), NULL, &dwType, (PBYTE)szInstallRoot, &dwSize);
					RegCloseKey(hkey);
					installdir.assign(szInstallRoot);
					installdir += str(boost::format("%cshare%copenrave-%d.%d") % s_filesep%s_filesep%OPENRAVE_VERSION_MAJOR%OPENRAVE_VERSION_MINOR);
					RAVELOG_VERBOSE(str(boost::format("window registry data dir '%s'") % installdir));
				}
				else
#endif
				{
					RAVELOG_WARN(str(boost::format("%s doesn't exist") % installdir));
				}
			}

			boost::filesystem::path datafilename = boost::filesystem::absolute(boost::filesystem::path(installdir));
			FOREACH(itname, _vdatadirs) {
				if (datafilename == boost::filesystem::absolute(boost::filesystem::path(*itname))) {
					bExists = true;
					break;
				}
			}
#else
			std::string datafilename = installdir;
			for (auto itname : _vdatadirs)
			{
				if (itname == installdir)
				{
					bExists = true;
					break;
				}
			}
#endif
			if (!bExists) {
				_vdatadirs.push_back(installdir);
			}
			for (auto itdir : _vdatadirs)
			{
				RAVELOG_VERBOSE(str(boost::format("data dir: %s") % itdir));
			}

#ifdef HAVE_BOOST_FILESYSTEM
			_vBoostDataDirs.resize(0);
			FOREACHC(itfilename, _vdatadirs) {
				boost::filesystem::path fullfilename = boost::filesystem::absolute(boost::filesystem::path(*itfilename));
				_CustomNormalizePath(fullfilename);
				if (fullfilename.filename() == ".") {
					// fullfilename ends in '/', so remove it
					fullfilename = fullfilename.parent_path();
				}
				_vBoostDataDirs.push_back(fullfilename);
			}
#endif
		}

#ifdef HAVE_BOOST_FILESYSTEM

		void _CustomNormalizePath(boost::filesystem::path& p)
		{
#ifndef BOOST_FILESYSTEM_NO_DEPRECATED
			p.normalize();
#else
			boost::filesystem::path result;
			for (boost::filesystem::path::iterator it = p.begin(); it != p.end(); ++it)
			{
				if (*it == "..") {
					// /a/b/.. is not necessarily /a if b is a symbolic link
					if (boost::filesystem::is_symlink(result)) {
						result /= *it;
					}
					// /a/b/../.. is not /a/b/.. under most circumstances
					// We can end up with ..s in our result because of symbolic links
					else if (result.filename() == "..") {
						result /= *it;
					}
					// Otherwise it should be safe to resolve the parent
					else {
						result = result.parent_path();
					}
				}
				else if (*it == ".") {
					// Ignore
				}
				else {
					// Just cat other path entries
					result /= *it;
				}
			}
			p = result;
#endif
		}

		bool _ValidateFilename(const boost::filesystem::path& filename, const boost::filesystem::path& curdir)
		{
			if (!boost::filesystem::exists(filename)) {
				return false;
			}

			if (data_access_options_ & 1) {
				// check if filename is within _vBoostDataDirs
				boost::filesystem::path fullfilename = boost::filesystem::absolute(filename, curdir.empty() ? boost::filesystem::current_path() : curdir);
				_CustomNormalizePath(fullfilename);
				bool bfound = false;
				FOREACHC(itpath, _vBoostDataDirs) {
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
		std::map<int, EnvironmentBase*> _mapenvironments;
		std::list<boost::function<void()> > _listDestroyCallbacks;
		std::string _homedirectory;
		std::string _defaultviewertype; ///< the default viewer type from the environment variable OPENRAVE_DEFAULT_VIEWER
		std::vector<std::string> _vdbdirectories;
		int global_environment_id_;
		SpaceSamplerBasePtr _pdefaultsampler;
#ifdef USE_CRLIBM
		long long _crlibm_fpu_state;
		bool is_crlibm_init_; ///< true if crlibm is initialized
#endif
		int data_access_options_;

		std::vector<std::string> _vdatadirs;
#ifdef HAVE_BOOST_FILESYSTEM
		std::vector<boost::filesystem::path> _vBoostDataDirs; ///< \brief returns absolute filenames of the data
#endif

#if OPENRAVE_LOG4CXX
		log4cxx::LoggerPtr _logger;
#endif

		friend void RaveInitializeFromState(UserDataPtr);
		friend UserDataPtr RaveGlobalState();
	};



}