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
#include <boost/thread/once.hpp>
#include <map>
#include <openrave/openrave.h>
#include <openrave/plugindatabase.h>
#include <openrave/utils.h>

namespace OpenRAVE
{
#ifdef _WIN32
	const char s_filesep = '\\';
#else
	const char s_filesep = '/';
#endif
	static boost::once_flag _onceRaveInitialize = BOOST_ONCE_INIT;
	/// there is only once global openrave state. It is created when openrave
/// is first used, and destroyed when the program quits or RaveDestroy is called.
	class RaveGlobal : private boost::noncopyable, public std::enable_shared_from_this<RaveGlobal>, public UserData
	{
		typedef std::map<std::string, CreateXMLReaderFn, CaseInsensitiveCompare> READERSMAP;

		RaveGlobal()
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
			for(auto it: _mapikparameterization)
			{
				std::string name = it.second;
				std::transform(name.begin(), name.end(), name.begin(), ::tolower);
				_mapikparameterizationlower[it.first] = name;
			}
		}
	public:
		virtual ~RaveGlobal()
		{
			Destroy();
		}

		static std::shared_ptr<RaveGlobal>& instance()
		{
			boost::call_once(_create, _onceRaveInitialize);
			return _state;
		}

		int Initialize(bool is_load_all_plugins, int level)
		{
			if (_IsInitialized())
			{
				return 0;     // already initialized
			}

			_InitializeLogging(level);

#ifdef USE_CRLIBM
			if (!_bcrlibmInit)
			{
				_crlibm_fpu_state = crlibm_init();
				_bcrlibmInit = true;
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
				RAVELOG_WARN("failed to set to C locale: %s\n", e.what());
			}

			_pdatabase.reset(new RaveDatabase());
			if (!_pdatabase->Init(is_load_all_plugins))
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

		void Destroy()
		{
			if (!!_pdatabase) {
				// notify all plugins that about to destroy
				_pdatabase->OnRavePreDestroy();
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
			_mapreaders.clear();

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

			if (!!_pdatabase) {
				// force destroy in case some one is holding a pointer to it
				_pdatabase->Destroy();
				_pdatabase.reset();
			}
#ifdef USE_CRLIBM

#ifdef HAS_FENV_H
			feclearexcept(-1); // clear any cached exceptions
#endif
			if (_bcrlibmInit) {
				crlibm_exit(_crlibm_fpu_state);
				_bcrlibmInit = false;
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
				_oldfn = global->_mapreaders[_type][_xmltag];
				global->_mapreaders[_type][_xmltag] = fn;
			}
			virtual ~XMLReaderFunctionData()
			{
				std::shared_ptr<RaveGlobal> global = _global.lock();
				if (!!global) {
					boost::mutex::scoped_lock lock(global->_mutexinternal);
					global->_mapreaders[_type][_xmltag] = _oldfn;
				}
			}
		protected:
			CreateXMLReaderFn _oldfn;
			std::weak_ptr<RaveGlobal> _global;
			InterfaceType _type;
			std::string _xmltag;
		};

		UserDataPtr RegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn)
		{
			return UserDataPtr(new XMLReaderFunctionData(type, xmltag, fn, shared_from_this()));
		}

		const BaseXMLReaderPtr CallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts)
		{
			READERSMAP::iterator it = _mapreaders[type].find(xmltag);
			if (it == _mapreaders[type].end()) {
				//throw OpenRAVEException(str(boost::format(_("No function registered for interface %s xml tag %s"))%GetInterfaceName(type)%xmltag),ORE_InvalidArguments);
				return BaseXMLReaderPtr();
			}
			return it->second(pinterface, atts);
		}

		std::shared_ptr<RaveDatabase> GetDatabase() const {
			return _pdatabase;
		}
		const std::map<InterfaceType, std::string>& GetInterfaceNamesMap() const {
			return _mapinterfacenames;
		}
		const std::map<IkParameterizationType, std::string>& GetIkParameterizationMap(int alllowercase = 0) {
			if (alllowercase) {
				return _mapikparameterizationlower;
			}
			return _mapikparameterization;
		}

		const std::string& GetInterfaceName(InterfaceType type)
		{
			std::map<InterfaceType, std::string>::const_iterator it = _mapinterfacenames.find(type);
			if (it == _mapinterfacenames.end()) {
				throw OPENRAVE_EXCEPTION_FORMAT(_("Invalid type %d specified"), type, ORE_Failed);
			}
			return it->second;
		}

		// have to take in pointer instead of shared_ptr since method will be called in EnvironmentBase constructor
		int RegisterEnvironment(EnvironmentBase* penv)
		{
			BOOST_ASSERT(!!_pdatabase);
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

		void SetDataAccess(int options) {
			boost::mutex::scoped_lock lock(_mutexinternal);
			_nDataAccessOptions = options;
		}
		int GetDataAccess() {
			boost::mutex::scoped_lock lock(_mutexinternal);
			return _nDataAccessOptions;
		}
		std::string GetDefaultViewerType() {
			if (_defaultviewertype.size() > 0) {
				return _defaultviewertype;
			}

			// get the first viewer that can be loadable, with preferenace to qtosg, qtcoin
			std::shared_ptr<RaveDatabase> pdatabase = _pdatabase;
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
		static void _create()
		{
			if (!_state) {
				_state.reset(new RaveGlobal());
			}
		}

		bool _IsInitialized() const {
			return !!_pdatabase;
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
			for(auto itname: _vdatadirs) 
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
			for(auto itdir: _vdatadirs)
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

			if (_nDataAccessOptions & 1) {
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

		void _InitializeLogging(int level) {
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
		static std::shared_ptr<RaveGlobal> _state;
		// state that is always present

		// state that is initialized/destroyed
		std::shared_ptr<RaveDatabase> _pdatabase;
		int debug_level_;
		boost::mutex _mutexinternal;
		std::map<InterfaceType, READERSMAP > _mapreaders;
		std::map<InterfaceType, std::string> _mapinterfacenames;
		std::map<IkParameterizationType, std::string> _mapikparameterization, _mapikparameterizationlower;
		std::map<int, EnvironmentBase*> _mapenvironments;
		std::list<boost::function<void()> > _listDestroyCallbacks;
		std::string _homedirectory;
		std::string _defaultviewertype; ///< the default viewer type from the environment variable OPENRAVE_DEFAULT_VIEWER
		std::vector<std::string> _vdbdirectories;
		int global_environment_id_;
		SpaceSamplerBasePtr _pdefaultsampler;
#ifdef USE_CRLIBM
		long long _crlibm_fpu_state;
		bool _bcrlibmInit; ///< true if crlibm is initialized
#endif
		int _nDataAccessOptions;

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