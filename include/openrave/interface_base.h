﻿// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file   interface_base.h
	\brief  Base interface definition that all exported interfaces derive from.

	Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_INTERFACE_BASE_
#define OPENRAVE_INTERFACE_BASE_

#if OPENRAVE_RAPIDJSON
#include <rapidjson/document.h>
#endif // OPENRAVE_RAPIDJSON
#include <openrave/xml_process.h>
#include <openrave/environment_base.h>

namespace OpenRAVE
{
	/// \brief Enumeration of all the interfaces.
	enum InterfaceType
	{
		PT_Planner = 1, ///< describes \ref PlannerBase interface
		PT_Robot = 2, ///< describes \ref RobotBase interface
		PT_SensorSystem = 3, ///< describes \ref SensorSystemBase interface
		PT_Controller = 4, ///< describes \ref ControllerBase interface
		PT_Module = 5, ///< describes \ref ModuleBase interface
		PT_ProblemInstance = 5, ///< describes \ref ModuleBase interface
		PT_IkSolver = 6, ///< describes \ref IkSolverBase interface
		PT_InverseKinematicsSolver = 6, ///< describes \ref IkSolverBase interface
		PT_KinBody = 7, ///< describes \ref KinBody
		PT_PhysicsEngine = 8, ///< describes \ref PhysicsEngineBase
		PT_Sensor = 9, ///< describes \ref SensorBase
		PT_CollisionChecker = 10, ///< describes \ref CollisionCheckerBase
		PT_Trajectory = 11, ///< describes \ref TrajectoryBase
		PT_Viewer = 12, ///< describes \ref ViewerBase
		PT_SpaceSampler = 13, ///< describes \ref SamplerBase
		PT_NumberOfInterfaces = 13 ///< number of interfaces, do not forget to update
	};

	/// serialization options for interfaces
	enum SerializationOptions
	{
		SO_Kinematics = 0x01, ///< kinematics information
		SO_Dynamics = 0x02, ///< dynamics information
		SO_BodyState = 0x04, ///< state of the body
		SO_NamesAndFiles = 0x08, ///< resource files and names
		SO_RobotManipulators = 0x10, ///< serialize robot manipulators
		SO_RobotSensors = 0x20, ///< serialize robot sensors
		SO_Geometry = 0x40, ///< geometry information (for collision detection)
		SO_InverseKinematics = 0x80, ///< information necessary for inverse kinematics. If Transform6D, then don't include the manipulator local transform
		SO_JointLimits = 0x100 ///< information of joint limits including velocity, acceleration, jerk, torque and inertia limits
	};

	/** \brief <b>[interface]</b> Base class for all interfaces that OpenRAVE provides. See \ref interface_concepts.
		\ingroup interfaces
	 */
	class OPENRAVE_API InterfaceBase : public std::enable_shared_from_this<InterfaceBase>
	{
	public:
		typedef std::map<std::string, XMLReadablePtr, CaseInsensitiveCompare> READERSMAP;

		InterfaceBase(InterfaceType type, EnvironmentBasePtr penv);
		virtual ~InterfaceBase();

		inline InterfaceType GetInterfaceType() const
		{
			return __type;
		}

		/// set internally by RaveDatabase <b>[multi-thread safe]</b>
		/// \return the unique identifier that describes this class type, case is ignored
		/// should be the same id used to create the object
		inline const std::string& GetXMLId() const 
		{
			return __strxmlid;
		}

		/// set internally by RaveDatabase <b>[multi-thread safe]</b>
		/// \return the pluginname this interface was loaded from
		inline const std::string& GetPluginName() const 
		{
			return __strpluginname;
		}

		/// \return The environment that this interface is attached to. <b>[multi-thread safe]</b>
		inline EnvironmentBasePtr GetEnv() const 
		{
			return __penv;
		}

		/// \brief Returns the raw map reference, this is \b not multithread safe and the GetInterfaceMutex should be locked before using.
		inline const READERSMAP& GetReadableInterfaces() const
		{
			return __mapReadableInterfaces;
		}

		/// \brief Returns the readable interface. <b>[multi-thread safe]</b>
		virtual XMLReadablePtr GetReadableInterface(const std::string& xmltag) const;

		/// \brief Set a new readable interface and return the previously set interface if it exists. <b>[multi-thread safe]</b>
		virtual XMLReadablePtr SetReadableInterface(const std::string& xmltag, XMLReadablePtr readable);

		/// \brief Documentation of the interface in reStructuredText format. See \ref writing_plugins_doc. <b>[multi-thread safe]</b>
		virtual const std::string& GetDescription() const 
		{
			return __description;
		}

		/// \brief sets a description <b>[multi-thread safe]</b>
		virtual void SetDescription(const std::string& description)
		{
			__description = description;
		}

		/// \brief set user data for a specific key. <b>[multi-thread safe]</b>
		///
		/// Because user data can be used for caching objects, it is necessary to allow functions taking const pointers of the interface can reset the pointers.
		virtual void SetUserData(const std::string& key, UserDataPtr data) const;

		/// \brief return the user custom data <b>[multi-thread safe]</b>
		virtual UserDataPtr GetUserData(const std::string& key = std::string()) const;

		/// \brief removes a user data pointer. if user data pointer does not exist, then return 0, otherwise 1. <b>[multi-thread safe]</b>
		virtual bool RemoveUserData(const std::string& key) const;

		/// \deprecated (12/12/11)
		virtual void SetUserData(UserDataPtr data) RAVE_DEPRECATED 
		{
			SetUserData(std::string(), data);
		}

		/// \brief the URI used to load the interface. <b>[multi-thread safe]</b>
		///
		/// Sometimes the URI could hold special markers like "#" like in COLLADA files in order to target objects insides a particular file.
		virtual const std::string& GetURI() const 
		{
			return __struri;
		}

		virtual const std::string& GetXMLFilename() const 
		{
			return __struri;
		}

		/// \brief Clone the contents of an interface to the current interface.
		///
		/// \param preference the interface whose information to clone
		/// \param cloningoptions mask of CloningOptions
		/// \throw OpenRAVEException if command doesn't succeed
		virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions);

		/// \brief return true if the command is supported
		virtual bool SupportsCommand(const std::string& cmd);

#if OPENRAVE_RAPIDJSON
		/// \brief return true if the command is supported
		virtual bool SupportsJSONCommand(const std::string& cmd);

#endif // OPENRAVE_RAPIDJSON

		/** \brief Used to send special commands to the interface and receive output.

			The command must be registered by \ref RegisterCommand. A special command '\b help' is
			always supported and provides a way for the user to query the current commands and the help
			string. The format of the returned help commands are in reStructuredText. The following commands are possible:
			- '\b help [command name]' - get the help string of just that command.
			- '\b help commands' - return the names of all the possible commands

			\param is the input stream containing the command
			\param os the output stream containing the output
			\exception OpenRAVEException Throw if the command is not supported.
			\return true if the command is successfully processed, otherwise false.
		 */
		virtual bool SendCommand(std::ostream& os, std::istream& is);

		/// \biref Similar to \ref SendCommand except the inputs and outputs are a string
		///
		/// This function should not be overridden by the user, therefore it isn't virtual.
		/// It is is slower than \ref SendCommand since it has to cast the strings to stringstreams.
		inline bool SendCommand(std::string& output, const std::string& input)
		{
			std::stringstream soutput, sinput(input);
			bool bSuccess = SendCommand(soutput, sinput);
			if (bSuccess) 
			{
				output = soutput.str();
			}
			return bSuccess;
		}

#if OPENRAVE_RAPIDJSON
		/** \brief Used to send special JSON commands to the interface and receive output.

			The command must be registered by \ref RegisterJSONCommand. A special command '\b help' is
			always supported and provides a way for the user to query the current commands and the help
			string.

			\param cmdname command name
			\param input the input rapidjson value
			\param output the output rapidjson value
			\param allocator allocator used to set alue on output rapidjson value
			\exception OpenRAVEException Throw if the command is not supported.
		 */
		virtual void SendJSONCommand(const std::string& cmdname, const rapidjson::Value& input, rapidjson::Value& output, rapidjson::Document::AllocatorType& allocator);

		/// \biref Similar to \ref SendJSONCommand except the output is a rapidjson document
		///
		/// This function should not be overridden by the user, therefore it isn't virtual.
		inline void SendJSONCommand(const std::string& cmdname, const rapidjson::Value& input, rapidjson::Document& output) {
			SendJSONCommand(cmdname, input, output, output.GetAllocator());
		}

#endif // OPENRAVE_RAPIDJSON

		/** \brief serializes the interface

			The readable interfaces are also serialized within the tag, for example:

			\code{.xml}
			<sometag> <!-- root writer -->
			  <interface> <!-- first child -->
				<readableinterface/> <!-- readable interface -->
			  </interface>
			</sometag>
			\endcode

			Depending on the writer format, extra tags might be created.
		 */
		virtual void Serialize(BaseXMLWriterPtr writer, int options = 0) const;

	protected:
		/// \brief The function to be executed for every command.
		///
		/// \param sinput - input of the command
		/// \param sout - output of the command
		/// \return If false, there was an error with the command, true if successful
		typedef boost::function<bool(std::ostream&, std::istream&)> InterfaceCommandFn;
		class OPENRAVE_API InterfaceCommand
		{
		public:
			InterfaceCommand() 
			{
			}

			InterfaceCommand(InterfaceCommandFn newfn, const std::string& newhelp)
				: fn(newfn), help(newhelp) 
			{
			}
			InterfaceCommandFn fn; ///< command function to run
			std::string help; ///< help string explaining command arguments
		};

		/// \brief Registers a command and its help string. <b>[multi-thread safe]</b>
		///
		/// \param cmdname - command name, converted to lower case
		/// \param fncmd function to execute for the command
		/// \param strhelp - help string in reStructuredText, see \ref writing_plugins_doc.
		/// \exception OpenRAVEException Throw if there exists a registered command already.
		virtual void RegisterCommand(const std::string& cmdname, InterfaceCommandFn fncmd, const std::string& strhelp);

		/// \brief Unregisters the command. <b>[multi-thread safe]</b>
		virtual void UnregisterCommand(const std::string& cmdname);

#if OPENRAVE_RAPIDJSON

		/// \brief The function to be executed for every JSON command.
		///
		/// \param input - input of the command
		/// \param output - output of the command
		/// \return If false, there was an error with the command, true if successful
		typedef boost::function<void(const rapidjson::Value&, rapidjson::Value&, rapidjson::Document::AllocatorType&)> InterfaceJSONCommandFn;
		class OPENRAVE_API InterfaceJSONCommand
		{
		public:
			InterfaceJSONCommand() 
			{
			}
			InterfaceJSONCommand(InterfaceJSONCommandFn newfn, const std::string& newhelp)
				: fn(newfn), help(newhelp) 
			{
			}
			InterfaceJSONCommandFn fn; ///< command function to run
			std::string help; ///< help string explaining command arguments
		};

		/// \brief Registers a command and its help string. <b>[multi-thread safe]</b>
		///
		/// \param cmdname - command name, converted to lower case
		/// \param fncmd function to execute for the command
		/// \param strhelp - help string in reStructuredText, see \ref writing_plugins_doc.
		/// \exception OpenRAVEException Throw if there exists a registered command already.
		virtual void RegisterJSONCommand(const std::string& cmdname, InterfaceJSONCommandFn fncmd, const std::string& strhelp);

		/// \brief Unregisters the command. <b>[multi-thread safe]</b>
		virtual void UnregisterJSONCommand(const std::string& cmdname);

#endif // OPENRAVE_RAPIDJSON

		virtual const char* GetHash() const = 0;
		std::string __description;     /// \see GetDescription()
		std::string __struri; ///< \see GetURI

		virtual boost::shared_mutex& GetInterfaceMutex() const 
		{
			return _mutexInterface;
		}

	private:
		/// Write the help commands to an output stream
		virtual bool _GetCommandHelp(std::ostream& sout, std::istream& sinput) const;

#if OPENRAVE_RAPIDJSON
		/// Write the help commands to an output stream
		virtual void _GetJSONCommandHelp(const rapidjson::Value& input, rapidjson::Value& output, rapidjson::Document::AllocatorType& allocator) const;
#endif // OPENRAVE_RAPIDJSON

		inline InterfaceBase& operator=(const InterfaceBase&r)
		{
			throw OpenRAVEException("InterfaceBase copying not allowed");
		}

		mutable boost::shared_mutex _mutexInterface; ///< internal mutex for protecting data from methods that might be access from any thread (those methods should be commented).
		InterfaceType __type; ///< \see GetInterfaceType
		UserDataPtr __plugin; ///< handle to plugin that controls the executable code. As long as this plugin pointer is present, module will not be unloaded.
		std::string __strpluginname; ///< the name of the plugin, necessary?
		std::string __strxmlid; ///< \see GetXMLId
		EnvironmentBasePtr __penv; ///< \see GetEnv
		mutable std::map<std::string, UserDataPtr> __mapUserData; ///< \see GetUserData

		READERSMAP __mapReadableInterfaces; ///< pointers to extra interfaces that are included with this object
		typedef std::map<std::string, std::shared_ptr<InterfaceCommand>, CaseInsensitiveCompare> CMDMAP;
		CMDMAP __mapCommands; ///< all registered commands

#if OPENRAVE_RAPIDJSON
		typedef std::map<std::string, std::shared_ptr<InterfaceJSONCommand>, CaseInsensitiveCompare> JSONCMDMAP;
		JSONCMDMAP __mapJSONCommands; ///< all registered commands
#endif

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
		friend class Environment;
		friend class OpenRAVEXMLParser::InterfaceXMLReader;
		friend class XFileReader;
#else
		friend class ::Environment;
		friend class ::OpenRAVEXMLParser::InterfaceXMLReader;
		friend class ::XFileReader;
#endif
#endif
		friend class ColladaReader;
		friend class RaveDatabase;
	};

} // end namespace OpenRAVE

#endif //OPENRAVE_INTERFACE_BASE_
