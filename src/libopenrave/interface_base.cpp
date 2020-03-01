// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
#include <openrave/interface_base.h>
#include <openrave/openrave.h>
#include <openrave/openrave_exception.h>
#include <openrave/openrave_macros.h>
#include <openrave/utils.h>

namespace OpenRAVE
{
	InterfaceBase::InterfaceBase(InterfaceType type, EnvironmentBasePtr penv)
		: type_(type), environment_(penv)
	{
		RaveInitializeFromState(penv->GlobalState()); // make sure global state is set
		RegisterCommand("help", std::bind(&InterfaceBase::_GetCommandHelp,
			this, std::placeholders::_1, std::placeholders::_2), "display help commands.");
#if OPENRAVE_RAPIDJSON
		RegisterJSONCommand("help", std::bind(&InterfaceBase::_GetJSONCommandHelp,
			this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), "display help commands.");
#endif // OPENRAVE_RAPIDJSON
	}

	InterfaceBase::~InterfaceBase()
	{
		boost::unique_lock< boost::shared_mutex > lock(mutex_interface_);
		commands_map_.clear();
		user_data_map_.clear();
		readable_interfaces_map_.clear();
		environment_.reset();
#if OPENRAVE_RAPIDJSON
		json_commands_map_.clear();
#endif // OPENRAVE_RAPIDJSON
	}

	void InterfaceBase::SetUserData(const std::string& key, UserDataPtr data) const
	{
		UserDataPtr olduserdata;
		{
			boost::unique_lock< boost::shared_mutex > lock(mutex_interface_);
			std::map<std::string, UserDataPtr>::iterator it = user_data_map_.find(key);
			if (it == user_data_map_.end())
			{
				user_data_map_[key] = data;
			}
			else
			{
				olduserdata = it->second;
				it->second = data;
			}
		}
		olduserdata.reset();
	}

	UserDataPtr InterfaceBase::GetUserData(const std::string& key) const
	{
		boost::shared_lock< boost::shared_mutex > lock(mutex_interface_);
		std::map<std::string, UserDataPtr>::const_iterator it = user_data_map_.find(key);
		if (it == user_data_map_.end())
		{
			return UserDataPtr();
		}
		return it->second;
	}

	bool InterfaceBase::RemoveUserData(const std::string& key) const
	{
		// have to destroy the userdata pointer outside the lock, otherwise can get into a deadlock
		UserDataPtr olduserdata;
		{
			boost::unique_lock< boost::shared_mutex > lock(mutex_interface_);
			std::map<std::string, UserDataPtr>::iterator it = user_data_map_.find(key);
			if (it == user_data_map_.end())
			{
				return false;
			}
			olduserdata = it->second;
			user_data_map_.erase(it);
		}
		olduserdata.reset();
		return true;
	}

	void InterfaceBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
	{
		if (!preference)
		{
			throw OpenRAVEException(("invalid cloning reference"), ORE_InvalidArguments);
		}
		// cannot clone the user data since it can be environment dependent!
		//user_data_map_ = preference->user_data_map_;
		str_uri_ = preference->str_uri_;
		readable_interfaces_map_ = preference->readable_interfaces_map_;
		description_ = preference->description_;
	}

	bool InterfaceBase::SupportsCommand(const std::string& cmd)
	{
		boost::shared_lock< boost::shared_mutex > lock(mutex_interface_);
		return commands_map_.find(cmd) != commands_map_.end();
	}

	bool InterfaceBase::SendCommand(std::ostream& sout, std::istream& sinput)
	{
		std::string cmd;
		sinput >> cmd;
		if (!sinput)
		{
			throw OpenRAVEException(("invalid command"), ORE_InvalidArguments);
		}
		std::shared_ptr<InterfaceCommand> interfacecmd;
		{
			boost::shared_lock< boost::shared_mutex > lock(mutex_interface_);
			CMDMAP::iterator it = commands_map_.find(cmd);
			if (it == commands_map_.end())
			{
				throw OpenRAVEException(format("failed to find command '%s' in interface %s\n"
					, cmd.c_str(), GetXMLId()), ORE_CommandNotSupported);
			}
			interfacecmd = it->second;
		}
		if (!interfacecmd->fn(sout, sinput))
		{
			RAVELOG_VERBOSE(format("command failed in interface %s: %s\n", GetXMLId(), cmd));
			return false;
		}
		return true;
	}

	void InterfaceBase::Serialize(BaseXMLWriterPtr writer, int options) const
	{
		for (auto it : readable_interfaces_map_)
		{
			// sometimes interfaces might be disabled
			if (!!it.second)
			{
				it.second->Serialize(writer, options);
			}
		}

	}

	void InterfaceBase::RegisterCommand(const std::string& cmdname,
		InterfaceBase::InterfaceCommandFn fncmd, const std::string& strhelp)
	{
		boost::unique_lock< boost::shared_mutex > lock(mutex_interface_);
		if ((cmdname.size() == 0)
			|| !utils::IsValidName(cmdname)
			|| (_stricmp(cmdname.c_str(), "commands") == 0))
		{
			throw OpenRAVEException(format("command '%s' invalid", cmdname), ORE_InvalidArguments);
		}
		if (commands_map_.find(cmdname) != commands_map_.end())
		{
			throw OpenRAVEException(format("command '%s' already registered", cmdname), ORE_InvalidArguments);
		}
		commands_map_[cmdname] = std::shared_ptr<InterfaceCommand>(new InterfaceCommand(fncmd, strhelp));
	}

	void InterfaceBase::UnregisterCommand(const std::string& cmdname)
	{
		boost::unique_lock< boost::shared_mutex > lock(mutex_interface_);
		CMDMAP::iterator it = commands_map_.find(cmdname);
		if (it != commands_map_.end())
		{
			commands_map_.erase(it);
		}
	}

	bool InterfaceBase::_GetCommandHelp(std::ostream& o, std::istream& sinput) const
	{
		boost::shared_lock< boost::shared_mutex > lock(mutex_interface_);
		std::string cmd, label;
		CMDMAP::const_iterator it;
		while (!sinput.eof())
		{
			sinput >> cmd;
			if (!sinput)
			{
				break;
			}
			std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

			if (cmd == "commands")
			{
				for (it = commands_map_.begin(); it != commands_map_.end(); ++it)
				{
					o << it->first << " ";
				}
				return true;
			}
			else if (cmd == "label")
			{
				sinput >> label;
			}
			else
			{
				it = commands_map_.find(cmd);
				if (it != commands_map_.end())
				{
					o << it->second->help;
					return true;
				}
			}

			if (!sinput)
			{
				RAVELOG_ERROR(format("failed processing command %s\n", cmd));
				return false;
			}
		}

		// display full help string
		o << std::endl << GetXMLId() << " Commands" << std::endl;
		for (size_t i = 0; i < GetXMLId().size(); ++i)
		{
			o << "=";
		}
		o << "=========" << std::endl << std::endl;
		for (it = commands_map_.begin(); it != commands_map_.end(); ++it)
		{
			if (label.size() > 0)
			{
				std::string strlower = it->first;
				std::transform(strlower.begin(), strlower.end(), strlower.begin(), ::tolower);
				o << std::endl << ".. _" << label << strlower << ":" << std::endl << std::endl;
			}
			o << std::endl << it->first << std::endl;
			for (size_t i = 0; i < it->first.size(); ++i)
			{
				o << "~";
			}
			o << std::endl << std::endl << it->second->help << std::endl << std::endl << "~~~~" << std::endl << std::endl;
		}
		return true;
	}

#if OPENRAVE_RAPIDJSON

	bool InterfaceBase::SupportsJSONCommand(const std::string& cmd)
	{
		boost::shared_lock< boost::shared_mutex > lock(mutex_interface_);
		return json_commands_map_.find(cmd) != json_commands_map_.end();
	}

	void InterfaceBase::RegisterJSONCommand(const std::string& cmdname,
		InterfaceBase::InterfaceJSONCommandFn fncmd, const std::string& strhelp)
	{
		boost::unique_lock< boost::shared_mutex > lock(mutex_interface_);
		if ((cmdname.size() == 0) || !utils::IsValidName(cmdname))
		{
			throw OpenRAVEException(format("command '%s' invalid", cmdname), ORE_InvalidArguments);
		}
		if (json_commands_map_.find(cmdname) != json_commands_map_.end())
		{
			throw OpenRAVEException(format("command '%s' already registered", cmdname), ORE_InvalidArguments);
		}
		json_commands_map_[cmdname] = std::shared_ptr<InterfaceJSONCommand>(new InterfaceJSONCommand(fncmd, strhelp));
	}

	void InterfaceBase::UnregisterJSONCommand(const std::string& cmdname)
	{
		boost::unique_lock< boost::shared_mutex > lock(mutex_interface_);
		JSONCMDMAP::iterator it = json_commands_map_.find(cmdname);
		if (it != json_commands_map_.end())
		{
			json_commands_map_.erase(it);
		}
	}

	void InterfaceBase::SendJSONCommand(const std::string& cmdname,
		const rapidjson::Value& input, rapidjson::Value& output,
		rapidjson::Document::AllocatorType& allocator)
	{
		output.SetNull();

		std::shared_ptr<InterfaceJSONCommand> interfacecmd;
		{
			boost::shared_lock< boost::shared_mutex > lock(mutex_interface_);
			JSONCMDMAP::iterator it = json_commands_map_.find(cmdname);
			if (it == json_commands_map_.end())
			{
				throw OpenRAVEException(format("failed to find JSON command '%s' in interface %s\n", 
					cmdname.c_str() , GetXMLId()), ORE_CommandNotSupported);
			}
			interfacecmd = it->second;
		}
		interfacecmd->fn(input, output, allocator);
	}

	void InterfaceBase::_GetJSONCommandHelp(const rapidjson::Value& input,
		rapidjson::Value& output, rapidjson::Document::AllocatorType& allocator) const
	{
		output.SetObject();

		for (JSONCMDMAP::const_iterator it = json_commands_map_.begin(); it != json_commands_map_.end(); ++it) 
		{
			output.AddMember(rapidjson::Value().SetString(it->first.c_str(), allocator), 
				rapidjson::Value().SetString(it->second->help.c_str(), allocator), allocator);
		}
	}

#endif // OPENRAVE_RAPIDJSON

	XMLReadablePtr InterfaceBase::GetReadableInterface(const std::string& xmltag) const
	{
		boost::shared_lock< boost::shared_mutex > lock(mutex_interface_);
		READERSMAP::const_iterator it = readable_interfaces_map_.find(xmltag);
		return it != readable_interfaces_map_.end() ? it->second : XMLReadablePtr();
	}

	XMLReadablePtr InterfaceBase::SetReadableInterface(const std::string& xmltag, XMLReadablePtr readable)
	{
		boost::unique_lock< boost::shared_mutex > lock(mutex_interface_);
		READERSMAP::iterator it = readable_interfaces_map_.find(xmltag);
		if (it == readable_interfaces_map_.end()) 
		{
			if (!!readable) 
			{
				readable_interfaces_map_[xmltag] = readable;
			}
			return XMLReadablePtr();
		}
		XMLReadablePtr pprev = it->second;
		if (!!readable) 
		{
			it->second = readable;
		}
		else 
		{
			readable_interfaces_map_.erase(it);
		}
		return pprev;
	}


}
