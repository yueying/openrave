// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include <openrave/openrave_exception.h>

namespace OpenRAVE
{
	OpenRAVEException::OpenRAVEException()
		: std::exception(), exception_message_("unknown exception"), error_code_(ORE_Failed)
	{
	}

	OpenRAVEException::OpenRAVEException(const std::string& exception_message, OpenRAVEErrorCode error_code)
		: std::exception()
	{
		error_code_ = error_code;
		exception_message_ = "openrave (";
		exception_message_ += RaveGetErrorCodeString(error_code_);
		exception_message_ += "): ";
		exception_message_ += exception_message;
	}

	char const* OpenRAVEException::what() const throw() 
	{
		return exception_message_.c_str();
	}

	const std::string& OpenRAVEException::message() const 
	{
		return exception_message_;
	}

	OpenRAVEErrorCode OpenRAVEException::GetCode() const 
	{
		return error_code_;
	}

	const char* RaveGetErrorCodeString(OpenRAVEErrorCode error)
	{
		switch (error) 
		{
		case ORE_Failed: return "Failed";
		case ORE_InvalidArguments: return "InvalidArguments";
		case ORE_EnvironmentNotLocked: return "EnvironmentNotLocked";
		case ORE_CommandNotSupported: return "CommandNotSupported";
		case ORE_Assert: return "Assert";
		case ORE_InvalidPlugin: return "InvalidPlugin";
		case ORE_InvalidInterfaceHash: return "InvalidInterfaceHash";
		case ORE_NotImplemented: return "NotImplemented";
		case ORE_InconsistentConstraints: return "InconsistentConstraints";
		case ORE_NotInitialized: return "NotInitialized";
		case ORE_InvalidState: return "InvalidState";
		case ORE_Timeout: return "Timeout";
		}
		// should throw an exception?
		return "";
	}
}
