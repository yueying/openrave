// -*- coding: utf-8 -*-
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
#ifndef OPENRAVE_OPENRAVE_EXCEPTION_H_
#define OPENRAVE_OPENRAVE_EXCEPTION_H_

#include <openrave/config.h>
#include <string>
#include <exception>
#include <boost/format.hpp>

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#else
#endif

namespace OpenRAVE
{
	/// %OpenRAVE error codes
	enum OpenRAVEErrorCode 
	{
		ORE_Failed = 0,
		ORE_InvalidArguments = 1, //!< passed in input arguments are not valid
		ORE_EnvironmentNotLocked = 2,
		ORE_CommandNotSupported = 3, //!< string command could not be parsed or is not supported
		ORE_Assert = 4,
		ORE_InvalidPlugin = 5, //!< shared object is not a valid plugin
		ORE_InvalidInterfaceHash = 6, //!< interface hashes do not match between plugins
		ORE_NotImplemented = 7, //!< function is not implemented by the interface.
		ORE_InconsistentConstraints = 8, //!< returned solutions or trajectories do not follow the constraints of the planner/module. The constraints invalidated here are planning constraints, not programming constraints.
		ORE_NotInitialized = 9, //!< when object is used without it getting fully initialized
		ORE_InvalidState = 10, //!< the state of the object is not consistent with its parameters, or cannot be used. This is usually due to a programming error where a vector is not the correct length, etc.
		ORE_Timeout = 11, //!< process timed out
	};

	inline const char* GetErrorCodeString(OpenRAVEErrorCode error)
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

	/// \brief Exception that all OpenRAVE internal methods throw; the error codes are held in \ref OpenRAVEErrorCode.
	class OPENRAVE_API OpenRAVEException : public std::exception
	{
	public:
		OpenRAVEException() : std::exception(), _s("unknown exception"), _error(ORE_Failed) 
		{
		}
		OpenRAVEException(const std::string& s, OpenRAVEErrorCode error = ORE_Failed) 
			: std::exception() 
		{
			_error = error;
			_s = "openrave (";
			_s += GetErrorCodeString(_error);
			_s += "): ";
			_s += s;
		}
		virtual ~OpenRAVEException() throw()
		{
		}

		char const* what() const throw()
		{
			return _s.c_str();
		}

		const std::string& message() const
		{
			return _s;
		}

		OpenRAVEErrorCode GetCode() const 
		{
			return _error;
		}
	private:
		std::string _s;
		OpenRAVEErrorCode _error;
	};

#define OPENRAVE_EXCEPTION_FORMAT0(s, errorcode) OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] %s")%(__PRETTY_FUNCTION__)%(__LINE__)%(s)),errorcode)

	/// adds the function name and line number to an openrave exception
#define OPENRAVE_EXCEPTION_FORMAT(s, args, errorcode) OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] ")%(__PRETTY_FUNCTION__)%(__LINE__)) + boost::str(boost::format(s)%args),errorcode)

#define OPENRAVE_ASSERT_FORMAT(testexpr, s, args, errorcode) { if( !(testexpr) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] (%s) failed ")%(__PRETTY_FUNCTION__)%(__LINE__)%(# testexpr)) + boost::str(boost::format(s)%args),errorcode); } }

#define OPENRAVE_ASSERT_FORMAT0(testexpr, s, errorcode) { if( !(testexpr) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] (%s) failed %s")%(__PRETTY_FUNCTION__)%(__LINE__)%(# testexpr)%(s)),errorcode); } }

	// note that expr1 and expr2 will be evaluated twice if not equal
#define OPENRAVE_ASSERT_OP_FORMAT(expr1,op,expr2,s, args, errorcode) { if( !((expr1) op (expr2)) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] %s %s %s, (eval %s %s %s) ")%(__PRETTY_FUNCTION__)%(__LINE__)%(# expr1)%(# op)%(# expr2)%(expr1)%(# op)%(expr2)) + boost::str(boost::format(s)%args),errorcode); } }

#define OPENRAVE_ASSERT_OP_FORMAT0(expr1,op,expr2,s, errorcode) { if( !((expr1) op (expr2)) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] %s %s %s, (eval %s %s %s) %s")%(__PRETTY_FUNCTION__)%(__LINE__)%(# expr1)%(# op)%(# expr2)%(expr1)%(# op)%(expr2)%(s)),errorcode); } }

#define OPENRAVE_ASSERT_OP(expr1,op,expr2) { if( !((expr1) op (expr2)) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] %s %s %s, (eval %s %s %s) ")%(__PRETTY_FUNCTION__)%(__LINE__)%(# expr1)%(# op)%(# expr2)%(expr1)%(# op)%(expr2)),ORE_Assert); } }

#define OPENRAVE_DUMMY_IMPLEMENTATION { throw OPENRAVE_EXCEPTION_FORMAT0("not implemented",ORE_NotImplemented); }
}

#endif // OPENRAVE_OPENRAVE_EXCEPTION_H_