// Copyright (C) 2020 fengbing <fengbing123@gmail.com>
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
#include <openrave/openrave_macros.h>
#include <vector>
#include <cstdarg>

namespace OpenRAVE
{
	std::string format(const char *fmt, ...)
	{
		if (!fmt) return std::string("");

		int result = -1, length = 2048;
		std::vector<char> buffer;
		while (result == -1)
		{
			buffer.resize(length + 10);

			va_list args;
			va_start(args, fmt);
#if defined(_MSC_VER)
			result = ::vsnprintf_s(&buffer[0], length, _TRUNCATE, fmt, args);
#else
			result = ::vsnprintf(&buffer[0], length, fmt, args);
#endif
			va_end(args);

			if (result >= length) result = -1;
			length *= 2;
		}
		std::string s(&buffer[0]);
		return s;
	}

	const char *RaveGetLocalizedTextForDomain(const std::string& domainname, const char *msgid)
	{
#ifndef _WIN32
		if (_gettextDomainsInitialized.find(domainname) == _gettextDomainsInitialized.end())
		{
			bindtextdomain(domainname.c_str(), OPENRAVE_LOCALE_INSTALL_DIR);
			_gettextDomainsInitialized.insert(domainname);
		}
		return dgettext(domainname.c_str(), msgid);
#else
		return msgid;
#endif
	}
}
