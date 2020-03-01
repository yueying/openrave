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

#ifndef OPENRAVE_OPENRAVE_MACROS_H_
#define OPENRAVE_OPENRAVE_MACROS_H_
#include <openrave/config.h>

#include <string>
#include <sstream> // ostringstream
#include <stdexcept> // logic_error
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <string>

#define FOREACH(it, v) for(auto it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(auto it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

namespace OpenRAVE
{

#if OPENRAVE_PRECISION // 1 if double precision
	typedef double dReal;
#define g_fEpsilon 1e-15
#else
	typedef float dReal;
#define g_fEpsilon 2e-7f
#endif

#define SERIALIZATION_PRECISION 4

#define GTS_M_ICOSAHEDRON_X /* sqrt(sqrt(5)+1)/sqrt(2*sqrt(5)) */ \
    (dReal)0.850650808352039932181540497063011072240401406
#define GTS_M_ICOSAHEDRON_Y /* sqrt(2)/sqrt(5+sqrt(5))         */ \
    (dReal)0.525731112119133606025669084847876607285497935
#define GTS_M_ICOSAHEDRON_Z (dReal)0.0
}
// <cmath>
#ifndef M_PI
#	define M_PI 3.14159265358979323846	// PI
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923   // pi/2
#endif

#ifndef M_2PI
#	define M_2PI 6.283185307179586476925286766559	// 2*PI
#endif

#define M_PIf  3.14159265358979f
#define M_2PIf 6.28318530717959f

#if defined(HAVE_LONG_DOUBLE) && !defined(M_PIl)
#	define M_PIl 3.14159265358979323846264338327950288L
#	define M_2PIl (2.0L*3.14159265358979323846264338327950288L)
#endif

#define OPENRAVE_DECLARE_PTR(Name, Type)              \
  typedef std::shared_ptr<Type> Name##Ptr;            \
  typedef std::shared_ptr<const Type> Name##ConstPtr; \
  typedef std::weak_ptr<Type> Name##WeakPtr;

#define OPENRAVE_CLASS_FORWARD(C)         \
  class C;                             \
  OPENRAVE_DECLARE_PTR(C, C);

#define OPENRAVE_STRUCT_FORWARD(C)       \
  struct C;                           \
  OPENRAVE_DECLARE_PTR(C, C);

namespace OpenRAVE
{
	std::string OPENRAVE_API format(const char *fmt, ...);

	/** \brief Returns the gettext translated string of the given message id

	\param domainname translation domain name
	\param msgid message id to look for
	\return if a translation was found, it is converted to the locale's codeset and returned. The resulting string is statically allocated and must not be modified or freed. Otherwise msgid is returned.
 */
	OPENRAVE_API const char *RaveGetLocalizedTextForDomain(const std::string& domainname, const char *msgid);
}

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave", msgid)

#endif // OPENRAVE_OPENRAVE_MACROS_H_