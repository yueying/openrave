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

// shouldn't include openrave.h!
#ifndef OPENRAVE_PYTHON_BINDINGS_
#define OPENRAVE_PYTHON_BINDINGS_

#include <openravepy/openravepy_config.h>

#include <cstdint>
// numpy
#include <numpy/arrayobject.h>
#include <numpy/arrayscalars.h>

#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <stdexcept>


#define FOREACH(it, v) for(auto it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(auto it = (v).begin(); it != (v).end(); )
#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC
#include <memory>
#include <complex>
#include <algorithm>
// openrave
#include <openrave/config.h>
#include <openrave/logging.h>


#ifndef _RAVE_DISPLAY
#define _RAVE_DISPLAY(RUNCODE)                                               \
    {                                                                              \
        printf(                                                                    \
            "\n%s:%d, [ %s "                                                       \
            "]\n-----------------------------------------------------------------" \
            "--------------\n",                                                    \
            __FILE__, __LINE__, __func__ /*__PRETTY_FUNCTION__*/);                 \
        RUNCODE;                                                                   \
        printf("\n");                                                              \
    }
#endif // _RAVE_DISPLAY

#ifdef USE_PYBIND11_PYTHON_BINDINGS
#include "pybind11/pybind11_bindings.h"
#else
#include "boostpython/boostpython_bindings.h"
#endif

namespace openravepy
{

	template<typename T> inline NPY_TYPES getEnum(void)
	{
		PyErr_SetString(PyExc_ValueError, "no mapping available for this type");
		py::throw_error_already_set();
		return NPY_VOID;
	}

	template <>inline NPY_TYPES getEnum<unsigned char>(void) { return NPY_UBYTE; }

	template <>inline NPY_TYPES getEnum<signed char>(void) { return NPY_BYTE; }

	template <>inline NPY_TYPES getEnum<short>(void) { return NPY_SHORT; }

	template <>inline NPY_TYPES getEnum<unsigned short>(void) { return NPY_USHORT; }

	template <>inline NPY_TYPES getEnum<unsigned int>(void) { return NPY_UINT; }

	template <>inline NPY_TYPES getEnum<int>(void) { return NPY_INT; }

	template <>inline NPY_TYPES getEnum<long>(void) { return NPY_LONG; }

	template <>inline NPY_TYPES getEnum<unsigned long>(void) { return NPY_ULONG; }

	template <>inline NPY_TYPES getEnum<long long>(void) { return NPY_LONGLONG; }

	template <>inline NPY_TYPES getEnum<unsigned long long>(void) { return NPY_ULONGLONG; }

	template <>inline NPY_TYPES getEnum<float>(void) { return NPY_FLOAT; }

	template <>inline NPY_TYPES getEnum<double>(void) { return NPY_DOUBLE; }

	template <>inline NPY_TYPES getEnum<long double>(void) { return NPY_LONGDOUBLE; }

	template <>inline NPY_TYPES getEnum<std::complex<float> >(void) { return NPY_CFLOAT; }

	template <>inline NPY_TYPES getEnum<std::complex<double> >(void) { return NPY_CDOUBLE; }

	template <>inline NPY_TYPES getEnum<std::complex<long double> >(void) { return NPY_CLONGDOUBLE; }

} // namespace openravepy



namespace openravepy {

	class PyVoidHandle
	{
	public:
		PyVoidHandle()
		{
		}
		PyVoidHandle(std::shared_ptr<void> handle)
			: _handle(handle)
		{
		}
		void Close()
		{
			_handle.reset();
		}
		std::shared_ptr<void> _handle;
	};

	class PyVoidHandleConst
	{
	public:
		PyVoidHandleConst() {
		}
		PyVoidHandleConst(std::shared_ptr<void const> handle) : _handle(handle) {
		}
		void Close() {
			_handle.reset();
		}
		std::shared_ptr<void const> _handle;
	};

	inline std::vector<int8_t> ExtractArrayInt8(const py::object& o)
	{
		if (IS_PYTHONOBJECT_NONE(o)) {
			return {};
		}
		std::vector<int8_t> v;
		try {
			const size_t n = len(o);
			v.resize(n);
			for (size_t i = 0; i < n; ++i) {
				v[i] = (int8_t)(py::extract<int>(o[i]));
			}
		}
		catch (...) {
			RAVELOG_WARN("Cannot do ExtractArray for int");
		}
		return v;
	}

	template <typename T>
	inline std::vector<T> ExtractArray(const py::object& o)
	{
		if (IS_PYTHONOBJECT_NONE(o)) {
			return {};
		}
		std::vector<T> v;
		try {
			const size_t n = len(o);
			v.resize(n);
			for (size_t i = 0; i < n; ++i) {
				v[i] = py::extract<T>(o[i]);
			}
		}
		catch (...) {
			RAVELOG_WARN("Cannot do ExtractArray for " + std::string(typeid(T).name()));
		}
		return v;
	}

	template <typename T>
	inline std::set<T> ExtractSet(const py::object& o)
	{
		std::set<T> v;
		size_t nlen = len(o);
		for (size_t i = 0; i < nlen; ++i) {
			v.insert(py::extract<T>(o[i]));
		}
		return v;
	}

	inline std::string GetPyErrorString()
	{
		PyObject *error, *value, *traceback, *string;
		PyErr_Fetch(&error, &value, &traceback);
		PyErr_NormalizeException(&error, &value, &traceback);
		std::string s;
		if (error != nullptr) {
			string = PyObject_Str(value);
			if (string != nullptr) {
				s.assign(PyBytes_AsString(string));
				Py_DECREF(string);
			}
		}
		// Does nothing when the ptr is nullptr
		Py_DECREF(error);
		Py_DECREF(value);
		Py_DECREF(traceback);

		return s;
	}

	/// should call in the beginning of all BOOST_PYTHON_MODULE
	void init_python_bindings();

} // namespace openravepy

#endif // OPENRAVE_PYTHON_BINDINGS_
