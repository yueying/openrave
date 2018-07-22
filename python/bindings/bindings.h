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
#ifndef OPENRAVE_BOOST_PYTHON_BINDINGS
#define OPENRAVE_BOOST_PYTHON_BINDINGS

#include <numpy/arrayobject.h>
#include <numpy/arrayscalars.h>
#include <Python.h>
#include <boost/array.hpp>
#include <boost/multi_array.hpp>
#include <memory>
#include <boost/format.hpp>
#include <boost/python.hpp>
#include <boost/assert.hpp>
#include <boost/cstdint.hpp>
#include <boost/version.hpp>
#include <stdint.h>

#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <stdexcept>

// apparently there's a problem with higher versions of C++
#if __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)
#include <typeinfo>
#define FOREACH(it, v) for(decltype((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(decltype((v).begin()) it = (v).begin(); it != (v).end(); )
#else
#define FOREACH(it, v) for(auto it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(auto it = (v).begin(); it != (v).end(); )
#endif

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#include <complex>
#include <algorithm>

#if BOOST_VERSION < 106500
#include <boost/python/numeric.hpp>
typedef typename boost::python:: ArrayFunc::array NumpyArrayType;
amespace ArrayFunc = boost::python::numeric;
#else
#include <boost/python/numpy.hpp>
typedef typename boost::python::numpy::ndarray NumpyArrayType;
namespace ArrayFunc = boost::python::numpy;
#endif

// is_none is not supported by older versions of python
#if BOOST_VERSION >= 104300
#define IS_PYTHONOBJECT_NONE(o) (o).is_none()
#else
#define IS_PYTHONOBJECT_NONE(o) (!!(o))
#endif

namespace openravepy 
{

	using namespace boost::python;

	inline boost::python::object ConvertStringToUnicode(const std::string& s)
	{
		return boost::python::object(boost::python::handle<>(PyUnicode_Decode(s.c_str(), s.size(), "utf-8", NULL)));
	}

	class PyVoidHandle
	{
	public:
		PyVoidHandle() {
		}
		PyVoidHandle(std::shared_ptr<void> handle) : _handle(handle) {
		}
		void Close() {
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

	template <typename T>
	inline std::vector<T> ExtractArray(const object& o)
	{
		if (IS_PYTHONOBJECT_NONE(o)) {
			return std::vector<T>();
		}
		std::vector<T> v(len(o));
		for (size_t i = 0; i < v.size(); ++i) {
			v[i] = boost::python::extract<T>(o[i]);
		}
		return v;
	}

	template <typename T>
	inline std::set<T> ExtractSet(const object& o)
	{
		std::set<T> v;
		size_t nlen = len(o);
		for (size_t i = 0; i < nlen; ++i) {
			v.insert(extract<T>(o[i]));
		}
		return v;
	}

	/// class for truly registering a C++ exception to a python exception
	/// add this definition to BOOST_PYTHON_MODULE:
	//
	//    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
	//    class_< T >( "_custom_exception_" )
	//        .def( init<const std::string&>() )
	//        .def( init<const T&>() )
	//        .def( "message", &T::message, return_copy_const_ref() )
	//        .def( "__str__", &T::message, return_copy_const_ref() )
	//        ;
	// inside python do:
	//
	//class custom_exception(Exception):
	//    """wrap up the C++ custom_exception"""
	//    def __init__( self, app_error ):
	//        Exception.__init__( self )
	//        self._pimpl = app_error
	//    def __str__( self ):
	//        return self._pimpl.message()
	//    def __getattribute__(self, attr):
	//        my_pimpl = super(custom_exception, self).__getattribute__("_pimpl")
	//        try:
	//            return getattr(my_pimpl, attr)
	//        except AttributeError:
	//            return super(custom_exception,self).__getattribute__(attr)
	//
	// import custom_module
	// custom_module._custom_exception_.py_err_class = custom_exception

	//template <typename ExceptionType>
	//struct ExceptionTranslator
	//{
	//   typedef ExceptionTranslator<ExceptionType> type;
	//
	//   static PyObject *pyException;
	//
	//   static void RegisterExceptionTranslator(scope & scope, const char*
	//moduleName, const char* name)
	//   {
	//     // Add the exception to the module scope
	//     std::strstream exName;
	//     exName << moduleName << "." << name << '\0';
	//     pyException = PyErr_NewException(exName.str(), NULL, NULL);
	//     handle<> instanceException(pyException);
	//     scope.attr(name) = object(instanceException);
	//
	//     // Register a translator for the type
	//     register_exception_translator< ExceptionType >
	//       (
	//        &ExceptionTranslator<ExceptionType>::translateException
	//        );
	//   }
	//
	//   static void translateException(const ExceptionType& ex)
	//   {
	//     PyErr_SetString(pyException, ex.getMessage().ptr());
	//   }
	//};
	//
	//template<typename ExceptionType>
	//PyObject* ExceptionTranslator<ExceptionType>::pyException;
	//
	//// Convenience macro
	//#define REGISTER_EXCEPTION(scopeRef, moduleName, className) ExceptionTranslator<className>::RegisterExceptionTranslator(scopeRef, moduleName, #className)
	//
	//
	//// Module
	//======================================================================
	//BOOST_PYTHON_MODULE(my_module)
	//{
	//
	//   scope moduleScope;
	//
	//   REGISTER_EXCEPTION(moduleScope, "my_module", InstanceException);
	//
	//.....
	//}

	template <typename T>
	struct exception_translator
	{
		exception_translator() {
			register_exception_translator<T>(&exception_translator::translate);

			//Register custom r-value converter
			//There are situations, where we have to pass the exception back to
			//C++ library. This will do the trick
			converter::registry::push_back(&exception_translator::convertible, &exception_translator::construct, type_id<T>());
		}

		static void translate(const T& err)
		{
			object pimpl_err(err);
			object pyerr_class = pimpl_err.attr("py_err_class");
			object pyerr = pyerr_class(pimpl_err);
			PyErr_SetObject(pyerr_class.ptr(), incref(pyerr.ptr()));
		}

		//Sometimes, exceptions should be passed back to the library.
		static void* convertible(PyObject* py_obj) {
			if (1 != PyObject_IsInstance(py_obj, PyExc_Exception)) {
				return 0;
			}

			if (!PyObject_HasAttrString(py_obj, "_pimpl")) {
				return 0;
			}

			object pyerr(handle<>(borrowed(py_obj)));
			object pimpl = getattr(pyerr, "_pimpl");
			extract<T> type_checker(pimpl);
			if (!type_checker.check()) {
				return 0;
			}
			return py_obj;
		}

		static void construct(PyObject* py_obj, converter::rvalue_from_python_stage1_data* data)
		{
			typedef converter::rvalue_from_python_storage<T> storage_t;

			object pyerr(handle<>(borrowed(py_obj)));
			object pimpl = getattr(pyerr, "_pimpl");

			storage_t* the_storage = reinterpret_cast<storage_t*>(data);
			void* memory_chunk = the_storage->storage.bytes;
			new (memory_chunk) T(extract<T>(pimpl));
			data->convertible = memory_chunk;
		}
	};

	// register const versions of the classes
	//template <class T> inline T* get_pointer( std::shared_ptr<const T>
	//const& p){
	//     return const_cast<T*>(p.get());
	//}
	//
	//template <class T> struct pintee< std::shared_ptr<const T> >{
	//     typedef T type;
	//};
	//
	//boost::python::register_ptr_to_python< std::shared_ptr<const my_class> >();

	template<typename T>
	struct float_from_number
	{
		float_from_number()
		{
			converter::registry::push_back(&convertible, &construct, type_id<T>());
		}

		static void* convertible(PyObject* obj)
		{
			return PyNumber_Check(obj) ? obj : NULL;
		}

		static void construct(PyObject* _obj, converter::rvalue_from_python_stage1_data* data)
		{
			PyObject* tmp = PyNumber_Float(_obj);
			T* storage = (T*)((converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
			*storage = boost::python::extract<T>(tmp);
			Py_DECREF(tmp);
			data->convertible = storage;
		}
	};

	template<typename T>
	struct int_from_number
	{
		int_from_number()
		{
			converter::registry::push_back(&convertible, &construct, type_id<T>());
		}

		static void* convertible(PyObject* obj)
		{
			return PyNumber_Check(obj) ? obj : NULL;
		}

		static void construct(PyObject* _obj, converter::rvalue_from_python_stage1_data* data)
		{
			PyObject* tmp = PyNumber_Long(_obj);
			T* storage = (T*)((converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
			*storage = boost::python::extract<T>(tmp);
			Py_DECREF(tmp);
			data->convertible = storage;
		}
	};

	inline std::string GetPyErrorString()
	{
		PyObject *error, *value, *traceback, *string;
		PyErr_Fetch(&error, &value, &traceback);
		PyErr_NormalizeException(&error, &value, &traceback);
		std::string s;
		if (error != NULL) {
			string = PyObject_Str(value);
			if (string != NULL) {
				s.assign(PyString_AsString(string));
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

#ifdef OPENRAVE_BININGS_PYARRAY

	inline boost::python::object toPyArrayN(const float* pvalues, size_t N)
	{
		if (N == 0)
		{
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<float>());
		}
		npy_intp dims[] = { npy_intp(N) };
		PyObject *pyvalues = PyArray_SimpleNew(1, dims, PyArray_FLOAT);
		if (pvalues != NULL)
		{
			memcpy(PyArray_DATA(pyvalues), pvalues, N * sizeof(float));
		}
		return object(handle<>(pyvalues));
	}

	inline boost::python::object toPyArrayN(const float* pvalues, std::vector<npy_intp>& dims)
	{
		if (dims.size() == 0)
		{
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<float>());
		}
		size_t totalsize = 1;
		FOREACH(it, dims)
		{
			totalsize *= *it;
		}
		if (totalsize == 0)
		{
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<float>());
		}
		PyObject *pyvalues = PyArray_SimpleNew(dims.size(), &dims[0], PyArray_FLOAT);
		if (pvalues != NULL)
		{
			memcpy(PyArray_DATA(pyvalues), pvalues, totalsize * sizeof(float));
		}
		return object(handle<>(pyvalues));
	}

	inline boost::python::object toPyArrayN(const double* pvalues, size_t N)
	{
		if (N == 0)
		{
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<double>());
		}
		npy_intp dims[] = { npy_intp(N) };
		PyObject *pyvalues = PyArray_SimpleNew(1, dims, PyArray_DOUBLE);
		if (pvalues != NULL) {
			memcpy(PyArray_DATA(pyvalues), pvalues, N * sizeof(double));
		}
		return object(handle<>(pyvalues));
	}

	inline boost::python::object toPyArrayN(const double* pvalues, std::vector<npy_intp>& dims)
	{
		if (dims.size() == 0)
		{
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<double>());
		}
		size_t totalsize = 1;
		FOREACH(it, dims) {
			totalsize *= *it;
		}
		if (totalsize == 0) {
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<double>());
		}
		PyObject *pyvalues = PyArray_SimpleNew(dims.size(), &dims[0], PyArray_DOUBLE);
		if (pvalues != NULL) {
			memcpy(PyArray_DATA(pyvalues), pvalues, totalsize * sizeof(double));
		}
		return object(handle<>(pyvalues));
	}

	inline boost::python::object toPyArrayN(const uint8_t* pvalues, std::vector<npy_intp>& dims)
	{
		if (dims.size() == 0) {
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<uint8_t>());
		}
		size_t totalsize = 1;
		for (size_t i = 0; i < dims.size(); ++i) {
			totalsize *= dims[i];
		}
		if (totalsize == 0) {
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<uint8_t>());
		}
		PyObject *pyvalues = PyArray_SimpleNew(dims.size(), &dims[0], PyArray_UINT8);
		if (pvalues != NULL) {
			memcpy(PyArray_DATA(pyvalues), pvalues, totalsize * sizeof(uint8_t));
		}
		return object(handle<>(pyvalues));
	}

	inline boost::python::object toPyArrayN(const uint8_t* pvalues, size_t N)
	{
		if (N == 0)
		{
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<uint8_t>());
		}
		npy_intp dims[] = { npy_intp(N) };
		PyObject *pyvalues = PyArray_SimpleNew(1, &dims[0], PyArray_UINT8);
		if (pvalues != NULL)
		{
			memcpy(PyArray_DATA(pyvalues), pvalues, N * sizeof(uint8_t));
		}
		return object(handle<>(pyvalues));
	}

	inline boost::python::object toPyArrayN(const int* pvalues, size_t N)
	{
		if (N == 0)
		{
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<uint32_t>());
		}
		npy_intp dims[] = { npy_intp(N) };
		PyObject *pyvalues = PyArray_SimpleNew(1, &dims[0], PyArray_INT32);
		if (pvalues != NULL)
		{
			memcpy(PyArray_DATA(pyvalues), pvalues, N * sizeof(int));
		}
		return object(handle<>(pyvalues));
	}

	inline boost::python::object toPyArrayN(const uint32_t* pvalues, size_t N)
	{
		if (N == 0)
		{
			return ArrayFunc::array(boost::python::list(), ArrayFunc::dtype::get_builtin<uint32_t>());
		}
		npy_intp dims[] = { npy_intp(N) };
		PyObject *pyvalues = PyArray_SimpleNew(1, &dims[0], PyArray_UINT32);
		if (pvalues != NULL) {
			memcpy(PyArray_DATA(pyvalues), pvalues, N * sizeof(uint32_t));
		}
		return object(handle<>(pyvalues));
	}

	template <typename T>
	inline object toPyList(const std::vector<T>& v)
	{
		boost::python::list lvalues;
		FOREACHC(it, v)
		{
			lvalues.append(object(*it));
		}
		return lvalues;
	}

	template <typename T>
	inline boost::python::object toPyArray(const std::vector<T>& v)
	{
		if (v.size() == 0)
		{
			return toPyArrayN((T*)NULL, 0);
		}
		return toPyArrayN(&v[0], v.size());
	}

	template <typename T>
	inline boost::python::object toPyArray(const std::vector<T>& v, std::vector<npy_intp>& dims)
	{
		if (v.size() == 0) {
			return toPyArrayN((T*)NULL, dims);
		}
		size_t totalsize = 1;
		FOREACH(it, dims)
			totalsize *= *it;
		BOOST_ASSERT(totalsize == v.size());
		return toPyArrayN(&v[0], dims);
	}

	template <typename T, int N>
	inline boost::python::object toPyArray(const boost::array<T, N>& v)
	{
		if (v.size() == 0) {
			return toPyArrayN((T*)NULL, 0);
		}
		return toPyArrayN(&v[0], v.size());
	}

#endif

}

#endif
