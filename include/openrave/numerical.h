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
#ifndef OPENRAVE_NUMERICAL_H_
#define OPENRAVE_NUMERICAL_H_

#include <openrave/config.h>
#include <openrave/geometry.h>
#include <openrave/mathextra.h>

namespace OpenRAVE
{
#if OPENRAVE_PRECISION // 1 if double precision
	typedef double dReal;
#define g_fEpsilon 1e-15
#else
	typedef float dReal;
#define g_fEpsilon 2e-7f
#endif

	/// \brief openrave constant for PI, could be replaced by accurate precision number depending on choice of dReal.
	static const dReal PI = dReal(3.14159265358979323846);

	/// Wrappers of common basic math functions, allows OpenRAVE to control the precision requirements.
	/// \ingroup affine_math
	//@{

	/// \brief exponential
	OPENRAVE_API dReal RaveExp(dReal f);
	/// \brief logarithm
	OPENRAVE_API dReal RaveLog(dReal f);
	/// \brief cosine
	OPENRAVE_API dReal RaveCos(dReal f);
	/// \brief sine
	OPENRAVE_API dReal RaveSin(dReal f);
	/// \brief tangent
	OPENRAVE_API dReal RaveTan(dReal f);
	/// \brief base 2 logarithm
	OPENRAVE_API dReal RaveLog2(dReal f);
	/// \brief base 10 logarithm
	OPENRAVE_API dReal RaveLog10(dReal f);
	/// \brief arccosine
	OPENRAVE_API dReal RaveAcos(dReal f);
	/// \brief arcsine
	OPENRAVE_API dReal RaveAsin(dReal f);
	/// \brief arctangent2 covering entire circle
	OPENRAVE_API dReal RaveAtan2(dReal fy, dReal fx);
	/// \brief power x^y
	OPENRAVE_API dReal RavePow(dReal fx, dReal fy);
	/// \brief square-root
	OPENRAVE_API dReal RaveSqrt(dReal f);
	/// \brief absolute value
	OPENRAVE_API dReal RaveFabs(dReal f);
	/// \brief ceil
	OPENRAVE_API dReal RaveCeil(dReal f);

	//@}

	// define the math functions
#if OPENRAVE_PRECISION // 1 if double precision
#define OPENRAVE_MATH_EXP_DOUBLE RaveExp
#define OPENRAVE_MATH_LOG_DOUBLE RaveLog
#define OPENRAVE_MATH_COS_DOUBLE RaveCos
#define OPENRAVE_MATH_SIN_DOUBLE RaveSin
#define OPENRAVE_MATH_TAN_DOUBLE RaveTan
#define OPENRAVE_MATH_LOG2_DOUBLE RaveLog2
#define OPENRAVE_MATH_LOG10_DOUBLE RaveLog10
#define OPENRAVE_MATH_ACOS_DOUBLE RaveAcos
#define OPENRAVE_MATH_ASIN_DOUBLE RaveAsin
#define OPENRAVE_MATH_ATAN2_DOUBLE RaveAtan2
#define OPENRAVE_MATH_POW_DOUBLE RavePow
#define OPENRAVE_MATH_SQRT_DOUBLE RaveSqrt
#define OPENRAVE_MATH_FABS_DOUBLE RaveFabs
#else // 32bit float
#define OPENRAVE_MATH_EXP_FLOAT RaveExp
#define OPENRAVE_MATH_LOG_FLOAT RaveLog
#define OPENRAVE_MATH_COS_FLOAT RaveCos
#define OPENRAVE_MATH_SIN_FLOAT RaveSin
#define OPENRAVE_MATH_TAN_FLOAT RaveTan
#define OPENRAVE_MATH_LOG2_FLOAT RaveLog2
#define OPENRAVE_MATH_LOG10_FLOAT RaveLog10
#define OPENRAVE_MATH_ACOS_FLOAT RaveAcos
#define OPENRAVE_MATH_ASIN_FLOAT RaveAsin
#define OPENRAVE_MATH_ATAN2_FLOAT RaveAtan2
#define OPENRAVE_MATH_POW_FLOAT RavePow
#define OPENRAVE_MATH_SQRT_FLOAT RaveSqrt
#define OPENRAVE_MATH_FABS_FLOAT RaveFabs
#endif

	using geometry::RaveVector;
	using geometry::RaveTransform;
	using geometry::RaveTransformMatrix;
	typedef RaveVector<dReal> Vector;
	typedef RaveTransform<dReal> Transform;
	typedef std::shared_ptr< RaveTransform<dReal> > TransformPtr;
	typedef std::shared_ptr< RaveTransform<dReal> const > TransformConstPtr;
	typedef RaveTransformMatrix<dReal> TransformMatrix;
	typedef std::shared_ptr< RaveTransformMatrix<dReal> > TransformMatrixPtr;
	typedef std::shared_ptr< RaveTransformMatrix<dReal> const > TransformMatrixConstPtr;
	typedef geometry::obb<dReal> OBB;
	typedef geometry::aabb<dReal> AABB;
	typedef geometry::ray<dReal> RAY;

	// for compatibility
	//@{
	using mathextra::dot2;
	using mathextra::dot3;
	using mathextra::dot4;
	using mathextra::normalize2;
	using mathextra::normalize3;
	using mathextra::normalize4;
	using mathextra::cross3;
	using mathextra::inv3;
	using mathextra::inv4;
	using mathextra::lengthsqr2;
	using mathextra::lengthsqr3;
	using mathextra::lengthsqr4;
	using mathextra::mult4;
	//@}

	template <typename T>
	inline T NormalizeCircularAnglePrivate(T theta, T min, T max)
	{
		if (theta < min)
		{
			T range = max - min;
			theta += range;
			while (theta < min)
			{
				theta += range;
			}
		}
		else if (theta > max)
		{
			T range = max - min;
			theta -= range;
			while (theta > max)
			{
				theta -= range;
			}
		}
		return theta;
	}

}

#endif // OPENRAVE_NUMERICAL_H_