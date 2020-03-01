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
#include <openrave/openrave_macros.h>

namespace OpenRAVE
{

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

	template <typename IKReal>
	inline void polyroots2(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
	{
		IKReal det = rawcoeffs[1] * rawcoeffs[1] - 4 * rawcoeffs[0] * rawcoeffs[2];
		if (det < 0) {
			numroots = 0;
		}
		else if (det == 0) {
			rawroots[0] = -0.5*rawcoeffs[1] / rawcoeffs[0];
			numroots = 1;
		}
		else {
			det = RaveSqrt(det);
			rawroots[0] = (-rawcoeffs[1] + det) / (2 * rawcoeffs[0]);
			rawroots[1] = (-rawcoeffs[1] - det) / (2 * rawcoeffs[0]); //rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
			numroots = 2;
		}
	}

	/// \brief Durand-Kerner polynomial root finding method
	template <typename IKReal, int D>
	inline void polyroots(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
	{
		using std::complex;
		BOOST_ASSERT(rawcoeffs[0] != 0);
		const IKReal tol = 128.0*std::numeric_limits<IKReal>::epsilon();
		const IKReal tolsqrt = sqrt(std::numeric_limits<IKReal>::epsilon());
		complex<IKReal> coeffs[D];
		const int maxsteps = 110;
		for (int i = 0; i < D; ++i) {
			coeffs[i] = complex<IKReal>(rawcoeffs[i + 1] / rawcoeffs[0]);
		}
		complex<IKReal> roots[D];
		IKReal err[D];
		roots[0] = complex<IKReal>(1, 0);
		roots[1] = complex<IKReal>(0.4, 0.9); // any complex number not a root of unity works
		err[0] = 1.0;
		err[1] = 1.0;
		for (int i = 2; i < D; ++i) {
			roots[i] = roots[i - 1] * roots[1];
			err[i] = 1.0;
		}
		for (int step = 0; step < maxsteps; ++step) {
			bool changed = false;
			for (int i = 0; i < D; ++i) {
				if (err[i] >= tol) {
					changed = true;
					// evaluate
					complex<IKReal> x = roots[i] + coeffs[0];
					for (int j = 1; j < D; ++j) {
						x = roots[i] * x + coeffs[j];
					}
					for (int j = 0; j < D; ++j) {
						if (i != j) {
							if (roots[i] != roots[j]) {
								x /= (roots[i] - roots[j]);
							}
						}
					}
					roots[i] -= x;
					err[i] = abs(x);
				}
			}
			if (!changed) {
				break;
			}
		}

		numroots = 0;
		bool visited[D] = { false };
		for (int i = 0; i < D; ++i) {
			if (!visited[i]) {
				// might be a multiple root, in which case it will have more error than the other roots
				// find any neighboring roots, and take the average
				complex<IKReal> newroot = roots[i];
				int n = 1;
				for (int j = i + 1; j < D; ++j) {
					if (abs(roots[i] - roots[j]) < 8 * tolsqrt) {
						newroot += roots[j];
						n += 1;
						visited[j] = true;
					}
				}
				if (n > 1) {
					newroot /= n;
				}
				// there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
				if (RaveFabs(imag(newroot)) < tolsqrt) {
					rawroots[numroots++] = real(newroot);
				}
			}
		}
	}


	template<typename T>
	inline T SerializationValue(T f)
	{
		return (f > -1e-4f && f < 1e-4f) ? static_cast<T>(0) : f; //boost::math::round(10000*f)*0.0001;
	}

	inline void SerializeRound(std::ostream& o, float f)
	{
		o << SerializationValue(f) << " ";
	}

	inline void SerializeRound(std::ostream& o, double f)
	{
		o << SerializationValue(f) << " ";
	}

	template <class T>
	inline void SerializeRound(std::ostream& o, const RaveVector<T>& v)
	{
		o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " " << SerializationValue(v.w) << " ";
	}

	template <class T>
	inline void SerializeRound3(std::ostream& o, const RaveVector<T>& v)
	{
		o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " ";
	}

	template <class T>
	inline void SerializeRound(std::ostream& o, const RaveTransform<T>& t)
	{
		// because we're serializing a quaternion, have to fix what side of the hypershpere it is on
		Vector v = t.rot;
		for (int i = 0; i < 4; ++i) {
			if (v[i] < g_fEpsilon) {
				v = -v;
				break;
			}
			else if (v[i] > g_fEpsilon) {
				break;
			}
		}
		SerializeRound(o, v);
		SerializeRound(o, t.trans);
	}

	template <class T>
	inline void SerializeRound(std::ostream& o, const RaveTransformMatrix<T>& t)
	{
		o << SerializationValue(t.m[0]) << " " << SerializationValue(t.m[4]) << " " << SerializationValue(t.m[8]) << " "
			<< SerializationValue(t.m[1]) << " " << SerializationValue(t.m[5]) << " " << SerializationValue(t.m[9]) << " "
			<< SerializationValue(t.m[2]) << " " << SerializationValue(t.m[6]) << " " << SerializationValue(t.m[10]) << " ";
		SerializeRound(o, t.trans);
	}

	inline int CountCircularBranches(dReal angle)
	{
		if (angle > PI) {
			return static_cast<int>((angle + PI) / (2 * PI));
		}
		else if (angle < -PI) {
			return static_cast<int>((angle - PI) / (2 * PI));
		}
		return 0;
	}

	/// returns a value=angle+2*PI*N such that value is closest to testvalue
	inline dReal GetClosestValueAlongCircle(dReal angle, dReal testvalue)
	{
		int n = static_cast<int>((testvalue - angle) / PI);
		if (n >= 1) {
			return angle + static_cast<dReal>((n + 1) / 2) * 2 * PI;
		}
		else if (n <= -1) {
			return angle + static_cast<dReal>((n - 1) / 2) * 2 * PI;
		}
		return angle;
	}

	inline dReal TransformDistanceFast(const Transform& t1, const Transform& t2, 
		dReal frotweight = 1, dReal ftransweight = 1)
	{
		dReal e1 = (t1.rot - t2.rot).lengthsqr4();
		dReal e2 = (t1.rot + t2.rot).lengthsqr4();
		dReal e = e1 < e2 ? e1 : e2;
		return RaveSqrt((t1.trans - t2.trans).lengthsqr3() + frotweight * e);
	}

	inline dReal TransformDistance2(const Transform& t1, const Transform& t2, 
		dReal frotweight = 1, dReal ftransweight = 1)
	{
		//dReal facos = RaveAcos(min(dReal(1),RaveFabs(dot4(t1.rot,t2.rot))));
		dReal fcos1 = (t1.rot - t2.rot).lengthsqr4();
		dReal fcos2 = (t1.rot + t2.rot).lengthsqr4();
		dReal fcos = fcos1 < fcos2 ? fcos1 : fcos2;
		return (t1.trans - t2.trans).lengthsqr3() + frotweight * fcos; //*fcos;
	}




}

#endif // OPENRAVE_NUMERICAL_H_