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

#include <openrave/numerical.h>

namespace OpenRAVE
{
#if defined(USE_CRLIBM)

#ifdef HAS_FENV_H
#include <fenv.h>
#endif

#ifdef LIBM_ACCURACY_RESULTS_H
#include LIBM_ACCURACY_RESULTS_H
#endif

	// use round-nearest versions since it is the default
#ifdef LIBM_EXP_ACCURATE
	dReal RaveExp(dReal f) {
		return exp(f);
	}
#else
	dReal RaveExp(dReal f) {
		return exp_rn(f);
	}
#endif
#ifdef LIBM_LOG_ACCURATE
	dReal RaveLog(dReal f) {
		return log(f);
	}
#else
	dReal RaveLog(dReal f) {
		return log_rn(f);
	}
#endif
#ifdef LIBM_COS_ACCURATE
	dReal RaveCos(dReal f) {
		return cos(f);
	}
#else
	dReal RaveCos(dReal f) {
		return cos_rn(f);
	}
#endif
#ifdef LIBM_SIN_ACCURATE
	dReal RaveSin(dReal f) {
		return sin(f);
	}
#else
	dReal RaveSin(dReal f) {
		return sin_rn(f);
	}
#endif
#ifdef LIBM_TAN_ACCURATE
	dReal RaveTan(dReal f) {
		return tan(f);
	}
#else
	dReal RaveTan(dReal f) {
		return tan_rn(f);
	}
#endif
#ifdef LIBM_LOG2_ACCURATE
	dReal RaveLog2(dReal f) {
		return log2(f);
	}
#else
	dReal RaveLog2(dReal f) {
		return log2_rn(f);
	}
#endif
#ifdef LIBM_LOG10_ACCURATE
	dReal RaveLog10(dReal f) {
		return log10(f);
	}
#else
	dReal RaveLog10(dReal f) {
		return log10_rn(f);
	}
#endif
#ifdef LIBM_ACOS_ACCURATE
	dReal RaveAcos(dReal f) {
		return acos(f);
	}
#else
	dReal RaveAcos(dReal f) {
		return acos_rn(f);
	}
#endif
#ifdef LIBM_ASIN_ACCURATE
	dReal RaveAsin(dReal f) {
		return asin(f);
	}
#else
	dReal RaveAsin(dReal f) {
		return asin_rn(f);
	}
#endif
#ifdef LIBM_ATAN2_ACCURATE
	dReal RaveAtan2(dReal y, dReal x) {
		return atan2(y, x);
	}
#else
	dReal RaveAtan2(dReal y, dReal x) // unfortunately no atan2 in crlibm...
	{
		dReal absx, absy, val;
		if ((x == 0) && (y == 0)) {
			return 0;
		}
		absy = y < 0 ? -y : y;
		absx = x < 0 ? -x : x;
		if (absy - absx == absy) {
			// x negligible compared to y
			return y < 0 ? -M_PI_2 : M_PI_2;
		}
		if (absx - absy == absx) {
			// y negligible compared to x
			val = 0.0;
		}
		else val = atan_rn(y / x);
		if (x > 0) {
			// first or fourth quadrant; already correct
			return val;
		}
		if (y < 0) {
			// third quadrant
			return val - M_PI;
		}
		return val + M_PI;
	}
#endif
#ifdef LIBM_POW_ACCURATE
	dReal RavePow(dReal x, dReal y) {
		return pow(x, y);
	}
#else
	dReal RavePow(dReal x, dReal y) {
		return pow_rn(x, y);
	}
#endif
#ifdef LIBM_SQRT_ACCURATE
	dReal RaveSqrt(dReal f) {
		return sqrt(f);
	}
#else
	dReal RaveSqrt(dReal f) {
		return sqrt(f);
	}
	//dReal RaveSqrt(dReal f) { return pow_rn(f,0.5); } // NOTE: this is really slow, is it really worth the precision?
#endif
	dReal RaveFabs(dReal f) {
		return fabs(f);
	}
	dReal RaveCeil(dReal f) {
		return ceil(f);
	}

#else // use all standard libm

#if OPENRAVE_PRECISION == 0 // floating-point
	dReal RaveExp(dReal f) {
		return expf(f);
	}
	dReal RaveLog(dReal f) {
		return logf(f);
	}
	dReal RaveCos(dReal f) {
		return cosf(f);
	}
	dReal RaveSin(dReal f) {
		return sinf(f);
	}
	dReal RaveTan(dReal f) {
		return tanf(f);
	}
#ifdef HAS_LOG2
	dReal RaveLog2(dReal f) {
		return log2f(f);
	}
#else
	dReal RaveLog2(dReal f) {
		return logf(f) / logf(2.0f);
	}
#endif
	dReal RaveLog10(dReal f) {
		return log10f(f);
	}
	dReal RaveAcos(dReal f) {
		return acosf(f);
	}
	dReal RaveAsin(dReal f) {
		return asinf(f);
	}
	dReal RaveAtan2(dReal fy, dReal fx) {
		return atan2f(fy, fx);
	}
	dReal RavePow(dReal fx, dReal fy) {
		return powf(fx, fy);
	}
	dReal RaveSqrt(dReal f) {
		return sqrtf(f);
	}
	dReal RaveFabs(dReal f) {
		return fabsf(f);
	}
	dReal RaveCeil(dReal f) {
		return ceilf(f);
	}
#else
	dReal RaveExp(dReal f) {
		return exp(f);
	}
	dReal RaveLog(dReal f) {
		return log(f);
	}
	dReal RaveCos(dReal f) {
		return cos(f);
	}
	dReal RaveSin(dReal f) {
		return sin(f);
	}
	dReal RaveTan(dReal f) {
		return tan(f);
	}
#ifdef HAS_LOG2
	dReal RaveLog2(dReal f) {
		return log2(f);
	}
#else
	dReal RaveLog2(dReal f) {
		return log(f) / log(2.0f);
	}
#endif
	dReal RaveLog10(dReal f) {
		return log10(f);
	}
	dReal RaveAcos(dReal f) {
		return acos(f);
	}
	dReal RaveAsin(dReal f) {
		return asin(f);
	}
	dReal RaveAtan2(dReal fy, dReal fx) {
		return atan2(fy, fx);
	}
	dReal RavePow(dReal fx, dReal fy) {
		return pow(fx, fy);
	}
	dReal RaveSqrt(dReal f) {
		return sqrt(f);
	}
	dReal RaveFabs(dReal f) {
		return fabs(f);
	}
	dReal RaveCeil(dReal f) {
		return ceil(f);
	}
#endif

#endif

}