// -*- coding: utf-8 -*-
#ifndef PARABOLIC_PRAMP_H
#define PARABOLIC_PRAMP_H

#include "paraboliccommon.h"

namespace ParabolicRampInternal
{

	/**\brief suppose that the movement runs in a straight line at a uniform speed.
		x0 + t*dx0 + 0.5*a*t*t
	*/
	class ParabolicRamp
	{
	public:
		/**\brief specific position estimates after t ,x0 + t*dx0 + 0.5*a*t*t*/
		Real Evaluate(Real t) const;

		/**\brief derivative at time t,which is the velocity at time t, dx0 + a*t*/
		Real Derivative(Real t) const;

		/**\brief accelerate at time t,a*/
		Real Accel(Real t) const;

		/**\brief calcuate acceleration and time based on given maximum acceleration */
		bool Solve(Real amax);

		/**\brief calculate acceleration based on a given runtime.*/
		bool SolveFixedTime(Real endTime);

		/** \brief returns max speed (absolute value of velocity) along ramp,
		    which is the larger of absolute value of dx0 and dx1 */
		Real GetMaxSpeed() const;

		//input
		Real x0, dx0;//!<start position and speed
		Real x1, dx1;//!<end position and speed

		//calculated
		Real a; //!< calculated acceleration
		Real ttotal; //!<total time to run ramp;
	};

}

#endif
