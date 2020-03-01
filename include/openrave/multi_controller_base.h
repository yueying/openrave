// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_CONTROLLER_BASE_H_
#define OPENRAVE_CONTROLLER_BASE_H_

#include <openrave/controller_base.h>

namespace OpenRAVE
{
	/** \brief controller that manage multiple controllers, allows users to easily set multiple controllers for one robot.

	The class also make sure individual controllers do not have colliding DOF.
	- Init() removes all controllers and is <b>[multi-thread safe]</b>
	- IsDone() returns true only if all controllers return true
	- GetTime() return the maximum time
 */
	class OPENRAVE_API MultiControllerBase : public ControllerBase
	{
	public:
		MultiControllerBase(EnvironmentBasePtr penv);
		virtual ~MultiControllerBase();

		/// \brief initializes and adds a controller, must be called after being initialized. <b>[multi-thread safe]</b>
		///
		/// \param controller the controller to init
		/// \param dofindices robot dof indices to control
		/// \throw OpenRAVEException if the controller dofs interfere with current set dofs, will throw an exception
		virtual bool AttachController(ControllerBasePtr controller,
			const std::vector<int>& dofindices, int nControlTransformation) = 0;

		/// \brief removes a controller from being managed. <b>[multi-thread safe]</b>
		virtual void RemoveController(ControllerBasePtr controller) = 0;

		/// \brief gets the controller responsible for dof (in the robot). If dof < 0, returns the transform controller. <b>[multi-thread safe]</b>
		virtual ControllerBasePtr GetController(int dof) const = 0;
	};

	typedef std::shared_ptr<MultiControllerBase> MultiControllerBasePtr;
	typedef std::shared_ptr<MultiControllerBase const> MultiControllerBaseConstPtr;
}


#endif // OPENRAVE_CONTROLLER_BASE_H_