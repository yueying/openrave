// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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

#include <openrave/config.h>
#include <openrave/geometry.h>
#include <openrave/openrave_exception.h>
#include <memory>

namespace OpenRAVE
{
	/** \brief Handle holding the plot from the viewers. The plot will continue to be drawn as long as a reference to this handle is held.

	Designed to be multi-thread safe and destruction and modification of the viewer plot can be done at any time. The viewers
	internally handle synchronization and threading issues.
 */
	class OPENRAVE_API GraphHandle
	{
	public:
		virtual ~GraphHandle()
		{
		}

		/// \brief Changes the underlying transformation of the plot. <b>[multi-thread safe]</b>
		///
		/// \param t new transformation of the plot
		virtual void SetTransform(const RaveTransform<float>& t) OPENRAVE_DUMMY_IMPLEMENTATION;
		/// \brief Shows or hides the plot without destroying its resources. <b>[multi-thread safe]</b>
		virtual void SetShow(bool bshow) OPENRAVE_DUMMY_IMPLEMENTATION;
	};

	typedef std::shared_ptr<GraphHandle> GraphHandlePtr;
	typedef std::shared_ptr<GraphHandle const> GraphHandleConstPtr;
	typedef std::weak_ptr<GraphHandle const> GraphHandleWeakPtr;
}