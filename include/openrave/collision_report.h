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
#ifndef OPENRAVE_COLLISION_REPORT_H_
#define OPENRAVE_COLLISION_REPORT_H_
#include <openrave/config.h>
#include <openrave/numerical.h>
#include <openrave/kinbody.h>
#include <openrave/openrave_macros.h>

namespace OpenRAVE 
{
	/// \brief action to perform whenever a collision is detected between objects
	enum CollisionAction
	{
		CA_DefaultAction = 0, ///< let the physics/collision engine resolve the action
		CA_Ignore = 1, ///< do nothing
	};
	OPENRAVE_CLASS_FORWARD(CollisionReport);
	/// \brief Holds information about a particular collision that occured.
	class OPENRAVE_API CollisionReport
	{
	public:
		class OPENRAVE_API CONTACT
		{
		public:
			CONTACT() : depth(0) 
			{
			}
			CONTACT(const Vector &p, const Vector &n, dReal d) : pos(p), norm(n) 
			{
				depth = d;
			}

			Vector pos;  ///< where the contact occured
			Vector norm; ///< the normals of the faces
			dReal depth; ///< the penetration depth, positive means the surfaces are penetrating, negative means the surfaces are not colliding (used for distance queries)
		};

		CollisionReport() 
		{
			nKeepPrevious = 0;
			Reset();
		}

		/// \brief resets the report structure for the next collision call
		///
		/// depending on nKeepPrevious will keep previous data.
		virtual void Reset(int coloptions = 0);
		virtual std::string __str__() const;

		KinBody::LinkConstPtr plink1, plink2; ///< the colliding links if a collision involves a bodies. Collisions do not always occur with 2 bodies like ray collisions, so these fields can be empty.

		std::vector<std::pair<KinBody::LinkConstPtr, KinBody::LinkConstPtr> > vLinkColliding; ///< all link collision pairs. Set when CO_AllCollisions is enabled.

		std::vector<CONTACT> contacts; ///< the convention is that the normal will be "out" of plink1's surface. Filled if CO_UseContacts option is set.

		int options; ///< the options that the CollisionReport was called with. It is overwritten by the options set on the collision checker writing the report

		dReal minDistance; ///< minimum distance from last query, filled if CO_Distance option is set
		int numWithinTol; ///< number of objects within tolerance of this object, filled if CO_UseTolerance option is set

		uint8_t nKeepPrevious; ///< if 1, will keep all previous data when resetting the collision checker. otherwise will reset

		//KinBody::Link::GeomConstPtr pgeom1, pgeom2; ///< the specified geometries hit for the given links
	};
	
	
}

#endif // OPENRAVE_COLLISION_REPORT_H_