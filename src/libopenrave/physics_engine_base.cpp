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
#include <openrave/physics_engine_base.h>

namespace OpenRAVE
{
	bool PhysicsEngineBase::GetLinkForceTorque(KinBody::LinkConstPtr plink, Vector& force, Vector& torque)
	{
		force = Vector(0, 0, 0);
		torque = Vector(0, 0, 0);
		//Loop over all joints in the parent body
		FOREACHC(itjoint, plink->GetParent()->GetJoints()) {
			//if this joint's parent or child body is the current body
			bool bParentLink = (*itjoint)->GetHierarchyParentLink() == plink;
			bool bChildLink = (*itjoint)->GetHierarchyChildLink() == plink;
			if (bParentLink || bChildLink) {
				Vector f;
				Vector T;
				GetJointForceTorque(*itjoint, f, T);
				if (bParentLink) {
					Vector r = (*itjoint)->GetAnchor() - plink->GetGlobalCOM();
					force += f;
					torque += T;
					//Re-add moment due to equivalent load at body COM
					torque += r.cross(f);
				}
				else {
					//Equal but opposite sign
					Vector r = (*itjoint)->GetAnchor() - plink->GetGlobalCOM();
					force -= f;
					torque -= T;
					torque -= r.cross(f);
				}
			}
		}
		FOREACHC(itjoint, plink->GetParent()->GetPassiveJoints()) {
			bool bParentLink = (*itjoint)->GetHierarchyParentLink() == plink;
			bool bChildLink = (*itjoint)->GetHierarchyChildLink() == plink;
			if (bParentLink || bChildLink) {
				Vector f;
				Vector T;
				GetJointForceTorque(*itjoint, f, T);

				if (bParentLink) {
					Vector r = (*itjoint)->GetAnchor() - plink->GetGlobalCOM();
					force += f;
					torque += T;
					//Re-add moment due to equivalent load at body COM
					torque += r.cross(f);
				}
				else {
					//Equal but opposite sign
					Vector r = (*itjoint)->GetAnchor() - plink->GetGlobalCOM();
					force -= f;
					torque -= T;
					torque -= r.cross(f);
				}
			}
		}
		return true;
	}

}