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

#include <openrave/collision_report.h>
#include <openrave/logging.h>

namespace OpenRAVE
{

	void CollisionReport::Reset(int coloptions)
	{
		options = coloptions;
		if (!(nKeepPrevious & 1)) {
			minDistance = 1e20f;
			numWithinTol = 0;
			contacts.resize(0);
			vLinkColliding.resize(0);
			plink1.reset();
			plink2.reset();
		}
	}

	std::string CollisionReport::__str__() const
	{
		std::stringstream s;
		if (vLinkColliding.size() > 0)
		{
			s << "pairs=" << vLinkColliding.size();
			int index = 0;
			for(auto itlinkpair: vLinkColliding) 
			{
				s << ", [" << index << "](";
				if (!!itlinkpair.first)
				{
					KinBodyPtr parent = itlinkpair.first->GetParent(true);
					if (!!parent) 
					{
						s << parent->GetName() << ":" << itlinkpair.first->GetName();
					}
					else 
					{
						RAVELOG_WARN_FORMAT("could not get parent for link name %s when printing collision report",
							itlinkpair.first->GetName());
						s << "[deleted]:" << itlinkpair.first->GetName();
					}
				}
				s << ")x(";
				if (!!itlinkpair.second)
				{
					KinBodyPtr parent = itlinkpair.second->GetParent(true);
					if (!!parent) 
					{
						s << parent->GetName() << ":" << itlinkpair.second->GetName();
					}
					else 
					{
						RAVELOG_WARN_FORMAT("could not get parent for link name %s when printing collision report", 
							itlinkpair.second->GetName());
						s << "[deleted]:" << itlinkpair.second->GetName();
					}
				}
				s << ") ";
				++index;
			}
		}
		else {
			s << "(";
			if (!!plink1) {
				KinBodyPtr parent = plink1->GetParent(true);
				if (!!parent) {
					s << plink1->GetParent()->GetName() << ":" << plink1->GetName();
				}
				else {
					RAVELOG_WARN_FORMAT("could not get parent for link name %s when printing collision report", plink1->GetName());
					s << "[deleted]:" << plink1->GetName();
				}
			}
			s << ")x(";
			if (!!plink2) {
				KinBodyPtr parent = plink2->GetParent(true);
				if (!!parent) {
					s << plink2->GetParent()->GetName() << ":" << plink2->GetName();
				}
				else {
					RAVELOG_WARN_FORMAT("could not get parent for link name %s when printing collision report", plink2->GetName());
					s << "[deleted]:" << plink2->GetName();
				}
			}
			s << ")";
		}
		s << ", contacts=" << contacts.size();
		if (minDistance < 1e10) {
			s << ", mindist=" << minDistance;
		}
		return s.str();
	}

}