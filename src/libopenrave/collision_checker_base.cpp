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

#include <openrave/collision_checker_base.h>

namespace OpenRAVE
{

	CollisionOptionsStateSaver::CollisionOptionsStateSaver(CollisionCheckerBasePtr p, int newoptions, bool required)
	{
		_oldoptions = p->GetCollisionOptions();
		_p = p;
		if (!_p->SetCollisionOptions(newoptions)) 
		{
			if (required) 
			{
				throw OpenRAVEException(str(boost::format(("Failed to set collision options %d in checker %s\n"))
					% newoptions%_p->GetXMLId()));
			}
		}
	}

	CollisionOptionsStateSaver::~CollisionOptionsStateSaver()
	{
		_p->SetCollisionOptions(_oldoptions);
	}
}