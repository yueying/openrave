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
#ifndef OPENRAVE_USER_DATA_H_
#define OPENRAVE_USER_DATA_H_

#include <openrave/config.h>
#include <memory>

namespace OpenRAVE 
{
	/// \brief base class for all user data
	class OPENRAVE_API UserData
	{
	public:
		virtual ~UserData()
		{
		}
	};
	typedef std::shared_ptr<UserData> UserDataPtr;
	typedef std::weak_ptr<UserData> UserDataWeakPtr;
}

#endif // OPENRAVE_USER_DATA_H_