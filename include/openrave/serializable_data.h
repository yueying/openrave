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
#ifndef OPENRAVE_SERIALIZABLE_DATA_H_
#define OPENRAVE_SERIALIZABLE_DATA_H_

#include <openrave/user_data.h>

namespace OpenRAVE
{
	/// \brief user data that can serialize/deserialize itself
	class OPENRAVE_API SerializableData : public UserData
	{
	public:
		virtual ~SerializableData() 
		{
		}

		/// \brief output the data of the object
		virtual void Serialize(std::ostream& O, int options = 0) const = 0;

		/// \brief initialize the object
		virtual void Deserialize(std::istream& I) = 0;
	};
	typedef std::shared_ptr<SerializableData> SerializableDataPtr;
	typedef std::weak_ptr<SerializableData> SerializableDataWeakPtr;
}

#endif // OPENRAVE_SERIALIZABLE_DATA_H_