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
#ifndef OPENRAVE_JSON_PROCESS_H_
#define OPENRAVE_JSON_PROCESS_H_

#include <openrave/user_data.h>
#include <rapidjson/document.h>

namespace OpenRAVE
{
	/// base class for json readable interfaces
	class OPENRAVE_API JSONReadable : virtual public Readable
	{
	public:
		JSONReadable() { }
		virtual ~JSONReadable() {}
		virtual void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale = 1.0, int options = 0) const = 0;
		virtual void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale = 1.0) = 0;
	};
	typedef std::shared_ptr<JSONReadable> JSONReadablePtr;
	typedef std::shared_ptr<JSONReadable const> JSONReadableConstPtr;

	/// \brief base class for all json readers. JSONReaders are used to process data from json files.
	///
	/// Custom readers can be registered through \ref RaveRegisterJSONReader.
	class OPENRAVE_API BaseJSONReader : public std::enable_shared_from_this<BaseJSONReader>
	{
	public:

		BaseJSONReader() {}
		virtual ~BaseJSONReader() {}

		/// a readable interface that stores the information processsed for the current tag
		/// This pointer is used to the InterfaceBase class registered readers
		virtual JSONReadablePtr GetReadable()
		{
			return JSONReadablePtr();
		}

		/// by default, json reader will simply call readable's deserialize function
		virtual void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale = 1.0)
		{
			JSONReadablePtr pReadable = GetReadable();
			if (!!pReadable)
			{
				pReadable->DeserializeJSON(value, fUnitScale);
			}
		}
	};
	typedef std::shared_ptr<BaseJSONReader> BaseJSONReaderPtr;
	typedef std::shared_ptr<BaseJSONReader const> BaseJSONReaderConstPtr;
	
}

#endif // OPENRAVE_JSON_PROCESS_H_