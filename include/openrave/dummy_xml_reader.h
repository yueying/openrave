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
#ifndef OPENRAVE_DUMMY_XML_READER_H_
#define OPENRAVE_DUMMY_XML_READER_H_

#include <openrave/openrave.h>

namespace OpenRAVE
{
	/// \brief reads until the tag ends
	class OPENRAVE_API DummyXMLReader : public BaseXMLReader
	{
	public:
		DummyXMLReader(const std::string& fieldname, const std::string& parentname,
			std::shared_ptr<std::ostream> osrecord = std::shared_ptr<std::ostream>());
		virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
		virtual bool endElement(const std::string& name);
		virtual void characters(const std::string& ch);
		const std::string& GetFieldName() const
		{
			return _fieldname;
		}
		virtual std::shared_ptr<std::ostream> GetStream() const
		{
			return _osrecord;
		}
	private:
		std::string _parentname;     /// XML filename
		std::string _fieldname;
		std::shared_ptr<std::ostream> _osrecord;     ///< used to store the xml data
		std::shared_ptr<BaseXMLReader> _pcurreader;
	};
}

#endif // OPENRAVE_DUMMY_XML_READER_H_