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
#include <openrave/dummy_xml_reader.h>
#include <openrave/xml_process.h>

namespace OpenRAVE
{
	// Dummy Reader
	DummyXMLReader::DummyXMLReader(const std::string& fieldname,
		const std::string& pparentname, std::shared_ptr<std::ostream> osrecord)
		: _fieldname(fieldname), _osrecord(osrecord)
	{
		_parentname = pparentname;
		_parentname += ":";
		_parentname += _fieldname;
	}

	BaseXMLReader::ProcessElement DummyXMLReader::startElement(const std::string& name, const AttributesList &atts)
	{
		if (!!_pcurreader)
		{
			if (_pcurreader->startElement(name, atts) == PE_Support)
			{
				return PE_Support;
			}
			return PE_Ignore;
		}

		if (!!_osrecord)
		{
			*_osrecord << "<" << name << " ";
			for (auto itatt : atts)
			{
				*_osrecord << itatt.first << "=\"" << itatt.second << "\" ";
			}
			*_osrecord << ">" << std::endl;
		}

		// create a new parser
		_pcurreader.reset(new DummyXMLReader(name, _parentname, _osrecord));
		return PE_Support;
	}

	bool DummyXMLReader::endElement(const std::string& name)
	{
		if (!!_pcurreader)
		{
			if (_pcurreader->endElement(name))
			{
				_pcurreader.reset();
				if (!!_osrecord)
				{
					*_osrecord << "</" << name << ">" << std::endl;
				}
			}
			return false;
		}

		if (name == _fieldname)
		{
			return true;
		}
		RAVELOG_ERROR(str(boost::format("invalid xml tag %s\n") % name));
		return false;
	}

	void DummyXMLReader::characters(const std::string& ch)
	{
		if (!_pcurreader)
		{
			if (!!_osrecord)
			{
				*_osrecord << ch;
			}
		}
		else
		{
			_pcurreader->characters(ch);
		}
	}
}