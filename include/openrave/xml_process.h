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
#ifndef OPENRAVE_XML_PROCESS_H_
#define OPENRAVE_XML_PROCESS_H_

#include <openrave/user_data.h>
#include <string>
#include <list>

namespace OpenRAVE
{
	class BaseXMLReader;
	typedef std::shared_ptr<BaseXMLReader> BaseXMLReaderPtr;
	typedef std::shared_ptr<BaseXMLReader const> BaseXMLReaderConstPtr;
	class BaseXMLWriter;
	typedef std::shared_ptr<BaseXMLWriter> BaseXMLWriterPtr;
	typedef std::shared_ptr<BaseXMLWriter const> BaseXMLWriterConstPtr;

	/// base class for readable interfaces
	class OPENRAVE_API XMLReadable : public UserData
	{
	public:
		XMLReadable(const std::string& xmlid) : xml_id_(xmlid)
		{
		}
		virtual ~XMLReadable()
		{
		}
		virtual const std::string& GetXMLId() const
		{
			return xml_id_;
		}
		/// \brief serializes the interface
		virtual void Serialize(BaseXMLWriterPtr writer, int options = 0) const
		{
		}
	private:
		std::string xml_id_;
	};

	typedef std::shared_ptr<XMLReadable> XMLReadablePtr;
	typedef std::shared_ptr<XMLReadable const> XMLReadableConstPtr;

	/// \brief a list of key-value pairs. It is possible for keys to repeat.
	typedef std::list<std::pair<std::string, std::string> > AttributesList;

	/// \brief base class for all xml readers. XMLReaders are used to process data from xml files.
	///
	/// Custom readers can be registered through \ref RaveRegisterXMLReader.
	class OPENRAVE_API BaseXMLReader : public std::enable_shared_from_this<BaseXMLReader>
	{
	public:
		enum ProcessElement
		{
			PE_Pass = 0,     ///< current tag was not supported, so pass onto another class
			PE_Support = 1,     ///< current tag will be processed by this class
			PE_Ignore = 2,     ///< current tag and all its children should be ignored
		};
		BaseXMLReader()
		{
		}
		virtual ~BaseXMLReader()
		{
		}

		/// a readable interface that stores the information processsed for the current tag
		/// This pointer is used to the InterfaceBase class registered readers
		virtual XMLReadablePtr GetReadable()
		{
			return XMLReadablePtr();
		}

		/// Gets called in the beginning of each "<type>" expression. In this case, name is "type"
		/// \param name of the tag, will be always lower case
		/// \param atts string of attributes where the first std::string is the attribute name and second is the value
		/// \return true if tag is accepted and this class will process it, otherwise false
		virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) = 0;

		/// Gets called at the end of each "</type>" expression. In this case, name is "type"
		/// \param name of the tag, will be always lower case
		/// \return true if XMLReader has finished parsing (one condition is that name==_fieldname) , otherwise false
		virtual bool endElement(const std::string& name) = 0;

		/// gets called for all data in between tags.
		/// \param ch a string to the data
		virtual void characters(const std::string& ch) = 0;

		/// XML filename/resource used for this class (can be empty)
		std::string file_name_;
	};

	typedef boost::function<BaseXMLReaderPtr(InterfaceBasePtr, const AttributesList&)> CreateXMLReaderFn;

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

	/// \brief base class for writing to XML files.
	///
	/// OpenRAVE Interfaces accept a BaseXMLWriter instance and call its write methods to write the data.
	class OPENRAVE_API BaseXMLWriter : public std::enable_shared_from_this<BaseXMLWriter>
	{
	public:
		virtual ~BaseXMLWriter()
		{
		}
		/// \brief return the format for the data writing, should be all lower capitals.
		///
		/// Samples formats are 'openrave', 'collada'
		virtual const std::string& GetFormat() const = 0;

		/// \brief saves character data to the child. Special characters like '<' are automatically converted to fit inside XML.
		///
		/// \throw OpenRAVEException throws if this element cannot have character data or the character data was not written
		virtual void SetCharData(const std::string& data) = 0;

		/// \brief returns a writer for child elements
		virtual BaseXMLWriterPtr AddChild(const std::string& xmltag, const AttributesList& atts = AttributesList()) = 0;
	};
}

#endif // OPENRAVE_XML_PROCESS_H_