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

#include <openrave/sensor_base.h>
#include <streambuf>
#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>

namespace OpenRAVE
{
	bool SensorBase::SensorData::serialize(std::ostream& O) const
	{
		RAVELOG_WARN("SensorData XML serialization not implemented\n");
		return true;
	}

	bool SensorBase::LaserSensorData::serialize(std::ostream& O) const
	{
		RAVELOG_WARN("LaserSensorData XML serialization not implemented\n");
		return true;
	}

	bool SensorBase::CameraSensorData::serialize(std::ostream& O) const
	{
		RAVELOG_WARN("CameraSensorData XML serialization not implemented\n");
		return true;
	}

	void SensorBase::SensorGeometry::Serialize(BaseXMLWriterPtr writer, int options) const
	{
		AttributesList atts;
		if (hardware_id.size() > 0) 
		{
			writer->AddChild("hardware_id", atts)->SetCharData(hardware_id);
		}
	}

	void SensorBase::CameraGeomData::Serialize(BaseXMLWriterPtr writer, int options) const
	{
		SensorGeometry::Serialize(writer, options);
		AttributesList atts;
		std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
		ss << KK.fx << " 0 " << KK.cx << " 0 " << KK.fy << " " << KK.cy;
		writer->AddChild("intrinsic", atts)->SetCharData(ss.str());
		ss.str("");
		ss << KK.focal_length;
		writer->AddChild("focal_length", atts)->SetCharData(ss.str());
		if (KK.distortion_model.size() > 0)
		{
			writer->AddChild("distortion_model", atts)->SetCharData(KK.distortion_model);
			if (KK.distortion_coeffs.size() > 0) 
			{
				ss.str("");
				for(auto it: KK.distortion_coeffs) 
				{
					ss << it << " ";
				}
				writer->AddChild("distortion_coeffs", atts)->SetCharData(ss.str());
			}
		}
		ss.str("");
		ss << width << " " << height; // _numchannels=3
		writer->AddChild("image_dimensions", atts)->SetCharData(ss.str());
		writer->AddChild("measurement_time", atts)->SetCharData(boost::lexical_cast<std::string>(measurement_time));
		writer->AddChild("gain", atts)->SetCharData(boost::lexical_cast<std::string>(gain));
		//writer->AddChild("format",atts)->SetCharData(_channelformat.size() > 0 ? _channelformat : std::string("uint8"));
		if (sensor_reference.size() > 0) 
		{
			atts.emplace_back("url", sensor_reference);
			writer->AddChild("sensor_reference", atts);
			atts.clear();
		}
		if (target_region.size() > 0) 
		{
			atts.emplace_back("url", target_region);
			writer->AddChild("target_region", atts);
			atts.clear();
		}
	}

	void SensorBase::Serialize(BaseXMLWriterPtr writer, int options) const
	{
		RAVELOG_WARN(str(boost::format("sensor %s does not implement Serialize") % GetXMLId()));
	}

}