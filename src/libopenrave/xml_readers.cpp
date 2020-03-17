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
#include "libopenrave.h"
#include <openrave/xml_readers.h>

#include <boost/lexical_cast.hpp>

namespace OpenRAVE 
{
namespace xmlreaders 
{

StringXMLReadable::StringXMLReadable(const std::string& xmlid, const std::string& data) 
	: XMLReadable(xmlid), _data(data)
{
}

void StringXMLReadable::Serialize(BaseXMLWriterPtr writer, int options) const
{
    if( writer->GetFormat() == "collada" )
	{
        AttributesList atts;
        atts.emplace_back("type", "stringxmlreadable");
        atts.emplace_back("name", GetXMLId());
        BaseXMLWriterPtr child = writer->AddChild("extra",atts);
        atts.clear();
        atts.emplace_back("profile", "OpenRAVE");
        writer = child->AddChild("technique",atts)->AddChild("data");
    }

    writer->SetCharData(_data);
}

const std::string& StringXMLReadable::GetData() const
{
    return _data;
}

HierarchicalXMLReadable::HierarchicalXMLReadable(const std::string& xmlid, const AttributesList& atts) : XMLReadable(xmlid), _atts(atts)
{
}

void HierarchicalXMLReadable::Serialize(BaseXMLWriterPtr writer, int options) const
{
    writer->SetCharData(_data);
    for(std::list<HierarchicalXMLReadablePtr>::const_iterator it = _listchildren.begin(); it != _listchildren.end(); ++it) {
        BaseXMLWriterPtr childwriter = writer->AddChild((*it)->GetXMLId(), (*it)->_atts);
        (*it)->Serialize(childwriter,options);
    }
}

TrajectoryReader::TrajectoryReader(EnvironmentBasePtr penv, TrajectoryBasePtr ptraj, const AttributesList& atts) : _ptraj(ptraj)
{
    _bInReadable = false;
    _datacount = 0;
    FOREACHC(itatt, atts) {
        if( itatt->first == "type" ) {
            if( !!_ptraj ) {
                OPENRAVE_ASSERT_OP(_ptraj->GetXMLId(),==,itatt->second );
            }
            else {
                _ptraj = RaveCreateTrajectory(penv, itatt->second);
            }
        }
    }
    if( !_ptraj ) {
        _ptraj = RaveCreateTrajectory(penv, "");
    }
}

BaseXMLReader::ProcessElement TrajectoryReader::startElement(const std::string& name, const AttributesList& atts)
{
    _ss.str("");
    if( !!_pcurreader ) {
        if( _pcurreader->startElement(name, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }
    if( _bInReadable ) {
        _pcurreader.reset(new HierarchicalXMLReader(name,atts));
        return PE_Support;
    }
    if( name == "trajectory" ) {
        _pcurreader.reset(new TrajectoryReader(_ptraj->GetEnv(), _ptraj, atts));
        return PE_Support;
    }
    else if( name == "configuration" ) {
        _pcurreader.reset(new ConfigurationSpecification::Reader(_spec));
        return PE_Support;
    }
    else if( name == "readable" ) {
        _bInReadable = true;
        return PE_Support;
    }
    else if( name == "data" ) {
        _vdata.resize(0);
        _datacount = 0;
        FOREACHC(itatt,atts) {
            if( itatt->first == "count" ) {
                _datacount = boost::lexical_cast<int>(itatt->second);
            }
        }
        return PE_Support;
    }
    else if( name == "description" ) {
        return PE_Support;
    }
    return PE_Pass;
}

bool TrajectoryReader::endElement(const std::string& name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(name) ) {
            if( _bInReadable ) {
                HierarchicalXMLReaderPtr reader = std::dynamic_pointer_cast<HierarchicalXMLReader>(_pcurreader);
                _ptraj->SetReadableInterface(reader->GetReadable()->GetXMLId(), reader->GetReadable());
                _pcurreader.reset();
            }
            else {
                if( !!std::dynamic_pointer_cast<ConfigurationSpecification::Reader>(_pcurreader) ) {
                    BOOST_ASSERT(_spec.IsValid());
                    _ptraj->Init(_spec);
                }
                bool bret = !!std::dynamic_pointer_cast<TrajectoryReader>(_pcurreader);
                _pcurreader.reset();
                if( bret ) {
                    return true;
                }
            }
        }
    }
    else if( name == "data" ) {
        _vdata.resize(_spec.GetDOF()*_datacount);
        for(size_t i = 0; i < _vdata.size(); ++i) {
            _ss >> _vdata[i];
        }
        if( !_ss ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed reading %d numbers from trajectory <data> element"), _vdata.size(), ORE_Assert);
        }
        else {
            _ptraj->Insert(_ptraj->GetNumWaypoints(),_vdata);
        }
    }
    else if( name == "description" ) {
        _ptraj->SetDescription(_ss.str());
    }
    else if( name == "trajectory" ) {
        return true;
    }
    else if( name == "readable" ) {
        _bInReadable = false;
    }
    return false;
}

void TrajectoryReader::characters(const std::string& ch)
{
    if( !!_pcurreader ) {
        _pcurreader->characters(ch);
    }
    else {
        _ss.clear();
        _ss << ch;
    }
}

GeometryInfoReader::GeometryInfoReader(KinBody::GeometryInfoPtr pgeom, const AttributesList& atts) : geometry_info_(pgeom)
{
    is_overwrite_diffuse_ = is_overwrite_ambient_ = is_overwrite_transparency_ = false;
    _sGroupName = "self";
    string type, name;
    bool bVisible = true, bModifiable = true;
    FOREACHC(itatt,atts) {
        if( itatt->first == "type") {
            type = itatt->second;
        }
        else if( itatt->first == "render" ) {
            // set draw to false only if atts[i]==false
            bVisible = _stricmp(itatt->second.c_str(), "false")!=0 && itatt->second!="0";
        }
        else if( itatt->first == "modifiable" ) {
            bModifiable = !(_stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0");
        }
        else if( itatt->first == "group" && !itatt->second.empty() ) {
            _sGroupName = itatt->second;
        }
        else if( itatt->first == "name" && !itatt->second.empty() ) {
            name = itatt->second;
        }
    }

    if( type.size() == 0 ) {
        RAVELOG_INFOA("no geometry type, defaulting to box\n");
        type = "box";
    }

    geometry_info_.reset(new KinBody::GeometryInfo());
    geometry_info_->name_ = name;
    if( _stricmp(type.c_str(), "none") == 0 ) {
        geometry_info_->type_ = GT_None;
    }
    else if( _stricmp(type.c_str(), "box") == 0 ) {
        geometry_info_->type_ = GT_Box;
    }
    else if( _stricmp(type.c_str(), "sphere") == 0 ) {
        geometry_info_->type_ = GT_Sphere;
    }
    else if( _stricmp(type.c_str(), "cylinder") == 0 ) {
        geometry_info_->type_ = GT_Cylinder;
    }
    else if( _stricmp(type.c_str(), "trimesh") == 0 ) {
        geometry_info_->type_ = GT_TriMesh;
    }
    else if( _stricmp(type.c_str(), "container") == 0 ) {
        geometry_info_->type_ = GT_Container;
    }
    else if( _stricmp(type.c_str(), "cage") == 0 ) {
        geometry_info_->type_ = GT_Cage;
    }
    else {
        RAVELOG_WARN(str(boost::format("type %s not supported\n")%type));
    }
    geometry_info_->is_visible_ = bVisible;
    geometry_info_->is_modifiable_ = bModifiable;
}

BaseXMLReader::ProcessElement GeometryInfoReader::startElement(const std::string& xml_name,
	const AttributesList& atts)
{
    string_stream_.str("");
    if( !!cur_reader_ ) 
	{
        if( cur_reader_->startElement(xml_name, atts) == PE_Support )
		{
            return PE_Support;
        }
        return PE_Ignore;
    }

    if( xml_name == "geometry" || xml_name == "geom" ) 
	{
        cur_reader_.reset(new GeometryInfoReader(geometry_info_, atts));
        return PE_Support;
    }
    if( xml_name == "render" )
	{
        // check the attributes first
        for(auto itatt:atts) 
		{
            if( itatt.first == "file" ) 
			{
                geometry_info_->render_file_name_ = itatt.second;
            }
            else if( itatt.first == "scale" ) 
			{
                geometry_info_->render_scale_vec_ = Vector(1,1,1);
                std::stringstream sslocal(itatt.second);
                sslocal >> geometry_info_->render_scale_vec_.x; 
				geometry_info_->render_scale_vec_.y = geometry_info_->render_scale_vec_.z = geometry_info_->render_scale_vec_.x;
                sslocal >> geometry_info_->render_scale_vec_.y >> geometry_info_->render_scale_vec_.z;
            }
        }
    }
    else if( xml_name == "collision" )
	{
        // check the attributes first
        for(auto itatt:atts) 
		{
            if( itatt.first == "file" ) 
			{
                geometry_info_->collision_file_name_ = itatt.second;
            }
            else if( itatt.first == "scale" )
			{
                geometry_info_->collision_scale_vec_ = Vector(1,1,1);
                std::stringstream sslocal(itatt.second);
                sslocal >> geometry_info_->collision_scale_vec_.x; 
				geometry_info_->collision_scale_vec_.y = geometry_info_->collision_scale_vec_.z = geometry_info_->collision_scale_vec_.x;
                sslocal >> geometry_info_->collision_scale_vec_.y >> geometry_info_->collision_scale_vec_.z;
            }
        }
    }

    static std::array<std::string,14> tags = { { "translation", "rotationmat", "rotationaxis",
		"quat", "diffusecolor", "ambientcolor", 
		"transparency", "render", "extents",
		"halfextents", "fullextents", "radius", "height", "name"}};
    if(std::find(tags.begin(),tags.end(),xml_name) != tags.end() ) 
	{
        return PE_Support;
    }
    switch(geometry_info_->type_) 
	{
    case GT_TriMesh:
        if(xml_name=="collision"|| xml_name=="data" || xml_name=="vertices" )
		{
            return PE_Support;
        }
        break;
    default:
        break;
    }
    return PE_Pass;
}

bool GeometryInfoReader::endElement(const std::string& xmlname)
{
    if( !!cur_reader_ ) 
	{
        if( cur_reader_->endElement(xmlname) ) 
		{
            bool bret = !!std::dynamic_pointer_cast<GeometryInfoReader>(cur_reader_);
            cur_reader_.reset();
            if( bret ) 
			{
                return true;
            }
        }
        return false;
    }

    if( xmlname == "geometry" || xmlname == "geom" )
	{
        return true;
    }
    else if( xmlname == "translation" )
	{
        Vector v;
        string_stream_ >>v.x >> v.y >> v.z;
        geometry_info_->transform_.trans += v;
    }
    else if( xmlname == "rotationmat" )
	{
        TransformMatrix tnew;
        string_stream_ >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> 
			tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> 
			tnew.m[8] >> tnew.m[9] >> tnew.m[10];
        geometry_info_->transform_.rot = (Transform(tnew)*geometry_info_->transform_).rot;
    }
    else if( xmlname == "rotationaxis" ) 
	{
        Vector vaxis; dReal fangle=0;
        string_stream_ >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
        Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
        geometry_info_->transform_.rot = (tnew*geometry_info_->transform_).rot;
    }
    else if( xmlname == "quat" )
	{
        Transform tnew;
        string_stream_ >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
        tnew.rot.normalize4();
        geometry_info_->transform_.rot = (tnew*geometry_info_->transform_).rot;
    }
    else if( xmlname == "render" ) 
	{
        if( geometry_info_->render_file_name_.size() == 0 ) 
		{
            geometry_info_->render_scale_vec_ = Vector(1,1,1);
            string_stream_ >> geometry_info_->render_file_name_;
            string_stream_ >> geometry_info_->render_scale_vec_.x;
			geometry_info_->render_scale_vec_.y = geometry_info_->render_scale_vec_.z = geometry_info_->render_scale_vec_.x;
            string_stream_ >> geometry_info_->render_scale_vec_.y >> geometry_info_->render_scale_vec_.z;
        }
    }
    else if( xmlname == "diffusecolor" )
	{
        is_overwrite_diffuse_ = true;
        string_stream_ >> geometry_info_->diffuse_color_vec_.x >>
			geometry_info_->diffuse_color_vec_.y >> geometry_info_->diffuse_color_vec_.z;
    }
    else if( xmlname == "ambientcolor" )
	{
        is_overwrite_ambient_ = true;
        string_stream_ >> geometry_info_->ambient_color_vec_.x >> 
			geometry_info_->ambient_color_vec_.y >> geometry_info_->ambient_color_vec_.z;
    }
    else if( xmlname == "transparency" ) 
	{
        is_overwrite_transparency_ = true;
        string_stream_ >> geometry_info_->transparency_;
    }
    else if( xmlname == "name" ) 
	{
        string_stream_ >> geometry_info_->name_;
    }
    else
	{
        // could be type specific features
        switch(geometry_info_->type_)
		{
        case GT_None:
            // Do nothing
            break;

        case GT_Sphere:
            if( xmlname == "radius" ) 
			{
                string_stream_ >> geometry_info_->gemo_outer_extents_data_.x;
            }
            break;
        case GT_Box:
            if( xmlname == "extents" || xmlname == "halfextents" ) 
			{
                string_stream_ >> geometry_info_->gemo_outer_extents_data_.x 
					>> geometry_info_->gemo_outer_extents_data_.y 
					>> geometry_info_->gemo_outer_extents_data_.z;
            }
            else if( xmlname == "fullextents" ) 
			{
                string_stream_ >> geometry_info_->gemo_outer_extents_data_.x 
					>> geometry_info_->gemo_outer_extents_data_.y 
					>> geometry_info_->gemo_outer_extents_data_.z;
                geometry_info_->gemo_outer_extents_data_ *= 0.5;
            }

            break;
        case GT_Container:
            if( xmlname == "outer_extents" )
			{
                string_stream_ >> geometry_info_->gemo_outer_extents_data_.x 
					>> geometry_info_->gemo_outer_extents_data_.y 
					>> geometry_info_->gemo_outer_extents_data_.z;
            }
            if( xmlname == "inner_extents" )
			{
                string_stream_ >> geometry_info_->geom_inner_extents_data_.x 
					>> geometry_info_->geom_inner_extents_data_.y 
					>> geometry_info_->geom_inner_extents_data_.z;
            }
            if( xmlname == "bottom_cross" )
			{
                string_stream_ >> geometry_info_->geom_bottom_cross_data_.x 
					>> geometry_info_->geom_bottom_cross_data_.y 
					>> geometry_info_->geom_bottom_cross_data_.z;
            }
            if( xmlname == "bottom" )
			{
                string_stream_ >> geometry_info_->geom_bottom_data_.x 
					>> geometry_info_->geom_bottom_data_.y 
					>> geometry_info_->geom_bottom_data_.z;
            }

            break;
        case GT_Cage:
            if( xmlname == "sidewall" ) 
			{
                geometry_info_->side_walls_vector_.push_back({});
            }
            if( xmlname == "transf" )
			{
                string_stream_ >> geometry_info_->side_walls_vector_.back().transf;
            }
            if( xmlname == "vExtents" ) 
			{
                string_stream_ >> geometry_info_->side_walls_vector_.back().vExtents;
            }
            if( xmlname == "type" )
			{
                int32_t type;
                string_stream_ >> type;
                geometry_info_->side_walls_vector_.back().type = static_cast<KinBody::GeometryInfo::SideWallType>(type);
            }

            break;
        case GT_Cylinder:
            if( xmlname == "radius") 
			{
                string_stream_ >> geometry_info_->gemo_outer_extents_data_.x;
            }
            else if( xmlname == "height" ) 
			{
                string_stream_ >> geometry_info_->gemo_outer_extents_data_.y;
            }
            break;
        case GT_TriMesh:
            if(( xmlname == "data") ||( xmlname == "collision") ) 
			{
                if( geometry_info_->collision_file_name_.size() == 0 ) 
				{
                    // check the attributes first
                    geometry_info_->collision_scale_vec_ = Vector(1,1,1);
                    string_stream_ >> geometry_info_->collision_file_name_;
                    string_stream_ >> geometry_info_->collision_scale_vec_.x; 
					geometry_info_->collision_scale_vec_.y = geometry_info_->collision_scale_vec_.z = geometry_info_->collision_scale_vec_.x;
                    string_stream_ >> geometry_info_->collision_scale_vec_.y 
						>> geometry_info_->collision_scale_vec_.z;
                }
            }
            else if( xmlname == "vertices" ) 
			{
                std::vector<dReal> values((std::istream_iterator<dReal>(string_stream_)),
					std::istream_iterator<dReal>());
                if( (values.size()%9) ) 
				{
                    RAVELOG_WARN(str(boost::format("number of points specified in the vertices field \
                     needs to be a multiple of 3 (it is %d), ignoring...\n")%values.size()));
                }
                else 
				{
                    geometry_info_->mesh_collision_.vertices.resize(values.size()/3);
                    geometry_info_->mesh_collision_.indices.resize(values.size()/3);
					std::vector<dReal>::iterator itvalue = values.begin();
                    size_t i = 0;
                    for(auto& itv:geometry_info_->mesh_collision_.vertices)
					{
                        itv.x = *itvalue++;
                        itv.y = *itvalue++;
                        itv.z = *itvalue++;
                        geometry_info_->mesh_collision_.indices[i] = i;
                        ++i;
                    }
                }
            }
            break;
        default:
            cur_reader_.reset(new DummyXMLReader(xmlname,"geom"));
        }
    }


    return false;
}

void GeometryInfoReader::characters(const std::string& ch)
{
    if( !!cur_reader_ ) 
	{
        cur_reader_->characters(ch);
    }
    else 
	{
        string_stream_.clear();
        string_stream_ << ch;
    }
}


ElectricMotorActuatorInfoReader::ElectricMotorActuatorInfoReader(ElectricMotorActuatorInfoPtr pinfo,
	const AttributesList& atts) : _pinfo(pinfo)
{
	std::string type;
    FOREACHC(itatt,atts) {
        if( itatt->first == "type") {
            type = itatt->second;
        }
    }

    if( type.size() == 0 ) {
        RAVELOG_INFOA("no actuator type, defaulting to electric_motor\n");
        type = "electric_motor";
    }

    if( type != "electric_motor" ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("does not support actuator '%s' type"), type, ORE_InvalidArguments);
    }

    _pinfo.reset(new ElectricMotorActuatorInfo());
}

BaseXMLReader::ProcessElement ElectricMotorActuatorInfoReader::startElement(const std::string& xmlname, const AttributesList& atts)
{
    _ss.str("");
    if( !!_pcurreader ) {
        if( _pcurreader->startElement(xmlname, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }

    if( xmlname == "actuator" ) {
        _pcurreader.reset(new ElectricMotorActuatorInfoReader(_pinfo, atts));
        return PE_Support;
    }

    static std::array<string, 18> tags = { { "gear_ratio", "assigned_power_rating", "max_speed", "no_load_speed", "stall_torque", "nominal_speed_torque_point", "max_speed_torque_point", "nominal_torque", "rotor_inertia", "torque_constant", "nominal_voltage", "speed_constant", "starting_current", "terminal_resistance", "coloumb_friction", "viscous_friction", "model_type", "max_instantaneous_torque", } };
    if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
        return PE_Support;
    }
    return PE_Pass;
}

bool ElectricMotorActuatorInfoReader::endElement(const std::string& xmlname)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(xmlname) ) {
            bool bret = !!std::dynamic_pointer_cast<ElectricMotorActuatorInfoReader>(_pcurreader);
            _pcurreader.reset();
            if( bret ) {
                return true;
            }
        }
        return false;
    }

    if( xmlname == "actuator" ) {
        return true;
    }
    else if( xmlname == "model_type" ) {
        _ss >> _pinfo->model_type;
    }
    else if( xmlname == "assigned_power_rating" ) {
        _ss >> _pinfo->assigned_power_rating;
    }
    else if( xmlname == "max_speed" ) {
        _ss >> _pinfo->max_speed;
    }
    else if( xmlname == "no_load_speed" ) {
        _ss >> _pinfo->no_load_speed;
    }
    else if( xmlname == "stall_torque" ) {
        _ss >> _pinfo->stall_torque;
    }
    else if( xmlname == "max_instantaneous_torque" ) {
        _ss >> _pinfo->max_instantaneous_torque;
    }
    else if( xmlname == "nominal_speed_torque_point" ) {
        dReal speed=0, torque=0;
        _ss >> speed >> torque;
        // should be from increasing speed.
        size_t insertindex = 0;
        while(insertindex < _pinfo->nominal_speed_torque_points.size()) {
            if( speed < _pinfo->nominal_speed_torque_points.at(insertindex).first ) {
                break;
            }
            ++insertindex;
        }
        _pinfo->nominal_speed_torque_points.insert(_pinfo->nominal_speed_torque_points.begin()+insertindex, make_pair(speed,torque));
    }
    else if( xmlname == "max_speed_torque_point" ) {
        dReal speed=0, torque=0;
        _ss >> speed >> torque;
        // should be from increasing speed.
        size_t insertindex = 0;
        while(insertindex < _pinfo->max_speed_torque_points.size()) {
            if( speed < _pinfo->max_speed_torque_points.at(insertindex).first ) {
                break;
            }
            ++insertindex;
        }
        _pinfo->max_speed_torque_points.insert(_pinfo->max_speed_torque_points.begin()+insertindex, make_pair(speed,torque));
    }
    else if( xmlname == "nominal_torque" ) {
        _ss >> _pinfo->nominal_torque;
    }
    else if( xmlname == "rotor_inertia" ) {
        _ss >> _pinfo->rotor_inertia;
    }
    else if( xmlname == "torque_constant" ) {
        _ss >> _pinfo->torque_constant;
    }
    else if( xmlname == "nominal_voltage" ) {
        _ss >> _pinfo->nominal_voltage;
    }
    else if( xmlname == "speed_constant" ) {
        _ss >> _pinfo->speed_constant;
    }
    else if( xmlname == "starting_current" ) {
        _ss >> _pinfo->starting_current;
    }
    else if( xmlname == "terminal_resistance" ) {
        _ss >> _pinfo->terminal_resistance;
    }
    else if( xmlname == "gear_ratio" ) {
        _ss >> _pinfo->gear_ratio;
    }
    else if( xmlname == "coloumb_friction" ) {
        _ss >> _pinfo->coloumb_friction;
    }
    else if( xmlname == "viscous_friction" ) {
        _ss >> _pinfo->viscous_friction;
    }
    else {
        RAVELOG_WARN_FORMAT("could not process tag %s", xmlname);
    }

    return false;
}

void ElectricMotorActuatorInfoReader::characters(const std::string& ch)
{
    if( !!_pcurreader ) {
        _pcurreader->characters(ch);
    }
    else {
        _ss.clear();
        _ss << ch;
    }
}


HierarchicalXMLReader::HierarchicalXMLReader(const std::string& xmlid, const AttributesList& atts) : _xmlid(xmlid)
{
    _readable.reset(new HierarchicalXMLReadable(xmlid,atts));
}

BaseXMLReader::ProcessElement HierarchicalXMLReader::startElement(const std::string& name, const AttributesList& atts)
{
    if( !!_pcurreader ) {
        if( _pcurreader->startElement(name, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }

    _pcurreader.reset(new HierarchicalXMLReader(name,atts));
    return PE_Support;
}

bool HierarchicalXMLReader::endElement(const std::string& name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(name) ) {
            _readable->_listchildren.push_back(_pcurreader->_readable);
            _pcurreader.reset();
        }
        return false;
    }

    if( name == _xmlid ) {
        return true;
    }
    RAVELOG_ERROR(str(boost::format("invalid xml tag %s, expected %s\n")%name%_xmlid));
    return false;
}

void HierarchicalXMLReader::characters(const std::string& ch)
{
    if( !_pcurreader ) {
        _readable->_data += ch;
    }
    else {
        _pcurreader->characters(ch);
    }
}

XMLReadablePtr HierarchicalXMLReader::GetReadable()
{
    return _readable;
}


StreamXMLWriter::StreamXMLWriter(const std::string& xmltag, const AttributesList& atts) : _xmltag(xmltag), _atts(atts)
{
}

const std::string& StreamXMLWriter::GetFormat() const
{
    static const std::string format("xml");
    return format;
}

void StreamXMLWriter::SetCharData(const std::string& data)
{
    _data = data;
}

BaseXMLWriterPtr StreamXMLWriter::AddChild(const std::string& xmltag, const AttributesList& atts)
{
    std::shared_ptr<StreamXMLWriter> child(new StreamXMLWriter(xmltag,atts));
    _listchildren.push_back(child);
    return child;
}

void StreamXMLWriter::Serialize(std::ostream& stream)
{
    if( _xmltag.size() > 0 ) {
        stream << "<" << _xmltag << " ";
        FOREACHC(it, _atts) {
            stream << it->first << "=\"" << it->second << "\" ";
        }
        // don't skip any lines since could affect reading back _data
        stream << ">";
    }
    if( _data.size() > 0 ) {
        if( _xmltag.size() > 0 ) {
            stream << "<![CDATA[" << _data << "]]>";
        }
        else {
            // there's no tag, so can render plaintext
            stream << _data;
        }
    }
    FOREACHC(it, _listchildren) {
        (*it)->Serialize(stream);
    }
    if( _xmltag.size() > 0 ) {
        stream << "</" << _xmltag << ">" << std::endl;
    }
}


} // xmlreaders
} // OpenRAVE
